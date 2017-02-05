(load "package://eus_qpoases/euslisp/eus-qpoases.l")
(load "log-tools.l")
(unless (boundp '*irtviewer*) (make-irtviewer))
(send *irtviewer* :change-background #f(0.5 0.5 0.5))

(defun update-robot-state (robot-state &key (mode :reference-root) (rleg-fix nil))
  (let (target-coord)
    (when rleg-fix
      (setq target-coord (send *robot* :rleg :end-coords :copy-worldcoords)))
    (send *robot* :angle-vector (send robot-state :angle-vector))

    (cond
     ( (equal mode :reference-root)
       (send *robot* :newcoords (send robot-state :reference-root-coords))
       (send *robot* :angle-vector (send robot-state :reference-angle-vector)) )
     ( (equal mode :root) (send *robot* :newcoords (send  robot-state :root-coords)) )
     ( (equal mode :lleg) (send *robot* :fix-leg-to-coords (make-coords) :lleg) )
     ( (equal mode :rleg) (send *robot* :fix-leg-to-coords (make-coords) :rleg) )
     (t (warn "not supported mode ~s~%" mode) )
     )

    (when rleg-fix
      (send *robot* :rleg :inverse-kinematics target-coord))
    ))

(defun get-local-force-point-list (left-top right-bottom grid)
  (let* ( (rows (car grid)) (cols (cadr grid))
          (lt-rb (v- right-bottom left-top))
          (x-step (/ (elt lt-rb 0) cols)) (y-step (/ (elt lt-rb 1) rows))
          (force-point-list nil) )
    (dotimes (i (1+ rows))
      (dotimes (j (1+ cols))
        (push (v+ left-top (float-vector (* j x-step) (* i y-step) 0)) force-point-list)
        ))
    (reverse force-point-list)
  ))

(defun get-convert-matrix (cur-pos cur-rot v-dt w-dt mu &key (draw nil) (size 1) (grid '(1 1)) (flush nil)
                                   (vertex-list (list (scale 1e-3 #f(130 75 0)) (scale 1e-3 #f(-100 -55 0)))));; '(left-top right-bottom) [m]
  (let ( (a-list nil) (b-list nil) (vi-dt) (force-point-list nil) (dxy-list nil)
         (cur-pos (scale 1e-3 cur-pos)) (v-dt (scale 1e-3 v-dt));; [m]
         (ri-local-list (if (< (length vertex-list) 3)
                            (get-local-force-point-list (car vertex-list) (cadr vertex-list) grid)
                            vertex-list))
         )
    (dolist (ri-local ri-local-list);; [m] <- left-top, right-bottom

      (setq ri (send (make-coords :rot cur-rot) :transform-vector  ri-local);; 3dof world [m]
            vi-dt (v+ v-dt (v* w-dt ri));; 3dof world [m]
            dxy (subseq (normalize-vector vi-dt) 0 2));; 2dof world [m]

      (if draw (send *viewer* :draw-arrow (scale 1e3 (v+ cur-pos ri)) (scale 1e3 (v+ (v+ cur-pos ri) (scale size vi-dt))) :color #f(0 1 1)))

      (setq r-2dof-closs-vec (float-vector (elt ri 1) (* -1 (elt ri 0))));; 2dof [m]
      (push (transpose (matrix (concatenate float-vector #f(1) r-2dof-closs-vec))) a-list)
      (push (transpose (matrix (concatenate float-vector
                                            (scale mu (concatenate float-vector (scale -1 dxy)))
                                            #f(1)
                                            r-2dof-closs-vec
                                            (float-vector (* mu (v. dxy r-2dof-closs-vec))))
                                            )) b-list)
      (push (scale 1e3 (v+ cur-pos ri)) force-point-list);; [mm]
      (push dxy dxy-list)
      )

    (if flush (send *viewer* :flush))

    (list (apply #'concatenate-matrix-row (reverse a-list))
          (apply #'concatenate-matrix-row (reverse b-list))
          (reverse force-point-list)
          (reverse dxy-list))
  ))

(defun calc-distributed-fz (&key (force-vec) (moment-vec) (cur-pos #f(0 0 0)) (cur-rot (unit-matrix)) (v-dt #f(0 0 0)) (w-dt #f(0 0 0)) (mu 0.5) (grid '(1 1)) (flush nil)
                                 (vertex-list '(#f(0.5 0 0) #f(-0.5 0 0) #f(0 0.01 0) #f(0 -0.01 0))) (unilateral t) (draw nil))
  (let ((color-list '(#f(1 0 0) #f(0 1 0) #f(0 0 1))) (distributed-fz-vec) (point-num))

    (multiple-value-setq (A B force-point-list dxy-list) (get-convert-matrix cur-pos cur-rot v-dt w-dt mu :draw draw :size 3 :grid grid :vertex-list vertex-list));; [m]
    (setq point-num (length (matrix-row A 0)))

    (if unilateral
        (setq distributed-fz-vec (solve-qpoases-qp ;; with inequality constraints
                                  :eval-weight-matrix (unit-matrix point-num)
                                  :eval-coeff-vector (make-array point-num :element-type float-vector :initial-element 0)
                                  :inequality-matrix (unit-matrix point-num)
                                  :inequality-min-vector (make-array point-num :element-type float-vector :initial-element 0)
                                  :equality-matrix A
                                  :equality-vector (float-vector (elt force-vec 2) (elt moment-vec 0) (elt moment-vec 1))))
      (setq distributed-fz-vec (solve-qpoases-qp ;; without inequality constraints
                                :eval-weight-matrix (unit-matrix point-num)
                                :eval-coeff-vector (make-array point-num :element-type float-vector :initial-element 0)
                                :equality-matrix A
                                :equality-vector (float-vector (elt force-vec 2) (elt moment-vec 0) (elt moment-vec 1))))
      )

    (when distributed-fz-vec
      ;; draw foot
      (send (instance face :init :vertices (quickhull force-point-list)) :draw-on :flush nil :color #f(0 0 1))

      ;; draw distributed force
      (mapc #'(lambda (force-point fz dxy)
                (send *viewer* :draw-arrow force-point (v+ force-point (concatenate float-vector (scale 1 dxy) #f(0))) :color #f(1 1 1))
                (send *viewer* :draw-arrow force-point (v+ force-point (concatenate float-vector (scale (* mu fz -1) dxy) (float-vector fz))) :color #f(1 1 1)))
            force-point-list (coerce distributed-fz-vec cons) dxy-list)

      ;; draw moment
      (dotimes (i 3)
        ;; (send *viewer* :draw-arrow cur-pos (v+ cur-pos (scale (elt moment-vec i) (matrix-column cur-rot i))))
        (send *viewer* :draw-arrow cur-pos (v+ cur-pos (scale (* 10 (elt (send (make-coords :rot cur-rot) :inverse-transform-vector moment-vec) i)) (matrix-column cur-rot i))) :color (elt color-list i))
        )

      ;; draw cop
      (send *irtviewer* :viewer :draw-circle (make-coords :pos (v+ cur-pos (scale 1000 (float-vector (/ (elt moment-vec 1) (elt force-vec 2) -1) (/ (elt moment-vec 0) (elt force-vec 2)) 0)))) :radius 10 :flush nil)
      )

    (when flush
      (send *viewer* :flush)
      (send *viewer* :draw-objects :flush nil))

    (list A B distributed-fz-vec)
    ))

(defun estimate-foot-force-distribution (&key (grid '(1 1)) (step 20) (mode :reference-root) (mu 0.6) (sleep-time 0) (start-time 5) (stop-time 15) (rleg-fix t)
                                              (vertex-list (list (scale 1e-3 #f(130 75 0)) (scale 1e-3 #f(-100 -55 0)))))
  (let ( (xrange (calc-xrange (list start-time stop-time) *rs-list*))
                 last-pos last-rot cur-coord cur-pos cur-rot (dt (calc-dt *rs-list*))v-dt w-dt r-2dof-closs-vec A B
                 point-num force-vec moment-vec distributed-fz-vec
                 (color-list '(#f(1 0 0) #f(0 1 0) #f(0 0 1)))
                 (tm-list nil) (act-fx-list nil) (estimated-fx-list nil) (act-fy-list nil) (estimated-fy-list nil)
                 (act-nz-list nil) (estimated-nz-list nil) (vy-list nil) (wz-list nil))
    (warn "~%estimate-foot-force-distribution()~%")
    (warn "~s->~s[sec]~%" (car xrange) (cadr xrange))
    (send *robot* :reset-pose);; for rleg-fix
    (update-robot-state (elt *rs-list* (car xrange)) :mode mode :rleg-fix rleg-fix)
    (setq last-pos (send *robot* :lleg :end-coords :worldpos)
          last-rot (send *robot* :lleg :end-coords :worldrot))

    (let ((i (car xrange)))
      (do-until-key-with-check (< i (cadr xrange))
       (when (= (mod i step) 0)
         (setq rs (elt *rs-list* i))
         (setq cur-coord (send *robot* :lleg :end-coords :copy-worldcoords)
               cur-pos (send cur-coord :worldpos)
               cur-rot (send cur-coord :worldrot))

         ;; (setq force-vec (scale -1 (elt (car (send rs :wrench-list)) 1));; left foot
         ;;       moment-vec (elt (cadr (send rs :wrench-list)) 1));; left foot
         (setq force-vec (send rs :get :lleg-force-vector)
               moment-vec (send rs :get :lleg-moment-vector))
         (format t "~s[sec] force: ~s  moment: ~s~%" (* i dt) force-vec moment-vec)

         (setq v-dt (v- cur-pos last-pos);; [mm]
               w-dt (matrix-log (m* cur-rot (inverse-matrix last-rot))));; [rad]

         (multiple-value-setq (A B distributed-fz-vec) (calc-distributed-fz :force-vec force-vec :moment-vec moment-vec :cur-pos cur-pos :cur-rot cur-rot
                                                                          :v-dt v-dt :w-dt w-dt :mu mu :grid grid :vertex-list vertex-list))
         (when distributed-fz-vec
           ;; fx,fy,nz plot
           (let* ((convert-coord (make-coords :rot cur-rot))
                  (local-force  (send convert-coord :inverse-transform-vector force-vec))
                  (local-moment (send convert-coord :inverse-transform-vector moment-vec))
                  (world-wrench (m* B (transpose (matrix distributed-fz-vec))))
                  (local-estimated-force  (send convert-coord :inverse-transform-vector (subseq world-wrench 0 3)))
                  (local-estimated-moemnt (send convert-coord :inverse-transform-vector (subseq world-wrench 3 6))))
             (push-back (* i dt) tm-list)
             (push-back (elt local-force 0)  act-fx-list)
             (push-back (elt local-force 1)  act-fy-list)
             (push-back (elt local-moment 2) act-nz-list)
             (push-back (elt local-estimated-force 0)  estimated-fx-list)
             (push-back (elt local-estimated-force 1)  estimated-fy-list)
             (push-back (elt local-estimated-moemnt 2) estimated-nz-list)
             (push-back (* 1e-3 (/ (elt v-dt 1) (* dt step))) vy-list);; [m/sec]
             (push-back (/ (rad2deg (elt w-dt 2)) (* dt step)) wz-list);; [rad/sec]
             )
           )

         (setq last-pos cur-pos
               last-rot cur-rot)
         (update-robot-state rs :mode mode :rleg-fix rleg-fix)
         (send *viewer* :flush)
         (send *viewer* :draw-objects :flush nil)
         (unix:usleep sleep-time)
         )
       (incf i)
       )
      )
    ;; (graph-view-with-filter (list act-nz-list estimated-nz-list act-fy-list estimated-fy-list wz-list) tm-list
    ;;                         :keylist '("act nz" "estimated nz" "act fy" "estimated fy" "wz")
    (graph-view-with-filter (list act-nz-list estimated-nz-list (mapcar #'(lambda (wz) (* wz 0.2)) wz-list)) tm-list
                            :keylist '("act nz" "estimated nz" "wz*0.2")
                            :xlabel "Time [s]"  :ylabel "nz[Nm]"
                            :xrange (list start-time stop-time)
                            :title "Comparison between act and estiamted wrenches"
                            )
    (list tm-list wz-list act-nz-list estimated-nz-list)
    ))

(defun compare-grid-difference (&key (start-time 7.8) (stop-time 8.7) (step 3) (grid-list '((1 1) (2 2) (3 3) (4 4))) (mu 0.6) (save nil) (add-fname nil))
  (let (estimated-nz-lists tm-list wz-list act-nz-list estimated-nz-list (fname "Comparison of yaw moment estimation among division number"))
    (setq estimated-nz-lists 
          (mapcar #'(lambda (grid) 
                      (multiple-value-setq (tm-list wz-list act-nz-list estimated-nz-list) (estimate-foot-force-distribution :grid grid :step step :sleep-time 0 :mode :reference-root :start-time start-time :stop-time stop-time :mu mu))
                      estimated-nz-list)
                  grid-list))
    (graph-view-with-filter (append (list (mapcar #'(lambda (wz) (* wz 0.2)) wz-list) act-nz-list) estimated-nz-lists) tm-list
                            :keylist (append '("yaw angular velocity" "actual moment z") (mapcar #'(lambda (grid) (format nil "estimated moment z(~sx~s division)" (1+ (car grid)) (1+ (cadr grid)))) grid-list))
                            :xlabel "Time [s]"  :ylabel "moment z[Nm]"
                            :xrange (list start-time stop-time)
                            :title fname
                            :graph-fname (reg-replace* " " "_" (if add-fname (format nil "~a ~a.eps" fname add-fname) (format nil "~a.eps" fname)))
                            :dump-graph save
                            )
  ))
