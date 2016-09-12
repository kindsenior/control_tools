(load "play-hrpsysfile.l")

(defun move-foot (&key (translate-vec #f(100 0 0)) (yaw-angle 30) (tm 10000) (fz 100) (nr 0) (np 1) (p 0) (r 0) (limb :lleg) )
  (let ( (target-coord) (init-pose (send *robot* :angle-vector)) (return-pose) (offset-vec #f(0 0 0)) )

    (unix:sleep 5)

    (send *robot* :fix-leg-to-coords (make-coords) :rleg)

    ;; (send *ri* :start-impedance limb :force-gain #f(0 0 0.2) :moment-gain #f(0.8 0.8 0) :m-p 10 :d-p 2500 :k-p 20 :d-r 100 :k-r 20)
    ;; (send *ri* :set-ref-force (float-vector 0 0 fz) 5000 limb)
    ;; ;; (send *ri* :wait-interpolation-seq)
    ;; (send *ri* :set-ref-moment (float-vector nr np 0) 5000 limb)
    ;; (send *ri* :wait-interpolation-seq)
    
    (send *robot* :lleg :ankle-p :joint-angle (+ (send *robot* :lleg :ankle-p :joint-angle) p))
    (send *robot* :lleg :ankle-r :joint-angle (+ (send *robot* :lleg :ankle-r :joint-angle) r))
    (send *ri* :angle-vector (send *robot* :angle-vector))
    (send *ri* :wait-interpolation)


    (setq target-coord (send *robot* limb :end-coords :copy-worldcoords)
          return-pose (send *robot* :angle-vector))

    (send *ri* :start-log)

    (send target-coord :translate translate-vec)
    (send target-coord :rotate (deg2rad yaw-angle) :z)
    (send *robot* limb :inverse-kinematics target-coord)
    (setq target-pose (send *robot* :angle-vector))

    (send *ri* :angle-vector target-pose tm)
    (send *ri* :wait-interpolation)
    (print (send *ri* :state :force-vector))
    (send *ri* :angle-vector return-pose tm)
    (send *ri* :wait-interpolation)
    (print (send *ri* :state :force-vector))

    ;; (send *ri* :stop-impedance :legs)
    ;; (save-log :fname (format nil "move-foot_~s,~s,~svec_~syaw_~stm_~sfz_~snr_~snp" (elt translate-vec 0) (elt translate-vec 1) (elt translate-vec 2) yaw-angle tm fz nr np))
    (save-log :fname (format nil "move-foot_paper_~s,~s,~svec_~syaw_~stm_~sp_~sr" (elt translate-vec 0) (elt translate-vec 1) (elt translate-vec 2) yaw-angle tm p r))

    (send *ri* :angle-vector (send *robot* :angle-vector init-pose))
    (send *ri* :wait-interpolation)

    ))
