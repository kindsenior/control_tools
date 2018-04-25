(defmacro create-robot-instance (&optional (robot nil))
  (unless robot (setq robot (read-from-string (unix:getenv "ROBOT"))))
  (warning-message 3 "~%loading robot interface file (~a)...~%" robot)
  (list
   'progn
   (list 'require
         (read-from-string (format nil ":~a-interface" robot))
         (read-from-string (format nil "\"package://hrpsys_ros_bridge_tutorials/euslisp/~a-interface.l\"" robot)))
   (list 'setq
         (read-from-string (format nil "*~a*" robot)) (list 'instance (read-from-string (format nil "~a-robot" robot)) :init)
         '*robot* (read-from-string (format nil "*~a*" robot)))
   )
  )

(if (equal (unix:getenv "ROBOT") "")
    (warning-message 1 "!!!!!!!!!!No ROBOT TYPE!!!!!!!!!!~%")
    (unless (boundp '*robot*) (create-robot-instance)))

;; disable needless module loading
(unless (assoc :hrp2jsk-interface     *modules*) (nconc *modules* (list (list* :hrp2jsk-interface     nil))))
(unless (assoc :hrp2jsknt-interface   *modules*) (nconc *modules* (list (list* :hrp2jsknt-interface   nil))))
(unless (assoc :hrp2jsknts-interface  *modules*) (nconc *modules* (list (list* :hrp2jsknts-interface  nil))))
(unless (assoc :urataleg-interface    *modules*) (nconc *modules* (list (list* :urataleg-interface    nil))))
(unless (assoc :staro-interface       *modules*) (nconc *modules* (list (list* :staro-interface       nil))))
(unless (assoc :jaxon-interface       *modules*) (nconc *modules* (list (list* :jaxon-interface       nil))))
(unless (assoc :jaxon_red-interface   *modules*) (nconc *modules* (list (list* :jaxon_red-interface   nil))))
(unless (assoc :samplerobot-interface *modules*) (nconc *modules* (list (list* :samplerobot-interface nil))))

;; for log-parse
;; (require "package://hrpsys_ros_bridge/euslisp/datalogger-log-parser.l")
;; (require "euslib/irteus_proposals/motion-lib-proposal.l")
(warning-message 3 "loading test-st-state-estimate-data.l...~%")
;; (require (format nil "~a/ros/~a/src/jsk-ros-pkg/euslib/demo/nozawa/hrpsys-test/test-st-state-estimate-data.l" (unix:getenv "HOME") (unix:getenv "ROS_DISTRO")))
(require "~/prog/euslib/demo/nozawa/hrpsys-test/test-st-state-estimate-data.l")
(defvar *usage-funcs* nil)
(unless (assoc :nozawa-log-plotter *usage-funcs*)
  (push (cons :nozawa-log-plotter
              #'(lambda ()
                  (warn "~%hrpsys-test/test-st-state-estimate-data.l~%")
                  (log-plot-usage)
                  (warn "~%"))) *usage-funcs*))

(load "euslib-tools.l")
(load "util.l")

(unless (assoc :hrpsys-base-log-parser *usage-funcs*)
  (push (cons :hrpsys-base-log-parser
              #'(lambda ()
                  (warn "~%hrpsys-base-log-parser.l~%")
                  (warning-message 1 "Execute in log dir!!~%")
                  (warning-message 1 "Or log file name has to correspond to log directory name!!~%")
                  (warning-message 3 "(create-robot-instance jaxon)~%")
                  (warning-message 3 "(prepare-plot :robot hrp2jsknts-robot)~%")
                  (warning-message 3 "(plot-rs :mode :ee-pos :axis 2 :proc-mode :press-enter :yrange (list 0 0.1) :robot hrp2jsknts-robot :save t :create t)~%")
                  (warning-message 3 "(plot-foot-force :axis 2 :save t :create t)~%")
                  (warning-message 3 "(plot-foot-cop :axis 2 :save t :create t)~%")
                  (warning-message 3 "(plot-imu :fname \"HRP2JSKNTS/go-vel-test_hoge\" :axis 1 :start-tm 7 :stop-tm 8 :robot hrp2jsknts-robot :save t :create t :correct-offset t)~%")
                  (warn "~%"))) *usage-funcs*))

(defun create-log-interface (fname-with-path &key (robot))
  (instance datalogger-log-parser-controller :init fname-with-path :robot robot) )

(defun create-rs-list (&key (ri *log*))
  ;; (setq *rs-list* (convert-to-robot-state-data-from-ri :ri *log*))
  (convert-to-robot-state-data-for-st-debug)
  (add-data-to-rs-list)
  *rs-list*
  )

(defun byte-convert (val &key from to)
  (let ((str (make-string 4)))
    (sys::poke val str 0 from)
    (sys::peek str 0 to)
  ))

(defun convert-to-servo-alarm (val) (ash (logand #x0007fff8 val) -3))

;; デフォルトではない値を*rs-list*に追加
(defun add-data-to-rs-list ()
  (let* ((ret) (init-tm) (idx 0) (urata-len 17) (num-joints (length (send *robot* :joint-list))) (is-real-machine)
         (limb-list
          (mapcar #'(lambda (x) (find-if #'(lambda (l) (member x (send (send *log* :robot) l :force-sensors))) '(:rleg :lleg :rarm :larm)))
                  (send (send *log* :robot) :force-sensors)))
         (limb-idx (let ((cnt -1)) (mapcar #'(lambda (x) (incf cnt)) limb-list))))
    (warn "(add-data-to-rs-list)~%")
    (do-until-key-with-check
     (progn (null-output (send *log* :state)) (if (find-method *log* :eof-p) (not (send *log* :eof-p))))
     (if (= (mod idx 100) 0) (warn " ~a" idx))
     (unless init-tm
       (setq init-tm (send *log* :time)))
     ;; use reference from st, instead of sh
     (send *log* :set-robot-state1
           :reference-vector
           ;; (map float-vector #'rad2deg (send (send self :parser-list "sh_q") :read-state)))
           (map float-vector #'rad2deg (send (send *log* :parser-list "sh_qOut") :read-state)))
     ;; (map float-vector #'rad2deg (send (send *log* :parser-list "sh_q") :read-state)))
     (if (send *log* :parser-list "RobotHardware0_servoState")
           (let ((raw-state (coerce (send (send *log* :parser-list "RobotHardware0_servoState") :read-state) cons)))
             (setq is-real-machine (= (length raw-state) (* num-joints urata-len)))
             (if is-real-machine
                 (send *log* :set-robot-state1
                       :servo-alarm
                       (mapcar #'(lambda (x) (convert-to-servo-alarm (byte-convert x :from :float :to :integer)))
                               (mapcar #'(lambda (joint-idx) (elt raw-state (* joint-idx urata-len))) (range num-joints)))))
             ))

     (let ((rs (elt *rs-list* idx)))
       (send rs :put :sh-lleg-ref-force (send (send *log* :parser-list "sh_lfsensorOut") :read-state))
       (send rs :put :sh-rleg-ref-force (send (send *log* :parser-list "sh_rfsensorOut") :read-state))
       (send rs :put :sh-larm-ref-force (send (send *log* :parser-list "sh_lhsensorOut") :read-state))
       (send rs :put :sh-rarm-ref-force (send (send *log* :parser-list "sh_rhsensorOut") :read-state))
       (send rs :put :stabilizer-end-coords-list (send *log* :stabilizer-end-coords-list))
       (send rs :put :imucoords (send *log* :imucoords))
       (if is-real-machine (send rs :put :servo-alarm (send *log* :get-robot-state :servo-alarm)))
       (setf (elt *rs-list* idx) rs)
       )
     (incf idx)
     )
    (warn "~% finished (add-data-to-rs-list)~%")
    (length *rs-list*)))

(defun plot-rs (&key (fname nil) (mode :root-rot) (axis 2) (limb nil) (proc-mode :sleep) (robot hrp2jsknts-robot)
                     (save nil) (xrange nil) (yrange nil) (create nil) (rs-list *rs-list*))
  (let* ( (xrange (calc-xrange xrange rs-list)) (data (subseq rs-list (car xrange) (cadr xrange)))
         fname-with-path start-idx stop-idx )

    (setq fname-with-path (prepare-plot :fname fname :create create :robot robot));; rs-list生成 fname-with-pathをparse

    (setq start-idx 0 stop-idx (length *rs-list*))

    ;; (when xrange
    ;;   (setq dt (calc-dt *rs-list*) start-idx (round (/ (car xrange) dt)) stop-idx (round (/ (cadr xrange) dt)) ))


    ;; (send *gp* :command "set grid;set size 1,1")
    (if yrange (set-yrange yrange))
    ;; (plot-robot-state-data-list (subseq *rs-list* start-idx stop-idx) :mode mode :axis axis :limb limb :robot (send *log* :robot) :proc-mode proc-mode)
    (plot-robot-state-data-list data :mode mode :axis axis :limb limb :robot (send *log* :robot) :proc-mode proc-mode)

    (when save
      ;; (send *gp* :command "set grid;set size 2,1")
      ;; (plot-robot-state-data-list (subseq *rs-list* start-idx stop-idx) :mode mode :axis axis :limb limb
      ;;                             :robot (send *log* :robot) :proc-mode :sleep :fname (concatenate string fname-with-path "_"))
      (plot-robot-state-data-list data :mode mode :axis axis :limb limb
                                  :robot (send *log* :robot) :proc-mode :sleep :fname (concatenate string fname-with-path "_"))

      ;; (mapc #'(lambda (val) (if (reg-match ".eps" val) (unix:system (format nil "convert ~s ~s" val (reg-replace ".eps" ".png" val))))) (directory))
      )

    ;; (unix:system "beep")
    ))

(defun plot-imu (&key (fname nil) (axis 1) (proc-mode :sleep) (save nil) (yrange nil) 
                      (robot hrp2jsknts-robot) (create nil) (correct-offset nil)
                      (start-tm nil) (stop-tm nil))
  (let* ( (imu-msg) (init-tm)
         (ylabel "rot [deg]") 
         fname-with-path
         (gname (format nil "chest-rpy ~A ~A" (case axis (0 "y") (1 "p") (2 "r")) (read-from-string ylabel))))

    (prepare-plot :fname fname :create create :robot robot :create create);; rs-list生成
    (setq tm-list (mapcar #'(lambda (x) (send x :time)) *rs-list*))

    (if (not (boundp '*gp*)) (setq *gp* (gnuplot)))

    (cond
     ( (equal robot hrp2jsknts-robot)
       (if (not (boundp '*hrp2jsknts*)) (hrp2jsknts-init))
       (setq *robot* *hrp2jsknts*) )
     ( (equal robot testmdofarm-robot)
       (if (not (boundp '*testmdofarm*)) (testmdofarm-init))
       (setq *robot* *testmdofarm*) )
     (t (warning-message 1 "Robot-instance not defined"))
     )

    ;; create imu list
    ;; (when (or (not (boundp 'true-p-list)) (not (boundp 'sensor-p-list)) create)
      (setq sensor-p-list nil  true-p-list nil)
      (warning-message 3 "now creating imu list~%")
      (do-until-key-with-check
       (progn (send *log* :state) (if (find-method *log* :eof-p) (not (send *log* :eof-p))))
       (progn
         (unless init-tm (setq init-tm (send *log* :time)))

         ;; (car (rpy-angle (quaternion2matrix (send (send *log* :imu) :orientation))))
         ;; (send *robot* :newcoords (make-coords)) (send *robot* :angle-vector (send *log* :potentio-vector)) (car (rpy-angle (car (send *robot* :imu-sensors))))

         (setq imu-msg (send (cdr (assoc :imu ( *log* . robot-state))) :orientation))
         (push-back 
          (rad2deg (elt (car (rpy-angle 
                              ;; (quaternion2matrix (float-vector  (send imu-msg :w) (send imu-msg :x) (send imu-msg :y) (send imu-msg :z) ))
                              (ros::tf-quaternion->rot imu-msg)
                              )) axis))
          sensor-p-list)

         (send *robot* :newcoords (make-coords)) (send *robot* :angle-vector (send *log* :potentio-vector))
         (push-back (rad2deg (elt (car (rpy-angle (send  (car (send *robot* :imu-sensors)) :worldrot))) axis)) true-p-list)
         ))
      (when correct-offset (setq true-p-list (mapcar #'(lambda (val) (+ val (car sensor-p-list))) true-p-list)))
      ;; )
    
    (setq start-tm (if start-tm start-tm (car tm-list))
          stop-tm  (if stop-tm  stop-tm  (car (last tm-list))) )

    ;; (send *gp* :command "set xlabel \"Time[sec]\"") (send *gp* :command "set ylabel \"rot [deg]\"")
    ;; (send *gp* :command "set tics font \"Helvetica,10\"")
    ;; (send *gp* :command "set xlabel font \"Helvetica,10\"") (send *gp* :command "set ylabel font \"Helvetica,10\"")
    (send *gp* :command "set grid;set size 1,1")
    (graph-view-with-filter (list true-p-list sensor-p-list) tm-list 
                            :keylist '("true" "sensor")
                            :xlabel "Time [s]"  :ylabel ylabel
                            :xrange (list start-tm stop-tm) :yrange yrange
                            :title gname
                            )

    (when save
      (send *gp* :command "set grid;set size 2,1")
      (graph-view-with-filter (list true-p-list sensor-p-list) tm-list 
                              :keylist '("true" "senseor")
                              :xlabel "Time [s]" :ylabel ylabel
                              :xrange (list start-tm stop-tm) :yrange yrange
                              :title gname
                              :dump-graph t
                              :graph-fname (format nil "~A~A.eps" fname-with-path (reg-replace* " " "_" gname))
                              )
      (mapc #'(lambda (val) (if (reg-match ".eps" val) (unix:system (format nil "convert ~a ~a" val (reg-replace ".eps" ".png" val))))) (directory))
      )

    ))

(defun plot-mu
  (&key (rs-list *rs-list*)
        (add-fname nil) (fname "Friction Coefficient")
        (cutoff-f nil) (xrange nil) (limb '(:lleg :rleg)) (save t) )
  (let* ( (xrange (calc-xrange xrange rs-list)) (data (subseq rs-list (car xrange) (cadr xrange))))
    (graph-view
     (delete nil 
             (list (if (find :rleg limb)
                       (low-pass-filter (mapcar #'(lambda (x) (/ (norm (subseq (send x :get :rleg-force-vector) 0 2)) (elt (send x :get :rleg-force-vector) 2))) data)
                                        :cutoff-f cutoff-f))
                   (if (find :lleg limb)
                       (low-pass-filter (mapcar #'(lambda (x) (/ (norm (subseq (send x :get :lleg-force-vector) 0 2)) (elt (send x :get :lleg-force-vector) 2))) data) 
                                        :cutoff-f cutoff-f))))
     (mapcar #'(lambda (x) (send x :time)) data)
     :keylist (delete nil (list (if (find :rleg limb) "Right Foot(stop)") (if (find :lleg limb) "Left Foot(slipping)")))
     :title fname
     :graph-fname (reg-replace* " " "_" (if add-fname (format nil "~a ~a.eps" fname add-fname) (format nil "~a.eps" fname)))
     :dump-graph save
     :xlabel "Time[s]"
     :ylabel "Friction Coefficient"
     )
    ))

(defun calc-dt (target-list)
  (let ( (accuracy 0.001) )
    (* (round (/ (- (send (cadr target-list) :time) (send (car target-list) :time)) accuracy)) accuracy)
  ))

;; (defun plot-foot-pos-rot
(defun plot-foot-rot
  (&key (rs-list *rs-list*)
        (add-fname nil) (fname "Reference foot yaw angle") ;; (fname "Reference foot position and yaw angle")
        (axis 0) (cutoff-f nil) (xrange nil) (yrange) (limb '(:lleg :rleg)) (save t))
  (let* ( (xrange (calc-xrange xrange rs-list)) (data (subseq *rs-list* (car xrange) (cadr xrange))) (ly-list nil) (lyaw-list nil) (ry-list nil) (ryaw-list nil))
    (if yrange (set-yrange yrange))
    (mapc #'(lambda (rs)
              (send *robot* :newcoords (send rs :root-coords))
              (send *robot* :angle-vector (send rs :reference-angle-vector))
              (when (find :rleg limb)
                (push-back (elt (send *robot* :rleg :end-coords :worldpos) 1) ry-list)
                (push-back (rad2deg (elt (car (rpy-angle (send *robot* :rleg :end-coords :worldrot))) 0)) ryaw-list))
              (when (find :lleg limb)
                (push-back (elt (send *robot* :lleg :end-coords :worldpos) 1) ly-list)
                (push-back (- (rad2deg (elt (car (rpy-angle (send *robot* :lleg :end-coords :worldrot))) 0)) 0.5) lyaw-list)))
          data)
    (graph-view
     (delete nil (list 
                  (if (find :rleg limb) ryaw-list)
                  ;; (if (find :rleg limb) ry-list)
                  (if (find :lleg limb) lyaw-list)
                  ;; (if (find :lleg limb) ly-list)
                  ))
     (mapcar #'(lambda (x) (send x :time)) data)
     :keylist (delete nil (list (if (find :rleg limb) "rleg yaw") ;; (if (find :rleg limb) "rleg y")
                                (if (find :lleg limb) "lleg yaw") ;; (if (find :lleg limb) "lleg y")
                                ))
     :title fname
     :graph-fname (reg-replace* " " "_" (if add-fname (format nil "~a ~a.eps" fname add-fname) (format nil "~a.eps" fname)))
     :dump-graph save
     :xlabel "Time[s]"
     ;; :ylabel "Position[m] or angle[degree]"
     :ylabel "Angle[degree]"
     )
    ))

(defun plot-foot-force
  (&key (rs-list *rs-list*)
        (add-fname nil) (fname "Actual and Reference and Modified Reference Force")
        (axis 2) (cutoff-f nil) (xrange nil) (limb '(:lleg :rleg)) (save t))
  (let* ( (xrange (calc-xrange xrange rs-list)) (data (subseq *rs-list* (car xrange) (cadr xrange))))
    (graph-view
     (delete nil (list (if (find :rleg limb) (low-pass-filter (mapcar #'(lambda (x) (elt (send x :get :rleg-off-force-vector) axis)) data) :cutoff-f cutoff-f))
                       (if (find :lleg limb) (low-pass-filter (mapcar #'(lambda (x) (elt (send x :get :lleg-off-force-vector) axis)) data) :cutoff-f cutoff-f))
                       ;; (if (find :rleg limb) (mapcar #'(lambda (x) (elt (send x :get :sh-rleg-ref-force) axis)) data))
                       ;; (if (find :lleg limb) (mapcar #'(lambda (x) (elt (send x :get :sh-lleg-ref-force) axis)) data))
                       (if (find :rleg limb) (mapcar #'(lambda (x) (elt (send x :get :st-rleg-ref-force) axis)) data))
                       (if (find :lleg limb) (mapcar #'(lambda (x) (elt (send x :get :st-lleg-ref-force) axis)) data))))
     (mapcar #'(lambda (x) (send x :time)) data)
     :keylist (delete nil (list (if (find :rleg limb) "act rleg force z") (if (find :lleg limb) "act lleg force z")
                                (if (find :rleg limb) "ref rleg force z") (if (find :lleg limb) "ref lleg force z")
                                ;; (if (find :rleg limb) "modified ref rleg force z") (if (find :lleg limb) "modified ref lleg force z")
                                ))
     :title ""
     :graph-fname (reg-replace* " " "_" (if add-fname (format nil "~a ~a.eps" fname add-fname) (format nil "~a.eps" fname)))
     :dump-graph save
     :xlabel "Time [s]"
     :ylabel "Foot force z [N]"
     )
    ))

(defun calc-cop (axis force-vector moment-vector)
  (/ (elt moment-vector (if (= axis 0) 1 0)) (elt force-vector 2) (if (= axis 0) -1 1)))

(defun plot-foot-cop
  (&key (rs-list *rs-list*)
        (add-fname nil) (fname "Actual and Reference COP")
       (axis 0) (cutoff-f nil) (xrange nil) (yrange) (start-offset 0) (limb '(:lleg :rleg)) (save nil))
  (let* ( (xrange (calc-xrange xrange rs-list)) (data (subseq *rs-list* (car xrange) (cadr xrange))))
    (if yrange (set-yrange yrange))
    (if save (set-plot-common-config))
    (graph-view
     (delete nil (list (if (find :rleg limb) (mapcar #'(lambda (x) (calc-cop axis (subseq (send x :get :sh-rleg-ref-force) 0 3) (subseq (send x :get :sh-rleg-ref-force) 3 6))) data))
                       (if (find :rleg limb) (low-pass-filter (mapcar #'(lambda (x) (calc-cop axis (send x :get :rleg-force-vector) (send x :get :rleg-moment-vector))) data) :cutoff-f cutoff-f))
                       (if (find :lleg limb) (mapcar #'(lambda (x) (calc-cop axis (subseq (send x :get :sh-lleg-ref-force) 0 3) (subseq (send x :get :sh-lleg-ref-force) 3 6))) data))
                       (if (find :lleg limb) (low-pass-filter (mapcar #'(lambda (x) (calc-cop axis (send x :get :lleg-force-vector) (send x :get :lleg-moment-vector))) data) :cutoff-f cutoff-f))
                       ))
     (mapcar #'(lambda (x) (- (send x :time) start-offset)) data)
     :keylist (delete nil (list (if (find :rleg limb) "ref rleg COP x") (if (find :rleg limb) "act rleg COP x")
                                (if (find :lleg limb) "ref lleg COP x") (if (find :lleg limb) "act lleg COP x")
                                ))
     :title fname
     :graph-fname (reg-replace* " " "_" (if add-fname (format nil "~a ~a.eps" fname add-fname) (format nil "~a.eps" fname)))
     :dump-graph save
     :xlabel "Time[s]"
     :ylabel "COP[m]"
     )
    (if save (unset-plot-common-config))
    ))

(defun calc-r (alpha-zmp)
  (let ( (steepness 8) )
    ;; (+ (/ -1 (+ 1 (exp (* -6 steepness (+ alpha-zmp -1 (/ 1.0 steepness))))))  (/ 1 (+ 1 (exp (* -6 steepness (- alpha-zmp (/ 1.0 steepness)))))))
    (+ (/ 1 (+ 1 (exp (+ (* -48 alpha-zmp) 6)))) (/ -1 (+ 1 (exp (+ (* -48 alpha-zmp) 42)))))
  ))

(defun plot-alpha
  (&key (rs-list *rs-list*) (xrange nil) (axis 0)
        (add-fname nil) (fname (format nil "state data ~A" add-fname))
        (cutoff-f nil) (save t)
        (leg-margin 65))
  (let* ( (xrange (calc-xrange xrange rs-list)) (data (subseq *rs-list* (car xrange) (cadr xrange)))
          (zmpy) (redge) (alpha-zmp) (r) (lleg-ref-force-z) (rleg-ref-force-z) (alpha-sfc) (alpha)
          (alpha-zmp-list nil) (alpha-sfc-list nil) (r-list nil) (alpha-list nil)
          (foot-edge 0) )
    (mapc #'(lambda (x) 
              (send *robot* :angle-vector (send x :angle-vector))
              (setq zmpy (* 1e3 (elt (send x :get :reference-zmp) 1))
                    redge (+ (elt (send (send *robot* :rleg :end-coords) :worldpos) 1) leg-margin) ;; rleg=stop
                    alpha-zmp (/ (max (- zmpy redge) 0) (- (- (elt (send (send *robot* :lleg :end-coords) :worldpos) 1) leg-margin) redge))
                    r (calc-r alpha-zmp)
                    lleg-ref-force-z (elt (send x :get :sh-lleg-ref-force) 2)
                    rleg-ref-force-z (elt (send x :get :sh-rleg-ref-force) 2)
                    alpha-sfc (/ lleg-ref-force-z (+ lleg-ref-force-z rleg-ref-force-z))
                    alpha (+ (* r alpha-sfc) (* (- 1 r) alpha-zmp)))
              (push-back alpha-zmp alpha-zmp-list)
              (push-back alpha-sfc alpha-sfc-list)
              (push-back r r-list)
              (push-back alpha alpha-list)
              ) data)
    (graph-view
     (list (low-pass-filter alpha-zmp-list :cutoff-f cutoff-f)
           ;; (low-pass-filter r-list :cutoff-f cutoff-f)
           ;; (low-pass-filter alpha-sfc-list :cutoff-f cutoff-f)
           ;; (low-pass-filter alpha-list :cutoff-f cutoff-f)
           ;; (mapcar #'(lambda (x) (* 1e3 (- (elt (send x :get :reference-zmp) 1) foot-edge))) data)
           )
     (mapcar #'(lambda (x) (send x :time)) data)
     :keylist (list "alpha-zmp" "r" "alpha-sfc" "alpha")
     ;; :keylist (list "refzmp y" "refzmp y from edge")
     :title fname
     :xlabel "Time [s]" :ylabel "ZMP, COG[mm]"
     :dump-graph save
     )
    ))

(defun plot-voltage-drop (&key (step 10) (rs-list *rs-list*) (start-tm 0) (stop-tm (send (car (last rs-list)) :time)) (yrange '(0 70))
                               (fname "voltage-drop") (add-fname nil) (save nil) (mode "lines")
                                (dt 0.002) (rotation-const 466))
  (let* ( (start-idx) (stop-idx) (servo-state-list) (error-list nil) (prev-av) (idx) (tm-list nil) (urata-len 17)
         (joint-name-list (get-joint-name-list))
         (plot-joint-name-list (get-joint-name-list :limb :legs))
         (servo-state-key-list '(:dummy-alarm :motor-core-temp :motor-output :board-vin :ref-angle :enc-angle :abs-angle :enc-abs-diff :motor-outer-temp :ref-angle-shm
                                        :board-vdd :pgain :dgain :current :comm-normal :error0 :error1))
         (parse-key-list '(:board-vin :current)) (data-key-list (append parse-key-list '(:dq :voltage-drop :servo-alarm)))
         (plot-key-list '(:voltage-drop :board-vin :servo-alarm))
         (plot-key-name-list '("Voltage drop" "Vin" "error"))
         (gear-ratio-list (append (flatten (make-list 2 :initial-element '(227.67 255.49 208.71 196.88 203.70 2202.58))) (make-list 19 :initial-element 1)))
         ;; (gear-ratio-list (append (flatten (make-list 2 :initial-element '(227.67 332.14 233.75 252 275 241.071))) (make-list 19 :initial-element 1)))
         )
    (multiple-value-setq (start-idx stop-idx) (calc-xrange (list start-tm stop-tm) rs-list))

    (send *gp* :command "set size 2,1.5")
    (send *gp* :command "set key below")
    (send *gp* :command "set key font \"Times New Roman,15\"")

    (if yrange (set-yrange yrange) (send *gp* :command "set autoscale y"))
    (setq rotation-const (* rotation-const 360.0 (/ 1.0 60)))
    (setq prev-av (send (elt rs-list start-idx) :angle-vector))

    (setq idx start-idx
          data-list (mapcar #'(lambda (x) (make-list (length (send *robot* :angle-vector)))) data-key-list))

    (do-until-key-with-check
     (< idx stop-idx)

     (setq rs (elt rs-list idx))
     (push-back (send rs :time) tm-list)

     (setq servo-state-list (send rs :get :servo-state))
     (mapc #'(lambda (joint-idx dq servo-alarm gear-ratio)
               (setq servo-state (subseq  servo-state-list (* joint-idx urata-len) (* (1+ joint-idx) urata-len)))
               (mapc #'(lambda (key)
                         (let ( (state-key-idx (position key servo-state-key-list)) (data-key-idx (position key data-key-list)) )
                           (push-back (elt servo-state state-key-idx) (elt (elt data-list data-key-idx) joint-idx)) ))
                     parse-key-list)
               (push-back dq (elt (elt data-list (position :dq data-key-list)) joint-idx))
               (push-back (* (abs dq) gear-ratio (/ 1.0 rotation-const)) (elt (elt data-list (position :voltage-drop data-key-list)) joint-idx))
               (push-back servo-alarm (elt (elt data-list (position :servo-alarm data-key-list)) joint-idx))
               )
           (range (length (send *robot* :angle-vector)))
           (coerce (scale (/ 1.0 (* dt step)) (v- (send rs :angle-vector) prev-av)) cons)
           (coerce (send rs :get :servo-alarm) cons)
           gear-ratio-list)

     (setq prev-av (send rs :angle-vector))
     (incf idx step)
     )

    (graph-view
     (reduce #'(lambda (ret plot-key)
                 (append ret
                         (mapcar #'(lambda (joint-name) (elt (elt data-list (position plot-key data-key-list)) (position joint-name joint-name-list :test #'equal))) plot-joint-name-list)))
             (append '(nil) plot-key-list))
     tm-list
     :keylist (reduce #'(lambda (ret key-name)
                          (append ret (mapcar #'(lambda (joint-name) (concatenate string key-name " " joint-name)) plot-joint-name-list)))
                      (append '(nil) plot-key-name-list))
     :title fname
     :graph-fname (reg-replace* " " "_" (if add-fname (format nil "~a ~a.eps" fname add-fname) (format nil "~a.eps" fname)))
     :dump-graph save
     :xlabel "Time[s]"
     :ylabel "Voltage drop[V]"
     :mode mode
     )

    ))

;; 複数の*rs-list*に対して関数を呼んでreplotすると上手くプロットできない時がある
(defun plot-joint-data (&key (step 10) (rs-list *rs-list*) (start-tm 0) (stop-tm (send (car (last rs-list)) :time)) (yrange nil) (replot nil)
                             (plot-joint-name-list (get-joint-name-list :limb :lleg))
                             (plot-keys-list (list '(:torque-vector) '(:ref-dq :act-dq))) (plot-key-names-list (list '("") '("ref" "act"))) (fname-list '("joint-torque" "joint-vel")) (ylabel-list '("Joint torque [Nm]" "Joint velocity [deg/s]"))
                             (add-fname nil) (save nil) (add-key-name nil) (mode "lines")
                             (dt 0.002) (rotation-const 466))
  (let* ( (start-idx) (stop-idx) (act-torque-vector) (error-list nil) (prev-act-av) (prev-ref-av) (idx) (tm-list nil) (urata-len 17)
         (joint-name-list (get-joint-name-list))
         (parse-key-list '(:torque-vector)) (data-key-list (append parse-key-list '(:ref-dq :act-dq)))
         )
    (multiple-value-setq (start-idx stop-idx) (calc-xrange (list start-tm stop-tm) rs-list))

    (if (or (not (boundp '*gp-list*)) (not (= (length *gp-list*) (length plot-keys-list))))
        (setq *gp-list* (mapcar #'(lambda (x) (gnuplot)) plot-keys-list)))

    (mapc #'(lambda (gp)
              (send gp :command "set size 1,0.5")
              (send gp :command "set key below")
              (send gp :command "set key font \"Times New Roman,15\"")
              (send gp :command "set tics font \"Times New Roman,15\"")
              (send gp :command "set ytics autofreq")
              (send gp :command "set lmargin 8"))
          *gp-list*)

    (if yrange (set-yrange yrange) (send *gp* :command "set autoscale y"))
    (setq prev-act-av (send (elt rs-list start-idx) :angle-vector)
          prev-ref-av (send (elt rs-list start-idx) :get :auto-balancer-reference-vector))

    (setq idx start-idx
          data-list (mapcar #'(lambda (x) (make-list (length (send *robot* :angle-vector)))) data-key-list))

    (do-until-key-with-check
     (< idx stop-idx)

     (setq rs (elt rs-list idx))
     (push-back (send rs :time) tm-list)

     (mapc #'(lambda (joint-idx ref-dq act-dq act-torque)
               (push-back ref-dq (elt (elt data-list (position :ref-dq data-key-list)) joint-idx))
               (push-back act-dq (elt (elt data-list (position :act-dq data-key-list)) joint-idx))
               (push-back act-torque (elt (elt data-list (position :torque-vector data-key-list)) joint-idx))
               )
           (range (length (send *robot* :angle-vector)))
           (coerce (scale (/ 1.0 (* dt step)) (v- (send rs :get :auto-balancer-reference-vector) prev-ref-av)) cons)
           (coerce (scale (/ 1.0 (* dt step)) (v- (send rs :angle-vector) prev-act-av)) cons)
           (coerce (send rs :torque-vector) cons)
           )

     (setq prev-act-av (send rs :angle-vector)
           prev-ref-av (send rs :get :auto-balancer-reference-vector))
     (incf idx step)
     )

    (if add-key-name (setq add-key-name (concatenate string add-key-name " ")) (setq add-key-name ""))
    (mapc #'(lambda (gp plot-keys plot-key-names fname ylabel)
              (graph-view
               (reduce #'(lambda (ret plot-key)
                           (append ret
                                   (mapcar #'(lambda (joint-name) (elt (elt data-list (position plot-key data-key-list)) (position joint-name joint-name-list :test #'equal))) plot-joint-name-list)))
                       (append '(nil) plot-keys))
               tm-list
               :keylist (reduce #'(lambda (ret key-name)
                                    (append ret (mapcar #'(lambda (joint-name) (concatenate string add-key-name key-name " " joint-name)) plot-joint-name-list)))
                                (append '(nil) plot-key-names))
               :graph-instance gp
               :title ""
               :graph-fname (reg-replace* " " "_" (if add-fname (format nil "~a ~a.eps" fname add-fname) (format nil "~a.eps" fname)))
               :dump-graph save
               ;; :xlabel "Time [s]"
               :xlabel ""
               :ylabel ylabel
               :mode mode
               :replot replot
               ))
          *gp-list*
          plot-keys-list
          plot-key-names-list
          fname-list
          ylabel-list)
    ))

(defun plot-high-jump-motion-data ()
  (let ( (start-tm 2) (stop-tm 3.5) )
    (mapc #'(lambda (robot) (mapc #'(lambda (link mass) (send link :weight mass)) (send robot :torso :links) (list 800 4889.43 7500))) (list *ref-robot* *robot*))

    (unless (boundp '*high-jump-rs-list*)
      (prepare-plot :fname "/home/k-kojima/k-kojima@jsk.imi.i.u-tokyo.ac.jp/Documents/log/JAXON_BLUE/jump-370msec_postq_interpolate-remaineder-frame_0delay-xg-dxg_JAXON_BLUE_2018_03-20_22-46_23")
      (setq *high-jump-rs-list* (copy-object *rs-list*)))

    (analyze-jump-motion :rs-list *high-jump-rs-list* :start-tm start-tm :stop-tm stop-tm :step 2 :takeoff-fz-thresh 100 :landing-fz-thresh 100)

    (send *gp* :command "set size 1,0.5")
    (send *gp* :command "set key right inside")
    (send *gp* :command "set key font \"Times New Roman,15\"")
    (send *gp* :command "set tics font \"Times New Roman,15\"")
    (send *gp* :command "set ytics autofreq")
    (send *gp* :command "set lmargin 8")

    (graph-view (list ref-cmz-list act-cmz-list) tm-list :keylist '("reference" "actual") :title "" :dump-graph t :xlabel "" :graph-fname "high-jump-cmz.eps" :ylabel "COG velocity z [mm/s]")
    (graph-view (list ref-dcmz-list act-dcmz-list) tm-list :keylist '("reference" "actual") :title "" :dump-graph t :xlabel "" :graph-fname "high-jump-dcmz.eps" :ylabel "COG z [mm]")
    (graph-view (list ref-l-fz-list ref-r-fz-list act-l-fz-list act-r-fz-list) tm-list :keylist '("reference left foot fz" "reference right foot fz" "actual left foot fz" "actual right foot fz")
                :title "" :dump-graph t :xlabel "" :graph-fname "high-jump-fz.eps" :ylabel "Force z [N]")

    (plot-joint-data :rs-list *high-jump-rs-list* :start-tm start-tm :stop-tm stop-tm :save t :step 2
                     :plot-keys-list '((:torque-vector)) :plot-key-names-list '(("")) :fname-list '("high-jump-joint-torque") :ylabel-list '("Joint torque [Nm]"))
    ))

(defun plot-different-mass-jump-comparison ()
  (mapc #'(lambda (robot) (mapc #'(lambda (link mass) (send link :weight mass)) (send robot :torso :links) (list 800 4889.43 3600))) (list *ref-robot* *robot*))

  (unless (boundp '*correct-mass-rs-list*)
    (prepare-plot :fname "/home/k-kojima/k-kojima@jsk.imi.i.u-tokyo.ac.jp/Documents/log/JAXON_BLUE/jump-250msec_postq_old-chest-mass_0dxgfz_0delay-xg-dxg_L10,50-1,10-0.5,1-0.1,3_S5,70-30,70-10,50-5,40-0.5,1-0.1,3_JAXON_BLUE_2018_03-01_20-54_31")
    (setq *correct-mass-rs-list* (copy-object *rs-list*)))

  (unless (boundp '*incorrect-mass-rs-list*)
    (prepare-plot :fname "/home/k-kojima/k-kojima@jsk.imi.i.u-tokyo.ac.jp/Documents/log/JAXON_BLUE/jump-250msec_postq_modify-chest-mass_0dxgfz_0delay-xg-dxg_L10,50-1,10-0.5,1-0.1,3_S5,70-30,70-10,50-5,40-0.5,1-0.1,3_little-back-tilt_JAXON_BLUE_2018_03-01_20-45_25")
    (setq *incorrect-mass-rs-list* (copy-object *rs-list*)))

  ;; (analyze-jump-motion :rs-list *correct-mass-rs-list* :start-tm 2.2 :stop-tm 3.2 :step 2 :takeoff-fz-thresh 100 :landing-fz-thresh 100)
  ;; (setq correct-mass-ref-dcmz-list ref-dcmz-list
  ;;       correct-mass-act-l-fz-list act-l-fz-list)
  ;; (setq correct-tm-list tm-list)

  ;; (analyze-jump-motion :rs-list *incorrect-mass-rs-list* :start-tm 2.2 :stop-tm 3.2 :step 2 :takeoff-fz-thresh 100 :landing-fz-thresh 100)
  ;; (setq incorrect-mass-ref-dcmz-list ref-dcmz-list
  ;;       incorrect-mass-act-l-fz-list act-l-fz-list)

  (send *gp* :command "set size 1,0.5")
  (send *gp* :command "set key right inside")
  (send *gp* :command "set key font \"Times New Roman,15\"")
  (send *gp* :command "set tics font \"Times New Roman,15\"")
  (send *gp* :command "set ytics autofreq")
  (send *gp* :command "set lmargin 8")

  (graph-view (list correct-mass-ref-dcmz-list incorrect-mass-ref-dcmz-list) tm-list :keylist '("corect mass" "incorrect mass") :title "" :dump-graph t :xlabel "" :graph-fname "different-mass-dcmz-comparison.eps" :ylabel "COG velocity z [mm/s]")
  (graph-view (list correct-mass-act-l-fz-list incorrect-mass-act-l-fz-list) tm-list :keylist '("corect mass" "incorrect mass") :title "" :dump-graph t :xlabel "" :graph-fname "different-mass-fz-comparison.eps" :ylabel "fz [N]")

  (plot-joint-data :rs-list *correct-mass-rs-list* :start-tm 2.2 :stop-tm 3.2 :plot-joint-name-list '("LLEG-JOINT3") :save t :step 2 :add-key-name "correct mass"
                   :plot-keys-list '((:torque-vector) (:act-dq)) :plot-key-names-list '(("") ("")) :fname-list '("different-mass-joint-torque-comparison" "different-mass-joint-velocity-comparison") :ylabel-list '("Joint torque [Nm]" "Joint velocity [deg/s]"))
  (plot-joint-data :rs-list *incorrect-mass-rs-list* :start-tm 2.2 :stop-tm 3.2 :plot-joint-name-list '("LLEG-JOINT3") :save t :step 2 :replot t :add-key-name "incorrect mass"
                   :plot-keys-list '((:torque-vector) (:act-dq)) :plot-key-names-list '(("") ("")) :fname-list '("different-mass-joint-torque-comparison" "different-mass-joint-velocity-comparison") :ylabel-list '("Joint torque [Nm]" "Joint velocity [deg/s]"))
  )

(defun plot-walk-shuffle-root-link-comparison (&key (log-dir "k-kojima@jsk.imi.i.u-tokyo.ac.jp/Documents/log/"))
  (unless (boundp '*walk-rs-list*)
    (prepare-plot :fname (concatenate string (unix:getenv "HOME") "/" log-dir "JAXON_BLUE/choreonoid_250step-len_5step_1step-tm_pos-eefmqpcop2_JAXON_BLUE_2018_03-30_07-14_27"))
    (setq *walk-rs-list* (copy-object *rs-list*)))

  (unless (boundp '*shuffle-rs-list*)
    ;; (prepare-plot :fname "/home/k-kojima/k-kojima@jsk.imi.i.u-tokyo.ac.jp/Documents/log/JAXON_BLUE/choreonoid_slide-forward-250step-len-5step_JAXON_BLUE_2018_03-30_07-05_00")
    (prepare-plot :fname (concatenate string (unix:getenv "HOME") "/" log-dir "JAXON_BLUE/slide-forward-200step-len-5step_pos-eefmqocop2_0delay-xg-dxg_0.06xoff_0.15-0.15-0.2xstep_0.05soleoff_sfc-0.6smu-0.6dmu_500CM_500P_200L_1Lz_0.6f_10n_200nz_toe-float_JAXON_BLUE_2018_04-13_00-21_12"))
    (setq *shuffle-rs-list* (copy-object *rs-list*)))

  (analyze-jump-motion :rs-list *walk-rs-list* :stop-tm 7)
  (setq walk-ref-root-y-list ref-root-y-list
        walk-ref-root-dy-list ref-root-dy-list)

  (analyze-jump-motion :rs-list *shuffle-rs-list* :stop-tm 7)
  (setq shuffle-ref-root-y-list ref-root-y-list
        shuffle-ref-root-dy-list ref-root-dy-list)

  (send *gp* :command "set size 1,0.5")
  (send *gp* :command "set key right inside")
  (send *gp* :command "set key font \"Times New Roman,15\"")
  (send *gp* :command "set tics font \"Times New Roman,15\"")
  (send *gp* :command "set ytics autofreq")
  (send *gp* :command "set lmargin 8")
  (send *gp* :command "set ylabel offset 2")

  (graph-view (list walk-ref-root-y-list shuffle-ref-root-y-list) tm-list :keylist '("walk" "shuffle") :title "" :dump-graph t :xlabel "" :graph-fname "walk-shuffle-root-y-comparison.eps" :ylabel "Waist side\\n position [mm]")
  (graph-view (list walk-ref-root-dy-list shuffle-ref-root-dy-list) tm-list :keylist '("walk" "shuffle") :title "" :dump-graph t :xlabel "Time [s]" :graph-fname "walk-shuffle-root-dy-comparison.eps" :ylabel "Waist side\\n velocity [mm/s]")
  )

(defun plot-pos-tq-conrol-jump-comparison ()
  (unless (boundp '*pos-rs-list*)
    (prepare-plot :fname "/home/k-kojima/k-kojima@jsk.imi.i.u-tokyo.ac.jp/Documents/log/JAXON_BLUE/jump-200msec_3spline-TO-LA_2700pdamp_20rdamp_JAXON_BLUE_2018_01-18_20-30_39")
    (setq *pos-rs-list* (copy-object *rs-list*)))

  (unless (boundp '*tq-rs-list*)
    (prepare-plot :fname "/home/k-kojima/k-kojima@jsk.imi.i.u-tokyo.ac.jp/Documents/log/JAXON_BLUE/jump-200msec_postq_calc-extforce-only-onground_0dxgfz_0.1delay-xg-dxg_L10,50-1,10-0.5,1-0.1,3_S5,70-30,70-10,50-5,40-0.5,1-0.1,3_JAXON_BLUE_2018_02-28_23-25_20")
    (setq *tq-rs-list* (copy-object *rs-list*)))

  (analyze-jump-motion :rs-list *pos-rs-list* :start-tm 1 :stop-tm 5 :step 2 :takeoff-fz-thresh 50 :landing-fz-thresh 50)
  (setq pos-act-l-fz-list act-l-fz-list)
  (setq correct-tm-list tm-list)

  (analyze-jump-motion :rs-list *tq-rs-list* :start-tm 1.5 :stop-tm 5.5 :step 2 :takeoff-fz-thresh 50 :landing-fz-thresh 50)
  (setq tq-act-l-fz-list act-l-fz-list)

  (send *gp* :command "set size 1,0.5")
  (send *gp* :command "set key right inside")
  (send *gp* :command "set key font \"Times New Roman,15\"")
  (send *gp* :command "set tics font \"Times New Roman,15\"")
  (send *gp* :command "set ytics autofreq")
  (send *gp* :command "set lmargin 8")
  (send *gp* :command "set yrange [-100:500]")

  (graph-view (list pos-act-l-fz-list tq-act-l-fz-list) tm-list :keylist '("Ordinary PD control" "Torque/PD gain shift control") :title "" :dump-graph t :xlabel "" :graph-fname "pos-tq-jump-force-comparison.eps" :ylabel "Foot force z [N]")
  )

(defun plot-different-joint-gain-jump-comparison (&key (log-dir "k-kojima@jsk.imi.i.u-tokyo.ac.jp/Documents/log/"))
  (mapc #'(lambda (robot) (mapc #'(lambda (link mass) (send link :weight mass)) (send robot :torso :links) (list 800 4889.43 3600))) (list *ref-robot* *robot*))

  (unless (boundp '*low-sgain-rs-list*)
    (prepare-plot :fname (concatenate string (unix:getenv "HOME") "/" log-dir "JAXON_BLUE/jump-200msec_rev3-25_60Vcap_25PE_postq-eefmqp_0delay-xg-dxg_L5,30,10,1,0.5,0.1-70,70,50,10,1,3_S5,30,10,5,0.5,0.1-70,70,50,20,1,5_100CM_20000P_5000L_1Lz_10f_20n_200nz_gear-sound_JAXON_BLUE_2018_04-24_21-34_27"))
    (setq *low-sgain-rs-list* (copy-object *rs-list*)))

  (unless (boundp '*high-sgain-rs-list*)
    (prepare-plot :fname (concatenate string (unix:getenv "HOME") "/" log-dir "JAXON_BLUE/jump-200msec_rev3-25_60Vcap_25PE_postq-eefmqp_0delay-xg-dxg_L5,30,10,1,0.5,0.1-70,70,50,10,1,3_S5,30,10,5,0.5,0.1-70,70,50,40,1,5_100CM_20000P_5000L_1Lz_10f_20n_200nz_JAXON_BLUE_2018_04-24_21-28_41"))
    (setq *high-sgain-rs-list* (copy-object *rs-list*)))

  (analyze-jump-motion :rs-list *low-sgain-rs-list* :start-tm 2.2 :stop-tm 3.2 :step 2 :takeoff-fz-thresh 80 :landing-fz-thresh 40 :dcm-lpr 0.3)
  (setq low-sgain-act-dcmz-list (low-pass-filter act-dcmz-list :dt 0.002 :cutoff-f 15)
        low-sgain-act-l-fz-list act-l-fz-list)
  (setq low-stain-tm-list tm-list)

  (analyze-jump-motion :rs-list *high-sgain-rs-list* :start-tm 2.2 :stop-tm 3.2 :step 2 :takeoff-fz-thresh 50 :landing-fz-thresh 5 :dcm-lpr 0.3)
  (setq high-sgain-act-dcmz-list (low-pass-filter act-dcmz-list :dt 0.002 :cutoff-f 15)
        high-sgain-act-l-fz-list act-l-fz-list)

  (send *gp* :command "set size 1,0.5")
  (send *gp* :command "set key right inside")
  (send *gp* :command "set key font \"Times New Roman,15\"")
  (send *gp* :command "set tics font \"Times New Roman,15\"")
  (send *gp* :command "set ytics autofreq")

  (send *gp* :command "set lmargin 8")

  (graph-view (list low-sgain-act-dcmz-list high-sgain-act-dcmz-list) tm-list :keylist '("Low takeoff dgain" "Dynamic servo gain shift control") :title "" :dump-graph t :xlabel "" :graph-fname "different-servo-gain-dcmz-comparison.eps" :ylabel "COG velocity z [mm/s]")
  (graph-view (list low-sgain-act-l-fz-list high-sgain-act-l-fz-list) tm-list :keylist '("Low takeoff dgain" "Dynamic servo gain shift control") :title "" :dump-graph t :xlabel "Time [s]" :graph-fname "different-servo-gain-fz-comparison.eps" :ylabel "fz [N]")

  ;; (plot-joint-data :rs-list *low-sgain-rs-list* :start-tm 2.2 :stop-tm 3.2 :plot-joint-name-list '("LLEG-JOINT3") :save t :step 6 :add-key-name "low sgain"
  ;;                  :plot-keys-list '((:act-dq)) :plot-key-names-list '(("")) :fname-list '("different-servo-gain-joint-velocity-comparison") :ylabel-list '("Joint velocity [deg/s]"))
  ;; (plot-joint-data :rs-list *high-sgain-rs-list* :start-tm 2.2 :stop-tm 3.2 :plot-joint-name-list '("LLEG-JOINT3") :save t :step 6 :replot t :add-key-name "high sgain"
  ;;                  :plot-keys-list '((:act-dq)) :plot-key-names-list '(("")) :fname-list '("different-servo-gain-joint-velocity-comparison") :ylabel-list '("Joint velocity [deg/s]"))
  )
