
;; (defun make-foot-step-coords (&key (step-num 5) (stride-length 200) (initial-swing-limb :lleg) (step-width 200))
;;   (let* ( (limb-list '(:lleg :rleg)) (width-gain-list '(1 -1)) (initial-support-limb (car (remove initial-swing-limb limb-list)))
;;           (limb-idx (position initial-swing-limb limb-list))
;;           (current-coords (mapcar #'(lambda (limb) ())))
;;           (half-step-width (/ step-width 2.0)))
;;     (append (list (make-cords :pos (float-vector 0 (* half-step-width (elt width-gain-list (position initial-support-limb limb-list)))) :name initial-support-limb))
;;             (mapcar #'(lambda (limb))
;;             )
;;   ))

(defun test-walk (&key (step-length 100) (step-time 1.2) (step-height 65))
  (let ( (stride-length (* step-length 2)) (org-st-algorithm (send (send *ri* :get-st-param) :st_algorithm)) (org-joint-control-mode (send (send *ri* :get-st-param) :joint_control_mode)) )
    (send *ri* :stop-st)
    (send *ri* :set-st-param :st-algorithm :eefmqpcop)
    (send *ri* :set-st-param :joint-control-mode 0)
    (send *ri* :stop-auto-balancer)
    (send *ri* :start-auto-balancer :limbs '(:lleg :rleg))
    (send *ri* :start-st)

    (send *ri* :set-foot-steps-with-param
          (list (make-coords :pos (float-vector (* stride-length 0) -100 0) :name :rleg) (make-coords :pos (float-vector (* stride-length 0.5) 100 0) :name :lleg)
                (make-coords :pos (float-vector (* stride-length 1) -100 0) :name :rleg) (make-coords :pos (float-vector (* stride-length 1.5) 100 0) :name :lleg)
                (make-coords :pos (float-vector (* stride-length 2) -100 0) :name :rleg) (make-coords :pos (float-vector (* stride-length 2.0) 100 0) :name :lleg))
          (list 0 step-height step-height step-height step-height step-height)
          (list step-time step-time step-time step-time step-time step-time)
          (list 0 0 0 0 0 0)
          (list 0 0 0 0 0 0))

    (send *ri* :stop-st)
    (send *ri* :set-st-param :st-algorithm org-st-algorithm)
    (send *ri* :set-st-param :joint-control-mode org-joint-control-mode)
    (send *ri* :start-st)
    ))

(defun test-shuffle ()
  (let ( (fname "_slide-forward-250step-len-5step") (sample-name "ShuffleSample") (motion-time) (local-path)
         (org-support-pgain (send-all (send (send *ri* :get-st-param) :joint_servo_control_parameters) :support_pgain)) )

    (setq local-path (concatenate string (ros::resolve-ros-path "package://jsk_choreonoid") "sample/" sample-name "/"))

    (setq *__fname__* (reg-replace* "^_" "" fname));; _fname->fname
    (setq motion-time (get-motion-time local-path *__fname__*))
    ;; (copy-files :fname "slide-forward-250step-len-5step" :sample-name "ShuffleSample")
    (send *ri* :set-st-param :joint-servo-control-parameters (list :support_pgain '(#f(5.0 30.0 10.0 5.0 0.5 1) #f(5.0 30.0 10.0 5.0 0.5 1) #f(100.0 100.0 100.0 100.0 100.0 100.0 100.0) #f(100.0 100.0 100.0 100.0 100.0 100.0 100.0))))
    (play-hrpsysfile :fname fname :sample-name sample-name :st-algorithm :eefmqp :add-fname "sim-1smu-0.2dmu_SP5,30,10,5,0.5,1" :tm 1 :save-log nil :wait-key nil)
    (unix:sleep (round motion-time))
    (send *ri* :set-st-param :joint-servo-control-parameters (list :support_pgain org-support-pgain))
    ))
