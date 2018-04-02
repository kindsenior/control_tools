(load "play-hrpsysfile.l")
(load "cross-step.l")

(defun slip-through-demo()
  (warn "slip-through-demo~%")
  (let ( (org-support-pgain (send-all (send (send *ri* :get-st-param) :joint_servo_control_parameters) :support_pgain)) )
    (send *ri* :set-st-param :joint-servo-control-parameters (list :support_pgain '(#f(5.0 30.0 10.0 5.0 0.5 1) #f(5.0 30.0 10.0 5.0 0.5 1) #f(100.0 100.0 100.0 100.0 100.0 100.0 100.0) #f(100.0 100.0 100.0 100.0 100.0 100.0 100.0))))
    (play-hrpsysfile :fname "_slip-through-right-200step-len-8step-low-waist-tilt" :sample-name "ShuffleSample" :st-algorithm :eefmqp :add-fname "sim-0.5smu-0.1dmu_SP5,30,10,5,0.5,1")
    (send *ri* :set-st-param :joint-servo-control-parameters (list :support_pgain org-support-pgain))
    ))

(defun shuffle-demo()
  (warn "shuffle-demo~%")
  (let ( (fname "_slide-forward-250step-len-5step") (sample-name "ShuffleSample") (motion-time) (local-path)
         (org-support-pgain (send-all (send (send *ri* :get-st-param) :joint_servo_control_parameters) :support_pgain)) )

    (setq local-path (concatenate string (ros::resolve-ros-path "package://jsk_choreonoid") "sample/" sample-name "/"))

    (setq *__fname__* (reg-replace* "^_" "" fname));; _fname->fname
    (setq motion-time (get-motion-time local-path *__fname__*))
    ;; (copy-files :fname "slide-forward-250step-len-5step" :sample-name "ShuffleSample")
    (send *ri* :set-st-param :joint-servo-control-parameters (list :support_pgain '(#f(5.0 30.0 10.0 5.0 0.5 1) #f(5.0 30.0 10.0 5.0 0.5 1) #f(100.0 100.0 100.0 100.0 100.0 100.0 100.0) #f(100.0 100.0 100.0 100.0 100.0 100.0 100.0))))
    ;; (play-hrpsysfile :fname fname :sample-name sample-name :st-algorithm :eefmqp :add-fname "sim-1smu-0.2dmu_SP5,30,10,5,0.5,1" :tm 1 :save-log nil)
    (play-hrpsysfile :fname fname :sample-name sample-name :st-algorithm :eefmqp :add-fname "sim-1smu-0.2dmu_SP5,30,10,5,0.5,1" :tm 1 :save-log nil :wait-key nil)
    (unix:sleep (round motion-time))
    (play-hrpsysfile :fname fname :sample-name sample-name :st-algorithm :eefmqp :add-fname "sim-1smu-0.2dmu_SP5,30,10,5,0.5,1" :tm 2 :save-log nil :prepare nil :wait-key nil)
    (send *ri* :set-st-param :joint-servo-control-parameters (list :support_pgain org-support-pgain))
    ))

(defun cross-step-demo()
  (warn "cross-step-demo~%")
  (read-line)
  (cross-step-init)

  (read-line)
  (start-cross)

  (read-line)
  (go-cross-step)

  (read-line)
  (stop-cross)

  (read-line)
  (cross-step-end)
  )

(defun jump-over-demo ()
  (warn "jump-over-demo~%")
  (send *ri* :stop-st)
  (play-hrpsysfile :fname "_jump-over-370msec" :sample-name "JumpSample" :st-algorithm :eefmqp :start-st nil)
  (send *ri* :start-st)
  )
