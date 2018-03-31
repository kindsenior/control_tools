(load "play-hrpsysfile.l")
(load "cross-step.l")

(defun slip-through-demo()
  (warn "slip-through-demo~%")
  (let ( (org-support-pgain (send-all (send (send *ri* :get-st-param) :joint_servo_control_parameters) :support_pgain)) )
    (send *ri* :set-st-param :joint-servo-control-parameters (list :support_pgain '(#f(5.0 30.0 10.0 5.0 0.5 1) #f(5.0 30.0 10.0 5.0 0.5 1) #f(100.0 100.0 100.0 100.0 100.0 100.0 100.0) #f(100.0 100.0 100.0 100.0 100.0 100.0 100.0))))
    (play-hrpsysfile :fname "_slip-through-right-200step-len-8step-low-waist-tilt" :sample-name "ShuffleSample" :st-algorithm :eefmqp :add-fname "sim-0.5smu-0.1dmu_SP5,30,10,5,0.5,1")
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
