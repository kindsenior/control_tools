;; (require "package://hrpsys_ros_bridge_tutorials/euslisp/jaxon_red-interface.l")
;; (load "package://drc_task_common/euslisp/test-drc-terrain-walk.l")
;; (load "/home/leus/ros/indigo/src/jsk-ros-pkg/jsk_demos/jsk_2015_06_hrp_drc/drc_task_common/euslisp/test-drc-terrain-walk.l")

;; (unless (boundp '*jaxon_red*)
;;   (jaxon_red-init)
;;   (setq *robot* *jaxon_red*)
;;   ;; (objects (list *robot*))
;;   ))


(defun cross-step-pose ()
  (send *robot* :reset-pose)
  (send *robot* :legs :move-end-pos #f(0 0 150))
  (send *robot* :legs :hip-p :joint-angle -70)
  (send *robot* :fix-leg-to-coords (make-coords))
  (send *robot* :move-centroid-on-foot :both '(:lleg :rleg) :target-centroid-pos #f(50 0 0))
  )


(defun cross-step-init ()
  (warn "cross-step-init~%")

  ;; (walking-pose *robot*
  ;;               :default-pose-method :reset-pose
  ;;               :root-link-height-offset -110
  ;;               :root-link-pitch-offset 20
  ;;               :chest-link-pitch-offset 20)
  (cross-step-pose)

  (send *ri* :stop-st)
  (send *ri* :stop-auto-balancer)
  (unix:sleep 2)
  (send *ri* :angle-vector (send *robot* :angle-vector))
  (unix:sleep 5)
  (send *ri* :start-auto-balancer)
  (send *ri* :start-st)

  (setq *cross-stride* #f(600 0 0))
  (setq *cross-step* (scale 0.5 *cross-stride*))
  (setq *width-offset* #f(0 -20 0))
  (setq *cross-step-time* 2.0)
  )

(defun start-cross ()
  (warn "start-cross~%")
  (send *ri* :set-gait-generator-param
        :default-orbit-type :cross
        :default-step-time 2.0
        ;; :cross-trajectory-start-way-point-offset #f(0.04 0.0 0) ;;https://github.com/fkanehiro/hrpsys-base/pull/1215 が必要
        )
  (send *ri* :set-foot-steps
        (list (make-coords :pos (float-vector 0 0 0) :name :lleg)
              (make-coords :pos (v+ (scale -1 *cross-step*) *width-offset*) :name :rleg)))
  (send *ri* :set-gait-generator-param
        :default-step-time *cross-step-time*
        ;; :cross-trajectory-start-way-point-offset #f(0.04 0.15 0)
        )
  )

(defun stop-cross ()
  (warn "stop-cross~%")
  (send *ri* :set-gait-generator-param
        :default-step-time 2.0
        ;; :cross-trajectory-goal-way-point-offset #f(0.04 0.0 0)
        )
  (send *ri* :set-foot-steps
        (list (make-coords :pos (float-vector 0 0 0) :name :lleg)
              (make-coords :pos (float-vector 0 -200 0) :name :rleg)))
  (send *ri* :set-gait-generator-param
        :default-orbit-type :cycloiddelay
        :default-step-height 0.065
        :default-step-time 1.2
        ;; :cross-trajectory-goal-way-point-offset #f(0.04 0.15 0)
        )
  )

(defun go-cross-step ()
  (warn "go-cross-step~%")
  (send *ri* :set-foot-steps
        (list (make-coords :pos (scale 0 *cross-step*) :name :lleg)
              (make-coords :pos (v+ (scale 1 *cross-step*) *width-offset*) :name :rleg)
              (make-coords :pos (scale 2 *cross-step*) :name :lleg)
              (make-coords :pos (v+ (scale 3 *cross-step*) *width-offset*) :name :rleg)
              (make-coords :pos (scale 4 *cross-step*) :name :lleg)
              ))
  )

(defun cross-step-end ()
  (warn "cross-step-end~%")
  (send *ri* :stop-st)
  (send *ri* :stop-auto-balancer)
  (unix:sleep 2)
  (send *ri* :angle-vector (send *robot* :reset-pose))
  (unix:sleep 5)
  (send *ri* :start-auto-balancer)
  (send *ri* :start-st)
  )
