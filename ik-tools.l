(defvar *usage-funcs* nil)
(unless (assoc :ik-tools *usage-funcs*)
  (push (cons :ik-tools
              #'(lambda ()
                  (warn "~%ik-tools.l~%")
                  (warning-message 3 "(set-arguments)~%")
                  (warn "~%"))) *usage-funcs*))

(defun set-arguments ()
  (setq *limb-list* '(:lleg :rleg :larm :rarm))
  (setq *move-targets* (mapcar #'(lambda (limb) (send *robot* limb :end-coords)) *limb-list*))
  (setq *target-coords* (mapcar #'(lambda (target) (send (send target :worldcoords) :copy-worldcoords)) *move-targets*))
  (setq *link-lists* (mapcar #'(lambda (target) (send *robot* :link-list (send target :parent))) *move-targets*)))
