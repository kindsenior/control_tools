(defvar *usage-funcs* nil)
(unless (assoc :util *usage-funcs*)
  (push (cons :util
              #'(lambda ()
                  (warn "~%util.l~%")
                  (warning-message 3 "(get-joint-name-list :underbar-replace <replace-pattern or nil>)~%")
                  (warn "~%"))) *usage-funcs*))

(defun get-joint-name-list (&key (robot *robot*) (limb nil) (underbar-replace "-"))
  (let ( (raw-joint-name (if limb
                             (if (or (equal limb :legs) (equal limb :arms))
                                 (send-all (flatten (send robot limb :joint-list)) :name)
                               (send-all (send robot limb :joint-list) :name))
                           (send-all (send robot :joint-list) :name))) )
    (if underbar-replace (mapcar #'(lambda (x) (char-replace "_" underbar-replace x)) raw-joint-name) raw-joint-name)
    ))
