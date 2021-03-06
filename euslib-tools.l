(defmacro push-back (el lst)
  `(if (null ,lst)
       (setf ,lst (list ,el))
     (nconc ,lst (list ,el))))

(defun reg-replace-impl (old-pattern new-pattern str delimiter g)
  (read-line (piped-fork (format nil "echo ~a|sed -e 's~a~a~a~a~a~a'" str delimiter old-pattern delimiter new-pattern delimiter g))))

(defun reg-replace* (old-pattern new-pattern str &key (delimiter "#"))
  (reg-replace-impl old-pattern new-pattern str delimiter "g"))

(defun reg-replace (old-pattern new-pattern str &key (delimiter "#"))
  (reg-replace-impl old-pattern new-pattern str delimiter ""))

(defun reg-split (split-str str &key (delimiter "#"))
  (let ( (p (piped-fork (format nil "echo ~a|sed -e 's~a~a~a~a~ag'" str delimiter split-str delimiter "\\n" delimiter)))
         (ret nil) (eof (gensym)) tmp )
    (while (not (eq eof (setq tmp (read-line p nil eof)))) (push tmp ret))
    (reverse ret)
  ))

(defun char-replace (old-char new-char str)
  (coerce (substitute (car (coerce new-char cons)) (car (coerce old-char cons)) (coerce str cons)) string))

(defun range (num)
  (let ((i 0) (ret-list nil))
    (while (< i num) (push i ret-list) (incf i))
    (reverse ret-list)
  ))
