(load "euslib-tools.l")

(defun parse-csv (fname)
  (let ((eof (gensym)) curline)
    (with-open-file
     (csvp fname :direction :input)
     (while (not (eq eof (setq curline (read-line csvp nil eof))))
       (setq curlist (reg-split "," curline))
       (multiple-value-setq (list sec nsec) (mapcar #'read-from-string (subseq curlist 2 4)))
       (print sec nsec)
       ))
    ))
