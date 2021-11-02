(load "util.l")
(load "euslib-tools.l")
(when (exist-euslib) (load "~/prog/euslib/jsk/gnuplotlib.l"))

(defun setup-plot-tools ()
  (when (find 'gnuplot (functions))
    (cond
     ( (boundp '*gnuplot*) (setq *gp* *gnuplot*) )
     ( (boundp '*gp*)      (setq *gnuplot* *gp*) )
     ( t                   (setq *gnuplot* (gnuplot) *gplot* *gnuplot* *gp* *gnuplot*) )
     )
    (send *gnuplot* :command "set grid")
    (send *gnuplot* :command "set xzeroaxis linewidth 1.5")
    ))

(defun set-plot-common-config (&key (gp *gp*))
  (send gp :command "set size 1,0.6")
  )
(defun unset-plot-common-config ()
  (send gp :command "set size 1,1")
  )

;; extend graph-view for replot
(defun graph-view
    (ordinate-list
     &optional (abscissa-list (user::range (length (car ordinate-list))))
     &key (title "Graph") (xlabel "X") (ylabel "Y") (zlabel "Z")
     (dump-graph nil) (graph-fname (format nil "~A.eps" (substitute #\_ (elt " " 0) title)))
     (lt-lw-params)
     ;;(mode "points")
     (mode "lines")
     keylist xrange yrange zrange
     x11 additional-func
     no-dump ((:graph-instance gp) (if (boundp 'user::*gp*) user::*gp* (setq user::*gp* (gnuplot))))
     ;; (fname (format nil "data~A" (sys::address gp)))
     (raw-date (unix:localtime))
     (fname (reg-replace* " " "0" (format nil "data_20~2d-~2d-~2d-~2d-~2d-~2d" 
                                          (- (elt raw-date 5) 100) (+ (elt raw-date 4) 1) (elt raw-date 3) (elt raw-date 2) (elt raw-date 1) (elt raw-date 0) )))
     (replot nil))

  ;; setup gnuplot instance
  (setup-plot-tools)

  (labels ((gen-range-string
            (range)
            (if range (format nil "[~A:~A]" (car range) (cadr range)) "[]"))
           (2d-or-3d (r-2d r-3d) (if (atom (car abscissa-list)) (eval r-2d) (eval r-3d))))
    (unless keylist (setq keylist (user::range (length ordinate-list))))
    (unless lt-lw-params (setq lt-lw-params (let ((idx -1)) (mapcar #'(lambda (x) (incf idx) "") (make-list (length ordinate-list))))))
    ;; dump dat file
    (unless no-dump
      (with-open-file
       (f (format nil "/tmp/~A.dat" fname) :direction :output)
       (format f (2d-or-3d "# x vals..~%" "# x y vals..~%"))
       (dotimes (i (length abscissa-list))
         (if (atom (car abscissa-list))
             (format f "~A " (elt abscissa-list i))
           (format f "~A ~A " (elt (elt abscissa-list i) 0) (elt (elt abscissa-list i) 1)))
         ;;(dolist (d ordinate-list) (format f "~A " (elt d i)))
         (dolist (d ordinate-list)
           (if (< i (length d))
               (format f "~A " (elt d i))))
         (format f "~%")
         )
       )
      )

    ;; plot
    (mapcar #'(lambda (d1 d2)
                (send gp :command (format nil "set ~A \"~A\"" d1 d2)))
            '(user::title user::xlabel user::ylabel user::zlabel)
            (list title xlabel ylabel zlabel))
    (if additional-func (funcall additional-func))
    (dotimes (i (length ordinate-list))
      (send gp :command
            (format nil "~A \"/tmp/~A.dat\" using ~A title \"~A\" ~A with ~A"
                    (cond
                     ((and (= i 0) (not replot))
                      (apply #'format
                             (list nil
                                   (2d-or-3d "plot ~A ~A" "splot ~A ~A ~A")
                                   (gen-range-string xrange)
                                   (gen-range-string yrange)
                                   (2d-or-3d nil (gen-range-string zrange)))))
                     (t "replot"))
                    fname
                    (format nil "~A:~A" (2d-or-3d "1" "1:2") (+ i (2d-or-3d 2 3)))
                    (elt keylist i)
                    (elt lt-lw-params i)
                    mode))
      )
    (if x11 (send gp :command "set terminal X11"))
    (when dump-graph
      (unix:usleep 200000)
      (send gp :save graph-fname)
      (unix:usleep 200000)
      )
    ))
