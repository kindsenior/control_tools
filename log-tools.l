(if (unix:getenv "OLD_HRPSYS")
    (progn ;; 旧システム用
      (warning-message 1 "!!!!!!!!!!using old hrpsys!!!!!!!!!!!~%")
      (load "hrpsys-old-log-parser.l")
      ;; (when (not (boundp '*gnuplot*))
      ;;   (setq *gnuplot* (gnuplot) *gplot* *gnuplot* *gp* *gnuplot*)
      ;;   (send *gnuplot* :command "set grid")(send *gnuplot* :command "set size 1,1"))
      )
  (progn ;; 新システム用
    (load "hrpsys-base-log-parser.l")
    ;; (when (not (boundp '*gp*))
    ;;   (setq *gp* (gnuplot)) (send *gp* :command "set grid") )
    )
  )

;; (load (format nil "~a/ros/~a/src/jsk-ros-pkg/euslib/jsk/gnuplotlib.l" (unix:getenv "HOME") (unix:getenv "ROS_DISTRO")))
(load "~/prog/euslib/jsk/gnuplotlib.l")

(defvar *usage-funcs* nil)
(unless (assoc :log-tools *usage-funcs*)
  (push (cons :log-tools
              #'(lambda ()
                  (warn "~%log-tools.l~%")
                  (warning-message 3 "(save-log :fname <file-name> :add-fname <file-name-option> :type <:new or :old>)~%")
                  (warning-message 3 "(prepare-plot) in log directory~%")
                  (warning-message 3 "(get-parsed-fname nil) in log directory~%")
                  (warning-message 3 "(make-list-from-input :fname <motion-name>  :span <time-span> )~%")
                  (warning-message 3 "(make-list-and-file-from-log :start <start-index> :span <time-span>)~%")
                  (warning-message 3 "(save-list '(<list-name1> <list-name2>) :make-script t :type <log-type> :multi-plot nil)~%")
                  (warning-message 3 "joint velocity and torque list name: joint-vel-tq-list~%")
                  (warn "~%"))) *usage-funcs*))
(defun usage()
  (warn "[[usage]]~%")
  (mapc #'(lambda (func-cons) (funcall (cdr func-cons))) *usage-funcs*))

(cond
 ( (boundp '*gnuplot*) (setq *gp* *gnuplot*) )
 ( (boundp '*gp*) (setq *gnuplot** *gp*) )
 ( t
   (setq *gnuplot* (gnuplot) *gplot* *gnuplot* *gp* *gnuplot*)
   (send *gnuplot* :command "set grid")(send *gnuplot* :command "set size 1,1") )
 )

(defun set-plot-common-config ()
  (send *gp* :command "set size 1,0.6")
  )
(defun unset-plot-common-config ()
  (send *gp* :command "set size 1,1")
  )

(defun get-date-string()
  (let ((raw-date (unix:localtime)))
    (reg-replace* " " "0" (format nil "20~2d_~2d-~2d_~2d-~2d_~2d" (- (elt raw-date 5) 100) (+ (elt raw-date 4) 1) (elt raw-date 3) (elt raw-date 2) (elt raw-date 1) (elt raw-date 0)  ))
    ))

(defun save-shm-log (&key (fname nil) (log-dir "k-kojima@jsk.imi.i.u-tokyo.ac.jp/Documents/log") (scp t))
  (let (machine-name user-name date path)
    (unless (is-simulation)
      (setq machine-name (get-control-machine-name) user-name (get-control-machine-user-name))
      (unix:system (format nil "ssh ~a@~a sudo pkill -2 ecbridge_shm~%" user-name machine-name))

      (when scp
        (if (not fname)
            (if (boundp '*__fname__*)
                (setq fname *__fname__*)
              (setq fname "test")))

        (setq date (get-date-string)
              fname (format nil "~a_~a_~a" fname (send *robot* :name) date)
              path (format nil "/home/~a/~a/~a/~a" (unix:getenv "USER") log-dir (send *robot* :name) fname) )
        (unix:system (format nil "mkdir -p ~a" path))

        (unix:system (format nil "scp ~a@~a:/tmp/shm.log ~a/~a.log" user-name machine-name path fname)))
    )))

(defun save-log (&key (fname nil) (add-fname nil) (type :new) (param-list nil) (parse-flg nil) (save-shm-log nil) (log-dir "k-kojima@jsk.imi.i.u-tokyo.ac.jp/Documents/log"))
  ;; デフォルトログファイル名
  (if (not fname)
      (if (boundp '*__fname__*)
          (setq fname *__fname__*)
        (setq fname "test")))
  (if (and add-fname (not (equal add-fname ""))) (setq fname (format nil "~a_~a" fname add-fname)))

  ;; param-listをファイル名に反映
  (push "" param-list)
  (setq fname (concatenate
               string fname (reduce #'(lambda (x y) (reg-replace "\\." "-" (concatenate string  x "_" (string (car y)) (cadr y))))  param-list)))

  (cond
   ( (equal type :old) (save-log-old fname) )
   ( (equal type :new) (setq fname (save-log-new fname parse-flg log-dir save-shm-log)) )
   )
  fname
  )
;; プラグインのログを保存してローカルにコピー
;; 旧システム
(defun save-log-old (fname)
  (let ( machine-name raw-date (date-list nil) path )

    (setq raw-date (unix:localtime))
    ;; 年 月 日 時(前後1時間のログをコピーする:hrp2内部時刻のPCの時刻のズレは1時間以下でなければならない)
    (push (reg-replace* " " "0" (format nil "~2d~2d~2d~2d" (- (elt raw-date 5) 100) (+ (elt raw-date 4) 1) (elt raw-date 3) (1- (elt raw-date 2)) )) date-list)
    (push (reg-replace* " " "0" (format nil "~2d~2d~2d~2d" (- (elt raw-date 5) 100) (+ (elt raw-date 4) 1) (elt raw-date 3)     (elt raw-date 2)  )) date-list)
    (push (reg-replace* " " "0" (format nil "~2d~2d~2d~2d" (- (elt raw-date 5) 100) (+ (elt raw-date 4) 1) (elt raw-date 3) (1+ (elt raw-date 2)) )) date-list)

    (send *ci* :stop-and-save-log fname)

    (cond
     ( (equal (send *robot* :name) "HRP2JSK")    (setq machine-name "hrp2007c") )
     ( (equal (send *robot* :name) "HRP2JSKNT")  (setq machine-name "hrp2016c") )
     ( (equal (send *robot* :name) "HRP2JSKNTS") (setq machine-name "hrp2017c") )
     ( t (warning-message 1 "cannot use save-log in this robot~%") )
     )

    (mapc #'(lambda (val)
              (unix:system (concatenate string "scp grxuser@" machine-name ":/tmp/" fname "-log-" machine-name "-" val "*.log /tmp/" ))) date-list)

    (setq path (format nil "/home/~a/Copy/Documents/log/~a/~a-log-~a-20~d_~d_~d_~d_~d_~d"
                       (unix:getenv "USER") (send *robot* :name) fname machine-name
                       (- (elt raw-date 5) 100) (+ (elt raw-date 4) 1) (elt raw-date 3) (elt raw-date 2) (elt raw-date 1) (elt raw-date 0) ) )
    (unix:system (format nil "mkdir -p ~a" path))
    (unix:system (format nil "find /tmp -name ~a* 2>/dev/null | head -7 | xargs -i cp {} ~a" (concatenate string fname "-log-") path))

    (send *ci* :start-log) ))

;; 新システム
(defun save-log-new (fname parse-flg log-dir save-shm-log)
  (let ( machine-name raw-date date path fstream uname (org-fname "test") (eof (gensym)) )

    ;; 日付追加
    (setq date (get-date-string))

    (setq org-fname (format nil "~a_~a" org-fname date))
    (while (not (send (send *ri* :save-log (format nil "/tmp/~a" org-fname) :set-robot-date-string nil) :operation_return)) (warn "do :save-log"))

    ;; save shm log (do not scp and copy at the same time with hrpsys log files)
    (when (or save-shm-log (is-servo-off :print t)) (save-shm-log :log-dir log-dir :scp nil))

    (setq fname (read-line (piped-fork (format nil "zenity --entry --title \"ファイル名\" --text=\"ログファイルの保存名\" --entry-text=\"~s\"" fname)) nil eof))
    (if (eq eof fname)
        nil
      (progn
        (let ((platform-name ""))
          (cond
           ((is-choreonoid) (setq platform-name "choreonoid_"))
           ((is-simulation) (setq platform-name "hrpsys-simulator_")))
          (setq fname (format nil "~a~a_~a_~a" platform-name fname (send *robot* :name) date)))

        ;; コピー先設定
        (setq path (format nil "/home/~a/~a/~a/~a"
                           (unix:getenv "USER") log-dir (send *robot* :name) fname) )
        (unix:system (format nil "mkdir -p ~a" path))

        ;; コピー
        (cond
         ( (is-simulation) 
           ;; (setq uname (unix:getenv "USER")
           ;; 			 cp-command "cp"
           ;; 			 machine-name (unix:system "uname -n"))
           ;; (with-open-file (fstream "/etc/hostname" :direction :input)
           ;;                 (let ( (eof (gensym)) ) (setq machine-name (read-line fstream nil eof)) ))
           (unix:system (concatenate string "cp /tmp/" org-fname "* " path))
           )
         ( t
           (setq cp-command "scp")
           (cond
            ( (equal (send *robot* :name) "HRP2JSK")    (setq uname "grxuser" machine-name "hrp2007c")  )
            ( (equal (send *robot* :name) "HRP2JSKNT")  (setq uname "grxuser" machine-name "hrp2016c")  )
            ( (equal (send *robot* :name) "HRP2JSKNTS") (setq uname "grxuser" machine-name "hrp2017c")  )
            ( (equal (send *robot* :name) "JAXON")      (setq uname "leus"    machine-name "jaxon")     )
            ( (equal (send *robot* :name) "JAXON_RED")  (setq uname "leus"    machine-name "jaxonred")  )
            ( (equal (send *robot* :name) "JAXON_BLUE") (setq uname "leus"    machine-name "jaxonblue") )
            ( t (warning-message 1 "cannot use save-log in this robot~%") )
            )
           (unix:system (concatenate string "scp " uname "@" machine-name ":/tmp/" org-fname "* " path))
           (when (or save-shm-log (is-servo-off)) (unix:system (concatenate string "scp " uname "@" machine-name ":/tmp/shm.log " path "/"org-fname ".log")))
           )
         )
        (unix:system (concatenate string "find " path "| rename 's/" org-fname "/" fname "/'"))
        (unix:system (concatenate string "find " path "| rename 's/" (send *robot* :name) "\\(Robot\\)/RobotHardware/'" ));; for old choreonoid log
        (unix:system (concatenate string "find " path "| rename 's/RobotHardware_choreonoid0/RobotHardware0/'" ));; for choreonoid log

        ;; (sys:thread-no-wait #'plot-rs :fname fname :mode :force :save t :create t)
        (if parse-flg (sys:thread-no-wait #'prepare-plot :fname fname :create t))
        ;; (sys:thread-no-wait #'plot-mu :fname fname :save t)

        fname
        ))
    ))

;; プロットをアニメーションで表示
(defun anim_gnuplot(&key (yrange "[-3:3]") (xtics 0.5) (ylabel "joint angular velocity [rad/s]")
                         (fname "./logs/tmplog.dat")
                         (indices (list 2))
                         (with "line")  
                         (speed 5) (tm-width 2) (offset-tm 1) (start-tm 0) (stop-tm 10) (data-span 3)
                         )
  (let (tmpstr (dt 0.005) )
    (send *gnuplot* :command "set grid")
    (send *gnuplot* :command "set xlabel \"time[s]\"")
    (send *gnuplot* :command (format nil "set ylabel \"~a\"" ylabel))
    (send *gnuplot* :command (format nil "set yrange ~a" yrange))
    (send *gnuplot* :command (format nil "set xtics ~a" xtics))
    (send *gnuplot* :command (format nil "set parametric;set trange ~a;" yrange))
    
    (dotimes (j (round (/ (- stop-tm start-tm) dt)))
      (when (= (mod j speed) 0)
        (send *gnuplot* :command (format nil "set xrange [~a:~a]" (+ start-tm (* j dt) (- offset-tm)) (+ start-tm (* j dt) (- offset-tm) tm-width)))
        (send *gnuplot* :command (format nil "const=~a;" (+ start-tm (* j dt))))

        (if (= j 0)
            (progn
              (setq tmpstr "plot const,t notitle;")
              (dolist (index indices)
                (setq tmpstr (concatenate string tmpstr (format nil "replot \"~a\" every ~a::::~a using 1:~a with ~a title columnhead;"
                                                                fname data-span (round (/ stop-tm dt)) index  with))) )
              (send *gnuplot* :command tmpstr)
              )
          (send *gnuplot* :command "replot")
          )
        )
      
      )
    )
  )

(defun parse-fname-with-path (fname);; fnameは変更されない
  (let ( (fname-with-path) )
    (if fname
        (progn 
          (case (probe-file fname)
            (:file (setq fname-with-path (car (reg-split "\\." fname))) )                       ;; fnameにログファイル名を指定した場合
            (:directory (setq fname-with-path (concatenate string fname "/" (car (last (reg-split "/" fname)))) )) ;; fnameにログディレクトリ名を指定した場合
            (nil (setq fname-with-path 
                       (concatenate string (unix:getenv "HOME") "/Copy/Documents/log/" (send *robot* :name) "/" fname "/" (car (last (reg-split "/" fname))))) )
            )
          ;; (setq fname (car (last (reg-split "/" fname))))
          )
      (setq fname (car (last (reg-split "/" (unix::getenv "PWD"))));; path名とfnameが同じと仮定
            fname-with-path (concatenate string (unix::getenv "PWD") "/" fname))
      )
    fname-with-path
    ))

(defun parse-fname (fname);; fnameは変更されない
  (if fname
      (setq fname (car (last (reg-split "/" fname))))
    (setq fname (car (last (reg-split "/" (unix::getenv "PWD")))));; path名とfnameが同じと仮定
    )
  fname
  )

(defun get-parsed-fname (fname)
  (list (parse-fname-with-path fname) (parse-fname fname)))

(defun set-yrange (yrange)
  (send *gp* :command (format nil " set yrange [~a:~a]" (car yrange) (cdr yrange))))

(defun calc-xrange (xrange rs-list)
  (let ( (start-idx 0) (stop-idx 0) start-time stop-time max-idx )
    (if xrange
        (progn
          (setq start-time (car xrange) stop-time (cadr xrange) max-idx (1- (length rs-list)))
          (until (or (> start-idx max-idx) (< start-time (send (elt rs-list start-idx) :time))) (incf start-idx))
          (until (or (> stop-idx  max-idx) (< stop-time  (send (elt rs-list stop-idx) :time))) (incf stop-idx))
          (list start-idx stop-idx) )
      (list 0 (length rs-list)))
    ))

(defun low-pass-filter (target-list &key (dt 0.004) (cutoff-f nil))
  (let ( rate )
    (when cutoff-f
      (setq rate (/ (* 2 pi cutoff-f dt) (+ 1 (* 2 pi cutoff-f dt))))
      (dotimes (i (1- (length target-list)))
        (setf (elt target-list (1+ i)) (+ (* (- 1 rate) (elt target-list i)) (* rate (elt target-list (1+ i)))) )) )
    target-list
    ))

(defun plot-foot-vel
  (&key (rs-list *rs-list*)
        (add-fname nil) (fname "Slide Foot Velocity")
        (cutoff-f nil) (xrange nil)
        (non-parse-flg nil) )
  (let* ( (xrange (calc-xrange xrange rs-list)) (data (subseq rs-list (car xrange) (cadr xrange)))
          (frame-rate 250) (lfoot-act-vel-list nil) (lfoot-ref-vel-list nil) )
    (send *robot* :angle-vector (send (car data) :reference-angle-vector))
    (send *robot* :fix-leg-to-coords (make-coords) :rleg)
    (setq last-lfoot-act-pos (send *robot* :lleg :end-coords :worldpos))

    (send *robot* :angle-vector (send (car data) :angle-vector))
    (send *robot* :fix-leg-to-coords (make-coords) :rleg)
    (setq last-lfoot-ref-pos (send *robot* :lleg :end-coords :worldpos))

    (dolist (x data)
      ;; センサ値代入
      (send *robot* :angle-vector (send x :angle-vector))
      (send *robot* :fix-leg-to-coords (make-coords) :rleg)
      (push-back (* (norm (subseq (v- (send *robot* :lleg :end-coords :worldpos) last-lfoot-act-pos) 0 2)) frame-rate) lfoot-act-vel-list)
      (setq last-lfoot-act-pos (send (send (send *robot* :lleg :end-coords) :copy-worldcoords) :worldpos))

      ;; 目標値代入
      (send *robot* :angle-vector (send x :reference-angle-vector))
      (send *robot* :fix-leg-to-coords (make-coords) :rleg)
      (push-back (* (norm (subseq (v- (send *robot* :lleg :end-coords :worldpos) last-lfoot-ref-pos) 0 2)) frame-rate) lfoot-ref-vel-list)
      (setq last-lfoot-ref-pos (send (send (send *robot* :lleg :end-coords) :copy-worldcoords) :worldpos))

      )
    (graph-view
     (list (low-pass-filter (if non-parse-flg *lfoot-act-vel-list* lfoot-act-vel-list) :cutoff-f cutoff-f) lfoot-ref-vel-list)
     (mapcar #'(lambda (x) (send x :time)) data)
     :keylist (list "Actual Slide Foot Velocity" "Reference Slide Foot Velocity")
     :title fname
     :graph-fname (reg-replace* " " "_" (if add-fname (format nil "~a ~a.eps" fname add-fname) (format nil "~a.eps" fname)))
     :dump-graph t
     :xlabel "Time[s]"
     :ylabel "Slide Foot Velocity"
     )
    (setq *lfoot-act-vel-list* lfoot-act-vel-list)
    (warn "")
    ))

(defun plot-root-rot
  (&key (axis 0) (rs-list *rs-list*)
        (add-fname nil)   (fname (format nil "Actual Root Rot ~a" (case axis (0 "Yaw") (1 "Pitch") (2 "Roll"))))
        (add-keylist nil) (def-keylist (list "Root Rot Angle"))
        (cutoff-f nil) (xrange nil) (replot nil) (init-zero nil))
  (let* ( (xrange (calc-xrange xrange rs-list)) 
          (data (subseq rs-list (car xrange) (cadr xrange))) )
    (graph-view
     (list (mapcar #'(lambda (x) (rad2deg (elt (car (send (send x :root-coords) :rpy-angle)) axis))) data))
     (mapcar #'(lambda (x) (if init-zero (- (send x :time) (send (car data) :time)) (send x :time))) data)
     :keylist (if add-keylist (mapcar #'(lambda (def-key add-key) (format nil "~a ~a" def-key add-key)) def-keylist add-keylist) def-keylist )
     :title fname
     :graph-fname (reg-replace* " " "_" (if add-fname (format nil "~a ~a.eps" fname add-fname) (format nil "~a.eps" fname)))
     :dump-graph t
     :xlabel "Time[s]"
     :ylabel "Angle [Degree]"
     :replot replot
     )
    ))

;; replotできるように拡張
(defun graph-view
  (ordinate-list
   &optional (abscissa-list (user::range (length (car ordinate-list))))
   &key (title "Graph") (xlabel "X") (ylabel "Y") (zlabel "Z")
   (dump-graph nil) (graph-fname (format nil "~A.eps" (substitute #\_ (elt " " 0) title)))
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
  (labels ((gen-range-string
            (range)
            (if range (format nil "[~A:~A]" (car range) (cadr range)) "[]"))
           (2d-or-3d (r-2d r-3d) (if (atom (car abscissa-list)) (eval r-2d) (eval r-3d))))
    (unless keylist (setq keylist (user::range (length ordinate-list))))
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
            (format nil "~A \"/tmp/~A.dat\" using ~A title \"~A\" with ~A"
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
                    mode))
      )
    (if x11 (send gp :command "set terminal X11"))
    (when dump-graph
      (unix:usleep 200000)
      (send gp :save graph-fname)
      (unix:usleep 200000)
      )
    ))

;; *log* *rs-list* の作成, fname-with-pathのparse
(defun prepare-plot ( &key (create nil) (fname nil) (robot (class *robot*)) (rs-list))
  (let ( fname-with-path fname rs-fname )
    (multiple-value-setq (fname-with-path fname) (get-parsed-fname fname))
    (setq rs-fname (concatenate string (reg-replace "[^\/]*$" "" fname-with-path) "rs-list.l"))

    (when (or (not (boundp '*log*)) create)
      (warning-message 3 "create new *log* from file:~a robot:~a~%" fname-with-path robot)
      (setq *log* (create-log-interface fname-with-path :robot robot)))
    
    (cond
     ( (and (probe-file rs-fname) (not create) )
       (warning-message 3 "load preserved robot-state list (~a)~%" rs-fname)
       (load rs-fname) )
     ( t
       (warning-message 3 "now converting *log* to robot-state list~%")
       (setq *rs-list* (create-rs-list :ri *log*))
       (unix:system "beep; notify-send \"finised convering log file to robot state list\" -t 5; say parsed")
       (dump-loadable-structure rs-fname *rs-list*) )
     )

    (if rs-list (setq 'rs-list (copy-object *rs-list*)))
    (unless (and (boundp '*viewer*) *viewer*) (objects *robot*))
    fname-with-path
    ))

(defun play-log (&key (step 10) (rs-list *rs-list*) (start-tm 0) (stop-tm (send (car (last rs-list)) :time)) (repeat nil))
  (let ((idx) (rs) (start-idx) (stop-idx))
    (multiple-value-setq (start-idx stop-idx) (calc-xrange (list start-tm stop-tm) rs-list))
    (setq idx start-idx)
    (unless (boundp '*ref-robot*)
      (setq *ref-robot* (instance (class *robot*) :init))
      (send-all (send-all (flatten (send-all (send *ref-robot* :links) :get-val 'geometry::bodies)) :glvertices) :set-color (float-vector 0 0 1) 0.3)
      (send-all (send-all (flatten (send-all (send *robot* :links) :get-val 'geometry::bodies)) :glvertices) :set-color (float-vector 1 0 0) 0.3)
      (objects (list *robot* *ref-robot*)))

    (send *robot* :reset-pose)
    (send *robot* :fix-leg-to-coords (make-coords))
    (send *robot* :reset-coords)
    (send *ref-robot* :reset-coords)
    (send *ref-robot* :reset-pose)
    (send *ref-robot* :fix-leg-to-coords (make-coords))

    (do-until-key-with-check
     (progn (if (< idx stop-idx) t (if repeat (progn (setq idx start-idx) t) nil)))
     (setq rs (elt rs-list idx))
     (setq contact-limbs (remove nil (mapcar #'(lambda (limb contact-state) (if (equal contact-state :support) limb)) '(:rleg :lleg :rarm :larm) (send rs :contact-state))))

     ;; (send *robot* :angle-vector (send rs :angle-vector))
     ;; (send *robot* :fix-leg-to-coords (send rs :root-coords))

     (send *ref-robot* :angle-vector (send rs :get :auto-balancer-reference-vector))
     (send *ref-robot* :move-coords (send rs :get :auto-balancer-reference-root-coords) (car (send *ref-robot* :links))) ;; root height 0
     (send (send rs :get :auto-balancer-reference-root-coords) :draw-on :flush t)

     (send *robot* :angle-vector (send rs :angle-vector))
     (send *robot* :move-coords (send (send (send rs :get :imucoords) :inverse-transformation) :transform (make-coords :rpy (list (deg2rad 180) 0 0))) (car (send *robot* :links)))

     ;; move robot for foot position adjusting
     (send *robot* :translate (v- (send (send *ref-robot* :lleg :end-coords) :worldpos) (send (send *robot* :lleg :end-coords) :worldpos)) :world)


     (send *irtviewer* :draw-objects :flush nil)
     ;; (send *irtviewer* :draw-objects)
     (x::window-main-one)
     ;; (unix:usleep 1)
     (incf idx step)
     )
  ))

(defun get-contact-state-list (rs &key (fz-thresh 20) &allow-other-keys)
  (mapcar #'(lambda (limb) (> (elt (send rs :get (read-from-string (format nil "~a-off-force-vector" limb))) 2) fz-thresh)) '(:rleg :lleg :rarm :larm)))

(defun get-contact-limbs (rs &rest args &key &allow-other-keys)
  (remove nil (mapcar #'(lambda (limb contact-state) (if contact-state limb)) '(:rleg :lleg :rarm :larm) (apply #'get-contact-state-list rs args))))

(defun get-contact-coords (robot contact-limbs)
  (mapcar #'(lambda (limb) (send robot limb :end-coords :copy-worldcoords)) contact-limbs))

(defun move-by-foot-step (robot prev-contact-coords prev-contact-limbs cur-cuntact-limbs)
  (let* ((common-contact-limbs (intersection prev-contact-limbs cur-cuntact-limbs))
         (common-contact-coords (mapcar #'(lambda (limb) (elt prev-contact-coords (position limb prev-contact-limbs))) common-contact-limbs))
         (common-contact-tf (mapcar #'(lambda (limb prev-coord) (send prev-coord :transformation (send robot limb :end-coords) :world)) common-contact-limbs common-contact-coords)))
    (cond
     ((= (length common-contact-limbs) 1) (send robot :fix-leg-to-coords (car common-contact-coords) (car common-contact-limbs)))
     (t (send robot :fix-leg-to-coords (apply #'midcoords 0.5 common-contact-coords) :both)))
    ;; (send robot :translate (scale (/ 1.0 (length common-contact-tf)) (apply #'v+ #f(0 0 0) (mapcar #'(lambda (tf) (send tf :pos)) common-contact-tf))) :world);; use only pos
    ))

(defun analyze-jump-motion (&key (step 10) (rs-list *rs-list*) (start-tm 0) (stop-tm (send (car (last rs-list)) :time)) (repeat nil) (add-fname nil) (fname (format nil "cmz"))
                                 (dt 0.002) (fz-thresh 50) (mode "lines") (sleep-time 0) (root-mode :real))
  (let ((idx) (rs) (start-idx) (stop-idx) (fz) (takeoff-idx nil) (landing-idx nil) (act-contact-state-list) (last-cm) (last-dcm) (ref-last-cm)
        (ref-dcm-list nil) (act-dcm-list nil) (ref-cm-list nil) (act-cm-list nil) (tm-list) (ref-cmz-list nil)
        (ref-r-fz-list nil) (ref-l-fz-list nil) (act-r-fz-list nil) (act-l-fz-list nil)
        (g-vec #f(0 0 -9800)) (time-step (* dt step)))
    (multiple-value-setq (start-idx stop-idx) (calc-xrange (list start-tm stop-tm) rs-list))
    (unless (boundp '*ref-robot*)
      (setq *ref-robot* (instance (class *robot*) :init))
      (send-all (send-all (flatten (send-all (send *ref-robot* :links) :get-val 'geometry::bodies)) :glvertices) :set-color (float-vector 0 0 1) 0.3)
      (send-all (send-all (flatten (send-all (send *robot* :links) :get-val 'geometry::bodies)) :glvertices) :set-color (float-vector 1 0 0) 0.3)
      (objects (list *robot* *ref-robot*)))

    (setq idx start-idx)
    (do-until-key-with-check
     (progn (if (< idx stop-idx) t (if repeat (progn (setq idx start-idx) t) nil)))
     (setq rs (elt rs-list idx))
     (push-back (send rs :time) tm-list)

     (when (= idx start-idx);; initial procedure
       (send *robot* :angle-vector (send rs :angle-vector))
       (send *robot* :fix-leg-to-coords (make-coords))
       ;; (send *robot* :reset-coords)
       (send *ref-robot* :reset-coords)
       (send *ref-robot* :angle-vector (send rs :get :auto-balancer-reference-vector))
       (send *ref-robot* :fix-leg-to-coords (make-coords))
       (setq abc-offset-tf (send (send rs :get :auto-balancer-reference-root-coords) :transformation (send (car (send *ref-robot* :links)) :worldcoords)))

       (setq ref-last-cm (copy-object (send *ref-robot* :centroid)))

       (setq last-cm (copy-object (send *robot* :centroid)))
       (setq prev-act-contact-limbs (get-contact-limbs (car rs-list) :fz-thresh fz-thresh))
       (setq prev-act-contact-coords (list (make-coords) (make-coords)))

       (setq ref-dcm-list nil act-dcm-list nil ref-cm-list nil act-cm-list nil
             ref-r-fz-list nil ref-l-fz-list nil act-r-fz-list nil act-l-fz-list nil)
       )

     (send *ref-robot* :angle-vector (send rs :get :auto-balancer-reference-vector))
     ;; (send *ref-robot* :move-coords (send rs :get :auto-balancer-reference-root-coords) (car (send *ref-robot* :links))) ;; root height 0
     (send *ref-robot* :move-coords (send (send rs :get :auto-balancer-reference-root-coords) :transform abc-offset-tf) (car (send *ref-robot* :links)))
     (push-back (copy-object (send *ref-robot* :centroid)) ref-cm-list)
     (push-back (scale (/ 1.0 time-step) (v- (car (last ref-cm-list)) ref-last-cm)) ref-dcm-list)
     (setq ref-last-cm (car (last ref-cm-list)))


     (if (and (> (length (setq act-contact-limbs (get-contact-limbs rs :fz-thresh fz-thresh))) 0) (> (length prev-act-contact-limbs) 0))
         (progn ;; supporting phase
           (send *robot* :angle-vector (send rs :angle-vector))
           (send *robot* :move-coords (send rs :root-coords) (car (send *robot* :links)));; for setting orientation
           (move-by-foot-step *robot* prev-act-contact-coords prev-act-contact-limbs act-contact-limbs)
           (when (and (not landing-idx) takeoff-idx)
             (setq landing-idx idx)
             (unless repeat (warn "takeoff: ~a (~a[sec]) landing: ~a (~a[sec])~%" takeoff-idx (send (elt rs-list takeoff-idx) :time) landing-idx (send (elt rs-list landing-idx) :time)))
             (setq takeoff-idx nil))
           ;; (warn "~a ~a ~a~%" (send rs :time) prev-act-contact-limbs (send prev-act-contact-coords :worldpos))
           (push-back (copy-object (send *robot* :centroid)) act-cm-list)
           (push-back (scale (/ 1.0 time-step) (v- (car (last act-cm-list)) last-cm)) act-dcm-list)
           (setq last-cm (car (last act-cm-list)))
           (setq last-dcm (car (last act-dcm-list)))

           (setq prev-act-contact-limbs act-contact-limbs)
           (setq prev-act-contact-coords (get-contact-coords *robot* prev-act-contact-limbs))
           )
       (progn ;; jump phase and just after jump phase
         (when (not takeoff-idx) (setq takeoff-idx idx landing-idx nil))
         (push-back (apply #'v+ (list last-cm (scale time-step last-dcm) (scale (* 0.5 time-step time-step) g-vec))) act-cm-list)
         (push-back (v+ last-dcm (scale time-step g-vec)) act-dcm-list)

         (send *robot* :angle-vector (send rs :angle-vector))
         (send *robot* :move-coords (send rs :root-coords) (car (send *robot* :links)))
         (send *robot* :translate (v- (car (last act-cm-list)) (send *robot* :centroid)) :world)

         (setq last-cm (car (last act-cm-list)))
         (setq last-dcm (car (last act-dcm-list)))

         (setq prev-act-contact-limbs (get-contact-limbs rs :fz-thresh fz-thresh))
         (setq prev-act-contact-coords (get-contact-coords *robot* prev-act-contact-limbs))
         )
       )
     ;; (cond
     ;;  ((equal root-mode :ref2act) (send *ref-robot* :move-coords (send (car (send *robot* :links)) :worldcoords) (car (send *ref-robot* :links))))
     ;;  ((equal root-mode :act2ref) (send *robot* :move-coords (send (car (send *ref-robot* :links)) :worldcoords) (car (send *robot* :links)))))
     (cond
      ((equal root-mode :ref2act) (send *ref-robot* :translate (v- (send (car (send *robot* :links)) :worldpos) (send (car (send *ref-robot* :links)) :worldpos)) :world))
      ((equal root-mode :act2ref) (send *robot* :translate (v- (send (car (send *ref-robot* :links)) :worldpos) (send (car (send *robot* :links)) :worldpos)) :world)))


     (send (send rs :get :auto-balancer-reference-root-coords) :draw-on :flush t)

     (push-back (elt (send rs :get :st-rleg-ref-force) 2) ref-r-fz-list)
     (push-back (elt (send rs :get :st-lleg-ref-force) 2) ref-l-fz-list)
     (push-back (elt (send rs :get :rleg-off-force-vector) 2) act-r-fz-list)
     (push-back (elt (send rs :get :lleg-off-force-vector) 2) act-l-fz-list)

     (send *irtviewer* :draw-objects :flush nil)
     ;; (send *irtviewer* :draw-objects)
     (x::window-main-one)
     (unix:usleep sleep-time)
     (incf idx step)
     )

    (unless repeat
      (let (ref-cmx-list ref-cmz-list act-cmx-list act-cmz-list)
        (setq ref-cmx-list (mapcar #'(lambda (cm) (elt cm 0)) ref-cm-list)
              ref-cmz-list (mapcar #'(lambda (cm) (elt cm 2)) ref-cm-list)
              ref-dcmz-list (mapcar #'(lambda (dcm) (* (elt dcm 2) 0.1)) ref-dcm-list)
              act-cmx-list (mapcar #'(lambda (cm) (elt cm 0)) act-cm-list)
              act-cmz-list (mapcar #'(lambda (cm) (elt cm 2)) act-cm-list)
              act-dcmz-list (mapcar #'(lambda (dcm) (* (elt dcm 2) 0.1)) act-dcm-list))

        (graph-view
         ;; (list ref-cmz-list act-cmz-list)
         (list ref-cmz-list act-cmz-list ref-dcmz-list act-dcmz-list ref-r-fz-list ref-l-fz-list act-r-fz-list act-l-fz-list)
         tm-list
         :keylist (list "Reference CMz" "Actual CMz" "Reference dCMz" "Actual dCMz" "Reference rFz" "Reference lFz" "Actual rFz" "Actual lFz")
         :title fname
         :graph-fname (reg-replace* " " "_" (if add-fname (format nil "~a ~a.eps" fname add-fname) (format nil "~a.eps" fname)))
         :dump-graph t
         :xlabel "Time[s]"
         :ylabel "CMz[mm]"
         :mode mode
         )
        ))
  ))
