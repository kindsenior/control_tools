(if (unix:getenv "OLD_HRPSYS")
    (progn ;; 旧システム用
      (warning-message 1 "!!!!!!!!!!using old hrpsys!!!!!!!!!!!~%")
      (load "hrpsys-old-log-parser.l")
      )
  (progn ;; 新システム用
    (load "hrpsys-base-log-parser.l")
    )
  )

(load "plot-tools.l")

(defvar *usage-funcs* nil)
(unless (assoc :log-tools *usage-funcs*)
  (push (cons :log-tools
              #'(lambda ()
                  (warn "~%log-tools.l~%")
                  (warning-message 3 "(save-log :fname <file-name> :add-fname <file-name-option>)~%")
                  (warning-message 3 "(prepare-plot) in log directory~%")
                  (warning-message 3 "  :create t for updating cache file rs-list.l (default nil)")
                  (warning-message 3 "(get-parsed-fname nil) in log directory~%")
                  ;; (warning-message 3 "(make-list-from-input :fname <motion-name>  :span <time-span> )~%")
                  ;; (warning-message 3 "(make-list-and-file-from-log :start <start-index> :span <time-span>)~%")
                  ;; (warning-message 3 "(save-list '(<list-name1> <list-name2>) :make-script t :type <log-type> :multi-plot nil)~%")
                  ;; (warning-message 3 "joint velocity and torque list name: joint-vel-tq-list~%")
                  (warning-message 3 "(analyze-motion)~%")
                  (warning-message 3 "  :step <int>       for using one data in <nums> control cycles (default 10)~%")
                  (warning-message 3 "  :start-tm <float> for setting start time (default 0)~%")
                  (warning-message 3 "  :stop-tm <float>  for setting end time   (default log's final time)~%")
                  (warning-message 3 "  :rs-list <list>   for giving robot state list (default *rs-list*)~%")
                  (warning-message 3 "  :repeat <t/nil>   for repeating animation (default nil)~%")
                  (warning-message 3 "  :dt <float>       for setting control cycle time (default 0.002 [ms])~%")
                  ;; (warning-message 3 "  :yrange [<float>:<float>]  for setting yrange (default auto)~%")
                  (warn "~%"))) *usage-funcs*))

(defun usage()
  (warn "[[usage]]~%")
  (mapc #'(lambda (func-cons) (funcall (cdr func-cons))) *usage-funcs*))

(unless (boundp '*irtviewer*) (make-irtviewer) (send *irtviewer* :change-background #f(0.6 0.6 0.6)))

(defun get-date-string()
  (let ((raw-date (unix:localtime)))
    (reg-replace* " " "0" (format nil "20~2d_~2d-~2d_~2d-~2d_~2d" (- (elt raw-date 5) 100) (+ (elt raw-date 4) 1) (elt raw-date 3) (elt raw-date 2) (elt raw-date 1) (elt raw-date 0)  ))
    ))

(defun save-shm-log (&key (fname nil) (log-dir "k-kojima@jsk.imi.i.u-tokyo.ac.jp/Documents/log") (scp t))
  (let (machine-name user-name date path (target-fname nil))
    (unless (is-simulation)
      (setq machine-name (get-control-machine-name) user-name (get-control-machine-user-name))
      (unix:system (format nil "ssh ~a@~a sudo pkill -2 ecbridge_shm~%" user-name machine-name))

      (when scp
        (if (not fname)
            (if (boundp '*__fname__*)
                (setq fname *__fname__*)
              (setq fname "test")))

        (setq date (get-date-string)
              robot-name (if (boundp '*robot*) (send *robot* :name) (unix:getenv "ROBOT"))
              fname (format nil "~a_~a_~a" fname robot-name date)
              path (format nil "/home/~a/~a/~a/~a" (unix:getenv "USER") log-dir robot-name fname)
              target-fname (format nil "~a/~a.log" path fname))
        (unix:system (format nil "mkdir -p ~a" path))

        (unix:system (format nil "scp ~a@~a:/tmp/shm.log ~a" user-name machine-name target-fname)))
      )
    target-fname
    ))

(defun save-log (&key (fname nil) (add-fname nil) (type :new) (param-list nil) (log-dir "k-kojima@jsk.imi.i.u-tokyo.ac.jp/Documents/log")
                      (parse-flg nil) (save-shm-log nil) (ask-to-save-shm-log t) (save-param t))
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
   ( (equal type :new) (setq fname (save-log-new fname parse-flg log-dir save-shm-log ask-to-save-shm-log save-param)) )
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
(defun save-log-new (fname parse-flg log-dir save-shm-log ask-to-save-shm-log save-param)
  (let ( machine-name raw-date date path fstream uname (org-fname "test") (eof (gensym)) shm-log-saved )

    ;; 日付追加
    (setq date (get-date-string))

    (setq org-fname (format nil "~a_~a" org-fname date))
    (while (not (send (send *ri* :save-log (format nil "/tmp/~a" org-fname) :set-robot-date-string nil) :operation_return)) (warn "do :save-log"))

    ;; save shm log (do not scp and copy at the same time with hrpsys log files)
    (when (setq shm-log-saved (or save-shm-log (and (is-servo-off :print t) (or (not ask-to-save-shm-log) (y-or-n-p "Save shm log?")))))
      (save-shm-log :log-dir log-dir :scp nil))

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
        (setq path (format nil "/home/~a/~a/~a/~a/"
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
           (when shm-log-saved (unix:system (concatenate string "scp " uname "@" machine-name ":/tmp/shm.log " path org-fname ".log")))
           (when save-param (save-param :param-dir path))
           )
         )
        (unix:system (concatenate string "find " path "| rename 's/" org-fname "/" fname "/'"))
        (unix:system (concatenate string "find " path "| rename 's/" (send *robot* :name) "\\(Robot\\)/RobotHardware/'" ));; for old choreonoid log
        (unix:system (concatenate string "find " path "| rename 's/RobotHardware_choreonoid0/RobotHardware0/'" ));; for choreonoid log

        (format t "saved logs in ~a%" path)

        ;; (sys:thread-no-wait #'plot-rs :fname fname :mode :force :save t :create t)
        (if parse-flg (sys:thread-no-wait #'prepare-plot :fname fname :create t))
        ;; (sys:thread-no-wait #'plot-mu :fname fname :save t)

        fname
        ))
    ))

(defun save-param (&key (param-list nil) (param-dir "k-kojima@jsk.imi.i.u-tokyo.ac.jp/Documents/log"))
  (let ( (hrpsys-param-fname (concatenate string param-dir "hrpsys-param.l")) )
  (setq *st-param* (send *ri* :get-st-param))
  (setq *abc-param* (send *ri* :get-auto-balancer-param))
  (dump-loadable-structure hrpsys-param-fname *st-param* *abc-param*)
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
            (:file (setq fname-with-path (reduce #'(lambda (ret val) (concatenate string ret val)) (butlast (reg-split "\\." fname)))) ) ;; fnameにログファイル名を指定した場合
            (:directory (setq fname-with-path (concatenate string fname "/" (car (last (reg-split "/" fname)))) )) ;; fnameにログディレクトリ名を指定した場合
            (t (setq fname-with-path 
                       (concatenate string (unix:getenv "HOME") "/k-kojima@jsk.imi.i.u-tokyo.ac.jp/Documents/log/" (send *robot* :name) "/" fname "/" (car (last (reg-split "/" fname))))) )
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

;; *log* *rs-list* の作成, fname-with-pathのparse
(defun prepare-plot ( &key (create nil) (fname nil) (robot (class *robot*)) (rs-list))
  (let ( (fname-with-path) (rs-fname) )
    (multiple-value-setq (fname-with-path fname) (get-parsed-fname fname))
    (warn "fname-with-path: ~a~%" fname-with-path)
    (warn "fname: ~a~%" fname)
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
       ;; (unix:system "beep; notify-send \"finised convering log file to robot state list\" -t 5; say parsed")
       (unix:system "beep; notify-send \"finised convering log file to robot state list\" -t 5; echo \"done\" | festival --tts")
       (dump-loadable-structure rs-fname *rs-list*) )
     )

    ;; (if rs-list (setq 'rs-list (copy-object *rs-list*)))
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

(defun avg-coords (coords-list &key weight-list)
  (let ((n 1))
    (if (= (length coords-list) 1)
        (car coords-list)
      (reduce #'(lambda (coord0 coord1)  (midcoords (/ 1.0 (incf n)) coord0 coord1)) coords-list))
    ))

(defun create-ref-com-robot (&key (ref-color #f(0 0 1)) (com-color #f(0 1 0)) (act-color #f(1 0 0)))
  (unless (boundp '*ref-robot*)
    (setq *ref-robot* (instance (class *robot*) :init))
    (send-all (send-all (flatten (send-all (send *ref-robot* :links) :get-val 'geometry::bodies)) :glvertices) :set-color ref-color 0.3)
    (send-all (send-all (flatten (send-all (send *robot* :links) :get-val 'geometry::bodies)) :glvertices) :set-color act-color 0.3)
    ;; (objects (list *robot* *ref-robot*))
    )
  (unless (boundp '*com-robot*)
    (setq *com-robot* (instance (class *robot*) :init))
    (send-all (send-all (flatten (send-all (send *com-robot* :links) :get-val 'geometry::bodies)) :glvertices) :set-color com-color 0.3))
  )

(defun get-nearest-point (target-point target-face)
  (car (sort (remove target-face
                     (append (mapcar #'(lambda (e) (send e :point (send e :foot target-point))) (send target-face :edges)) (send target-face :vertices))
                     :test #'(lambda (f p) (eq (send f :insidep p) :outside)))
             #'(lambda (p0 p1) (<= (distance p0 target-point) (distance p1 target-point))))))

(defun truncate-point-to-face (target-point target-face)
  (if (eq (send target-face :insidep target-point) :inside)
      target-point
    (get-nearest-point target-point target-face)))

(defun get-support-polygon (robot)
  (instance polygon :init :vertices (quickhull (flatten (send-all (send robot :support-polygons) :vertices)))))

(defun get-cop (rs limb &optional (wrt :world) &key (robot *robot*) (fz-thre))
  (let ((cop (send robot :calc-cop-from-force-moment
                    (send rs :get (read-from-string (format nil "~A-force-vector" limb)))
                    (send rs :get (read-from-string (format nil "~A-moment-vector" limb)))
                    (car (send robot limb :force-sensors)) (send robot limb :end-coords) :fz-thre fz-thre)))
    (if cop
        (setq cop (truncate-point-to-face cop (find limb (send robot :support-polygons) :test #'(lambda (limb face) (eq limb (send face :name))))))
      (setq cop (send (send *robot* limb :end-coords) :worldpos)))
    (cond
     ((eq wrt :world) cop)
     ((eq wrt :local) (send (send robot limb :end-coords) :inverse-transform-vector cop))
     (t (assert "wrt type error~%")))
    ))

(defun get-contact-state-list (rs &key (fz-thresh 20) (frame) &allow-other-keys)
  (cond
   ((equal frame :ref) (mapcar #'(lambda (state-int) (= state-int 1)) (concatenate cons (send rs :get :auto-balancer-contact-states))))
   ((equal frame :act) (mapcar #'(lambda (limb) (> (elt (send rs :get (read-from-string (format nil "~a-off-force-vector" limb))) 2) fz-thresh)) '(:rleg :lleg :rarm :larm)))))

(defun get-contact-limbs (rs &rest args &key &allow-other-keys)
  (remove nil (mapcar #'(lambda (limb contact-state) (if contact-state limb)) '(:rleg :lleg :rarm :larm) (apply #'get-contact-state-list rs args))))

(defun get-contact-coords (robot contact-limbs)
  (mapcar #'(lambda (limb) (send robot limb :end-coords :copy-worldcoords)) contact-limbs))

(defun move-by-foot-step (robot prev-contact-coords prev-contact-limbs cur-contact-limbs &key (cop))
  (let* ((local-offsets (if cop cop (mapcar #'(lambda (val) #f(0 0 0)) prev-contact-limbs)))
         (common-contact-limbs (intersection prev-contact-limbs cur-contact-limbs))
         (common-contact-offsets (mapcar #'(lambda (limb) (elt local-offsets (position limb prev-contact-limbs))) common-contact-limbs))
         (common-contact-coords (mapcar #'(lambda (limb) (elt prev-contact-coords (position limb prev-contact-limbs))) common-contact-limbs))
         (common-contact-tf (mapcar #'(lambda (limb prev-coord) (send prev-coord :transformation (send robot limb :end-coords) :world)) common-contact-limbs common-contact-coords)))

    ;; (cond
    ;;  ((= (length common-contact-limbs) 1) (send robot :fix-leg-to-coords (car common-contact-coords) (car common-contact-limbs)))
    ;;  (t (send robot :fix-leg-to-coords (apply #'midcoords 0.5 common-contact-coords) :both)))
    ;; ;; (send robot :translate (scale (/ 1.0 (length common-contact-tf)) (apply #'v+ #f(0 0 0) (mapcar #'(lambda (tf) (send tf :pos)) common-contact-tf))) :world);; use only pos

    (cond
     ((= (length common-contact-limbs) 0) (warn "  contact limbs: ~s -> ~s~%" prev-contact-limbs cur-contact-limbs))
     ((= (length common-contact-limbs) 1)
      (send robot :translate (v- (send (send (car common-contact-coords) :translate (car common-contact-offsets)) :worldpos) (send (send (send robot (car common-contact-limbs) :end-coords :copy-worldcoords) :translate (car common-contact-offsets)) :worldpos)) :world))
     (t
      (send robot :translate (v- (send (apply #'midcoords 0.5 (mapcar #'(lambda (ccoord coffset) (send ccoord :translate coffset)) common-contact-coords common-contact-offsets)) :worldpos)
                                 (send (apply #'midcoords 0.5 (mapcar #'(lambda (ccoord coffset) (send ccoord :translate coffset)) (send robot :legs :end-coords :copy-worldcoords) common-contact-offsets)) :worldpos)) :world)))
    ))

(defun set-contact-link-drawing (robot contact-limbs &key (color) (contact-opacity) (float-opacity))
  (send-all (send-all (flatten (send-all (mapcar #'(lambda (limb) (send robot limb :end-coords :parent)) '(:lleg :rleg :larm :rarm)) :get-val 'geometry::bodies)) :glvertices) :set-color color float-opacity)
  (send-all (send-all (flatten (send-all (mapcar #'(lambda (limb) (send robot limb :end-coords :parent)) contact-limbs) :get-val 'geometry::bodies)) :glvertices) :set-color color contact-opacity))

(defun resetup-angle-vector (joint-list &key (resetup-robot *robot*) (reference-robot *ref-robot*) (mode :act2ref))
  (mapc #'(lambda (joint-key)
            (mapc #'(lambda (joint-method) (send (send resetup-robot joint-method) :joint-angle (send (send reference-robot joint-method) :joint-angle)))
                  (remove (subseq (format nil "~s" joint-key) 1) (send resetup-robot :methods);; remove colon from joint-key
                          :test #'(lambda (key robot-method-symbol)
                                    (let ((robot-method (format nil "~s" robot-method-symbol)))
                                    ;;   (if (equal key (subseq robot-method (- (length robot-method) (length key)) (length robot-method)))
                                    ;;       (format t "key: ~s  robot-method: ~s~%" key robot-method))
                                      (not (equal key (subseq robot-method (- (length robot-method) (length key)) (length robot-method)))))))
                  ))
        joint-list)
  )

(defun analyze-motion (&key (step 10) (rs-list *rs-list*) (start-tm 0) (stop-tm (send (car (last rs-list)) :time)) (yrange '(-100 1000)) (xtics 0.5) (repeat nil) (plot t)
                                 (add-fname nil) (fname (format nil "cmz")) (save nil)
                                 (dt 0.002) (advance-tm 0.04) (fz-thresh 50) (takeoff-fz-thresh fz-thresh) (landing-fz-thresh fz-thresh) (min-jump-time 0.05)
                                 (mode "lines") (sleep-time 0) (root-mode :real) (model-mode :ref-act) (dcm-lpr 0.4)
                                 (resetup-joint-list nil)
                                 )
  (let ((idx) (rs) (advance-rs) (advance-idx (round (/ advance-tm (* dt step)))) (start-idx) (stop-idx) (fz) (takeoff-idx nil) (landing-idx nil) (act-contact-state-list) (last-cm) (last-dcm) (ref-last-cm) (last-ref-root-pos)
        ref-root-pos-list ref-root-vel-list
        ref-lfoot-pos-list act-lfoot-pos-list ref-lfoot-rot-list act-lfoot-rot-list act-root-accel-list
        ref-L-list act-L-list
        (prev-act-robot-state-list)
        (ref-color #f(0 0 1)) (com-color #f(0 1 0)) (act-color #f(1 0 0)) (contact-opacity 0.8) (float-opacity 0.3)
        (g-vec #f(0 0 -9800)) (time-step (* dt step)))
    (warn "analyze-motion~%")
    (multiple-value-setq (start-idx stop-idx) (calc-xrange (list start-tm stop-tm) rs-list))

    (setup-plot-tools)

    ;; (send *gp* :command "set size 2,1.5")
    (send *gp* :command "set key right outside")
    (send *gp* :command "set key font \"Times New Roman,15\"")
    (if yrange (send *gp* :command (format nil "set yrange [~s:~s]" (car yrange) (cadr yrange))) (send *gp* :command "unset yrange"))
    (if xtics (send *gp* :command (format nil "set xtics ~s" xtics)) (send *gp* :command "unset xtics"))

    (setq tm-list nil
          ref-r-fz-list nil ref-l-fz-list nil act-r-fz-list nil act-l-fz-list nil)

    (create-ref-com-robot :ref-color ref-color :com-color com-color :act-color act-color)
    (cond
     ((equal model-mode :ref-act) (objects (list *ref-robot* *robot*)))
     ((equal model-mode :com-act) (objects (list *com-robot* *robot*)))
     ((equal model-mode :ref-com) (objects (list *com-robot* *ref-robot*)))
     ((equal model-mode :all)     (objects (list *com-robot* *ref-robot* *robot*))))

    (setq idx start-idx)
    (do-until-key-with-check
     (progn (if (< idx stop-idx) t (if repeat (progn (setq idx start-idx) t) nil)))
     (setq rs (elt rs-list idx)
           advance-rs (elt rs-list (+ idx advance-idx)))
     (push-back (send rs :time) tm-list)

     ;; initial procedure
     (when (= idx start-idx)
       (setq ref-dcm-list nil act-dcm-list nil ref-cm-list nil act-cm-list nil
             ref-root-pos-list nil ref-root-vel-list nil
             ref-root-rot-list nil act-root-rot-list nil
             ref-zmp-list nil act-zmp-list nil
             ref-r-fz-list nil ref-l-fz-list nil act-r-fz-list nil act-l-fz-list nil
             ref-lfoot-pos-list nil act-lfoot-pos-list nil ref-lfoot-rot-list nil act-lfoot-rot-list nil
             ref-max-edge-list nil ref-min-edge-list nil
             ref-top-pos-list nil act-top-pos-list nil
             act-root-accel-list nil
             ref-L-list nil act-L-list nil
             prev-act-robot-state-list nil)

       ;; initialization for actual robot
       (send *robot* :angle-vector (send rs :angle-vector))
       (send *robot* :fix-leg-to-coords (make-coords))
       (let ((rsd (instance robot-state-data)))
         (send rsd :set-val 'angle-vector (send *robot* :angle-vector))
         (send rsd :set-val 'root-coords  (send (car (send *robot* :links)) :worldcoords :copy-worldcoords))
         (dotimes (i 3) (push-back rsd prev-act-robot-state-list)))

       ;; initialization for reference robot
       ;; (send *robot* :reset-coords)
       ;; (send *ref-robot* :reset-coords)
       (send *ref-robot* :angle-vector (send rs :get :auto-balancer-reference-vector))
       ;; (send *ref-robot* :fix-leg-to-coords (make-coords))
       ;; (setq abc-offset-tf (send (send rs :get :auto-balancer-reference-root-coords) :transformation (send (car (send *ref-robot* :links)) :worldcoords)))
       (send *ref-robot* :move-coords (send rs :get :auto-balancer-reference-root-coords) (car (send *ref-robot* :links)))
       (setq abc-offset-tf (send (apply #'midcoords 0.5 (send *ref-robot* :legs :end-coords)) :inverse-transformation));; assume initialpose is both foot contact
       (dotimes (i 3)
         (send *ref-robot* :angle-vector (send rs :get :auto-balancer-reference-vector))
         (send *ref-robot* :move-coords (send (send (send rs :get :auto-balancer-reference-root-coords) :inverse-transformation) :transformation abc-offset-tf) (car (send *ref-robot* :links))))

       ;; initialization for command robot
       (send *com-robot* :angle-vector (send rs :reference-angle-vector))
       (send *com-robot* :fix-leg-to-coords (make-coords))

       (setq ref-last-cm (copy-object (send *ref-robot* :centroid)))
       (setq last-ref-root-pos (send (send (car (send *ref-robot* :links)) :copy-worldcoords) :worldpos))

       (setq last-cm (copy-object (send *robot* :centroid)))
       (setq prev-act-contact-limbs (get-contact-limbs advance-rs :fz-thresh fz-thresh :frame :act))
       (setq prev-act-contact-coords (list (make-coords) (make-coords)))

       )

     ;; procedure for getting reference data
     (send *ref-robot* :angle-vector (send rs :get :auto-balancer-reference-vector))
     (send *ref-robot* :move-coords (send (send (send rs :get :auto-balancer-reference-root-coords) :inverse-transformation) :transformation abc-offset-tf) (car (send *ref-robot* :links)))
     ;; (send *ref-robot* :move-coords (send (send rs :get :auto-balancer-reference-root-coords) :transform abc-offset-tf) (car (send *ref-robot* :links)))
     ;; (send *ref-robot* :move-coords (send (send (send rs :get :auto-balancer-reference-root-coords) :copy-worldcoords) :transform abc-offset-tf) (car (send *ref-robot* :links)))
     (push-back (copy-object (send *ref-robot* :centroid)) ref-cm-list)
     (push-back (scale (/ 1.0 time-step) (v- (car (last ref-cm-list)) ref-last-cm)) ref-dcm-list)
     (setq ref-last-cm (car (last ref-cm-list)))

     (send *ref-robot* :calc-torque :dt time-step :calc-statics-p nil)
     (push-back (v- (reduce #'v+ (send-all (send *ref-robot* :links) :get-val 'angular-momentum))
                    (scale (* 1e-3 (send *ref-robot* :weight)) (v* (scale 1e-3 (car (last ref-cm-list))) (scale 1e-3 (car (last ref-dcm-list)))))) ref-L-list)

     (push-back (send (send (car (send *ref-robot* :links)) :copy-worldcoords) :worldpos) ref-root-pos-list)
     (push-back (scale (/ 1.0 time-step) (v- (car (last ref-root-pos-list)) last-ref-root-pos)) ref-root-vel-list)
     (setq last-ref-root-pos (car (last ref-root-pos-list)))
     (push-back (send (send (car (send *ref-robot* :links)) :copy-worldcoords) :worldrot) ref-root-rot-list)

     ;; procedure for getting actual data
     ;; set fz-thresh (need to modify)
     (if (and takeoff-idx (> idx (+ takeoff-idx (/ min-jump-time time-step)))) (setq fz-thresh landing-fz-thresh) (setq fz-thresh takeoff-fz-thresh));; takeoff-idx is nil during support phase

     ;; move act robot in world coords using foot step and cog velocity
     (if (and (> (length (setq act-contact-limbs (get-contact-limbs advance-rs :fz-thresh fz-thresh :frame :act))) 0) (> (length prev-act-contact-limbs) 0))
         (progn ;; supporting phase
           (send *robot* :angle-vector (send rs :angle-vector))
           (resetup-angle-vector resetup-joint-list :resetup-robot *robot* :reference-robot *ref-robot*);; resetup angle-vector
           (send *robot* :move-coords (send rs :root-coords) (car (send *robot* :links)));; for setting orientation
           (move-by-foot-step *robot* prev-act-contact-coords prev-act-contact-limbs act-contact-limbs :cop (mapcar #'(lambda (limb) (get-cop advance-rs limb :local :fz-thre fz-thresh)) prev-act-contact-limbs))
           (mapcar #'(lambda (limb) (send (make-coords :pos (get-cop advance-rs limb :world :fz-thre fz-thresh)) :draw-on :flush nil :size 50 :color #f(1 1 0))) prev-act-contact-limbs)

           (send *com-robot* :angle-vector (send rs :reference-angle-vector))
           (send *com-robot* :move-coords (make-coords :rpy (map float-vector #'deg2rad (send (elt *rs-list* 1000) :get :stabilizer-current-base-rpy))) (car (send *com-robot* :links)))
           (move-by-foot-step *com-robot* prev-act-contact-coords prev-act-contact-limbs act-contact-limbs)
           (when (and (not landing-idx) takeoff-idx)
             (setq landing-idx idx)
             (unless repeat (warn "takeoff: ~a (~a[sec]) landing: ~a (~a[sec]) flight: ~a(~a[sec])~%" takeoff-idx (send (elt rs-list takeoff-idx) :time) landing-idx (send (elt rs-list landing-idx) :time)
                                  (- landing-idx takeoff-idx) (- (send (elt rs-list landing-idx) :time) (send (elt rs-list takeoff-idx) :time))))
             (setq takeoff-idx nil))
           ;; (warn "~a ~a ~a~%" (send rs :time) prev-act-contact-limbs (send prev-act-contact-coords :worldpos))
           (push-back (copy-object (send *robot* :centroid)) act-cm-list)
           (push-back (v+ (scale (* dcm-lpr (/ 1.0 time-step)) (v- (car (last act-cm-list)) last-cm)) (scale (- 1 dcm-lpr) (if act-dcm-list (car (last act-dcm-list)) #f(0 0 0)))) act-dcm-list)
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
         (resetup-angle-vector resetup-joint-list :resetup-robot *robot* :reference-robot *ref-robot*);; resetup angle-vector
         (send *robot* :move-coords (send rs :root-coords) (car (send *robot* :links)))
         (send *robot* :translate (v- (car (last act-cm-list)) (send *robot* :centroid)) :world)
         (send *com-robot* :angle-vector (send rs :reference-angle-vector))
         (send *com-robot* :move-coords (make-coords :rpy (map float-vector #'deg2rad (send (elt *rs-list* 1000) :get :stabilizer-current-base-rpy))) (car (send *com-robot* :links)))
         (send *com-robot* :translate (v- (car (last act-cm-list)) (send *com-robot* :centroid)) :world)

         (setq last-cm (car (last act-cm-list)))
         (setq last-dcm (car (last act-dcm-list)))

         (setq prev-act-contact-limbs (get-contact-limbs advance-rs :fz-thresh fz-thresh :frame :act))
         (setq prev-act-contact-coords (get-contact-coords *robot* prev-act-contact-limbs))
         )
       )

     (push-back (send (send *ref-robot* :lleg :end-coords :copy-worldcoords) :worldpos) ref-lfoot-pos-list)
     (push-back (send (send *ref-robot* :lleg :end-coords :copy-worldcoords) :worldrot) ref-lfoot-rot-list)
     (push-back (send (send *robot*     :lleg :end-coords :copy-worldcoords) :worldpos) act-lfoot-pos-list)
     (push-back (send (send *robot*     :lleg :end-coords :copy-worldcoords) :worldrot) act-lfoot-rot-list)
     (push-back (send (send (car (send *robot* :links)) :copy-worldcoords) :worldrot) act-root-rot-list)

     (push-back (send (send (car (send *ref-robot* :head)) :copy-worldcoords) :worldpos) ref-top-pos-list)
     (push-back (send (send (car (send *robot*     :head)) :copy-worldcoords) :worldpos) act-top-pos-list)

     ;; calculate zmp in world coords
     ;; (push-back (send (avg-coords (get-contact-coords *ref-robot* (get-contact-limbs rs :frame :ref))) :transform-vector (send rs :get :stabilizer-reference-zmp)) ref-zmp-list)
     ;; (push-back (send (avg-coords (get-contact-coords *ref-robot* (get-contact-limbs rs :frame :ref))) :transform-vector (send rs :get :stabilizer-zmp))           act-zmp-list);; use ref-robot
     ;; foot origin coords
     (push-back (send rs :get :stabilizer-reference-zmp) ref-zmp-list)
     (push-back (send rs :get :stabilizer-zmp)           act-zmp-list)

     ;; calculate edge
     (push-back (reduce #'(lambda (ret vec) (mapcar #'(lambda (x y) (max x y)) ret vec)) (mapcar #'(lambda (val) (coerce val cons)) (send (get-support-polygon *ref-robot*) :vertices))) ref-max-edge-list)
     (push-back (reduce #'(lambda (ret vec) (mapcar #'(lambda (x y) (min x y)) ret vec)) (mapcar #'(lambda (val) (coerce val cons)) (send (get-support-polygon *ref-robot*) :vertices))) ref-min-edge-list)

     ;; add new robot-state and remove old robot-state
     (let ((rsd (instance robot-state-data)))
       (send rsd :set-val 'angle-vector (send *robot* :angle-vector))
       (send rsd :set-val 'root-coords  (send (car (send *robot* :links)) :worldcoords :copy-worldcoords))
       (push-back (copy-object rsd) prev-act-robot-state-list)
       (pop prev-act-robot-state-list))

     ;; set recent 3 poses
     (dolist (act-rs prev-act-robot-state-list)
       (send *robot* :angle-vector (send act-rs :angle-vector))
       (send *robot* :move-coords  (send act-rs :root-coords) (car (send *robot* :links))))

     (send *robot* :calc-torque :dt time-step :calc-statics-p nil)
     (push-back (v- (reduce #'v+ (send-all (send *robot* :links) :get-val 'angular-momentum))
                    ;; (scale (* 1e-3 (send *robot* :weight)) (v* (scale 1e-3 (car (last act-cm-list))) (scale 1e-3 (car (last act-dcm-list)))))) act-L-list)
                    (scale (* 1e-3 (send *robot* :weight)) (v* (scale 1e-3 (car (last ref-cm-list))) (scale 1e-3 (car (last ref-dcm-list)))))) act-L-list);; use ref cm,dcm

     ;; overwrite root position
     ;; (cond
     ;;  ((equal root-mode :ref2act) (send *ref-robot* :move-coords (send (car (send *robot* :links)) :worldcoords) (car (send *ref-robot* :links))))
     ;;  ((equal root-mode :act2ref) (send *robot* :move-coords (send (car (send *ref-robot* :links)) :worldcoords) (car (send *robot* :links)))))
     (cond
      ((equal root-mode :ref2act) (send *ref-robot* :translate (v- (send (car (send *robot* :links)) :worldpos) (send (car (send *ref-robot* :links)) :worldpos)) :world))
      ((equal root-mode :act2ref) (send *robot* :translate (v- (send (car (send *ref-robot* :links)) :worldpos) (send (car (send *robot* :links)) :worldpos)) :world)))

     (set-contact-link-drawing *ref-robot* (get-contact-limbs advance-rs :fz-thresh fz-thresh :frame :ref) :color ref-color :contact-opacity contact-opacity :float-opacity float-opacity)
     (set-contact-link-drawing *robot*     (get-contact-limbs advance-rs :fz-thresh fz-thresh :frame :act) :color act-color :contact-opacity contact-opacity :float-opacity float-opacity)

     ;; drawing
     (send (make-coords :pos (car (last ref-cm-list))) :draw-on :flush nil :size 100 :color #f(0 1 1))
     (send (make-coords :pos (car (last act-cm-list))) :draw-on :flush nil :size 100 :color #f(1 1 0))
     (send (send rs :get :auto-balancer-reference-root-coords) :draw-on :flush t)

     (push-back (elt (send rs :get :st-rleg-ref-force) 2) ref-r-fz-list)
     (push-back (elt (send rs :get :st-lleg-ref-force) 2) ref-l-fz-list)
     (push-back (elt (send rs :get :rleg-off-force-vector) 2) act-r-fz-list)
     (push-back (elt (send rs :get :lleg-off-force-vector) 2) act-l-fz-list)

     (push-back (transform (send (send *robot* :gsensor) :worldrot) (send (car (last *rs-list*)) :get :accel-vector)) act-root-accel-list)

     (send *irtviewer* :draw-objects :flush nil)
     ;; (send *irtviewer* :draw-objects)
     (x::window-main-one)
     (unix:usleep sleep-time)
     (incf idx step)
     )

    (warning-message 3 "Stopped at ~a sec (~a)~%" (send rs :time) idx)

    (let ((pos-scale 1) (vel-scale 0.5) (acc-scale 100) (rot-scale 10) (Lscale 10))
      (setq ref-cmx-list (mapcar #'(lambda (cm) (elt cm 0)) ref-cm-list)
            act-cmx-list (mapcar #'(lambda (cm) (elt cm 0)) act-cm-list)
            ref-cmz-list (mapcar #'(lambda (cm) (elt cm 2)) ref-cm-list)
            act-cmz-list (mapcar #'(lambda (cm) (elt cm 2)) act-cm-list)
            ref-dcmz-list (mapcar #'(lambda (dcm) (* (elt dcm 2) vel-scale)) ref-dcm-list)
            act-dcmz-list (mapcar #'(lambda (dcm) (* (elt dcm 2) vel-scale)) act-dcm-list)
            ref-root-y-list (mapcar #'(lambda (pos) (elt pos 1)) ref-root-pos-list)
            ref-root-dy-list (mapcar #'(lambda (pos) (elt pos 1)) ref-root-vel-list)
            ref-lfoot-z-list (mapcar #'(lambda (pos) (* (- (elt pos 2) 4) pos-scale)) ref-lfoot-pos-list)
            act-lfoot-z-list (mapcar #'(lambda (pos) (* (elt pos 2) pos-scale)) act-lfoot-pos-list)
            ref-zmp-x-list (mapcar #'(lambda (zmp) (elt zmp 0)) ref-zmp-list)
            act-zmp-x-list (mapcar #'(lambda (zmp) (elt zmp 0)) act-zmp-list)
            ref-zmp-y-list (mapcar #'(lambda (zmp) (elt zmp 1)) ref-zmp-list)
            act-zmp-y-list (mapcar #'(lambda (zmp) (elt zmp 1)) act-zmp-list)
            act-root-accel-z-list (mapcar #'(lambda (accel) (* (elt accel 2) acc-scale)) act-root-accel-list)
            ref-lfoot-pi-list (mapcar #'(lambda (rot) (* (rad2deg (elt (car (rpy-angle rot)) 1)) rot-scale)) ref-lfoot-rot-list)
            act-lfoot-pi-list (mapcar #'(lambda (rot) (* (rad2deg (elt (car (rpy-angle rot)) 1)) rot-scale)) act-lfoot-rot-list)
            ref-max-edge-y-list (mapcar #'(lambda (e) (elt e 1)) ref-max-edge-list)
            ref-min-edge-y-list (mapcar #'(lambda (e) (elt e 1)) ref-min-edge-list)
            ref-lfoot-pi-list (mapcar #'(lambda (rot) (* (rad2deg (elt (car (rpy-angle rot)) 1)) rot-scale)) ref-lfoot-rot-list)
            act-lfoot-pi-list (mapcar #'(lambda (rot) (* (rad2deg (elt (car (rpy-angle rot)) 1)) rot-scale)) act-lfoot-rot-list)
            ref-root-pi-list (mapcar #'(lambda (rot) (* (rad2deg (elt (car (rpy-angle rot)) 1)) rot-scale)) ref-root-rot-list)
            act-root-pi-list (mapcar #'(lambda (rot) (* (rad2deg (elt (car (rpy-angle rot)) 1)) rot-scale)) act-root-rot-list)
            ref-top-z-list (mapcar #'(lambda (pos) (elt pos 2)) ref-top-pos-list)
            act-top-z-list (mapcar #'(lambda (pos) (elt pos 2)) act-top-pos-list)
            ref-Ly-list (mapcar #'(lambda (angular-momentum) (* (elt angular-momentum 1) Lscale)) ref-L-list)
            act-Ly-list (mapcar #'(lambda (angular-momentum) (* (elt angular-momentum 1) Lscale)) act-L-list))

      (when (and (not repeat) plot)
        (send *gp* :command "set ytics 100")
        (graph-view
         ;; (list ref-cmz-list act-cmz-list)
         (list ref-cmx-list act-cmx-list
               ref-cmz-list act-cmz-list ref-dcmz-list act-dcmz-list
               ref-r-fz-list ref-l-fz-list act-r-fz-list act-l-fz-list
               ref-lfoot-z-list act-lfoot-z-list act-root-accel-z-list ref-lfoot-pi-list act-lfoot-pi-list
               ref-root-pi-list act-root-pi-list
               ref-Ly-list act-Ly-list
               )
         tm-list
         :keylist (list "ref CMx" "act CMx"
                        "ref CMz" "act CMz" "ref dCMz" "act dCMz"
                        "ref rFz" "ref lFz" "act rFz" "act lFz"
                        "ref lz" "act lz" "act root accel z" "ref lpi" "act lpi"
                        "ref root pi" "act root pi"
                        "ref Ly" "act Ly"
                        )
         :title fname
         :graph-fname (reg-replace* " " "_" (if add-fname (format nil "~a ~a.eps" fname add-fname) (format nil "~a.eps" fname)))
         :dump-graph save
         :xlabel "Time[s]"
         :ylabel "CMz[mm]"
         :mode mode
         )
        ))
  ))
