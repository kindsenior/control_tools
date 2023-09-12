;; hrpsysシーケンスファイルから直接、実機またはシミュレータを動かす
;; OpenHRP simulatorの使いかた
;; 8号館602 avocadoにて
;; cd ${CVSDIR}/hrp2/OpenHRP/eus
;; eus test-sim.l
;; (init) or (init :nosim nil);; simulationが起動，あとは実機と同じ
;; (send *ci* :stop-zmp-override)
;; (send *ci* :set-abc-param :check-shuffling nil)
;; (send *ci* :load-pattern "/tmp/****" 5000)
;; (send *ci* :send "seq" ":load ***.pos nil ***.zmp ***.waist")
;; (send *ci* :stop-and-save-log "choreonoid-test-***")


;; (load "~/prog/hrp2/plugins/test-log-parser.l")
(if (unix:getenv "OLD_HRPSYS") (warning-message 1 "!!!!!!!!!!Using Old hrpsys!!!!!!!!!!~%") (warning-message 3 "Using New hrpsys~%"))

(defmacro create-robot-interface (&optional (robot (read-from-string (string-downcase (unix:getenv "ROBOT")))))
  (warning-message 3 "~%loading robot interface file (~a)...~%" robot)
  (list
   'progn
   (list 'require
         (read-from-string (format nil ":~a-interface" robot))
         (read-from-string (format nil "\"package://hrpsys_ros_bridge_tutorials/euslisp/~a-interface.l\"" robot)))
   (list (read-from-string (format nil "~a-init" robot)))
   (list 'setq '*robot* (read-from-string (format nil "*~a*" robot)))
   )
  )

(if (not (unix:getenv "OLD_HRPSYS"))
    (progn ;; new hrpsys
      (setq robot-string (unix:getenv "ROBOT"))
      (if robot-string
          (progn
            (setq interface-file-path (format nil (ros::resolve-ros-path "package://hrpsys_ros_bridge_tutorials/euslisp/~a-interface.l") (read-from-string (string-downcase robot-string))))
            (if (probe-file interface-file-path)
                (unless (boundp '*robot*) (create-robot-interface))
              (progn
                (warning-message 1 "~a not exists~%" interface-file-path)
                (setq *servo-state-msg* nil))
            ))
        (warning-message 1 "Please set ROBOT variable for (create-robot-interface)~%")
        ))
  (when (not (boundp '*hrp2*)) (setup-model-for-hrp2) (setq *robot* *hrp2*)  );; old hrpsys
  )

;; 実機かシミュレーションか判定
(defun is-simulation ()
  (if (ros::get-param "use_sim_time" nil) t nil))

(defun is-choreonoid ()
  (and (is-simulation) (> (read-from-string (read-line (piped-fork "rosnode list |grep choreonoid | wc -l"))) 0)))

(ros::roseus "play-hrpsysfile.l")
(ros::load-ros-manifest "trans_ros_bridge")

(load "log-tools.l")

(defvar *usage-funcs* nil)
(unless (assoc :play-hrpsysfile *usage-funcs*)
  (push (cons :play-hrpsysfile
              #'(lambda ()
                  (warn "~%play-hrpsysfile.l~%")
                  ;; (warning-message 3 "(setup-real-for-hrp2)~%")
                  (warning-message 3 "(create-robot-interface jaxon)~%")
                  (warning-message 3 "(scale-hrpsysfile :fname <FILE NAME>)~%")
                  (warning-message 3 "(copy-files :fname <FILE NAME> :sample-name <SAMPLE NAME> :real t)~%")
                  (warning-message 3 "(prepare-experiment :start-st t :set-pose t :st-algorithm :eefmqpcop2)~%")
                  (warning-message 3 "(init-pose :fname <FILE NAME> :sample-name <SAMPLE NAME> :real t)~%")
                  (warning-message 3 "(play-hrpsysfile :fname <FILE NAME> :sample-name <SAMPLE NAME> :use-init-file nil :real t)~%")
                  (warning-message 3 "(save-log :fname <FILE NAME> :add-fname <FILE NAME OPTION> :log-dir <log directory name from ${HOME}>)~%")
                  (warn "~%")
                  (warning-message 3 "(set-pgain <l-crotch dgain>)~%")
                  (warning-message 3 "(set-dgain <l-crotch dgain>)~%")
                  (warn "~%"))) *usage-funcs*))

(usage)

;; (if (not (boundp '*ci*)) (setup-real-for-hrp2))

(defun get-control-machine-name (&key (robot (if (boundp '*robot*) *robot* nil)))
  (if robot
      (string-downcase (reg-replace "_" "" (send *robot* :name)))
    "localhost") ;; return local machine name for ichijiku test bed
  )

(defun get-control-machine-user-name (&key (robot (if (boundp '*robot*) *robot* nil)))
  (let ((machine-name (get-control-machine-name :robot robot)) (user-name nil))
    (cond
     ((substringp "jaxon" machine-name) (setq user-name "leus"))
     ((substringp "hrp" machine-name) (setq user-name "grxuser"))
     ((equal "root" (unix:getenv "USER")) (setq user-name "k-kojima"))
     (t (warn "~s is not supported robot~%" robot) (setq user-name (unix:getenv "USER"))) ;; return local user name for ichijiku test bed
     )
    user-name
    ))

;; servo-state判定
(defun get-servo-states (&key (print nil))
  (ros::spin-once)

  (let (servo-states)
    (setq servo-states (coerce (send *servo-state-msg* :states) cons))
    (when print (warning-message 3 "servo-states: ") (mapc #'(lambda (val) (warning-message 3 " 0x~0,4X" val)) servo-states) (warn "~%"))
    servo-states
    )
  )

(defun is-servo-off (&key (print nil))
  (if (is-simulation)
      nil ;; always servo-on in simulation
    (find t (mapcar #'(lambda (state) (not (zerop (logand state #X0800)))) (get-servo-states :print print)))
    )
  )

;; posファイルの関節数を修正する
(defun modify-posfile (path fname)
    (when (or (equal (send *robot* :name) "HRP2JSKNT") (equal (send *robot* :name) "HRP2JSKNTS"))
      (unix:system (concatenate string "cat " path fname ".pos | cut -d' ' -f1-35 | cat > " path fname ".pos_; mv " path fname ".pos_ " path fname ".pos")));; 1-35 34関節
    (when (and (equal (send *robot* :name) "JAXON_RED") (not (is-choreonoid)))
      (unix:system (concatenate string "cat " path fname ".pos | cut -d' ' -f1-34 | cat > " path fname ".pos_; mv " path fname ".pos_ " path fname ".pos")));; 1-34 33関節
    )

;; 実際に実機またはローカルの/tmpにファイルをコピー
(defun exec-copy (path fname)
  (warn "~%(exec-copy) now copying files ~a~a~%" path fname)
  (if (is-simulation) 
      ;; hrpsysシミュレータ用 localの/tmpにコピー
      (unix::system (concatenate string "cp " path fname ".* /tmp/" ) )
    ;; 実機
    (progn
      (cond
       ( (equal (send *robot* :name) "HRP2JSK")    (setq uname "grxuser" machine-name "hrp2007c") )
       ( (equal (send *robot* :name) "HRP2JSKNT")  (setq uname "grxuser" machine-name "hrp2016c") )
       ( (equal (send *robot* :name) "HRP2JSKNTS") (setq uname "grxuser" machine-name "hrp2017c") )
       ( (equal (send *robot* :name) "JAXON")      (setq uname "leus"    machine-name "jaxon")    )
       ( (equal (send *robot* :name) "JAXON_RED")  (setq uname "leus"    machine-name "jaxonred") )
       ( (equal (send *robot* :name) "JAXON_BLUE") (setq uname "leus"    machine-name "jaxonblue") )
       ( t (warning-message 1 "No Setting for The Robot~%") )
       )
      (unix:system (concatenate string "scp " path fname ".* " uname "@" machine-name ":/tmp"))
      )
    )
  )

;; 関節角速度ファイル作成
;; 自分のPCから実行
(defun copy-files (&key (fname "SampleMotion") (ofname nil) (real t) (plugin t) (sample-name "ShuffleSample") (remove-last-row t))
  (let ( motionname
         zmpline velfile optfile posfile zmpfile path
         ;; (extention-list '("pos" "hip" "waist" "zmp" "vel" "optionaldata" "wrenches"))
         (extention-list '("pos" "waist" "zmp" "hip" "optionaldata" "wrenches"))
         )

    (if (not ofname) (setq ofname fname))

    (setq zmp-x-list nil zmp-y-list nil zmp-z-list nil);; zmpリスト

    (setq motionname fname)
    (if plugin
      (setq path (concatenate string (ros::resolve-ros-path "package://jsk_choreonoid") "sample/" sample-name "/"))
      (setq path (concatenate string "/home/" (unix::getenv "USER") "/" (unix::getenv "CNOID_WORKSPACE") "/" (send *robot* :name) "/" motionname "/")))
    (setq hipfile (concatenate string path fname ".hip")
          posfile (concatenate string path fname ".pos")
          waistfile (concatenate string path fname ".waist")
          zmpfile (concatenate string path fname ".zmp")
          numjoints (length (send *robot* :angle-vector))
          )

    ;; 最終2行削除 fname -> _fname
    (if remove-last-row
        (mapc #'(lambda (extemtion)
                  (unix:system (concatenate string "sed  -e '$d' " path fname "." extemtion "| sed -e '$d' | cat >  " path "_" fname "." extemtion)) )
              extention-list)
      (mapc #'(lambda (extemtion)
                  (unix:system (concatenate string "mv " path fname "." extemtion  " " path "_" fname "." extemtion)) )
            extention-list))

    ;; waist/hip合成 _fname->fname_->_fname
    (warn "~%now converting waist from waist and hip of _~s~%" fname)
    (unix:system (concatenate string "cat " path "_" fname ".waist | cut -d' ' -f1-4 | cat > " waistfile "_"))
    (unix:system (concatenate string "cat " path "_" fname ".hip   | cut -d' ' -f2-4 | cat > " hipfile   "_"))
    (unix:system (concatenate string "paste " waistfile "_ " hipfile "_ | cat > " path "_" fname ".waist"))
    (unix:system (concatenate string "rm -f " waistfile "_ " hipfile "_ " path "_" fname ".hip"))

    ;; posファイル 関節角度個数合わせ
    (modify-posfile path (concatenate string "_" fname))

    ;; ファイルコピー
    (exec-copy path (concatenate string "_" fname))

    (unix::system (concatenate string "rm -f " path "_" fname ".*"));; 一次生成ファイル削除

    ))

;; simの時はavocadで実行
(defun play-hrpsysfile-old (&key (fname "SampleMotion_200") (real nil) (impedance nil) (tm 5000) (save-log t) )
  (when real
    (when save-log (send *ci* :stop-and-save-log "trash"))
    (send *ci* :start-grasp)
    (send *ci* :stop-zmp-override)
    (send *ci* :set-abc-param :check-shuffling nil)
    (warning-message 3 "Push Enter to start~%")
    (do-until-key)
    (when save-log (send *ci* :start-log))
    (send *ci* :load-pattern (concatenate string "/tmp/" fname) tm)
    (when impedance (send *ci* :start-impedance :lleg))
  ))

(defun prepare-experiment (&key (start-st t) (st-algorithm :eefmqpcop2) (set-pose t) (delay-vec #f(0.055 0.055)) (zmp-offset #f(0 0 0)))
  (warn "(prepare-experiment)~%")
  (when (= (send (send *ri* :get-auto-balancer-param) :controller_mode) 1)
    (warn "stop-st~%")
    (send *ri* :stop-st)
    ;; (unix:sleep 1)
    (warn "stop-auto-balancer~%")
    (send *ri* :stop-auto-balancer)
    ;; (unix:sleep 1)
    (warn "stop-impedance~%")
    (send *ri* :stop-impedance :arms)
    ;; (unix:sleep 1)
    )

  (send *robot* :angle-vector (send *ri* :state :potentio-vector))
  (when set-pose (set-current-pose :tm 2 :wait t :zmp-offset zmp-offset))

  (unless (eq (send *ri* :get-st-algorithm) st-algorithm)
    (warn "stop-st for change st-algorithm~%")
    (send *ri* :stop-st)
    (warn "set st-algorithm to ~s~%" st-algorithm)
    (send *ri* :set-st-param :st-algorithm st-algorithm))
  (send *ri* :set-st-param :eefm-ee-forcemoment-distribution-weight
        (list #f(0 0 1 1 1 0) #f(0 0 1 1 1 0) #f(0 0 0 0 0 0) #f(0 0 0 0 0 0)))
  (unless (= (send (send *ri* :get-st-param) :emergency_check_mode) 0)
    (warn "disable emergency-check~%")
    (send *ri* :set-st-param :emergency-check-mode 0))

  ;; jump setting
  (send *ri* :set-st-param :eefm-use-force-difference-control nil)
  (send *ri* :set-st-param :detection-time-to-air 1.0)
  ;; (send *ri* :set-st-param :emergency-check-mode 0)

  ;; (warn "set eefm-zmp-delay-time-const ~s~%" delay-vec)
  ;; (send *ri* :set-st-param :eefm-zmp-delay-time-const delay-vec)

  (send *ri* :set-log-maxlength (* 500 25))
  (if start-st
      (when (and (or (not (is-simulation)) (is-choreonoid)))
        (warn "start-st~%")
        (send *ri* :start-st)
        (unix:sleep 3))
    (progn
      (warn "stop-st~%")
      (send *ri* :stop-st)))
  )

(defun get-motion-time (path fname)
  (* 0.002 (read (piped-fork (concatenate string "cat " path fname ".pos" " | wc -l"))));; dt=0.002 sec
  )

(defun calc-default-zmp-offset ()
  (let ( (support-vertices (send (send *robot* :support-polygon '(:lleg :rleg)) :vertices)) )
    (float-vector (* 0.5 (+ (apply #'min (mapcar #'(lambda (x) (elt x 0)) support-vertices)) (apply #'max (mapcar #'(lambda (x) (elt x 0)) support-vertices)))) 0 0)))

(defun play-hrpsysfile-impl (&key fname (real t) (tm 5) (save-log t) (save-shm-log nil) (plugin t) (use-init-file nil) (start-st t) (set-pose t) (add-fname nil) (zmp-offset nil)
                                  (sample-name "ShuffleSample") (st-algorithm :eefmqpcop2) (prepare t) (wait-key t))
  (let (path local-path motion-time)
   (if plugin
       (setq local-path (concatenate string (ros::resolve-ros-path "package://jsk_choreonoid") "sample/" sample-name "/"))
     (setq local-path (concatenate string "/home/" (unix::getenv "USER") "/" (unix::getenv "CNOID_WORKSPACE") "/" (send *robot* :name) "/" motionname "/")))

  (setq *__fname__* (reg-replace* "^_" "" fname));; _fname->fname
  (setq motion-time (get-motion-time local-path *__fname__*))

  (unless zmp-offset (setq zmp-offset (calc-default-zmp-offset)))

  (if prepare (prepare-experiment :start-st start-st :set-pose set-pose :st-algorithm st-algorithm :zmp-offset zmp-offset))
  (if use-init-file
      (let ((__init-fname__ (concatenate string *__fname__* "-init"));; fname-init
            (init-fname (concatenate string fname "-init")));; _fname-init
        (copy-files :fname __init-fname__ :sample-name sample-name :real t)
        (warn "now moving to initial pose by .pseq file~%")
        (send *ri* :load-pattern (concatenate string path init-fname) 3)
        (unix:sleep 3)
        )
    (init-pose :fname *__fname__* :sample-name sample-name :real t :wait t :tm tm));; tm sec

  ;; (warning-message 3 "Push Enter to start~%")
  ;; (do-until-key)
  (if (or (= (send *ri* :state :emergency-mode) 0) (not (send *ri* :state :emergency-mode)));; emergency-mode-off or st-off
      (progn
        (warning-message 3 "Push y to start or n to resign~%")
        (when (or (not wait-key) (y-or-n-p))
          (when save-log
            (warn "start-log~%")
            (send *ri* :start-log))
          (when real
            (warn "load-pattern~%")
            (setq path "/tmp/")
            (send *ri* :load-pattern (concatenate string path fname) 1) );; default 1sec in load-pattern?
          (setq sleep-time 1)
          (warn "sleep ~a[sec] before initial pose~%" sleep-time);; 1sec
          (unix:sleep sleep-time)

          (setq sleep-time (round (+ 2 motion-time)))
          (when save-log
            (warn "sleep ~a[sec] during motion~%" sleep-time)
            (unix:sleep sleep-time)
            (save-log :add-fname add-fname :save-shm-log save-shm-log))
          ))
    (warning-message 1 "Please release emergency~%")
    )
  ))

(defun play-hrpsysfile (&rest args)
  (when (boundp '*ci*) (apply #'play-hrpsysfile-old args))
  (when (boundp '*ri*) (apply #'play-hrpsysfile-impl args))
  )

(defun play-demo ( &key (music-file "/home/jskuser/love-cir_demo_x82.wmv") )
  (when (not (boundp '*sh*))
    (setq *sh* (piped-fork "sh"))
    (format *sh* "vlc -I rc --no-loop --no-repeat --volume 450~%") (unix:sleep 1) )
  (format *sh* "clear~%")
  (format *sh* "add ~A~%" (reg-replace* "/" "\/" music-file))
  (unix:usleep 10000)
  (format *sh* "pause~%")

  (play-hrpsysfile :fname "love-cir-slow-moderate-side-point_" :real t :save-log nil)
  (unix::usleep (round (* 1000 1000 9.6)))

  ;; (format *sh* "add ~A~%" (reg-replace* "/" "\/" music-file))
  (format *sh* "play~%")
  ;; (if music-file
  ;;     (unix::system (concatenate string "vlc --no-loop --no-repeat --volume 450 -f --play-and-exit " music-file))
  ;;   (unix::system "vlc --no-loop --no-repeat --volume 450 -f --play-and-exit ~/Dropbox/Videos/research（研究）/love-cir/love-cir_demo_x82.wmv"))

  )

;; hrpsysファイルのタイムスケールを変更する (足りない場合は線形補間)
;; 現状100fpsから200fpsの変更のみ可能
(defun scale-hrpsysfile (&key (fname "SampleMotion"))
  (let (motionname hipfile posfile waistfile zmpfile path )

    (setq motionname fname
          path (concatenate string "/home/" (unix::getenv "USER") "/" (unix::getenv "CNOID_WORKSPACE") "/" (send *robot* :name) "/" motionname "/")
          hipfile (concatenate string path fname "_100fps.hip")
          posfile (concatenate string path fname "_100fps.pos")
          waistfile (concatenate string path fname "_100fps.waist")
          zmpfile (concatenate string path fname "_100fps.zmp")

          hipfile_ (concatenate string path fname ".hip")
          posfile_ (concatenate string path fname ".pos")
          waistfile_ (concatenate string path fname ".waist")
          zmpfile_ (concatenate string path fname ".zmp")

          numjoints (length (send *robot* :angle-vector))
          )

    ;; 各種ファイルをスケール
    (warn "~%now scaling posfile~%")
    (mapc 
     #'(lambda (infile outfile)
         (with-open-file
          (inf infile :direction :input)
          (with-open-file
           (outf outfile :direction :output)
           (let ( (eof (gensym))  curline prevec curvec midvec )
             (while (not (eq eof (setq curline (read-line inf nil eof))))
               (setq curvec (coerce (mapcar #'read-from-string (reg-split " +" curline)) float-vector))

               ;; 補間時刻データ書き込み
               (when prevec
                 (setq midpos (scale 0.5 (v+ prevec curvec)))
                 (dotimes (i (length midpos)) (format outf "~a " (elt midpos i)))
                 (format outf "~%")
                 )

               ;; 非補間時刻データ書き込み
               (format outf "~a~%" curline)

               (setq prevec curvec)
               )))))
     (list hipfile posfile waistfile zmpfile) (list hipfile_ posfile_ waistfile_ zmpfile_)
     )

    ))

;; (defun get-init-contact-state (path fname)
;;   (let ( (file (concatenate string path fname ".optionaldata")) )
;;     (with-open-file
;;      (f file :direction :input)
;;      (let ( (eof (gensym)) )
;;        (subseq (mapcar #'read-from-string (reg-split " " (read-line f nil eof))) 1 5);; return contact state list(4 limbs)
;;        ))
;;     ))

;; モーションの初期姿勢へ遷移 (hrpsysシーケンスファイルの1行目のみを実行)
(defun init-pose (&key (fname "SampleMotion") (tm 10) (real t) (plugin t) (sample-name "ShuffleSample") (wait nil))
  (let* ( (motionname (car (reg-split "_"  fname))) (path) )
    (warn "(init-pose) now moving to initial pose of ~s's ~s in ~s sec~%" sample-name fname tm)

    (if plugin
      (setq path (concatenate string (ros::resolve-ros-path "package://jsk_choreonoid") "sample/" sample-name "/"))
      (setq path (concatenate string "/home/" (unix::getenv "USER") "/" (unix::getenv "CNOID_WORKSPACE") "/" (send *robot* :name) "/" motionname "/")))

    ;; (setq contact-state-list (get-init-contact-state path fname))
    ;; (set-current-pose :contact-state-list '(1 1 0 0) :leg-option :lleg :tm 5 :zmp-offset #f(0 -10 0))
    ;; (warn "now moving centroid on foot of init pose's contact state by load-pattern~%")
    ;; (unix:sleep 5)
    ;; (set-current-pose :contact-state-list contact-state-list :leg-option :lleg :tm 0 :zmp-offset #f(0 -10 0))
    ;; (warn "change contact state by load-pattern~%")

    ;;fname->fname_init
    (mapc #'(lambda (extemtion)
              (unix:system (concatenate string "head -n 1 " path fname "." extemtion "| cat >  " path fname "_init." extemtion)) )
          ;; (list "hip" "waist" "zmp" "vel" "pos" "optionaldata" "wrenches")
          (list "waist" "hip" "zmp" "pos" "optionaldata" "wrenches") )

    ;; fname_init->_fname_init
    (copy-files :fname (concatenate string fname "_init") :sample-name sample-name :real real :remove-last-row nil)

    (when real
      (cond
       ( (boundp '*ci*)
         (send *ci* :start-grasp)
         (send *ci* :stop-zmp-override)
         (send *ci* :set-abc-param :check-shuffling nil)

         (send *ci* :set-impedance-param :moment-gain 0 :legs)
         (send *ci* :set-impedance-param :axis-p :z :legs)
         (send *ci* :set-impedance-param :mdk #f(0.01 5 3) :legs)
         (send *ci* :set-ref-force #f(0 0 200) 1000 :legs)

         (send *ci* :load-pattern (concatenate string "/tmp/_" fname "_init") tm) )
       ( (boundp '*ri*)
         (send *ri* :load-pattern (concatenate string "/tmp/_" fname "_init") tm) )
       )
      )

    (unix::system (concatenate string "rm -f " path fname "_init.*"));; 一次生成ファイル削除
    (unix::system (concatenate string "rm -f " path fname "_init_.*"));; 一次生成ファイル削除

    (if wait (unix:sleep (round tm)))
    ))

(defun init-landing-pose (&key (fname "SampleMotion") (tm 2000) (real t) (plugin t) (sample-name "ShuffleSample"))
  (let* ( (motionname (car (reg-split "_"  fname))) (path) )
    (if plugin
      (setq path (concatenate string (ros::resolve-ros-path "package://jsk_choreonoid") "sample/" sample-name "/"))
      (setq path (concatenate string "/home/" (unix::getenv "USER") "/" (unix::getenv "CNOID_WORKSPACE") "/" (send *robot* :name) "/" motionname "/")))

    (send *robot* :angle-vector (coerce (mapcar #'rad2deg (cdr (mapcar #'read-from-string (reg-split " " (read-line (piped-fork (concatenate string "head -n 1 " path fname ".pos"))))))) float-vector))
    (send *robot* :fix-leg-to-coords (make-coords))
    (setq target-endcoords (mapcar #'(lambda (limb) (send *robot* limb :end-coords :copy-worldcoords)) '(:lleg :rleg)))

    (send *robot* :reset-landing-pose)
    (send *robot* :fix-leg-to-coords (make-coords))
    (mapc #'(lambda (limb target-coord) (send *robot* limb :inverse-kinematics target-coord)) '(:lleg :rleg) target-endcoords)

    (set-current-pose)
  ))

(defun set-current-pose (&key (tm 10) (zmp-offset #f(0 0 0)) (lcop-offset #f(0 0)) (rcop-offset #f(0 0)) (wait nil)
                              (contact-state-list '(1 1 0 0)) (leg-option :both) (finger #f(-80 80 -80 80)))
  (let* ( (zmp) (fz) (lwrench) (rwrench)
          (fname-with-path "/tmp/set-current-pose")
          ;; (waist-coord)
          (posfile      (format nil "~a.pos"          fname-with-path)) pf
          (waistfile    (format nil "~a.waist"        fname-with-path)) waf
          (zmpfile      (format nil "~a.zmp"          fname-with-path)) zf
          (wrenchfile   (format nil "~a.wrenches"     fname-with-path)) wf
          (optionalfile (format nil "~a.optionaldata" fname-with-path)) of )

    (warn "set-current-pose :zmp-offset ~a~%" zmp-offset)
    (warn "now moving to current pose by load-pattern in ~a sec~%" tm)

    (send *robot* :fix-leg-to-coords (make-coords) leg-option)
    (send *robot* :move-centroid-on-foot leg-option '(:lleg :rleg) :target-centroid-pos zmp-offset)
    ;; (setq waist-coord (reduce #'(lambda (coord0 coord1) (make-coords :pos (scale 0.5 (v+ (send coord0 :worldpos) (send coord1 :worldpos)))))
    ;;                          (send-all (send-all (send *robot* :legs :crotch-r) :child-link) :worldcoords)))
    (setq waist-coord (send (car (send *robot* :links)) :worldcoords))

    (setq zmp (send *robot* :centroid))
    (setf (elt zmp 2) 0)

    ;; waistファイル作成
    (with-open-file
     (waf waistfile :direction :output)
     (format waf "0 ")
     (mapc #'(lambda (val) (format waf "~s " (/ val 1000))) (coerce (send waist-coord :worldpos) cons))
     (mapc #'(lambda (val) (format waf "~s " val)) (car (send waist-coord :rpy-angle)))
     )
    
    ;; zmpファイル作成
    (with-open-file
     (zf zmpfile :direction :output)
     (format zf "0 ")
     (mapc #'(lambda (val)
               (format zf "~s " (/ val 1000.0)))
           (coerce (send waist-coord :inverse-transform-vector zmp) cons)
           )
     )

    ;; posfile作成  
    (with-open-file
     (pf posfile :direction :output)
     (let (av)
       (setq av (send *robot* :angle-vector))
       (if (and (equal (send *robot* :name) "JAXON_RED") (is-choreonoid)) (setq av (concatenate float-vector av finger)))
       (format pf "0 ")
       (mapc #'(lambda (angle) (format pf "~s " (deg2rad angle))) (coerce av cons))
       (format pf "~%")
       ))

    ;; wrenchfile作成
    (setq lcop-offset zmp rcop-offset zmp)
    (setq fz (* 9.8 1e-3 0.5 (send *robot* :weight)));; [N]
    (setq lf (float-vector 0 0 fz)
          ln (float-vector (* fz (elt lcop-offset 1) 1e-3) (* -1 fz (elt lcop-offset 0) 1e-3) 0)
          rf (float-vector 0 0 fz)
          rn (float-vector (* fz (elt rcop-offset 1) 1e-3) (* -1 fz (elt rcop-offset 0) 1e-3) 0))
    (setq lf (m* (send (send *robot* :lleg :end-coords) :worldrot) (transpose (matrix lf)))
          ln (m* (send (send *robot* :lleg :end-coords) :worldrot) (transpose (matrix ln)))
          rf (m* (send (send *robot* :rleg :end-coords) :worldrot) (transpose (matrix rf)))
          rn (m* (send (send *robot* :rleg :end-coords) :worldrot) (transpose (matrix rn))))
    (setq lwrench (concatenate float-vector (matrix-row (transpose lf) 0) (matrix-row (transpose ln) 0))
          rwrench (concatenate float-vector (matrix-row (transpose rf) 0) (matrix-row (transpose rn) 0)))
    (with-open-file
     (wf wrenchfile :direction :output)
     (format wf "0");; time
     (dotimes (i 6) (format wf " ~s" (elt rwrench i)))
     (dotimes (i 6) (format wf " ~s" (elt lwrench i)))
     (format wf " 0 0 0 0 0 0  0 0 0 0 0 0~%")
     )

    ;; optionalfile作成
    (with-open-file
     (of optionalfile :direction :output)
     ;; (format of "0.0 1 1 0 0 5 5 5 5~%")
     (format of "0.0")
     (mapc #'(lambda (contact-state) (format of " ~s" contact-state)) contact-state-list)
     (format of " 5 5 5 5~%")
     )

    (exec-copy "/tmp/" "set-current-pose")

    (send *ri* :load-pattern "/tmp/set-current-pose" tm)

    (if wait (unix:sleep (round tm)))
    ))

(defun reset-pose (&rest args &key (zmp-offset nil) &allow-other-keys)
  (send *robot* :reset-pose)
  ;; (send *robot* :move-centroid-on-foot :both '(:lleg :rleg))
  (send *robot* :fix-leg-to-coords (make-coords))
  (unless zmp-offset (setq zmp-offset (calc-default-zmp-offset)))
  (apply #'set-current-pose (append args (list :zmp-offset zmp-offset))))

(defun set-gain ( &optional (gain-ratio 1.0) )
  (setq vec (fill (instantiate float-vector (send *hrp2* :dof)) 1.0))
  (setf (elt vec (send *hrp2* :lleg :crotch-r :servo :no)) gain-ratio)
  (send *ci* :gain-vector vec 1000 t nil)
  )
(defun set-pgain ( &optional (gain-ratio 1.0) )
  (setq vec (fill (instantiate float-vector (send *hrp2* :dof)) 1.0))
  (setf (elt vec (send *hrp2* :lleg :crotch-r :servo :no)) gain-ratio)
  (send *ci* :gain-vector vec 1000 t nil)
  (setq vec (fill (instantiate float-vector (send *hrp2* :dof)) 1.0))
  (send *ci* :dgain-vector vec 1000)
  )
(defun set-dgain ( &optional (gain-ratio 1.0) )
  (setq vec (fill (instantiate float-vector (send *hrp2* :dof)) 1.0))
  (setf (elt vec (send *hrp2* :lleg :crotch-r :servo :no)) gain-ratio)
  (send *ci* :gain-vector vec 1000 t nil)
  (setq vec (fill (instantiate float-vector (send *hrp2* :dof)) 1.0))
  (send *ci* :pgain-vector vec 1000 t)
  )


(defun shuffle-walk-old ( &optional (term 1) )
  (send *ci* :stop-st)
  (send *ci* :start-grasp)
  (send *ci* :stop-zmp-override)
  (send *ci* :set-abc-param :check-shuffling nil)
  (init-pose :fname "2dof-slide_short_1smu_1dmu_2D" :real t :tm 5000)
  ;; (init-pose :fname "2dof-slide_left-high-waist_1smu_1dmu_1e-7w_2D" :real t)
  (unix:sleep 2)
  (dotimes (i term)
    (send *ci* :load-pattern "/tmp/2dof-slide_short_1smu_1dmu_2D_" 2000)
    ;; (send *ci* :load-pattern "/tmp/2dof-slide_left-high-waist_1smu_1dmu_1e-7w_2D_" 2000)
    (unix:usleep (round (* 1e6 (+ 3 1.6))))
    (send *ci* :load-pattern "/tmp/2dof-slide_right_1smu_3dmu_2D_" 2000)
    ;; (send *ci* :load-pattern "/tmp/2dof-slide_right-high-waist_1smu_1dmu_1e-7w_2D_" 2000)
    (unix:usleep (round (* 1e6 (+ 3 1.6))))
    )
  )

(defun shuffle-side-walk-old ( &optional (term 1) )
  (send *ri* :start-st)
  (init-pose :fname "2dof-slide-left_short_open_58-5mass_0-5smu_0-5dmu_1w_2D" :real t :tm 5)
  ;; (init-pose :fname "2dof-slide_left-high-waist_1smu_1dmu_1e-7w_2D" :real t)
  (unix:sleep 5)
  (dotimes (i term)
    (send *ri* :load-pattern "/tmp/2dof-slide-left_short_open_58-5mass_0-5smu_0-5dmu_1w_2D_" 1)
    (unix:usleep (round (* 1e6 (+ 1 1.6))))
    (send *ri* :load-pattern "/tmp/2dof-slide-left_short_close_58-5mass_1smu_1-5dmu_1w_2D_" 1.5)
    (unix:usleep (round (* 1e6 (+ 1.5 1.6))))
    )
  )

(defun shuffle-side-walk ( &optional (term 1) )
  (setq *__fname__* (format t "shuffle-side-walk-~s" term))
  ;; (copy-files :fname "2dof-slide-left_short_comb_1smu_1dmu_1w_2D" :real t)
  (prepare-experiment)
  (init-pose :fname "2dof-slide-left_short_comb_1smu_1dmu_1w_2D" :real t :tm 3)
  ;; (init-pose :fname "2dof-slide_left-high-waist_1smu_1dmu_1e-7w_2D" :real t)
  (unix:sleep 3)
  (dotimes (i term)
    (send *ri* :load-pattern "/tmp/2dof-slide-left_short_comb_1smu_1dmu_1w_2D_" 1)
    (unix:usleep (round (* 1e6 (+ 1 2.5))))
    )
  )

(defun shuffle-forward-walk ( &optional (term 1) &key (tm 1) )
  (setq *__fname__* (format t "shuffle-forward-walk-~s" term))
  (prepare-experiment)
  (init-pose :fname "3dof-slide-forward_short_continue_1smu_1-1dmu_1w_2D" :real t :tm 3)
  (unix:sleep 3)
  (dotimes (i term)
    (send *ri* :load-pattern "/tmp/3dof-slide-forward_short_continue_1smu_1-1dmu_1w_2D_" 1)
    (unix:usleep (round (* 1e6 (+ tm 2.5))))
    )
  )

(defun slip-through-demo (&key (turn 1))
  (let ( (fname "_slip-through-right-200step-len-8step-low-waist-tilt") (sample-name "ShuffleSample") (local-path))
    (setq local-path (concatenate string (ros::resolve-ros-path "package://jsk_choreonoid") "sample/" sample-name "/"))

    (setq *__fname__* (reg-replace* "^_" "" fname));; _fname->fname
    (setq motion-time (get-motion-time local-path *__fname__*))

    (prepare-experiment)
    (setq *__fname__* (reg-replace* "^_" "" fname))
    (init-pose :fname *__fname__* :sample-name sample-name :real t :wait t :tm 5)

    (warning-message 3 "Push y to start or n to resign~%")
    (when (y-or-n-p)
      (send *ri* :start-log)
      (dotimes (i turn)
        ;; (play-hrpsysfile :fname fname :sample-name sample-name :st-algorithm :eefmqpcop2 :save-log nil :tm 0.5 :prepare nil :wait-key nil)
        ;; (play-hrpsysfile :fname fname :sample-name sample-name :st-algorithm :eefmqpcop2 :save-log nil :tm 2 :prepare nil :wait-key nil)
        (if (= i 0) (setq tm 0.5) (setq tm 5))
        (play-hrpsysfile :fname fname :sample-name sample-name :st-algorithm :eefmqpcop2 :save-log nil :tm tm :prepare nil :wait-key nil)
        (unix:sleep (round motion-time)))
      (unix:sleep 2)
      ;; (send *ri* :stop-st)
      (reset-pose :tm 15)
      (save-log :fname (format nil "slip-through-demo_~aturn_~a" turn fname))
      )
    ))

(defun init-play-hrpsysfile()
  (when (and (not (is-simulation)) (unix:getenv "ROBOT"))
    (when (not (boundp '*servo-state-msg*))
      (warning-message 2 "subscribe /shm_servo_state~%")
      (ros::subscribe "/shm_servo_state"  trans_ros_bridge::ServoState #'(lambda (msg) (setq *servo-state-msg* msg)))
      )
    (warn "spin-once while not boundp servo-state-msg~%")
    (while (not (boundp '*servo-state-msg*)) (ros::spin-once))
    )
  )
(init-play-hrpsysfile)
