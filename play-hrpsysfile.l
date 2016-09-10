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

(defmacro create-robot-interface (&optional (robot nil))
  (unless robot (setq robot (read-from-string (unix:getenv "ROBOT"))))
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
    (if (equal (unix:getenv "ROBOT") "");; new hrpsys
        (warning-message 1 "!!!!!!!!!!No ROBOT TYPE!!!!!!!!!!~%")
      (unless (boundp '*robot*) (create-robot-interface)))
  (when (not (boundp '*hrp2*)) (setup-model-for-hrp2) (setq *robot* *hrp2*)  );; old hrpsys
  )

;; (load "euslib/jsk/gnuplotlib.l")
(load "log-tools.l")

(defvar *usage-funcs* nil)
(unless (assoc :play-hrpsysfile *usage-funcs*)
  (push (cons :play-hrpsysfile
              #'(lambda ()
                  (warn "~%play-hrpsysfile.l~%")
                  ;; (warning-message 3 "(setup-real-for-hrp2)~%")
                  (warning-message 3 "(create-robot-interface jaxon)~%")
                  (warning-message 3 "(scale-hrpsysfile :fname <FILE NAME>)~%")
                  (warning-message 3 "(copy-files :fname <FILE NAME> :real t)~%")
                  (warning-message 3 "(init-pose :fname <FILE NAME> :real t)~%")
                  (warning-message 3 "(play-hrpsysfile :fname <FILE NAME> :real t)~%")
                  (warning-message 3 "(save-log :fname <FILE NAME> :add-fname <FILE NAME OPTION>)~%")
                  (warn "~%")
                  (warning-message 3 "(set-pgain <l-crotch dgain>)~%")
                  (warning-message 3 "(set-dgain <l-crotch dgain>)~%")
                  (warn "~%"))) *usage-funcs*))

(usage)

;; (if (not (boundp '*ci*)) (setup-real-for-hrp2))

;; (when (not (boundp '*gnuplot*))
;;   (if (boundp '*gplot*) (setq *gnuplot* *gplot*)  (setq *gnuplot* (gnuplot)))
;;   (send *gnuplot* :command "set size 1.5,1")(send *gnuplot* :command "set size 1,1"))

;; 実機かシミュレーションか判定
(defun check-sim ()
  (if (ros::get-param "use_sim_time" nil);; シミュレーションフラグ
      t
    nil)
  )

(defun is-choreonoid ()
  (if (and (check-sim) (ros::get-param "/multisense/range_bridge/frame_id" nil)) t nil))

;; posファイルの関節数を修正する
(defun modify-posfile (path fname)
    (when (or (equal (send *robot* :name) "HRP2JSKNT") (equal (send *robot* :name) "HRP2JSKNTS"))
      (unix:system (concatenate string "cat " path fname ".pos | cut -d' ' -f1-35 | cat > " path fname ".pos_; mv " path fname ".pos_ " path fname ".pos")));; 1-35 34関節
    (when (and (equal (send *robot* :name) "JAXON_RED") (not (is-choreonoid)))
      (unix:system (concatenate string "cat " path fname ".pos | cut -d' ' -f1-34 | cat > " path fname ".pos_; mv " path fname ".pos_ " path fname ".pos")));; 1-34 33関節
    )

;; 実際に実機またはローカルの/tmpにファイルをコピー
(defun exec-copy (path fname)
  (if (check-sim) 
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
       ( t (warning-message 1 "No Setting for The Robot~%") )
       )
      (unix:system (concatenate string "scp " path fname ".* " uname "@" machine-name ":/tmp"))
      )
    )
  )

;; 関節角速度ファイル作成
;; 自分のPCから実行
(defun copy-files (&key (fname "SampleMotion") (ofname nil) (real nil) (plugin t) (sample-name "ShuffleSample") (remove-last-row t))
  (let ( motionname
         zmpline velfile optfile posfile zmpfile path
         ;; (extention-list '("pos" "hip" "waist" "zmp" "vel" "optionaldata" "wrenches"))
         (extention-list '("pos" "waist" "zmp" "hip" "optionalData" "wrenches"))
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
    (warn "~%now converting waist from waist and hip~%")
    (unix:system (concatenate string "cat " path "_" fname ".waist | cut -d' ' -f1-4 | cat > " waistfile "_"))
    (unix:system (concatenate string "cat " path "_" fname ".hip   | cut -d' ' -f2-4 | cat > " hipfile   "_"))
    (unix:system (concatenate string "paste " waistfile "_ " hipfile "_ | cat > " path "_" fname ".waist"))
    (unix:system (concatenate string "rm -f " waistfile "_ " hipfile "_ " path "_" fname ".hip"))

    ;; posファイル 関節角度個数合わせ
    (modify-posfile path (concatenate string "_" fname))

    ;; ファイルコピー
    (warn "~%now copying files~%")
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

(defun prepare-experiment ()
  (send *ri* :stop-st)
  (warn "stop-st~%")
  (unix:sleep 2)
  (send *ri* :stop-auto-balancer)
  (warn "stop-auto-balancer~%")
  (unix:sleep 2)
  (send *robot* :angle-vector (send *ri* :state :potentio-vector))
  (set-current-pose :tm 2)
  (warn "now moving to set-current-pose by load-pattern~%")
  (unix:sleep 2)
  (send *ri* :set-st-param :st-algorithm :eefmqpcop2)
  (unless (= (cdr (assoc 'ros::_emergency_check_mode (send (send *ri* :get-st-param) :slots))) 0)
    (send *ri* :set-st-param :emergency-check-mode 0)
    (warn "disable emergency-check~%"))
  (send *ri* :set-log-maxlength (* 500 25))
  (when (or (not (check-sim)) (is-choreonoid))
    (warn "start-st~%")
    (send *ri* :start-st))
  )

(defun play-hrpsysfile-impl (&key fname (real nil) (tm 10) (save-log t) (plugin t) (sample-name "ShuffleSample"))
  (let (path local-path)
   (if plugin
       (setq local-path (concatenate string (ros::resolve-ros-path "package://jsk_choreonoid") "sample/" sample-name "/"))
     (setq local-path (concatenate string "/home/" (unix::getenv "USER") "/" (unix::getenv "CNOID_WORKSPACE") "/" (send *robot* :name) "/" motionname "/")))

  (prepare-experiment)
  (warning-message 3 "Push Enter to start~%")
  (do-until-key)
  (setq *__fname__* (reg-replace* "^_" "" fname));; _fname->fname
  (when real
    (setq path "/tmp/")
    (send *ri* :load-pattern (concatenate string path fname) tm) )
  (setq sleep-time (round (* tm 0.8)))
  (warn "sleep ~a[sec]~%" sleep-time)
  (unix:sleep sleep-time)
  (when save-log
    (send *ri* :start-log)
    (setq sleep-time (round (+ (/ tm 2) (* 0.002 (read (piped-fork (concatenate string "cat " local-path fname ".pos" " | wc -l")))))))
    (warn "sleep ~a[sec]~%" sleep-time)
    (unix:sleep sleep-time)
    (unix:sleep sleep-time)
    (save-log))
  ))

(defun play-hrpsysfile (&rest args &key fname (real nil) (save-log t) (tm 10) &allow-other-keys)
  (when (boundp '*ci*) (play-hrpsysfile-old  :fname fname :real real :save-log save-log))
  (when (boundp '*ri*) (play-hrpsysfile-impl :fname fname :real real :save-log save-log :tm tm))
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

;; モーションの初期姿勢へ遷移 (hrpsysシーケンスファイルの1行目のみを実行)
(defun init-pose (&key (fname "SampleMotion") (tm 10) (real nil) (plugin t) (sample-name "ShuffleSample"))
  (let* ( (motionname (car (reg-split "_"  fname))) (path) )

    (if plugin
      (setq path (concatenate string (ros::resolve-ros-path "package://jsk_choreonoid") "sample/" sample-name "/"))
      (setq path (concatenate string "/home/" (unix::getenv "USER") "/" (unix::getenv "CNOID_WORKSPACE") "/" (send *robot* :name) "/" motionname "/")))

    ;;fname->fname_init
    (mapc #'(lambda (extemtion)
              (unix:system (concatenate string "head -n 1 " path fname "." extemtion "| cat >  " path fname "_init." extemtion)) )
          ;; (list "hip" "waist" "zmp" "vel" "pos" "optionaldata" "wrenches")
          (list "waist" "hip" "zmp" "pos" "optionalData" "wrenches") )

    ;; fname_init->_fname_init
    (copy-files :fname (concatenate string fname "_init") :real real :remove-last-row nil)

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

    ))

(defun init-landing-pose (&key (fname "SampleMotion") (tm 2000) (real nil) (plugin t) (sample-name "ShuffleSample"))
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

(defun set-current-pose (&key (tm 10) (zmp-offset #f(0 0 0)) (lcop-offset #f(0 0)) (rcop-offset #f(0 0)))
  (let* ( (zmp) (fz) (lwrench) (rwrench)
          (fname-with-path "/tmp/set-current-pose")
          ;; (waist-coord)
          (posfile      (format nil "~a.pos"          fname-with-path)) pf
          (waistfile    (format nil "~a.waist"        fname-with-path)) waf
          (zmpfile      (format nil "~a.zmp"          fname-with-path)) zf
          (wrenchfile   (format nil "~a.wrenches"     fname-with-path)) wf
          (optionalfile (format nil "~a.optionaldata" fname-with-path)) of )

    (send *robot* :fix-leg-to-coords (make-coords))
    (setq waist-coord (reduce #'(lambda (coord0 coord1) (make-coords :pos (scale 0.5 (v+ (send coord0 :worldpos) (send coord1 :worldpos)))))
                             (send-all (send-all (send *robot* :legs :crotch-r) :child-link) :worldcoords)))

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
               (format zf "~s " (/ val 1000)))
           (coerce (send waist-coord :inverse-transform-vector (v+ zmp zmp-offset)) cons))
     )

    ;; posfile作成  
    (with-open-file
     (pf posfile :direction :output)
     (let (av)
       (setq av (send *robot* :angle-vector))
       (if (and (equal (send *robot* :name) "JAXON_RED") (is-choreonoid)) (setq av (concatenate float-vector av #f(0 0 0 0))))
       (format pf "0 ")
       (mapc #'(lambda (angle) (format pf "~s " (deg2rad angle))) (coerce av cons))
       (format pf "~%")
       ))

    ;; wrenchfile作成
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
     (format wf "0 0 0 0 0 0  0 0 0 0 0 0~%")
     )

    ;; optionalfile作成
    (with-open-file
     (of optionalfile :direction :output)
     (format of "0.0 1 1 0 0 5 5 5 5~%")
     )

    (exec-copy "/tmp/" "set-current-pose")

    (send *ri* :load-pattern "/tmp/set-current-pose" tm)

    ))

(defun reset-pose (&rest args &key &allow-other-keys)
  (send *robot* :reset-pose)
  ;; (send *robot* :move-centroid-on-foot :both '(:lleg :rleg))
  (send *robot* :legs :move-end-pos #f(-70 0 0))
  (apply #'set-current-pose args))

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
