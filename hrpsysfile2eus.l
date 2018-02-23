(if (unix:getenv "OLD_HRPSYS")
    (progn ;; 旧システム用
      (warning-message 1 "!!!!!!!!!!using old hrpsys!!!!!!!!!!!~%")
      (load "hrpsys-old-log-parser.l")
      )
  (progn ;; 新システム用
    (load "hrpsys-base-log-parser.l")
    )
  )

(unless (boundp '*irtviewer*) (make-irtviewer) (send *irtviewer* :change-background #f(0.6 0.6 0.6)) (objects (list *robot*)))

(defun animate-hrpsysfile (&key fname (sample-name "ShuffleSample") (step 10) (sleep-time 0) )
  (let (local-path motion-name)
    (if sample-name
        (setq local-path (concatenate string (ros::resolve-ros-path "package://jsk_choreonoid") "sample/" sample-name "/"))
      (setq local-path (concatenate string "/home/" (unix::getenv "USER") "/" (unix::getenv "CNOID_WORKSPACE") "/" (send *robot* :name) "/" motionname "/")))

    (setq posfile (concatenate string local-path fname ".pos")
          waistfile (concatenate string local-path fname ".waist")
          hipfile (concatenate string local-path fname ".hip")
          zmpfile (concatenate string local-path fname ".zmp"))

    (setq idx 0)
    (with-open-file
     (posf posfile :direction :input)
     (with-open-file
      (waistf waistfile :direction :input)
      (with-open-file
       (hipf hipfile :direction :input)
       (with-open-file
        (zmpf zmpfile :direction :input)
        (let ( (poseof (gensym)) (waisteof (gensym)) (hipeof (gensym)) (zmpeof (gensym)) )
          ;; (while (and (not (eq poseof   (setq posl   (read-line posf   nil poseof))))
          ;;             (not (eq waisteof (setq waistl (read-line waistf nil waisteof))))
          ;;             (not (eq hipeof   (setq hipl   (read-line hipf   nil hipeof)))))
          (do-until-key-with-check
           (not (eq zmpeof (setq zmpl (read-line zmpf nil zmpeof))))
           ;; (print idx)
           (read posf)
           (setq av (coerce (mapcar #'(lambda (x) (rad2deg (read posf))) (coerce (send *robot* :angle-vector) cons)) float-vector))
           (read waistf)
           (read hipf)
           (setq root-coord (make-coords :pos (coerce (mapcar #'(lambda (x) (* (read waistf) 1000)) (range 3)) float-vector)
                                         :rpy (coerce (reverse (mapcar #'(lambda (x) (read hipf)) (range 3))) float-vector)))
           (read-line waistf nil waisteof)

           (when (= (mod idx step) 0)
             (send *robot* :angle-vector av)
             (send *robot* :move-coords root-coord (car (send *robot* :links)))

             ;; (send *irtviewer* :draw-objects :flush nil)
             (send *irtviewer* :draw-objects)
             (x::window-main-one)
             (unix:usleep sleep-time)
             )

           (incf idx)
           )
          )))))
    ))
