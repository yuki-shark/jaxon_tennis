;;please change jaxon_ros_bridge.launch
;;//hrpsys_ros_bridge_tutorials/launch/jaxon_ros_bridge.launch

;;(load "package://hrpsys_ros_bridge_tutorials/euslisp/jaxon-interface.l")
;;(load "../model/racket-model.l")
(load "package://hrpsys_ros_bridge_tutorials/euslisp/jaxon_red-interface.l")
(load "../model/racket-model2.l")

(load "fullbody-ik-jaxon.l")
;;;
(setq *real* nil)
(setq *range* (deg2rad 45))

(defun callback-point (msg)
  (setq *point-msg* msg)
  )

(defun init-volley-pose ()
  (if *real*
      (jaxon_red-init)
    (jaxon_red))

  (setq *robot* *jaxon_red*)
  (send *robot* :reset-pose)
  (send *robot* :fix-leg-to-coords (make-coords))

  (setq rootcds (send (car (send *robot* :links)) :copy-worldcoords))
  ;; root->l, root->r
  (setq rcds (send *robot* :rleg :end-coords :copy-worldcoords))
  (setq lcds (send *robot* :lleg :end-coords :copy-worldcoords))

  (send rootcds :translate #f(0 0 -50) :world)
  (send rcds :translate #f(-100 -150 0) :world)
  (send rcds :rotate (deg2rad -30) :z)
  (send lcds :translate #f(200 100 0) :world)

  (send *robot* :move-coords rootcds (send (car (send *robot* :links)) :worldcoords))
  (send *robot* :lleg :inverse-kinematics lcds)
  (send *robot* :rleg :inverse-kinematics rcds)

  (send *robot* :rotate (deg2rad -45) :z)
  (send *robot* :torso-waist-y :joint-angle 15)
  (send *robot* :head-neck-y :joint-angle 30)
  (send *robot* :move-centroid-on-foot :both (list :rleg :lleg))

  (setq *racket* (instance racket :init :move-origin nil))
  (send *racket* :move-coords (send *robot* :rarm :end-coords :copy-worldcoords) (send *racket* :get :gripper))

  (objects (list *robot* *racket*))

  (if *real*
      (send *ri* :angle-vector (send *robot* :angle-vector) 4000))
  )


(defun tracking_ball ()

  (ros::roseus "tracking_ball")

  (setq *tfl* (instance ros::transform-listener :init)) ;; subscribe

  (ros::subscribe "/centroid_calc_from_stereo_image"
                  geometry_msgs::PointStamped
                  #'callback-point 1)

  ;; (if *real*
  ;;     (jaxon_red-init)
  ;;   (jaxon_red))
  ;;
  ;; (setq *robot* *jaxon_red*)
  ;; (send *robot* :reset-pose)
  ;; (send *robot* :fix-leg-to-coords (make-coords))
  ;; (setq *racket* (instance racket :init :move-origin nil))
  ;; (send *racket* :move-coords (send *robot* :rarm :end-coords :copy-worldcoords) (send *racket* :get :gripper))
  (let* ((ed (send *robot* :rarm :end-coords))
         (pr (send ed :parent))
         (tg (make-cascoords :coords (send (send *racket* :get :target) :copy-worldcoords)))
         )
    (send pr :assoc tg)
    (send *robot* :put :racket-target tg)
    tg)
  (setq *prev-angle* (send *robot* :angle-vector))
  (when *real*
    (send *ri* :angle-vector (send *robot* :angle-vector) 6000)
    (send *ri* :wait-interpolation))

  (objects (list *robot* *racket*))

  ;(send *racket* :move-coords *target* (send *racket* :get :target))
  (setq *point-msg* nil)
  (ros::ros-info "start tracking")
  (while (ros::ok)

    (ros::spin-once)

    (when *point-msg*

      (when
          (send *tfl* :wait-for-transform
                "rleg_end_coords"
                (send *point-msg* :header :frame_id)
                (send *point-msg* :header :stamp)
                1.0)
        (setq body->camera-frame
              (send *tfl* :lookup-transform "rleg_end_coords"
                    (send *point-msg* :header :frame_id)
                    (send *point-msg* :header :stamp)))

        (print "point-msg")
        (format t "x: ~A   y: ~A   z: ~A~%" (send *point-msg* :point :x) (send *point-msg* :point :y) (send *point-msg* :point :z))

        (pprint body->camera-frame)
        (setq body->hitpoint
              (send body->camera-frame
                    :transform-vector
                    (ros::tf-point->pos (send *point-msg* :point))))
        (pprint body->hitpoint)
        )

      (send *robot* :fix-leg-to-coords (make-coords))
      (setq hitpoint-world
            (send (send *robot* :rleg :end-coords) :transform-vector body->hitpoint))

      (setq *target*
            (make-coords :pos (float-vector 1000 (elt hitpoint-world 1) (elt hitpoint-world 2))
                         :rpy (list 0 (deg2rad 25) 0)))

      (print "target")
      (format t "x: ~A   y: ~A   z: ~A~%" (elt (send *target* :pos) 0) (elt (send *target* :pos) 1) (elt (send *target* :pos) 2))

      (fullbody-ik-jaxon
       :rarm-target *target*
       ;:larm-target (send *shovel* :get :larm)
       :move-target-arms
       (list (send *robot* :get :racket-target)
             (send *robot* :larm :end-coords))
       :rotation-axis (list t t :x t)
       ;:rotation-axis (list t t t t)
       :translation-axis (list t t t t)
       :use-waist-r nil
       :use-waist-y t
       :root-link-virtual-joint-weight ;; root movement weight
       (float-vector 0.1 0.1 0.1 0.03 0.0 0.03) ;; x,y,z,roll,pitch,yaw
       :debug-view nil
       )
      ;; re-calc
      (let ((off (send *target* :transformation (send *robot* :get :racket-target)))
            roll re-calc)
        (pprint off)
        (setq roll (caddr (car (send off :rpy-angle))))
        (pprint roll)
        (if (> roll *range*)
            (setq roll *range* re-calc t)
          (if (< roll (* -1 *range*))
              (setq roll (* -1 *range*) re-calc t)
            ))
        (when re-calc
          (setq re-calc
               (send (send *target* :copy-worldcoords) :rotate roll :x))
          (pprint (list 're re-calc))
          (fullbody-ik-jaxon
           :rarm-target re-calc
           :move-target-arms
           (list (send *robot* :get :racket-target)
                 (send *robot* :larm :end-coords))
           :rotation-axis (list t t t t)
                                        ;:rotation-axis (list t t t t)
           :translation-axis (list t t t t)
           :use-waist-r nil
           :use-waist-y t
           :root-link-virtual-joint-weight ;; root movement weight
           (float-vector 0.1 0.1 0.1 0.03 0.0 0.03) ;; x,y,z,roll,pitch,yaw
           :debug-view nil
           ))
        )

      (send *robot* :head :look-at hitpoint-world)

      (print ";; ik solved~%")

      (send *racket* :move-coords *target* (send *racket* :get :target))
      (send *racket* :move-coords (send *robot* :rarm :end-coords :copy-worldcoords) (send *racket* :get :gripper))
      (send *irtviewer* :draw-objects)

      (when *real*
        (let ((av (send *robot*  :angle-vector))
              tm)
          (setq *prev-angle* (send *ri* :state :reference-vector))
          (setq tm (* 40 (norm (v- av *prev-angle*))))
          (when (< tm 300) (setq tm 300))
          (pprint (list :tm tm))
          (send *ri* :angle-vector (send *robot* :angle-vector) tm)
          ;;(send *ri* :angle-vector (send *robot* :angle-vector) 4000)
          ))

      (setq *point-msg* nil)
      )
    ;;(unix::usleep (* 1000 10));; 10ms
    (unix::usleep (* 1000 100));; 100ms
    (x::window-main-one)
    )
)

(warn "(init-volley-pose)~%")
(warn "(tracking_ball)~%")

;;(send *ri* :angle-vector (send *robot* :reset-pose) 6000)
;;(unix::usleep (* 2000 1000))
;;(send *ri* :angle-vector (send *robot* :reset-manip-pose) 1000)
;;(send *ri* :set-max-loglength (* 500 600))
;;(send *ri* :save-log "/home/leus/hrpsyslog")
