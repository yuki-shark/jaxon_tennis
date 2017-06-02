(load "package://hrpsys_ros_bridge_tutorials/euslisp/jaxon-interface.l")
(load "../model/racket-model.l")
(load "fullbody-ik-jaxon.l")

(defun callback-point (msg)
  (setq *point-msg* msg)
  )

(defun tracking_ball ()

  (ros::roseus "tracking_ball")

  (setq *tfl* (instance ros::transform-listener :init)) ;; subscribe

  (ros::subscribe "/centroid_calc_from_stereo_image"
                  geometry_msgs::PointStamped
                  #'callback-point 1)

  (jaxon)
  (setq *robot* *jaxon*)
  (send *robot* :reset-pose)
  (send *robot* :fix-leg-to-coords (make-coords))
  (setq *racket* (instance racket :init :move-origin nil))
  (send *racket* :move-coords (send *robot* :rarm :end-coords :copy-worldcoords) (send *racket* :get :gripper))
  (let* ((ed (send *robot* :rarm :end-coords))
         (pr (send ed :parent))
         (tg (make-cascoords :coords (send (send *racket* :get :target) :copy-worldcoords)))
         )
    (send pr :assoc tg)
    (send *robot* :put :racket-target tg)
    tg)

  (objects (list *robot* *racket*))

  ;(send *racket* :move-coords *target* (send *racket* :get :target))
  (setq *point-msg* nil)

  (while (ros::ok)

    (ros::spin-once)

    (when *point-msg*

      (when
          (send *tfl* :wait-for-transform
                "BODY"
                (send *point-msg* :header :frame_id)
                (send *point-msg* :header :stamp)
                1.0)
        (setq body->camera-frame
              (send *tfl* :lookup-transform "BODY"
                    (send *point-msg* :header :frame_id)
                    (send *point-msg* :header :stamp)))
        (pprint body->camera-frame)
        (setq body->hitpoint
              (send body->camera-frame
                    :transform-vector
                    (ros::tf-point->pos (send *point-msg* :point))))
        (pprint body->hitpoint)
        )

      ;; (setq x (send body->hitpoint :point :x))
      ;; (setq y (send body->hitpoint :point :y))
      ;; (setq z (send body->hitpoint :point :z))

      (setq x (send (elt body->hitpoint 0)))
      (setq y (send (elt body->hitpoint 1)))
      (setq z (send (elt body->hitpoint 2)))

      (setq *target*
      (make-coords :pos (float-vector x y z)
                   :rpy (list 0 (deg2rad 25) 0)))

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
       (float-vector 0.1 0.1 0.1 0.0 0.2 0.0) ;; x,y,z,roll,pitch,yaw
       )

      (print ";; ik solved~%")

      (send *racket* :move-coords *target* (send *racket* :get :target))
      (send *racket* :move-coords (send *robot* :rarm :end-coords :copy-worldcoords) (send *racket* :get :gripper))
      (send *irtviewer* :draw-objects)
      (setq *point-msg* nil)
      )
    (unix::usleep (* 1000 10));; 10ms
    (x::window-main-one)
    )
)

(tracking_ball)
