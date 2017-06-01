(load "package://hrpsys_ros_bridge_tutorials/euslisp/jaxon-interface.l")
(load "../model/racket-model.l")
(load "fullbody-ik-jaxon.l")

;; (setq *target*
;;       (make-coords :pos (float-vector 418.958 -218.164 1292.99)
;;                    :rpy (list -0.024 0.504 -0.371)))
;; (setq *target*
;;       (make-coords :pos (float-vector 1000 0 1800)
;;                    :rpy (list 0 (deg2rad 25) 0)))

(defun callback-target (msg)
  (setq *target-msg* msg)
  )

(defun jaxon_racket3 ()

  (ros::roseus "jaxon_racket3")
  (ros::subscribe "/target_point"
                  geometry_msgs::PointStamped
                  #'callback-target 1)

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
  (setq *target-msg* nil)

  (while (ros::ok)

    (ros::spin-once)

    (when *target-msg*
      ;; (setq *target*
      ;; (make-coords :pos (send *target-msg* :point)
      ;;              :rpy (list 0 (deg2rad 25) 0)))
      (setq x (send *target-msg* :point :x))
      (setq y (send *target-msg* :point :y))
      (setq z (send *target-msg* :point :z))

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
      (setq *target-msg* nil)
      )
    (unix::usleep (* 1000 10));; 10ms
    (x::window-main-one)
    )
)

(jaxon_racket3)

#|
(setq tm (instance mtimer :init))
(progn (send tm :start) (fullbody-ik-jaxon
   :rarm-target (send *racket* :get :gripper)
   ;:larm-target (send *shovel* :get :larm)
   :rotation-axis (list t t t t) ;; nil, :z
   :translation-axis (list t t t t)
   :use-waist-r nil
   :use-waist-y t
   :root-link-virtual-joint-weight ;; root movement weight
   (float-vector 0.1 0.1 0.1 0.0 0.2 0.0) ;; x,y,z,roll,pitch,yaw
   :debug-view nil
   ) (send tm :stop))
|#

