(load "package://hrpsys_ros_bridge_tutorials/euslisp/jaxon-interface.l")
(load "../model/racket-model.l")
(load "fullbody-ik-jaxon.l")

(defun callback-target (msg)
  (setq *target-msg* msg)
  )

(defun jaxon_racket3 (*range*)

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
       ;:rotation-axis (list t t t t)
       :translation-axis (list t t t t)
       :use-waist-r nil
       :use-waist-y t
       :root-link-virtual-joint-weight ;; root movement weight
       (float-vector 0.1 0.1 0.1 0.0 0.2 0.0) ;; x,y,z,roll,pitch,yaw
       :debug-view nil
       )

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
           (float-vector 0.1 0.1 0.1 0.0 0.2 0.0) ;; x,y,z,roll,pitch,yaw
           :debug-view nil
           ))
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

(jaxon_racket3 (deg2rad 45))
