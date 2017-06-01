(load "package://hrpsys_ros_bridge_tutorials/euslisp/jaxon-interface.l")
(load "../model/racket-model.l")

;; (setq *target*
;;       (make-coords :pos (float-vector 418.958 -218.164 1292.99)
;;                    :rpy (list -0.024 0.504 -0.371)))
(setq *target*
      (make-coords :pos (float-vector 1000 0 1800)
                   :rpy (list 0 (deg2rad 25) 0)))

(defun fullbody-ik-jaxon (&key (stop 100)
                               (debug-view :no-message)
                               (centroid (send (send *robot* :foot-midcoords) :worldpos))
                               (centroid-thre 20.0);;
                               (larm-target (send *robot* :larm :end-coords :copy-worldcoords))
                               (rarm-target (send *robot* :rarm :end-coords :copy-worldcoords))
                               (lleg-target (send *robot* :lleg :end-coords :copy-worldcoords))
                               (rleg-target (send *robot* :rleg :end-coords :copy-worldcoords))
                               (translation-axis (list t t t t)) ;; rleg, lleg, rarm, larm
                               (rotation-axis    (list t t t t)) ;; rleg, lleg, rarm, larm
                               (thre  (list 0.1 0.1 0.1 0.1))    ;; rleg, lleg, rarm, larm
                               (rthre (list 0.0017453 0.0017453 0.0017453 0.0017453)) ;; rleg, lleg, rarm, larm
                               (root-link-virtual-joint-weight ;; root movement weight
                                (float-vector 0.1 0.1 0.1 0.05 0.2 0.2)) ;; x,y,z,roll,pitch,yaw
                               (min (float-vector -500 -500 -500 -20 -20 -10)) ;; root movement limit, x,y,z,roll,pitch,yaw
                               (max (float-vector  500  500  500  20  20  10)) ;; root movement limit, x,y,z,roll,pitch,yaw
                               (use-collision-avoidance nil)
                               (avoid-collision-null-gain 40)
                               (avoid-collision-joint-gain 0.1)
                               (collision-avoidance-pair
                                (list
                                 (list (send *robot* :larm_link2_lk)
                                       (send *robot* :chest_link2_lk))
                                 (list (send *robot* :larm_link2_lk)
                                       (send *robot* :chest_link1_lk))
                                 (list (send *robot* :rarm_link2_lk)
                                       (send *robot* :chest_link2_lk))
                                 (list (send *robot* :rarm_link2_lk)
                                       (send *robot* :chest_link1_lk))
                                 ;;(list (send *robot* :larm_link4_lk)
                                 ;;*shovel*)
                                 ))
                               (use-waist-r nil)
                               (use-waist-y nil)
                               (revert-if-fail t)
                               (move-target-legs
                                (list (send *robot* :rleg :end-coords)
                                      (send *robot* :lleg :end-coords)))
                               (move-target-arms
                                (list (send *robot* :rarm :end-coords)
                                      (send *robot* :larm :end-coords)))
                               )
  (let* ((move-target (append move-target-legs move-target-arms))
         (target-coords
          (list
           rleg-target lleg-target
           rarm-target larm-target
           ))
         (additional-weight-list ;; jaxon specific
          (list
           (if (not use-waist-r)
               (list (send *robot* :torso :waist-r :child-link) 0)
             (if (numberp use-waist-r)
                 (list (send *robot* :torso :waist-r :child-link) use-waist-r)))
           (if (not use-waist-y)
               (list (send *robot* :torso :waist-y :child-link) 0)
             (if (numberp use-waist-y)
                 (list (send *robot* :torso :waist-y :child-link) use-waist-y)))
           ))
         (link-list)
         )
    (setq additional-weight-list (remove nil additional-weight-list));;

    ;;link-list
    (setq link-list
          (mapcar #'(lambda (limb)
                      (send *robot* :link-list (send limb :parent))) move-target))

    (let (args)
      (when use-collision-avoidance
        (push :avoid-collision-null-gain args)
        (push avoid-collision-null-gain args)
        (push :avoid-collision-joint-gain args)
        (push avoid-collision-joint-gain args)
        (push :collision-avoidance-link-pair args)
        (push collision-avoidance-pair args)
        (setq args (nreverse args))
        )
      (send* *robot* :fullbody-inverse-kinematics target-coords
             :move-target move-target
             :link-list link-list
             :translation-axis translation-axis
             :rotation-axis    rotation-axis
             :target-centroid-pos centroid
             :centroid-thre centroid-thre
             :thre thre
             :rthre rthre
             :min min
             :max max
             :additional-weight-list additional-weight-list
             :root-link-virtual-joint-weight root-link-virtual-joint-weight
             :debug-view debug-view ;;:no-message
             :stop stop ;;
             :revert-if-fail revert-if-fail
             :dump-command nil
             :warnp nil
             args
             )
      )
    ))

(defun jaxon_racket2 ()
  ;;(jaxon-init)
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

  (send *racket* :move-coords *target* (send *racket* :get :target))

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

  (send *racket* :move-coords (send *robot* :rarm :end-coords :copy-worldcoords) (send *racket* :get :gripper))
  (send *irtviewer* :draw-objects)
)

(jaxon_racket2)
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

