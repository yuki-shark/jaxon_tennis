;;(load "package://hrpsys_ros_bridge_tutorials/euslisp/jaxon-interface.l")
(load "package://hrpsys_ros_bridge_tutorials/euslisp/jaxon_red-interface.l")
;;(load "../model/racket-model.l")
(load "../model/racket-model2.l")

(setq *real* nil)

(defun jaxon_racket ()

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

  (setq racket (instance racket :init :move-origin nil))
  (send racket :move-coords (send *robot* :rarm :end-coords :copy-worldcoords) (send racket :get :gripper))

  (objects (list *robot* racket))

  (if *real*
      (send *ri* :angle-vector (send *robot* :angle-vector) 4000))

)

(jaxon_racket)
