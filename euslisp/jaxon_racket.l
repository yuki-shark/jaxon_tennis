;;(load "package://hrpsys_ros_bridge_tutorials/euslisp/jaxon-interface.l")
(load "package://hrpsys_ros_bridge_tutorials/euslisp/jaxon_red-interface.l")
;;(load "../model/racket-model.l")
(load "../model/racket-model2.l")

(defun jaxon_racket ()
  ;;(jaxon-init)
  ;; (jaxon)
  ;;(jaxon_red)
  (jaxon_red-init)
  ;;(setq *robot* *jaxon*)
  (setq *robot* *jaxon_red*)
  (send *robot* :reset-pose)
  ;; (setq racket (instance racket :init :move-origin nil))
  ;; (send racket :move-coords (send *robot* :rarm :end-coords :copy-worldcoords) (send racket :get :gripper))

  (setq rootcds (send (car (send *robot* :links)) :copy-worldcoords))
  ;; root->l, root->r
  (setq rcds (send *robot* :rleg :end-coords :copy-worldcoords))
  (setq lcds (send *robot* :lleg :end-coords :copy-worldcoords))

  (send rootcds :translate #f(0 0 -100) :world)
  (send rcds :translate #f(-200 -200 0) :world)
  (send lcds :translate #f(100 200 0) :world)

  ;;(send *robot* :fix-leg-to-coords (make-coords))

  (send *robot* :move-coords rootcds (send (car (send *robot* :links)) :worldcoords))
  (send *robot* :lleg :inverse-kinematics lcds)
  (send *robot* :rleg :inverse-kinematics rcds)

  (send *robot* :fix-leg-to-coords (make-coords))

  (send *robot* :rotate (deg2rad -30) :z)
  (send *robot* :head-neck-y :joint-angle 30)

  (setq racket (instance racket :init :move-origin nil))
  (send racket :move-coords (send *robot* :rarm :end-coords :copy-worldcoords) (send racket :get :gripper))

  (objects (list *robot* racket))

)

;; (setq rootcds (send (car (send *robot* :links)) :copy-worldcoords))
;; ;; root->l, root->r
;; (setq rcds (send *robot* :rleg :end-coords :copy-worldcoords))
;; (setq lcds (send *robot* :lleg :end-coords :copy-worldcoords))

;; (send rootcds :translate #f(0 0 -100) :world)
;; (send rcds :translate #f(-200 -200 0) :world)
;; (send lcds :translate #f(100 200 0) :world)

;; ;;(send *robot* :fix-leg-to-coords (make-coords))

;; (send *robot* :move-coords rootcds (send (car (send *robot* :links)) :worldcoords))
;; (send *robot* :lleg :inverse-kinematics lcds)
;; (send *robot* :rleg :inverse-kinematics rcds)

;; (send *robot* :fix-leg-to-coords (make-coords))

;; (send *robot* :rotate (deg2rad -30) :z)
;; (send *robot* :head-neck-y :joint-angle 30)
(jaxon_racket)
