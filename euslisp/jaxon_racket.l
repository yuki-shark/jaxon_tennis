(load "package://hrpsys_ros_bridge_tutorials/euslisp/jaxon-interface.l")
;;(load "../model/racket-model.l")
(load "../model/racket-model2.l")

(defun jaxon_racket ()
  ;;(jaxon-init)
  (jaxon)
  (setq *robot* *jaxon*)
  (send *robot* :reset-pose)
  (setq racket (instance racket :init :move-origin nil))
  (send racket :move-coords (send *robot* :rarm :end-coords :copy-worldcoords) (send racket :get :gripper))
  (objects (list *robot* racket))

)


(jaxon_racket)
