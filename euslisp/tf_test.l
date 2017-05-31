#!/usr/bin/env roseus

;(ros::roseus-add-msgs "opencv_apps")
(ros::roseus-add-msgs "sensor_msgs")
;(load "~/prog/euslib/jsk/gnuplotlib.l")

;;
;; hit-point [camera_frame]
;;


(defun callback-point (msg)
  (setq *point-msg* msg)
  )

(defun tf_test ()
  (ros::roseus "tf_test")

  (setq *tfl* (instance ros::transform-listener :init)) ;; subscribe
  ;;(setq *tfb* (instance ros::transform-broadcaster :init)) ;; publisher
  (ros::subscribe "/hit_point"
                  geometry_msgs::PointStamped
                  #'callback-point 1)
  ;;
  ;; body ->(angle-vector)-> head_link1 -> camera [fix]
  ;; (send *robot* :angle-vector (send *ri* :state :potentio-vector))
  ;;
  (let ()
    (while (ros::ok)

      (setq *point-msg* nil)
      (ros::spin-once)

      (when *point-msg*
        (print "receive message")
        (setq hit_point *point-msg*)

        ;;(setq tr (instance ros::transformer :init))
        ;;(send tr :set-transform (instance ros::tf-cascaded-coords :init 
        ;;:frame-id "BODY"
        ;;:child-frame-id "left"))

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

        #|
        (setq msg (instance geometry_msgs::PointStamped :init
                              :header header
                              :point
                              (instance geometry_msgs::Point :init
                                        :x x 
                                        :y y
                                        :z z)))
        (ros::publish "hit_point" msg)
        |#
        )
      )
    )
  )

(tf_test)
