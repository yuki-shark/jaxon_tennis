#!/usr/bin/env roseus

(ros::roseus-add-msgs "sensor_msgs")

(defun target_publisher ()
  (ros::roseus "target_publisher")
  (ros::advertise "target_point"
                  geometry_msgs::PointStamped)

  (let (theta rad x y z)

    (setq theta 0)

    (while (ros::ok)

      (ros::spin-once)

      (setq theta (mod (+ theta 9) 360))
      (setq rad (deg2rad theta))

      (setq x 1000)
      (setq y (+ -300 (* -150 (cos rad))))
      (setq z (+ 1700 (* 150 (sin rad))))

      (setq msg (instance geometry_msgs::PointStamped :init
                          :point
                          (instance geometry_msgs::Point :init
                                    :x x
                                    :y y
                                    :z z)))

      (print "target")
      (format t "x: ~A   y: ~A   z: ~A~%" x y z)

      (ros::publish "target_point" msg)

      (unix::usleep (* 1000 100));; 100ms
    )
  )
)

(target_publisher)
