(defun make-shovel (&key (shaft-length 820.0) (shaft-radius 15.0)
                         (handle-width 120.0) (handle-radius 15.0) (handle-radius-2 4.0)
                         (handle-length 130.0)
                         (blade-width 175.0) (blade-length 245.0)
                         )
  (let (
        blade
        shaft
        handle
        shovel
        )
    ;;
    (setq blade (make-cube blade-length blade-width 10.0))
    ;;(send blade :face-color)
    (setq shaft (make-cylinder shaft-radius shaft-length :segments 18))
    (let ((handle-base
           (make-prism
            (list
             (float-vector 0 (- shaft-radius) (- handle-radius))
             (float-vector (- handle-length) (- (/ handle-width 2.0)) (- handle-radius))
             (float-vector (- handle-length)    (/ handle-width 2.0)  (- handle-radius))
             (float-vector 0 shaft-radius (- handle-radius))
             )
            (float-vector 0 0 (* 2 handle-radius)))
           )
          (handle-hole
           (make-prism
            (list
             (float-vector (- (- (* 2 handle-radius)) 0) (- shaft-radius) (- (* 2 handle-radius)))
             (float-vector (+ (- handle-length)
                              (* 2 handle-radius))
                           (-
                            (/ (* (- (/ handle-width 2) (* 2 handle-radius-2))
                                  (- handle-length (* 4 handle-radius-2)))
                               handle-length))
                           (- (* 2 handle-radius)))
             (float-vector (+ (- handle-length)
                              (* 2 handle-radius))
                            (/ (* (- (/ handle-width 2) (* 2 handle-radius-2))
                                  (- handle-length (* 4 handle-radius-2)))
                               handle-length)
                           (- (* 2 handle-radius)))
             (float-vector (- (* 2 handle-radius)) shaft-radius (- (* 2 handle-radius)))
            )
            (float-vector 0 0 (* 4 handle-radius)))
           )
          )
      (setq handle (body- handle-base handle-hole))
      )
    (send shaft :rotate pi/2 :y)
    (send blade :translate (float-vector (+ shaft-length (/ blade-length 2)) 0 0))
    (send handle :assoc shaft)
    (send handle :assoc blade)
    (send handle :rotate pi/2 :x)
    (send handle :translate (float-vector (- handle-length handle-radius) 0 0))
    (setq shovel (instance bodyset :init (make-cascoords)
                           :bodies (list handle shaft blade)))
    (let ((rcoords (make-cascoords))
          (lcoords (make-cascoords))
          (target  (make-cascoords))
          (edge    (make-cascoords)))
      (send shovel :put :rarm rcoords)
      (send shovel :put :larm lcoords)
      (send shovel :put :target target)
      (send shovel :put :edge edge)

      (send rcoords :rotate pi :x)

      (send lcoords :translate (float-vector (/ shaft-length 2.0) 0 0))
      (send lcoords :rotate pi/2 :y) ;;
      (send lcoords :rotate pi :z)   ;;
      (send lcoords :rotate pi :x)   ;;??

      (send target :translate (float-vector (+ handle-length (- handle-radius)
                                               shaft-length (/ blade-length 2.0)) 0 0))
      (send target :rotate -pi/2 :x)

      (send edge :translate (float-vector (+ handle-length (- handle-radius)
                                               shaft-length blade-length 2.0) 0 0))
      (send edge :rotate -pi/2 :x)

      (send shovel :assoc rcoords)
      (send shovel :assoc lcoords)
      (send shovel :assoc target)
      (send shovel :assoc edge)
      )
    shovel
    ))
