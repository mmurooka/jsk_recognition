#!/usr/bin/env roseus

(ros::roseus-add-msgs "jsk_recognition_msgs")

(ros::roseus "check_polygon_area")

(ros::subscribe "~input" jsk_recognition_msgs::PolygonArray #'polygon-array-cb)


(defun polygon-array-cb
  (msg)
  (let* (poly-area-list)
    (dolist (poly-msg (send msg :polygons))
      (let* ((poly-points (send (send poly-msg :polygon) :points))
             (poly-pos-list (mapcar #'ros::tf-point->pos poly-points))
             (poly (instance polygon :init :vertices poly-pos-list))
             (poly-area (send poly :area))
             )
        (push poly-area poly-area-list)
        ))
    (print poly-area-list)
    ))
