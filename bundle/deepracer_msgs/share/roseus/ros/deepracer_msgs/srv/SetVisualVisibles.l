;; Auto-generated. Do not edit!


(when (boundp 'deepracer_msgs::SetVisualVisibles)
  (if (not (find-package "DEEPRACER_MSGS"))
    (make-package "DEEPRACER_MSGS"))
  (shadow 'SetVisualVisibles (find-package "DEEPRACER_MSGS")))
(unless (find-package "DEEPRACER_MSGS::SETVISUALVISIBLES")
  (make-package "DEEPRACER_MSGS::SETVISUALVISIBLES"))
(unless (find-package "DEEPRACER_MSGS::SETVISUALVISIBLESREQUEST")
  (make-package "DEEPRACER_MSGS::SETVISUALVISIBLESREQUEST"))
(unless (find-package "DEEPRACER_MSGS::SETVISUALVISIBLESRESPONSE")
  (make-package "DEEPRACER_MSGS::SETVISUALVISIBLESRESPONSE"))

(in-package "ROS")





(defclass deepracer_msgs::SetVisualVisiblesRequest
  :super ros::object
  :slots (_link_names _visual_names _visibles _block ))

(defmethod deepracer_msgs::SetVisualVisiblesRequest
  (:init
   (&key
    ((:link_names __link_names) (let (r) (dotimes (i 0) (push "" r)) r))
    ((:visual_names __visual_names) (let (r) (dotimes (i 0) (push "" r)) r))
    ((:visibles __visibles) (make-array 0 :initial-element 0 :element-type :integer))
    ((:block __block) nil)
    )
   (send-super :init)
   (setq _link_names __link_names)
   (setq _visual_names __visual_names)
   (setq _visibles __visibles)
   (setq _block __block)
   self)
  (:link_names
   (&optional __link_names)
   (if __link_names (setq _link_names __link_names)) _link_names)
  (:visual_names
   (&optional __visual_names)
   (if __visual_names (setq _visual_names __visual_names)) _visual_names)
  (:visibles
   (&optional __visibles)
   (if __visibles (setq _visibles __visibles)) _visibles)
  (:block
   (&optional __block)
   (if __block (setq _block __block)) _block)
  (:serialization-length
   ()
   (+
    ;; string[] _link_names
    (apply #'+ (mapcar #'(lambda (x) (+ 4 (length x))) _link_names)) 4
    ;; string[] _visual_names
    (apply #'+ (mapcar #'(lambda (x) (+ 4 (length x))) _visual_names)) 4
    ;; int8[] _visibles
    (* 1    (length _visibles)) 4
    ;; bool _block
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string[] _link_names
     (write-long (length _link_names) s)
     (dolist (elem _link_names)
       (write-long (length elem) s) (princ elem s)
       )
     ;; string[] _visual_names
     (write-long (length _visual_names) s)
     (dolist (elem _visual_names)
       (write-long (length elem) s) (princ elem s)
       )
     ;; int8[] _visibles
     (write-long (length _visibles) s)
     (dotimes (i (length _visibles))
       (write-byte (elt _visibles i) s)
       )
     ;; bool _block
       (if _block (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string[] _link_names
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _link_names (make-list n))
     (dotimes (i n)
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setf (elt _link_names i) (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
     ))
   ;; string[] _visual_names
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _visual_names (make-list n))
     (dotimes (i n)
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setf (elt _visual_names i) (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
     ))
   ;; int8[] _visibles
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _visibles (instantiate integer-vector n))
     (dotimes (i n)
     (setf (elt _visibles i) (sys::peek buf ptr- :char)) (incf ptr- 1)
     (if (> (elt _visibles i) 127) (setf (elt _visibles i) (- (elt _visibles i) 256)))
     ))
   ;; bool _block
     (setq _block (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(defclass deepracer_msgs::SetVisualVisiblesResponse
  :super ros::object
  :slots (_success _status_message _status _messages ))

(defmethod deepracer_msgs::SetVisualVisiblesResponse
  (:init
   (&key
    ((:success __success) nil)
    ((:status_message __status_message) "")
    ((:status __status) (make-array 0 :initial-element 0 :element-type :integer))
    ((:messages __messages) (let (r) (dotimes (i 0) (push "" r)) r))
    )
   (send-super :init)
   (setq _success __success)
   (setq _status_message (string __status_message))
   (setq _status __status)
   (setq _messages __messages)
   self)
  (:success
   (&optional __success)
   (if __success (setq _success __success)) _success)
  (:status_message
   (&optional __status_message)
   (if __status_message (setq _status_message __status_message)) _status_message)
  (:status
   (&optional __status)
   (if __status (setq _status __status)) _status)
  (:messages
   (&optional __messages)
   (if __messages (setq _messages __messages)) _messages)
  (:serialization-length
   ()
   (+
    ;; bool _success
    1
    ;; string _status_message
    4 (length _status_message)
    ;; int8[] _status
    (* 1    (length _status)) 4
    ;; string[] _messages
    (apply #'+ (mapcar #'(lambda (x) (+ 4 (length x))) _messages)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; bool _success
       (if _success (write-byte -1 s) (write-byte 0 s))
     ;; string _status_message
       (write-long (length _status_message) s) (princ _status_message s)
     ;; int8[] _status
     (write-long (length _status) s)
     (dotimes (i (length _status))
       (write-byte (elt _status i) s)
       )
     ;; string[] _messages
     (write-long (length _messages) s)
     (dolist (elem _messages)
       (write-long (length elem) s) (princ elem s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; bool _success
     (setq _success (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; string _status_message
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _status_message (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; int8[] _status
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _status (instantiate integer-vector n))
     (dotimes (i n)
     (setf (elt _status i) (sys::peek buf ptr- :char)) (incf ptr- 1)
     (if (> (elt _status i) 127) (setf (elt _status i) (- (elt _status i) 256)))
     ))
   ;; string[] _messages
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _messages (make-list n))
     (dotimes (i n)
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setf (elt _messages i) (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
     ))
   ;;
   self)
  )

(defclass deepracer_msgs::SetVisualVisibles
  :super ros::object
  :slots ())

(setf (get deepracer_msgs::SetVisualVisibles :md5sum-) "f1147e9c73d95b8955fad12bcf7a1ecd")
(setf (get deepracer_msgs::SetVisualVisibles :datatype-) "deepracer_msgs/SetVisualVisibles")
(setf (get deepracer_msgs::SetVisualVisibles :request) deepracer_msgs::SetVisualVisiblesRequest)
(setf (get deepracer_msgs::SetVisualVisibles :response) deepracer_msgs::SetVisualVisiblesResponse)

(defmethod deepracer_msgs::SetVisualVisiblesRequest
  (:response () (instance deepracer_msgs::SetVisualVisiblesResponse :init)))

(setf (get deepracer_msgs::SetVisualVisiblesRequest :md5sum-) "f1147e9c73d95b8955fad12bcf7a1ecd")
(setf (get deepracer_msgs::SetVisualVisiblesRequest :datatype-) "deepracer_msgs/SetVisualVisiblesRequest")
(setf (get deepracer_msgs::SetVisualVisiblesRequest :definition-)
      "string[] link_names
string[] visual_names
int8[] visibles
bool block
---
bool success
string status_message
int8[] status
string[] messages
")

(setf (get deepracer_msgs::SetVisualVisiblesResponse :md5sum-) "f1147e9c73d95b8955fad12bcf7a1ecd")
(setf (get deepracer_msgs::SetVisualVisiblesResponse :datatype-) "deepracer_msgs/SetVisualVisiblesResponse")
(setf (get deepracer_msgs::SetVisualVisiblesResponse :definition-)
      "string[] link_names
string[] visual_names
int8[] visibles
bool block
---
bool success
string status_message
int8[] status
string[] messages
")



(provide :deepracer_msgs/SetVisualVisibles "f1147e9c73d95b8955fad12bcf7a1ecd")


