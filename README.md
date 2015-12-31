# arm_ros_conn
[Ros](http://www.ros.org) adapter to my 5dof robot arm.

May this serve as an example (scripts/arm_ros.py) on how to interface [MoveIt](http://moveit.ros.org/) with hardware.

The new angle is send to hardware with the to_angle() call (not in this repos). The interface is 

    def to_angle(axis, speed=255, angle=0)

where axis is the axis number (0..5), speed the motor pwm speed and angle in rad.
