#!/bin/bash

rosrun collada_urdf urdf_to_collada urdf/arm.urdf /tmp/arm.dae
python `openrave-config --python-dir`/openravepy/_openravepy_/ikfast.py --robot=/tmp/arm.dae --iktype=translationdirection5d --baselink=2 --eelink=9
mv ik.cpp ../arm_ikfast_arm_plugin/src/arm_arm_ikfast_solver.cpp
