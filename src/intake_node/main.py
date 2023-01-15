#!/usr/bin/env python3

import tf2_ros
import rospy
from threading import Thread


from ck_utilities_py_node.motor import *
from frc_robot_utilities_py_node.frc_robot_utilities_py import *
from frc_robot_utilities_py_node.RobotStatusHelperPy import RobotStatusHelperPy, Alliance, RobotMode, BufferedROSMsgHandlerPy
from ck_ros_msgs_node.msg import Intake_Control, Intake_Status

def ros_func():
    global hmi_updates
    global robot_status

    control_sub = BufferedROSMsgHandlerPy(Intake_Control)
    control_sub.register_for_updates("IntakeControl")
    status_pub = rospy.Publisher(name="IntakeStatus", data_class=Intake_Status, queue_size=50, tcp_nodelay=True)

    intakeRollerMotor = Motor(11, MotorType.TalonFX)
    intakeRollerMotor.set_defaults()
    intakeRollerMotor.set_neutral_mode(NeutralMode.Brake)
    intakeRollerMotor.set_forward_soft_limit(18000.0)
    intakeRollerMotor.set_reverse_soft_limit(0.0)
    intakeRollerMotor.apply()




    intake_broadcaster = tf2_ros.TransformBroadcaster()


    rate = rospy.Rate(20)

    while not rospy.is_shutdown():

        if control_sub.get() is not None:
            if robot_status.get_mode() == RobotMode.DISABLED:
                intakeRollerMotor.set(ControlMode.PERCENT_OUTPUT, 0.0, 0.0)
                
            else:
                if control_sub.get().rollers_intake:
                    intakeRollerMotor.set(ControlMode.PERCENT_OUTPUT, 1.0, 0.0)
                elif control_sub.get().rollers_outake:
                    intakeRollerMotor.set(ControlMode.PERCENT_OUTPUT, -1.0, 0.0)
                else:
                    intakeRollerMotor.set(ControlMode.PERCENT_OUTPUT, 0.0, 0.0)
              

        intake_broadcaster.sendTransform(get_intake_transforms(intakeRollerMotor.get_sensor_position()))

        pubmsg = Intake_Status()
        #pubmsg.arm_base_actual_position = armUpperMotor.get_sensor_position()
        #pubmsg.arm_upper_actual_position = intakeRollerMotor.get_sensor_position()
        status_pub.publish(pubmsg)

        rate.sleep()


def ros_main(node_name):
    rospy.init_node(node_name)
    register_for_robot_updates()

    t1 = Thread(target=ros_func)
    t1.start()

    rospy.spin()

    t1.join(5)