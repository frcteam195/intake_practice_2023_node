#!/usr/bin/env python3

import rospy
from threading import Thread


from ck_utilities_py_node.motor import *
from ck_utilities_py_node.solenoid import *
from frc_robot_utilities_py_node.frc_robot_utilities_py import *
from frc_robot_utilities_py_node.RobotStatusHelperPy import RobotStatusHelperPy, Alliance, RobotMode, BufferedROSMsgHandlerPy
from ck_ros_msgs_node.msg import Intake_Control, Intake_Status


def ros_func():
    global hmi_updates
    global robot_status

    control_subscriber = BufferedROSMsgHandlerPy(Intake_Control)
    control_subscriber.register_for_updates("IntakeControl")
    status_publisher = rospy.Publisher(
        name="IntakeStatus", data_class=Intake_Status, queue_size=50, tcp_nodelay=True)

    intakeRollerMotor = Motor("intake", MotorType.TalonFX)
    # wristRollerMotor = Motor("wrist", MotorType.TalonFX)

    pincherSolenoid = Solenoid(0, SolenoidType.SINGLE)
    pincherSolenoid.set(SolenoidState.OFF)

    rate = rospy.Rate(20)

    while not rospy.is_shutdown():

        if control_subscriber.get() is not None:
            intake_ctrl_msg : Intake_Control = control_subscriber.get()
            if robot_status.get_mode() == RobotMode.DISABLED:
                intakeRollerMotor.set(ControlMode.PERCENT_OUTPUT, 0.0, 0.0)
                pass

            else:
                if intake_ctrl_msg.rollers_intake:
                    intakeRollerMotor.set(ControlMode.PERCENT_OUTPUT, 1.0, 0.0)
                    pass
                elif intake_ctrl_msg.rollers_outake:
                    intakeRollerMotor.set(ControlMode.PERCENT_OUTPUT, -1.0, 0.0)
                    pass
                else:
                    intakeRollerMotor.set(ControlMode.PERCENT_OUTPUT, 0.0, 0.0)
                    pass

                if intake_ctrl_msg.pincher_solenoid_on:
                    pincherSolenoid.set(True)
                else:
                    pincherSolenoid.set(False)

        status_message = Intake_Status()
        status_publisher.publish(status_message)

        rate.sleep()


def ros_main(node_name):
    rospy.init_node(node_name)
    register_for_robot_updates()

    t1 = Thread(target=ros_func)
    t1.start()

    rospy.spin()

    t1.join(5)
