#!/usr/bin/env python3
"""Configure PX4 SITL for conservative unattended indoor OFFBOARD demos."""
import rospy
from mavros_msgs.msg import ParamValue
from mavros_msgs.srv import ParamSet


def set_int(vehicle, param_id, value):
    service_name = '/%s/mavros/param/set' % vehicle
    rospy.wait_for_service(service_name, timeout=20.0)
    setter = rospy.ServiceProxy(service_name, ParamSet)
    result = setter(param_id=param_id, value=ParamValue(integer=int(value), real=0.0))
    rospy.loginfo('%s %s=%s success=%s', vehicle, param_id, value, result.success)


def set_real(vehicle, param_id, value):
    service_name = '/%s/mavros/param/set' % vehicle
    rospy.wait_for_service(service_name, timeout=20.0)
    setter = rospy.ServiceProxy(service_name, ParamSet)
    result = setter(param_id=param_id, value=ParamValue(integer=0, real=float(value)))
    rospy.loginfo('%s %s=%.3f success=%s', vehicle, param_id, value, result.success)


def main():
    rospy.init_node('configure_px4_sitl_offboard')
    for vehicle in ('uav0', 'uav1'):
        # Avoid RC-loss failsafes in unattended SITL OFFBOARD runs.
        set_int(vehicle, 'COM_RC_IN_MODE', 1)
        set_int(vehicle, 'COM_RCL_EXCEPT', 4)

        # Conservative translational limits for the narrow indoor world. These
        # parameters are applied at runtime before flight starts.
        set_real(vehicle, 'MPC_XY_VEL_MAX', 1.20)
        set_real(vehicle, 'MPC_Z_VEL_MAX_UP', 0.70)
        set_real(vehicle, 'MPC_Z_VEL_MAX_DN', 0.50)
        set_real(vehicle, 'MPC_ACC_HOR', 1.00)

    rospy.loginfo('PX4 SITL OFFBOARD and conservative indoor-speed parameters configured.')


if __name__ == '__main__':
    main()
