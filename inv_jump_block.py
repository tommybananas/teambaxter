#has Baxter move 1 block from a known location to a programmed location using cartesian points

import rospy
import baxter_interface
import struct
import sys

from geometry_msgs.msg import (
    PoseStamped,
    Pose,
    Point,
    Quaternion,
)
from std_msgs.msg import Header

from baxter_core_msgs.srv import (
    SolvePositionIK,
    SolvePositionIKRequest,
)

#define inverse kinematics function
def ik_test(xi,yi,zi):
    ns = "ExternalTools/right/PositionKinematicsNode/IKService"
    iksvc = rospy.ServiceProxy(ns, SolvePositionIK)
    ikreq = SolvePositionIKRequest()
    hdr = Header(stamp=rospy.Time.now(), frame_id='base')
    poses = {
        'right': PoseStamped(
            header=hdr,
            pose=Pose(
                position=Point(
                    x = xi,
                    y = yi,
                    z = zi,
                ),
                orientation=Quaternion(
                    x=1,
                    y=0,
                    z=0,
                    w=0,
                ),
            ),
        ),
    }

    ikreq.pose_stamp.append(poses['right'])
    try:
        rospy.wait_for_service(ns, 5.0)
        resp = iksvc(ikreq)
    except (rospy.ServiceException, rospy.ROSException), e:
        rospy.logerr("Service call failed: %s" % (e,))
        return 1

    # Check if result valid, and type of seed ultimately used to get solution
    # convert rospy's string representation of uint8[]'s to int's
    resp_seeds = struct.unpack('<%dB' % len(resp.result_type),
                               resp.result_type)
    if (resp_seeds[0] != resp.RESULT_INVALID):
        seed_str = {
                    ikreq.SEED_USER: 'User Provided Seed',
                    ikreq.SEED_CURRENT: 'Current Joint Angles',
                    ikreq.SEED_NS_MAP: 'Nullspace Setpoints',
                   }.get(resp_seeds[0], 'None')
        # Format solution into Limb API-compatible dictionary
        limb_joints = dict(zip(resp.joints[0].name, resp.joints[0].position))
        print limb_joints
    else:
        print("INVALID POSE - No Valid Joint Solution Found.")

    return limb_joints

#intialize ros node
rospy.init_node('inv_jump_block')

#create a limb object
limb = baxter_interface.Limb('right')

#create gripper object
gripper = baxter_interface.Gripper('right')
if gripper.calibrated() == 0:
    gripper.calibrate()

#move to first waypoint
way1 = ik_test(0.76,0,-0.10)
limb.move_to_joint_positions(way1)
rospy.sleep(1.0)

#close gripper
gripper.command_position(0)
rospy.sleep(1.0)

#move to second waypoint
way2 = ik_test(0.76,0,-0.03)
limb.move_to_joint_positions(way2)
rospy.sleep(1.0)

#move to third waypoint
way3 = ik_test(0.69, 0, -0.03)
limb.move_to_joint_positions(way3)
rospy.sleep(1.0)

#move to fourth waypoint
way4 = ik_test(0.69, 0, -0.10)
limb.move_to_joint_positions(way4)
rospy.sleep(1.0)

#open gripper
gripper.command_position(100)
rospy.sleep(1.0)

#return to first waypoint
limb.move_to_joint_positions(way1)

