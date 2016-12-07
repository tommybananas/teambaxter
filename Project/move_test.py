'''accepts x,y,z from command line and commands right arm to given point'''

#predefined libraries
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

#our custom modules
import ik

#get command line arguments
x = sys.argv[1]
y = sys.argv[2]
z = sys.argv[3]
print "\nx=", x, "\ty=", y, "\tz=", z

#intialize ros node
rospy.init_node('move_test')

#create a limb object
limb = baxter_interface.Limb('right')

#move to point
p0 = ik.calc(float(x), float(y), float(z))
limb.move_to_joint_positions(p0)
