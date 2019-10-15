from numpy import array, cos, sin
import tf
from tf import TransformListener
import rospy

"""
Show Homogeneous Transformation Matrices of each frame for comparison
"""
def show_HTM(ur5_param, joint_values):

    matrix = tf.TransformListener(True, rospy.Duration(10.0))
    rospy.sleep(0.5)

    th1, th2, th3, th4, th5, th6  = joint_values
    d1, SO, EO, a2, a3, d4, d45, d5, d6 = ur5_param

    print "T0_1:  -------------------------------------- SHOULDER_LINK "
    t0_1 = array([[cos(th1), -sin(th1), 0, 0],
                  [sin(th1),  cos(th1), 0, 0],
                  [0,                0, 1, d1],
                  [0,                0, 0, 1]])
    print(t0_1)

    print "T0_1:  -------------------------------------- SHOULDER_LINK_ROS"
    pos1, quat1 = matrix.lookupTransform("base_link", "shoulder_link", rospy.Time())
    t0_1_ros = matrix.fromTranslationRotation(pos1, quat1)
    print(t0_1_ros)

    print '\n'

    print "T0_2:  -------------------------------------- UPPER_ARM_LINK "
    t1_2 = array([[-sin(th2), 0, cos(th2),  0],
                  [0,         1,        0, SO],
                  [-cos(th2), 0,-sin(th2),  0],
                  [0,         0,        0,  1]])
    t0_2 = t0_1.dot(t1_2)
    print(t0_2)

    print "T0_2:  -------------------------------------- UPPER_ARM_LINK_ROS"
    pos2, quat2 = matrix.lookupTransform("base_link", "upper_arm_link", rospy.Time())
    t0_2_ros = matrix.fromTranslationRotation(pos2, quat2)
    print(t0_2_ros)

    print '\n'

    print "T0_3:  -------------------------------------- FOREARM_LINK "
    t2_3 = array([[ cos(th3), 0, sin(th3),  0],
                  [0,         1,        0, EO],
                  [-sin(th3), 0, cos(th3), a2],
                  [0,         0,        0,  1]])
    t0_3 = t0_2.dot(t2_3)
    print(t0_3)

    print "T0_3:  -------------------------------------- FOREARM_LINK_ROS"
    pos3, quat3 = matrix.lookupTransform("base_link", "forearm_link", rospy.Time())
    t0_3_ros = matrix.fromTranslationRotation(pos3, quat3)
    print(t0_3_ros)

    print '\n'

    print "T0_4:  -------------------------------------- WRIST_1_LINK "
    t3_4 = array([[-sin(th4), 0, cos(th4),  0],
                  [0,         1,        0,  0],
                  [-cos(th4), 0,-sin(th4), a3],
                  [0,         0,        0,  1]])
    t0_4 = t0_3.dot(t3_4)
    print(t0_4)

    print "T0_4:  -------------------------------------- WRIST_1_LINK_ROS"
    pos4, quat4 = matrix.lookupTransform("base_link", "wrist_1_link", rospy.Time())
    t0_4_ros = matrix.fromTranslationRotation(pos4, quat4)
    print(t0_4_ros)

    print '\n'

    print "T0_5:  -------------------------------------- WRIST_2_LINK "
    t4_5 = array([[cos(th5), -sin(th5), 0, 0],
                  [sin(th5),  cos(th5), 0, d45],
                  [0,                0, 1, 0],
                  [0,                0, 0, 1]])
    t0_5 = t0_4.dot(t4_5)
    print(t0_5)

    print "T0_5:  -------------------------------------- WRIST_2_LINK_ROS"
    pos5, quat5 = matrix.lookupTransform("base_link", "wrist_2_link", rospy.Time())
    t0_5_ros = matrix.fromTranslationRotation(pos5, quat5)
    print(t0_5_ros)

    print '\n'

    print "T0_6:  -------------------------------------- WRIST_3_LINK "
    t5_6 = array([[ cos(th6), 0, sin(th6),  0],
                  [0,         1,        0,  0],
                  [-sin(th6), 0, cos(th6), d5],
                  [0,         0,        0,  1]])
    t0_6 = t0_5.dot(t5_6)
    print(t0_6)

    print "T0_6:  -------------------------------------- WRIST_3_LINK_ROS"
    pos6, quat6 = matrix.lookupTransform("base_link", "wrist_3_link", rospy.Time())
    t0_6_ros = matrix.fromTranslationRotation(pos6, quat6)
    print(t0_6_ros)

    print '\n'

    print "T0_7:  -------------------------------------- TOOL0_LINK "
    t6_7 = array([[1, 0, 0,  0],
                  [0, 0, 1, d6],
                  [0,-1, 0,  0],
                  [0, 0, 0,  1]])
    t0_7 = t0_6.dot(t6_7)
    print(t0_7)

    print "T0_7:  -------------------------------------- TOOL0_LINK_ROS"
    pos7, quat7 = matrix.lookupTransform("base_link", "tool0", rospy.Time())
    t0_7_ros = matrix.fromTranslationRotation(pos7, quat7)
    print(t0_7_ros)
