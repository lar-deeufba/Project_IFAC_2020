#!/usr/bin/env python

# ROS import
import sys
import rospy
from tf.transformations import quaternion_from_euler, quaternion_matrix
import tf
from tf import TransformListener, Transformer
import publish_joint_states
from publish_joint_states import *

# Moveit Import
import moveit_commander
import moveit_python
from moveit_commander.conversions import pose_to_list

# Msg Import
from moveit_msgs.msg import *
from moveit_msgs.srv import *
from geometry_msgs.msg import *
from std_msgs.msg import String, Header, ColorRGBA
from visualization_msgs.msg import Marker
from shape_msgs.msg import SolidPrimitive
from sensor_msgs.msg import JointState

# Inverse Kinematics Import
from ur_inverse_kinematics import *

# Python Import
import numpy as np
from numpy import array, dot, pi
from numpy.linalg import det, norm

# Customized code
from get_geometric_jacobian import *
from get_ur5_position import *
from show_HTM import *
from get_dist3D import *
import CPA_classico
from CPA_classico import *


"""
### Use instructions

# First launch RViz:
roslaunch ur5_moveit_config demo.launch

# Second - run UR5_CPA file
rosrun UR5_CPA.py
"""

def get_param(name, value=None):
    private = "~%s" % name
    if rospy.has_param(private):
        return rospy.get_param(private)
    elif rospy.has_param(name):
        return rospy.get_param(name)
    else:
        return value

class MoveGroupPythonIntefaceTutorial(object):
    """MoveGroupPythonIntefaceTutorial"""

    def __init__(self):
        super(MoveGroupPythonIntefaceTutorial, self).__init__()

        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('move_group_python_interface_ur5_robot',
                        anonymous=True)

        self.display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                                       moveit_msgs.msg.DisplayTrajectory,
                                                       queue_size=20)
        rospy.sleep(0.5)

        # Topico para publicar marcadores para o Rviz
        self.marker_publisher = rospy.Publisher('visualization_marker', Marker, queue_size=100)
        rospy.sleep(0.5)

        self.tf = TransformListener()
        rospy.sleep(0.5)

        self.init_id_path = 13

        # Topico para publicar no /robot_state_publisher
        self.pub_joint_states = rospy.Publisher('joint_states_ur5', JointState, queue_size=50)


        self.marker = Marker()
        self.joint_states = JointState()
        self.joint_states.name = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint', 'wrist_2_joint',
    'wrist_3_joint']

        # d1, SO, EO, a2, a3, d4, d45, d5, d6
        self.ur5_param = (0.089159, 0.13585, -0.1197, 0.425, 0.39225, 0.10915, 0.093, 0.09465, 0.0823 + 0.15)

    """
    Calculate the initial robot position - Used before CPA application
    Need to update: pass analytical homogeneous transformation to invKine
    """
    def get_ik(self, pose):
        matrix = tf.TransformerROS()
        # The orientation of /grasping_link will be constant
        q = quaternion_from_euler(1.5707, 1.5707, 0)

        matrix2 = matrix.fromTranslationRotation((pose[0]*(-1), pose[1]*(-1), pose[2]), (q[0], q[1], q[2], q[3]))
        # print "The quaternion representation is %s %s %s %s." % (q[0], q[1], q[2], q[3])

        rospy.loginfo(matrix2)
        th = invKine(matrix2)
        sol1 = th[:, 0].transpose()
        joint_values_from_ik = np.array(sol1)

        joint_values = joint_values_from_ik[0, :]

        return joint_values.tolist()

    """
    Also gets each frame position through lookupTransform
    """
    def get_repulsive_cp(self, obs_pos, joint_values, diam_rep_ur5):

        marker_lines = Marker()

        ur5_links = [
            "shoulder_link",
            "upper_arm_link",
            "forearm_link",
            "wrist_1_link",
            "wrist_2_link",
            "wrist_3_link",
            "grasping_link"
        ]

        cp_position, cp_distance = [], []
        for i in range(len(ur5_links)):
            # link_pose = get_ur5_position(self.ur5_param, joint_values, ur5_links[i])
            link_pose, _ = self.tf.lookupTransform("base_link", ur5_links[i], rospy.Time())
            cp_position.append(link_pose)
            cp_distance.append(np.linalg.norm(link_pose - np.asarray(obs_pos)))
            # print(cp_distance)
            marker_lines.points.append(Point(obs_pos[0], obs_pos[1], obs_pos[2]))
            marker_lines.points.append(Point(link_pose[0], link_pose[1], link_pose[2]))
            self.add_sphere(link_pose, i, diam_rep_ur5, ColorRGBA(1.0, 0.0, 0.0, 0.3))

        return cp_position, cp_distance

    """
    Adds lines representing distances from obstacles to the robot control's point
    """
    def add_line(self, marker):
        marker.header.frame_id = "base_link"
        marker.type = marker.LINE_STRIP
        marker.action = marker.MODIFY
        marker.scale = Vector3(0.008, 0.009, 0.1)
        marker.color = ColorRGBA(0.0, 1.0, 0.0, 0.8)
        self.marker_publisher.publish(marker)

    """
    Adds the obstacles and repulsive control points on the robot
    """
    def add_sphere(self, pose, id, diam, color):
        marker = Marker()
        marker.header.frame_id = "base_link"
        marker.id = id
        marker.pose.position  = Point(pose[0], pose[1], pose[2])
        marker.type = marker.SPHERE
        marker.action = marker.MODIFY
        marker.scale = Vector3(diam, diam, diam)
        marker.color = color
        self.marker_publisher.publish(marker)

    """
    Plot robot's path to the RViz environment
    """
    def visualize_path_planned(self, path):
        self.marker.points.append(Point(path[0], path[1], path[2]))
        self.marker.header.frame_id = "base_link"
        self.marker.id = 14
        self.marker.type = self.marker.LINE_STRIP
        self.marker.action = self.marker.ADD
        self.marker.scale = Vector3(0.008, 0.009, 0.1)
        self.marker.color = ColorRGBA(0.0, 1.0, 0.0, 0.8)
        self.marker_publisher.publish(self.marker)

    """
    Delete all markers in Rviz
    """
    def delete_markers(self):
        marker = Marker()
        marker.action = marker.DELETEALL
        self.marker_publisher.publish(marker)

def main():
    ur5_robot = MoveGroupPythonIntefaceTutorial()
    ur5_robot.delete_markers()

    ### UR5 Initial position
    raw_input("' =========== Aperte enter para posicionar o UR5 \n")
    ur5_robot.joint_states.position = [0, -1.5707, 0, -1.5707, 0, 0] # Posicao configurada no fake_controller_joint_states
    ur5_robot.pub_joint_states.publish(ur5_robot.joint_states)

    raw_input("' =========== Aperte enter para carregar o obstaculo e objetivo \n")
    ### Final and obstacle points
    obs_pos = [0.45, 0.4, 0.4]
    diam_obs = 0.4
    ur5_robot.add_sphere(obs_pos, 11, diam_obs, ColorRGBA(1.0, 0.0, 0.0, 0.5))

    ptFinal = [0.45, 0.3, 0.5]
    oriFinal = [0.01, 0.01, 0.01]
    diam_goal = 0.04
    ur5_robot.add_sphere(ptFinal, 13, diam_goal, ColorRGBA(0.0, 1.0, 0.0, 0.8))

    ### CPA Parameters
    err = diam_goal/2
    max_iter = 5000
    zeta = [0.5 for i in range(7)]
    eta = [0.005 for i in range(6)]
    rho_0 = diam_obs
    dist_att = 0.05
    dist_att_config = 0.15
    alfa = 0.5
    alfa_rot = 0.2
    diam_rep_ur5 = 0.15

    raw_input("' =========== Pressione enter para posicionar o UR5 \n")
    ur5_robot.joint_states.position = [0, -1.5707, 0, -1.5707, 0, 0] # Posicao configurada no fake_controller_joint_states
    ur5_robot.pub_joint_states.publish(ur5_robot.joint_states)

    ### GET repulsive control point's position through LOOKUPTRANSFORM
    CP_pos, CP_dist = ur5_robot.get_repulsive_cp(obs_pos, ur5_robot.joint_states.position, diam_rep_ur5)

    ### Parameters
    CPAA_state = False
    Orientation_state = True

    hz = get_param("rate", 50)  # 10hz
    r = rospy.Rate(hz)

    ptAtual, oriAtual = ur5_robot.tf.lookupTransform("base_link", "grasping_link", rospy.Time())

    dist_EOF_to_Goal = np.linalg.norm(ptAtual - np.asarray(ptFinal))
    n = 0

    raw_input("' =========== Aperte enter para iniciar o algoritmo dos CPAs")
    while dist_EOF_to_Goal > err and not rospy.is_shutdown():
        Jacobian = get_geometric_jacobian(ur5_robot.ur5_param, ur5_robot.joint_states.position)
        CP_pos, CP_dist = ur5_robot.get_repulsive_cp(obs_pos, ur5_robot.joint_states.position, diam_rep_ur5)

        joint_att_force_p, joint_att_force_w, joint_rep_force = CPA_classico.get_joint_forces(ptAtual, ptFinal, oriAtual, oriFinal,
        dist_EOF_to_Goal, Jacobian, ur5_robot.joint_states.position, ur5_robot.ur5_param, zeta,
        eta, rho_0, dist_att, dist_att_config, CP_dist, CP_pos, obs_pos, CPAA_state, diam_rep_ur5)

        # Joint angles UPDATE
        ur5_robot.joint_states.position = ur5_robot.joint_states.position + alfa*joint_att_force_p[0]
        if Orientation_state:
            ur5_robot.joint_states.position = ur5_robot.joint_states.position + alfa_rot*joint_att_force_w[0]

        list = np.transpose(joint_rep_force[0]).tolist()
        for j in range(6):
            for i in range(6):
                ur5_robot.joint_states.position[i] = ur5_robot.joint_states.position[i] + alfa*list[j][i]
        ptAtual, oriAtual = ur5_robot.tf.lookupTransform("base_link", "grasping_link", rospy.Time())
        oriAtual += quaternion_from_euler(1.5707, 0, 0)

        if n % 10 == 0:
            ur5_robot.visualize_path_planned(ptAtual)
            print("Distance to the goal: " + str(dist_EOF_to_Goal))
        dist_EOF_to_Goal = np.linalg.norm(ptAtual - np.asarray(ptFinal))
        ur5_robot.pub_joint_states.publish(ur5_robot.joint_states)

        try:
            r.sleep()
        except rospy.exceptions.ROSTimeMovedBackwardsException:
            pass

        n += 1

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
    except KeyboardInterrupt:
        pass
