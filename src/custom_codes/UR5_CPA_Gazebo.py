#!/usr/bin/env python

# ROS import
import sys
import rospy
from tf.transformations import quaternion_from_euler, quaternion_matrix
import tf
from tf import TransformListener, Transformer
import actionlib

# Moveit Import
import moveit_commander
from moveit_commander.conversions import pose_to_list

# Msg Import
from moveit_msgs.msg import *
from geometry_msgs.msg import *
from std_msgs.msg import String, Header, ColorRGBA
from visualization_msgs.msg import Marker
from shape_msgs.msg import SolidPrimitive
from sensor_msgs.msg import JointState
from control_msgs.msg import *
from trajectory_msgs.msg import *

# Inverse Kinematics Import
from ur_inverse_kinematics import *

# Python Import
import numpy as np
from numpy import array, dot, pi
from numpy.linalg import det, norm

# Customized code
from get_geometric_jacobian import *
from get_ur5_position import *
from get_dist3D import *
import CPA
from CPA import *

def get_param(name, value=None):
    private = "~%s" % name
    if rospy.has_param(private):
        return rospy.get_param(private)
    elif rospy.has_param(name):
        return rospy.get_param(name)
    else:
        return value


"""
This function transforms from matrix to quaternion
"""


def quaternion_from_matrix(joint_values, isprecise=False):

    th1, th2, th3, th4, th5, th6 = joint_values

    matrix = [[-(sin(th1) * sin(th5) + cos(th1) * cos(th5) * cos(th2 + th3 + th4)) * cos(th6) + sin(th6) * sin(th2 + th3 + th4) * cos(th1), (sin(th1) * sin(th5) + cos(th1) * cos(th5) * cos(th2 + th3 + th4)) * sin(th6) + sin(th2 + th3 + th4) * cos(th1) * cos(th6), -sin(th1) * cos(th5) + sin(th5) * cos(th1) * cos(th2 + th3 + th4)],
              [(-sin(th1) * cos(th5) * cos(th2 + th3 + th4) + sin(th5) * cos(th1)) * cos(th6) + sin(th1) * sin(th6) * sin(th2 + th3 + th4), (sin(th1) * cos(th5) * cos(th2 + th3 + th4) - sin(th5) * cos(th1)) * sin(th6) + sin(th1) * sin(th2 + th3 + th4) * cos(th6), sin(th1) * sin(th5) * cos(th2 + th3 + th4) + cos(th1) * cos(th5)],
              [sin(th6) * cos(th2 + th3 + th4) + sin(th2 + th3 + th4) * cos(th5) * cos(th6), -sin(th6) * sin(th2 + th3 + th4) * cos(th5) + cos(th6) * cos(th2 + th3 + th4), -sin(th5) * sin(th2 + th3 + th4)]]

    M = np.array(matrix, dtype=np.float64, copy=False)[:4, :4]
    if isprecise:
        q = np.empty((4, ))
        t = np.trace(M)
        if t > M[3, 3]:
            q[0] = t
            q[3] = M[1, 0] - M[0, 1]
            q[2] = M[0, 2] - M[2, 0]
            q[1] = M[2, 1] - M[1, 2]
        else:
            i, j, k = 0, 1, 2
            if M[1, 1] > M[0, 0]:
                i, j, k = 1, 2, 0
            if M[2, 2] > M[i, i]:
                i, j, k = 2, 0, 1
            t = M[i, i] - (M[j, j] + M[k, k]) + M[3, 3]
            q[i] = t
            q[j] = M[i, j] + M[j, i]
            q[k] = M[k, i] + M[i, k]
            q[3] = M[k, j] - M[j, k]
            q = q[[3, 0, 1, 2]]
        q *= 0.5 / math.sqrt(t * M[3, 3])
    else:
        m00 = M[0, 0]
        m01 = M[0, 1]
        m02 = M[0, 2]
        m10 = M[1, 0]
        m11 = M[1, 1]
        m12 = M[1, 2]
        m20 = M[2, 0]
        m21 = M[2, 1]
        m22 = M[2, 2]
        # symmetric matrix K
        K = np.array([[m00 - m11 - m22, 0.0,         0.0,         0.0],
                      [m01 + m10,     m11 - m00 - m22, 0.0,         0.0],
                      [m02 + m20,     m12 + m21,     m22 - m00 - m11, 0.0],
                      [m21 - m12,     m02 - m20,     m10 - m01,     m00 + m11 + m22]])
        K /= 3.0
        # quaternion is eigenvector of K that corresponds to largest eigenvalue
        w, V = np.linalg.eigh(K)
        q = V[[3, 0, 1, 2], np.argmax(w)]
    if q[0] < 0.0:
        np.negative(q, q)
    return q


class MoveGroupPythonIntefaceTutorial(object):
    """MoveGroupPythonIntefaceTutorial"""

    def __init__(self):
        super(MoveGroupPythonIntefaceTutorial, self).__init__()

        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node(
            'move_group_python_interface_ur5_robot', anonymous=True)

        self.display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                                            moveit_msgs.msg.DisplayTrajectory,
                                                            queue_size=20)
        rospy.sleep(0.5)

        # Topico para publicar marcadores para o Rviz
        self.marker_publisher = rospy.Publisher(
            'visualization_marker', Marker, queue_size=100)
        rospy.sleep(0.5)

        self.tf = TransformListener()
        rospy.sleep(0.5)

        self.marker = Marker()
        self.joint_states = JointState()
        self.joint_states.name = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint', 'wrist_2_joint',
                                  'wrist_3_joint']

        # d1, SO, EO, a2, a3, d4, d45, d5, d6
        self.ur5_param = (0.089159, 0.13585, -0.1197, 0.425,
                          0.39225, 0.10915, 0.093, 0.09465, 0.0823 + 0.15)

        self.client = actionlib.SimpleActionClient(
            'arm_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
        print "Waiting for server (gazebo)..."
        self.client.wait_for_server()
        print "Connected to server (gazebo)"

        # Uncomment this if you are working with real UR5

        # self.clientreal = actionlib.SimpleActionClient('follow_joint_trajectory', FollowJointTrajectoryAction)
        # print "Waiting for server (real)..."
        # self.clientreal.wait_for_server()
        # print "Connected to server (real)"

        self.n = 1
        self.id = 100
        self.id2 = 130

    """
    Get UR5 Inverse Kinematics
    """

    def get_ik(self, pose):
        matrix = tf.TransformerROS()
        q = quaternion_from_euler(1.5707, 1.5707, 0)

        matrix2 = matrix.fromTranslationRotation(
            (pose[0] * (-1), pose[1] * (-1), pose[2]), (q[0], q[1], q[2], q[3]))

        rospy.loginfo(matrix2)
        th = invKine(matrix2)
        sol1 = th[:, 0].transpose()
        joint_values_from_ik = np.array(sol1)

        joint_values = joint_values_from_ik[0, :]

        return joint_values.tolist()

    """
    Also gets each frame position through lookupTransform
    """

    def get_repulsive_cp(self, obs_pos, joint_values, CP_ur5_rep):

        marker_lines = Marker()

        ur5_links = [
            "upper_arm_link",
            "forearm_link",
            "wrist_1_link",
            "wrist_2_link",
            "wrist_3_link",
            "tool0"
        ]

        CP_pos, CP_dist = [], []

        for i in range(len(ur5_links)):
            link_pose = get_ur5_position(
                self.ur5_param, joint_values, ur5_links[i])
            CP_pos.append(link_pose)
            CP_inter = []
            # self.add_sphere2(link_pose, CP_ur5_rep, ColorRGBA(1.0, 0.0, 0.0, 0.5)) # Plot UR5 repulsive fields
            for y in range(len(obs_pos)):
                CP_inter.append(np.linalg.norm(link_pose - obs_pos[y]))
            CP_dist.append(CP_inter)
        return CP_pos, CP_dist

    """
    Adds the obstacles and repulsive control points on the robot
    """

    def add_sphere(self, pose, diam, color):
        marker = Marker()
        marker.header.frame_id = "base_link"
        for i in range(len(pose)):
            marker.id = self.id
            marker.pose.position = Point(pose[i][0], pose[i][1], pose[i][2])
            marker.type = marker.SPHERE
            marker.action = marker.ADD
            marker.scale = Vector3(diam[i], diam[i], diam[i])
            marker.color = color
            self.marker_publisher.publish(marker)
            self.id += 1

    """
    This function plot UR5 Repulsive Fields
    """

    def add_sphere2(self, pose, diam, color):
        marker = Marker()
        if self.id2 == 137:
            self.id2 = 130
        marker.header.frame_id = "base_link"
        marker.id = self.id2
        marker.pose.position = Point(pose[0], pose[1], pose[2])
        marker.type = marker.SPHERE
        marker.action = marker.MODIFY
        marker.scale = Vector3(diam, diam, diam)
        marker.color = color
        self.marker_publisher.publish(marker)
        self.id2 += 1

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
        self.marker.color = ColorRGBA(0.0, 0.0, 1.0, 0.8)
        self.marker_publisher.publish(self.marker)

    """
    Send final trajectory to gazebo or real UR5
    """

    def move(self, way_points, target):
        g = FollowJointTrajectoryGoal()
        g.trajectory = JointTrajectory()
        g.trajectory.joint_names = self.joint_states.name

        # for joint in range(len(way_points)):
        hz = get_param("rate", 30)  # 10hz
        r = rospy.Rate(hz)

        try:
            i = 0
            while not rospy.is_shutdown() and i < len(way_points):
                g.trajectory.points.append(JointTrajectoryPoint(positions=way_points[i],
                                                                velocities=[0] * 6,
                                                                time_from_start=rospy.Duration(0.1 * i + 1)))
                i += 1

            if target == "gazebo":
                self.client.send_goal(g)
                self.client.wait_for_result()
            elif target == "real":
                self.clientreal.send_goal(g)
                self.clientreal.wait_for_result()

        except KeyboardInterrupt:
            self.client.cancel_goal()
            self.clientreal.cancel_goal()
            raise
        except:
            raise


def main():
    ur5_robot = MoveGroupPythonIntefaceTutorial()
    way_points = []

    # UR5 Initial position
    raw_input("' =========== Aperte enter para posicionar o UR5 \n")
    # Posicao configurada no fake_controller_joint_states
    ur5_robot.joint_states.position = [0, -1.5707, 0, -1.5707, 1.5707, 0]
    way_points.append(ur5_robot.joint_states.position)
    ur5_robot.move(way_points, "gazebo")

    raw_input("' =========== Aperte enter para carregar os param. dos CPAs \n")

    # Obstacle positions
    oc = [-0.9, 0, 0.375]  # Obstacle reference point - 3D printer
    d1 = -0.080
    s = 1
    obs_pos = [oc, np.add(oc, [s * 0.14, 0.0925, 0.255 + d1]), np.add(oc, [s * 0.14, 0.185, 0.255 + d1]), np.add(oc, [s * 0.14, 0, 0.255 + d1]), np.add(oc, [s * 0.14, -0.0925, 0.255 + d1]), np.add(oc, [s * 0.14, -0.185, 0.255 + d1]),
               np.add(oc, [s * 0.14, -0.185, 0.16 + d1]), np.add(oc, [s * 0.14, 0.185, 0.16 + d1]), np.add(oc, [s * 0.14, 0.0925, 0.05 + d1]), np.add(oc, [s * 0.14, 0.185, 0.05 + d1]), np.add(oc, [s * 0.14, 0, 0.05 + d1]),
               np.add(oc, [s * 0.14, -0.0925, 0.05 + d1]), np.add(oc, [s * 0.14, -0.185, 0.05 + d1])]
    diam_obs = [0.18] * len(obs_pos)  # Main obstacle repulsive field
    diam_obs[0] = 0.3
    ur5_robot.add_sphere(obs_pos, diam_obs, ColorRGBA(1.0, 0.0, 0.0, 0.5))

    # Final position
    ptFinal = [[-0.9, 0, 0.45]]
    oriFinal = [0.01, 0.01, 0.01]
    diam_goal = [0.05]
    ur5_robot.add_sphere(ptFinal, diam_goal, ColorRGBA(0.0, 1.0, 0.0, 0.8))

    # CPA Parameters
    err = diam_goal[0] / 4  # Max error allowed
    max_iter = 2500  # Max iterations
    zeta = [0.5 for i in range(7)]  # Attractive force gain of each obstacle
    eta = [0.00006 for i in range(6)]  # Repulsive gain of each obstacle
    rho_0 = [i / 2 for i in diam_obs]  # Influence distance of each obstacle
    dist_att = 0.05  # Influence distance in workspace
    dist_att_config = 0.15  # Influence distance in configuration space
    alfa = 0.5  # Learning rate of positioning
    alfa_rot = 0.4  # Learning rate of orientation
    CP_ur5_rep = 0.15  # Repulsive fields on UR5

    # Parameters
    CPAA_state = True  # If True, it enables CPAA only on end effector
    Orientation_state = True  # If True, UR5 will keep end effector orientation

    # Get current orientation and position of tool0 link
    q = quaternion_from_matrix(ur5_robot.joint_states.position)
    oriAtual = q[1], q[2], q[3], q[0]
    ptAtual = get_ur5_position(
        ur5_robot.ur5_param, ur5_robot.joint_states.position, "tool0")

    hz = get_param("rate", 60)
    r = rospy.Rate(hz)

    dist_EOF_to_Goal = np.linalg.norm(ptAtual - np.asarray(ptFinal[0]))
    n = 0

    raw_input("' =========== Aperte enter para iniciar o algoritmo dos CPAs")
    while dist_EOF_to_Goal > err and not rospy.is_shutdown() and n < max_iter:
        # Get UR5 Jacobian of each link
        Jacobian = get_geometric_jacobian(
            ur5_robot.ur5_param, ur5_robot.joint_states.position)

        # Get position and distance from each link to each obstacle
        CP_pos, CP_dist = ur5_robot.get_repulsive_cp(
            obs_pos, ur5_robot.joint_states.position, CP_ur5_rep)

        # Get attractive linear and angular forces and repulsive forces
        joint_att_force_p, joint_att_force_w, joint_rep_force = CPA.get_joint_forces(ptAtual, ptFinal, oriAtual, oriFinal,
                                                                                     dist_EOF_to_Goal, Jacobian, ur5_robot.joint_states.position, ur5_robot.ur5_param, zeta,
                                                                                     eta, rho_0, dist_att, dist_att_config, CP_dist, CP_pos, obs_pos, CPAA_state, CP_ur5_rep)

        # Joint angles UPDATE - Attractive force
        ur5_robot.joint_states.position = ur5_robot.joint_states.position + \
            alfa * joint_att_force_p[0]
        if Orientation_state:
            ur5_robot.joint_states.position = ur5_robot.joint_states.position + \
                alfa_rot * joint_att_force_w[0]
        way_points.append(ur5_robot.joint_states.position)

        # Joint angles UPDATE - Repulsive force
        list = np.transpose(joint_rep_force[0]).tolist()
        for j in range(6):
            for i in range(6):
                ur5_robot.joint_states.position[i] = ur5_robot.joint_states.position[i] + \
                    alfa * list[j][i]

        # Get current orientation of tool0 link
        q = quaternion_from_matrix(ur5_robot.joint_states.position)
        oriAtual = q[1], q[2], q[3], q[0]

        # Get current position of tool0 link
        ptAtual = get_ur5_position(
            ur5_robot.ur5_param, ur5_robot.joint_states.position, "tool0")

        # Angle offset between tool0 and base_link (base?)
        oriAtual += quaternion_from_euler(1.5707, 1.5707, 0)

        # Get distance from EOF to goal
        dist_EOF_to_Goal = np.linalg.norm(ptAtual - np.asarray(ptFinal))

        if n % 10 == 0:
            ur5_robot.visualize_path_planned(ptAtual)
            # ur5_robot.add_sphere2(ptAtual, ur5_robot.id2, 0.04, ColorRGBA(0.0, 1.0, 0.0, 0.8)) # plot path as spheres

        try:
            r.sleep()
        except rospy.exceptions.ROSTimeMovedBackwardsException:
            pass

        n += 1

    print("Iterations: " + str(n))
    print("Distance to goal: " + str(dist_EOF_to_Goal))

    raw_input("' =========== Press enter to send the trajectory to Gazebo \n")
    ur5_robot.move(way_points, "gazebo")

    # UR5 Initial position
    raw_input("' =========== Aperte enter para posicionar o UR5 na posicao UP\n")
    ur5_robot.move(([0, -1.5707, 0, -1.5707, 0, 0],), "gazebo")

    # Uncomment this if you are working with real UR5
    #raw_input("' =========== Aperte enter para posicionar o UR5 real na posicao UP\n")
    #ur5_robot.move(([0, -1.5707, 0, -1.5707, 0, 0],), "real")

    # Uncomment this if you are working with real UR5
    #raw_input("' =========== Aperte enter para enviar a trajetoria para o UR5 !!!REAL!!! \n")
    #ur5_robot.move(way_points, "real")


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
    except KeyboardInterrupt:
        pass
