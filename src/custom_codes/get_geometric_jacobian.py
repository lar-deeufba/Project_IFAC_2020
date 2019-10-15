from numpy import array, cos, sin
import numpy as np

"""
Anallytical jacobian
type:
    ur5_param: list
    joint_values: list
"""
def get_geometric_jacobian(ur5_param, joint_values):

    th1, th2, th3, th4, th5, th6 = joint_values
    d1, SO, EO, a2, a3, d4, d45, d5, d6 = ur5_param

    # J_1 = array([[0, 0, 0, 0, 0, 0, 0],
    #              [0, 0, 0, 0, 0, 0, 0],
    #              [0, 0, 0, 0, 0, 0, 0],
    #              [0, 0, 0, 0, 0, 0, 0],
    #              [1, 0, 0, 0, 0, 0, 0],
    #              [0, 0, 0, 0, 0, 0, 0]])

    J_2_T = ([[-SO*cos(th1), -SO*sin(th1), 0],
            [0, 0, 0],
            [0, 0, 0],
            [0, 0, 0],
            [0, 0, 0],
            [0, 0, 0]])


    J_3_T = ([[-EO*cos(th1) - SO*cos(th1) - a2*sin(th1)*cos(th2), -EO*sin(th1) - SO*sin(th1) + a2*cos(th1)*cos(th2), 0],
            [-a2*sin(th2)*cos(th1), -a2*sin(th1)*cos(th2), -a2*cos(th2)],
            [0, 0, 0],
            [0, 0, 0],
            [0, 0, 0],
            [0, 0, 0]])


    J_4_T = ([[-EO*cos(th1) - SO*cos(th1) - a2*sin(th1)*cos(th2) - a3*sin(th1)*cos(th2 + th3), -EO*sin(th1) - SO*sin(th1) + a2*cos(th1)*cos(th2) + a3*cos(th1)*cos(th2 + th3), 0],
            [(-a2*sin(th2) - a3*sin(th2 + th3))*cos(th1), (-a2*sin(th2) - a3*sin(th2 + th3))*sin(th1), -a2*cos(th2) - a3*cos(th2 + th3)],
            [-a3*sin(th2 + th3)*cos(th1), -a3*sin(th1)*sin(th2 + th3), -a3*cos(th2 + th3)],
            [0, 0, 0],
            [0, 0, 0],
            [0, 0, 0]])


    J_5_T = ([[-EO*cos(th1) - SO*cos(th1) - a2*sin(th1)*cos(th2) - a3*sin(th1)*cos(th2 + th3) - d45*cos(th1), -EO*sin(th1) - SO*sin(th1) + a2*cos(th1)*cos(th2) + a3*cos(th1)*cos(th2 + th3) - d45*sin(th1), 0],
            [(-a2*sin(th2) - a3*sin(th2 + th3))*cos(th1), (-a2*sin(th2) - a3*sin(th2 + th3))*sin(th1), -a2*cos(th2) - a3*cos(th2 + th3)],
            [-a3*sin(th2 + th3)*cos(th1), -a3*sin(th1)*sin(th2 + th3), -a3*cos(th2 + th3)],
            [0, 0, 0],
            [0, 0, 0],
            [0, 0, 0]])


    J_6_p_T = ([[-EO*cos(th1) - SO*cos(th1) - a2*sin(th1)*cos(th2) - a3*sin(th1)*cos(th2 + th3) - d45*cos(th1) + d5*sin(th1)*sin(th2 + th3 + th4), -EO*sin(th1) - SO*sin(th1) + a2*cos(th1)*cos(th2) + a3*cos(th1)*cos(th2 + th3) - d45*sin(th1) - d5*sin(th2 + th3 + th4)*cos(th1), 0],
              [(-a2*sin(th2) - a3*sin(th2 + th3) - d5*cos(th2 + th3 + th4))*cos(th1), (-a2*sin(th2) - a3*sin(th2 + th3) - d5*cos(th2 + th3 + th4))*sin(th1), -a2*cos(th2) - a3*cos(th2 + th3) + d5*sin(th2 + th3 + th4)],
              [(-a3*sin(th2 + th3) - d5*cos(th2 + th3 + th4))*cos(th1), (-a3*sin(th2 + th3) - d5*cos(th2 + th3 + th4))*sin(th1), -a3*cos(th2 + th3) + d5*sin(th2 + th3 + th4)],
              [-d5*cos(th1)*cos(th2 + th3 + th4), -d5*sin(th1)*cos(th2 + th3 + th4), d5*sin(th2 + th3 + th4)],
              [0, 0, 0],
              [0, 0, 0]])

    gain_3 = 2.0
    J_7_T = ([[-EO*cos(th1) - SO*cos(th1) - a2*sin(th1)*cos(th2) - a3*sin(th1)*cos(th2 + th3) - d45*cos(th1) + d5*sin(th1)*sin(th2 + th3 + th4) - d6*(sin(th1)*sin(th5)*cos(th2 + th3 + th4) + cos(th1)*cos(th5)), -EO*sin(th1) - SO*sin(th1) + a2*cos(th1)*cos(th2) + a3*cos(th1)*cos(th2 + th3) - d45*sin(th1) - d5*sin(th2 + th3 + th4)*cos(th1) - d6*(sin(th1)*cos(th5) - sin(th5)*cos(th1)*cos(th2 + th3 + th4)), 0],
            [(-a2*sin(th2) - a3*sin(th2 + th3) - d5*cos(th2 + th3 + th4) - d6*sin(th5)*sin(th2 + th3 + th4))*cos(th1), (-a2*sin(th2) - a3*sin(th2 + th3) - d5*cos(th2 + th3 + th4) - d6*sin(th5)*sin(th2 + th3 + th4))*sin(th1), -a2*cos(th2) - a3*cos(th2 + th3) + d5*sin(th2 + th3 + th4) - d6*sin(th5)*cos(th2 + th3 + th4)],
            [((-a3*sin(th2 + th3) - d5*cos(th2 + th3 + th4) - d6*sin(th5)*sin(th2 + th3 + th4))*cos(th1))*gain_3, ((-a3*sin(th2 + th3) - d5*cos(th2 + th3 + th4) - d6*sin(th5)*sin(th2 + th3 + th4))*sin(th1))*gain_3, (-a3*cos(th2 + th3) + d5*sin(th2 + th3 + th4) - d6*sin(th5)*cos(th2 + th3 + th4))*gain_3],
            [(-d5*cos(th2 + th3 + th4) - d6*sin(th5)*sin(th2 + th3 + th4))*cos(th1), (-d5*cos(th2 + th3 + th4) - d6*sin(th5)*sin(th2 + th3 + th4))*sin(th1), d5*sin(th2 + th3 + th4) - d6*sin(th5)*cos(th2 + th3 + th4)],
            [d6*(sin(th1)*sin(th5) + cos(th1)*cos(th5)*cos(th2 + th3 + th4)), d6*(sin(th1)*cos(th5)*cos(th2 + th3 + th4) - sin(th5)*cos(th1)), -d6*sin(th2 + th3 + th4)*cos(th5)],
            [0, 0, 0]])


    J_6_w_T = ([[0, 1, 0],
             [-sin(th1), cos(th1), 0],
             [-sin(th1), cos(th1), 0],
             [-sin(th1), cos(th1), 0],
             [-sin(th2 + th3 + th4)*cos(th1), -sin(th1)*sin(th2 + th3 + th4), -cos(th2 + th3 + th4)],
             [-sin(th1)*cos(th5) + sin(th5)*cos(th1)*cos(th2 + th3 + th4), sin(th1)*sin(th5)*cos(th2 + th3 + th4) + cos(th1)*cos(th5), -sin(th5)*sin(th2 + th3 + th4)]])

    return (J_2_T, J_3_T, J_4_T, J_5_T, J_6_p_T, J_7_T, J_6_w_T)
