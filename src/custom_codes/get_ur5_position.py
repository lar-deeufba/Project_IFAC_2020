from numpy import array, cos, sin, ndarray

"""
Equations of each frame's position
Types:
    joint_values: list
    frame: string
Caso seja exigido que somente o efetuador final alcance a posicao Final,
nao eh necessario obter a posicao de todas as juntas
"""

def get_ur5_position(ur5_param, joint_values, frame):

    #PJ_i -> Posicao da junta i
    th1, th2, th3, th4, th5, th6  = joint_values
    d1, SO, EO, a2, a3, d4, d45, d5, d6 = ur5_param

    if frame in ['shoulder_link', 'all']:
        PJ_1 = array([0, 0, d1])
        if frame in ['shoulder_link']:
            return PJ_1

    if frame in ['upper_arm_link', 'all']:
        PJ_2 = array([-SO*sin(th1),
                       SO*cos(th1),
                               d1])
        if frame in ['upper_arm_link']:
            return PJ_2

    if frame in ['forearm_link', 'all']:
        PJ_3 = array([-EO*sin(th1) - SO*sin(th1) + a2*cos(th1)*cos(th2),
                 EO*cos(th1) + SO*cos(th1) + a2*sin(th1)*cos(th2),
                -a2*sin(th2) + d1])
        if frame in ['forearm_link']:
            return PJ_3

    if frame in ['wrist_1_link', 'all']:
        PJ_4 = array([-EO*sin(th1) - SO*sin(th1) + a2*cos(th1)*cos(th2) + a3*(-sin(th2)*sin(th3)*cos(th1) + cos(th1)*cos(th2)*cos(th3)),
                 EO*cos(th1) + SO*cos(th1) + a2*sin(th1)*cos(th2) + a3*(-sin(th1)*sin(th2)*sin(th3) + sin(th1)*cos(th2)*cos(th3)),
                -a2*sin(th2) + a3*(-sin(th2)*cos(th3) - sin(th3)*cos(th2)) + d1])
        if frame in ['wrist_1_link']:
            return PJ_4

    if frame in ['wrist_2_link', 'all']:
        PJ_5 = array([-EO*sin(th1) - SO*sin(th1) + a2*cos(th1)*cos(th2) + a3*(-sin(th2)*sin(th3)*cos(th1) + cos(th1)*cos(th2)*cos(th3)) - d45*sin(th1),
                 EO*cos(th1) + SO*cos(th1) + a2*sin(th1)*cos(th2) + a3*(-sin(th1)*sin(th2)*sin(th3) + sin(th1)*cos(th2)*cos(th3)) + d45*cos(th1),
                -a2*sin(th2) + a3*(-sin(th2)*cos(th3) - sin(th3)*cos(th2)) + d1])
        if frame in ['wrist_2_link']:
            return PJ_5

    if frame in ['wrist_3_link', 'all']:
        PJ_6 = array([-EO*sin(th1) - SO*sin(th1) + a2*cos(th1)*cos(th2) + a3*cos(th1)*cos(th2 + th3) - d45*sin(th1) - d5*sin(th2 + th3 + th4)*cos(th1),
                 EO*cos(th1) + SO*cos(th1) + a2*sin(th1)*cos(th2) + a3*sin(th1)*cos(th2 + th3) + d45*cos(th1) - d5*sin(th1)*sin(th2 + th3 + th4),
                -a2*sin(th2) - a3*sin(th2 + th3) + d1 - d5*cos(th2 + th3 + th4)])
        if frame == "wrist_3_link":
            return PJ_6

    if frame in ['tool0', 'all']:
        PJ_7 = array([-EO*sin(th1) - SO*sin(th1) + a2*cos(th1)*cos(th2) + a3*cos(th1)*cos(th2 + th3) - d45*sin(th1) - d5*sin(th2 + th3 + th4)*cos(th1) - d6*(sin(th1)*cos(th5) - sin(th5)*cos(th1)*cos(th2 + th3 + th4)),
                 EO*cos(th1) + SO*cos(th1) + a2*sin(th1)*cos(th2) + a3*sin(th1)*cos(th2 + th3) + d45*cos(th1) - d5*sin(th1)*sin(th2 + th3 + th4) + d6*(sin(th1)*sin(th5)*cos(th2 + th3 + th4) + cos(th1)*cos(th5)),
                -a2*sin(th2) - a3*sin(th2 + th3) + d1 - d5*cos(th2 + th3 + th4) - d6*sin(th5)*sin(th2 + th3 + th4)])
        if frame in ['tool0']:
            return PJ_7

    if frame in ['all']:
        return [PJ_1, PJ_2, PJ_3, PJ_4, PJ_5, PJ_6, PJ_7]
