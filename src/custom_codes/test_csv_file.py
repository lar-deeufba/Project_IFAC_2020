import numpy as np
import csv

ur5_joint_positions_vec = np.array([1, 2, 3, 4, 5, 6, 7])
ur5_joint_positions_vec = np.vstack((ur5_joint_positions_vec, np.array([1, 2, 3, 4, 5, 6, 8])))

with open('employee_file.csv', mode='w') as employee_file:
    employee_writer = csv.writer(employee_file, delimiter=' ', quotechar='"', quoting=csv.QUOTE_MINIMAL)
    employee_writer.writerow(['Index', 'Joint1', 'Joint2', 'Joint3', 'Joint4', 'Joint5', 'Joint6'])
    pos = ur5_joint_positions_vec
    n = 0
    print(len(pos))
    print(pos)
    for position in pos:
        print(position)
        print("\n")
        employee_writer.writerow([position[0], position[1], position[2], position[3], position[4], position[5], position[6]])
        n+=1
