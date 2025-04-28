import numpy as np

def fkine(q):
    # Compute σ1, σ2, σ3
    sigma_2 = np.cos(q[0]) * np.cos(q[1]) - np.sin(q[0]) * np.sin(q[1])
    sigma_3 = np.cos(q[0]) * np.sin(q[1]) + np.cos(q[1]) * np.sin(q[0])
    sigma_1 = np.cos(q[3]) * sigma_3 - np.sin(q[3]) * sigma_2

    # Compute the transformation matrix
    matrix = np.array([
        [np.cos(q[3]) * sigma_2 + np.sin(q[3]) * sigma_3, sigma_1, 0, (17 * np.cos(q[0]) / 40) + (69 * np.cos(q[0]) * np.cos(q[1]) / 200) - (69 * np.sin(q[0]) * np.sin(q[1]) / 200)],
        [sigma_1, -np.cos(q[3]) * sigma_2 - np.sin(q[3]) * sigma_3, 0, (17 * np.sin(q[0]) / 40) + (69 * np.cos(q[0]) * np.sin(q[1]) / 200) + (69 * np.cos(q[1]) * np.sin(q[0]) / 200)],
        [0, 0, -1, 11 / 25 - q[2]],  # Assuming q3 is a scalar, not conjugated.
        [0, 0, 0, 1]
    ])

    return matrix

def jacobian(q):
    # Compute the Jacobian matrix for the SCARA robot
    J = np.zeros((4, 4))  # Jacobian matrix for a 4-DOF SCARA robot
    J[0, 0] = -np.sin(q[0]) * ((17 / 40) + (69 / 200) * (np.cos(q[1]) - np.sin(q[1])))
    J[0, 1] = -(69 / 200) * (np.sin(q[0]) * np.sin(q[1]) + np.sin(q[0]) * np.cos(q[1]))
    J[1, 0] = np.cos(q[0]) * ((17 / 40) + (69 / 200) * (np.cos(q[1]) - np.sin(q[1])))
    J[1, 1] = -(69 / 200) * (np.cos(q[0]) * np.sin(q[1]) + np.cos(q[0]) * np.cos(q[1]))
    J[2, 2] = -1  # Translation along z-axis
    J[3, 3] = 1   # Rotation about z-axis

    return J

def sysCall_init():
    sim = require('sim')

    self.joint_hdls = []
    self.joint_hdls.append(sim.getObject('../joint1'))
    self.joint_hdls.append(sim.getObject('../joint2'))
    self.joint_hdls.append(sim.getObject('../joint3'))
    self.joint_hdls.append(sim.getObject('../joint4'))

    for hdl in self.joint_hdls:
        sim.setJointPosition(hdl, 0.0)
        sim.setJointTargetPosition(hdl, 0.0)
    
    self.setpoint_hdl = sim.getObject('../setpoint')
    self.end_effector_hdl = sim.getObject('../tool0_visual')

    self.q_current = np.zeros(len(self.joint_hdls))
    self.pose_setpoint = np.array(sim.getObjectPose(self.setpoint_hdl, -1))

    self.pose_end_effector = np.array(sim.getObjectPose(self.end_effector_hdl, -1))
    

def sysCall_actuation():
    target_dq = np.zeros(4)
    dx = self.pose_setpoint[0] - self.pose_end_effector[0]
    dy = self.pose_setpoint[1] - self.pose_end_effector[1]
    dz = self.pose_setpoint[2] - self.pose_end_effector[2]
    dphi = self.pose_setpoint[3] - self.pose_end_effector[3]

    dX = np.array([[dx], [dy], [dz], [dphi]])
    J = jacobian(self.q_current)
    J_inv = np.linalg.pinv(J)  # Pseudoinverse of the Jacobian
    print(J.shape)
    target_dq = J_inv @ dX  # Compute the joint velocity
    target_q = self.q_current + target_dq.ravel()  # Update the joint angles
    print(target_q)
    for i, hdl in enumerate(self.joint_hdls):
        sim.setJointTargetPosition(hdl, target_q[i])

def sysCall_sensing():
    self.q_current = np.zeros(4)
    for i, hdl in enumerate(self.joint_hdls):
        self.q_current[i] = sim.getJointPosition(hdl)

    self.pose_setpoint = np.array(sim.getObjectPose(self.setpoint_hdl, -1))
    self.pose_end_effector = np.array(sim.getObjectPose(self.end_effector_hdl, -1))
    print(f'Translation error: {self.pose_setpoint[0:3] - self.pose_end_effector[0:3]}')
    print(f'Rotation error: {self.pose_setpoint[3:6] - self.pose_end_effector[3:6]}')

def sysCall_cleanup():
    # do some clean-up here
    pass