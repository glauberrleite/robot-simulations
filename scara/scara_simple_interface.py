import numpy as np

def invkine(x,y,z,phi, q_current):
    q = np.zeros(4)
    l1 = 0.425
    l2 = 0.345

    if z > 0.44:
        print('z is too high')
        return None
    
    q[2] = -z + 0.44

    #... 
    c2 = (x**2 + y**2 - l1**2 - l2**2)/(2*l1*l2)
    if c2 < -1 or c2 > 1:
        print('setpoint is out of range')
        return None
    s2_1 = np.sqrt(1 - c2**2)
    s2_2 = -s2_1

    theta_2_1 = np.arctan2(s2_1, c2) # elbow up
    theta_2_2 = np.arctan2(s2_2, c2) # elbow down

    k_1 = l1 + l2*c2
    
    k_2_1 = l2*s2_1
    k_2_2 = l2*s2_2

    theta_1_1 = np.arctan2(y, x) - np.arctan2(k_2_1, k_1) # elbow up
    theta_1_2 = np.arctan2(y, x) - np.arctan2(k_2_2, k_1) # elbow down

    theta_4_1 = phi - theta_1_1 - theta_2_1
    theta_4_2 = phi - theta_1_2 - theta_2_2
    #...

    q[3] = phi - q[0] - q[1]

    candidate_1 = np.array([theta_1_1, theta_2_1, q[2], theta_4_1])
    candidate_2 = np.array([theta_1_2, theta_2_2, q[2], theta_4_2])

    if (np.linalg.norm(candidate_1 - q_current) < np.linalg.norm(candidate_2 - q_current)):
        q[0] = theta_1_1
        q[1] = theta_2_1
        q[3] = theta_4_1
    else:
        q[0] = theta_1_2
        q[1] = theta_2_2
        q[3] = theta_4_2

    return q

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

def sysCall_actuation():
    target_q = invkine(self.pose_setpoint[0], self.pose_setpoint[1], self.pose_setpoint[2], self.pose_setpoint[3], self.q_current)
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