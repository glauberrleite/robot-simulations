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

def cost_function(q, target_position):
    """Compute the cost function (distance between the end effector and target)."""
    fk_result = fkine(q)
    end_effector_pos = fk_result[:3, 3]  # Extract position (x, y, z)
    return np.linalg.norm(end_effector_pos - target_position)

def gradient_descent(cost_function, q_initial, target_position, learning_rate=0.01, tolerance=1e-6, max_iters=1000):
    """Minimize the cost function using gradient descent."""
    q = np.copy(q_initial)
    for _ in range(max_iters):
        # Compute the gradient using numerical differentiation
        grad = np.zeros_like(q)
        epsilon = 1e-6  # Small perturbation for numerical gradient
        
        for i in range(len(q)):
            q_plus = np.copy(q)
            q_plus[i] += epsilon
            q_minus = np.copy(q)
            q_minus[i] -= epsilon
            
            grad[i] = (cost_function(q_plus, target_position) - cost_function(q_minus, target_position)) / (2 * epsilon)
        
        # Update the parameters
        q -= learning_rate * grad
        
        # Check if the cost function is below the tolerance
        if np.linalg.norm(grad) < tolerance:
            break
    
    return q

def invkine(x, y, z, phi, q_current):
    target_position = np.array([x, y, z])
    q_initial = np.copy(q_current)  # Start with current configuration
    # Use gradient descent to find the joint angles that minimize the cost
    q = gradient_descent(cost_function, q_initial, target_position)
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