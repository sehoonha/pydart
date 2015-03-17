import numpy as np
from numpy.linalg import inv
from jacobian_transpose import JTController


class Controller:
    """ Add damping force to the skeleton """
    def __init__(self, skel, h):
        self.h = h
        self.skel = skel
        ndofs = self.skel.ndofs
        self.qhat = self.skel.q
        self.Kp = np.diagflat([0.0] * 6 + [600.0] * (ndofs - 6))
        self.Kd = np.diagflat([0.0] * 6 + [40.0] * (ndofs - 6))

        # Init target poses
        self.init_target_poses()
        # Jacobian transpose
        self.jt = JTController(self.skel)

    def init_target_poses(self):
        skel = self.skel
        I_thigh = skel.dof_indices(["j_thigh_left_z", "j_thigh_right_z"])
        I_shin = skel.dof_indices(["j_shin_left", "j_shin_right"])
        I_heel = skel.dof_indices(["j_heel_left_1", "j_heel_right_1"])

        pose0 = self.skel.q
        pose0[I_thigh] = 1.2
        pose0[I_shin] = -2.0
        pose0[I_heel] = 0.8
        pose0[28], pose0[31] = 0.5, -0.5  # Shoulder

        pose1 = self.skel.q
        pose1[28], pose1[31] = -2.0, 2.0  # Shoulder
        pose1[29], pose1[32] = 0.5, -0.5  # Shoulder

        pose2 = self.skel.q
        pose2[I_thigh] = 0.3  # Thighs

        self.target_poses = [pose0, pose1, pose2]
        self.target_times = [0.0, 0.4, 0.8]

    def update_target_pose(self):
        if len(self.target_times) == 0:
            return
        t = self.skel.world.t
        if t > self.target_times[0]:
            self.qhat = self.target_poses[0]
            print('update pose! at %.4lf' % t)
            self.target_poses.pop(0)
            self.target_times.pop(0)

    def compute(self):
        self.update_target_pose()
        skel = self.skel

        invM = inv(skel.M + self.Kd * self.h)
        p = -self.Kp.dot(skel.q + skel.qdot * self.h - self.qhat)
        d = -self.Kd.dot(skel.qdot)
        qddot = invM.dot(-skel.c + p + d + skel.constraint_forces())
        tau = p + d - self.Kd.dot(qddot) * self.h

        t = self.skel.world.t
        if 0.3 < t and t < 0.5:
            heels = ['h_heel_left', 'h_heel_right']
            vf = self.jt.apply(heels, [0, -700, 0])
            tau += vf

        # Make sure the first six are zero
        tau[:6] = 0
        return tau
