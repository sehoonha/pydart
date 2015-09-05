import numpy as np
from numpy.linalg import inv


class Controller:
    """ Add damping force to the skeleton """
    def __init__(self, skel, h):
        self.h = h
        self.skel = skel
        ndofs = self.skel.ndofs
        self.qhat = self.skel.q
        self.Kp = np.diagflat([0.0] * 6 + [400.0] * (ndofs - 6))
        self.Kd = np.diagflat([0.0] * 6 + [40.0] * (ndofs - 6))
        self.preoffset = 0.0

    def compute(self):
        skel = self.skel

        # apply SPD (Stable PD Control - Tan et al.)
        invM = inv(skel.M + self.Kd * self.h)
        p = -self.Kp.dot(skel.q + skel.qdot * self.h - self.qhat)
        d = -self.Kd.dot(skel.qdot)
        qddot = invM.dot(-skel.c + p + d + skel.constraint_forces())
        tau = p + d - self.Kd.dot(qddot) * self.h

        # Check the balance
        COP = skel.body('h_heel_left').to_world([0.05, 0, 0])
        offset = skel.C[0] - COP[0]
        preoffset = self.preoffset
        # print ("offset = %f, preoffset = %f" % (offset, preoffset))

        # Adjust the target pose -- translated from bipedStand app of DART
        foot = skel.dof_indices(["j_heel_left_1", "j_toe_left",
                                 "j_heel_right_1", "j_toe_right"])
        if 0.0 < offset < 0.1:
            k1, k2, kd = 200.0, 100.0, 10.0
            k = np.array([-k1, -k2, -k1, -k2])
            tau[foot] += k * offset + kd * (preoffset - offset) * np.ones(4)
            self.preoffset = offset
        elif -0.2 < offset < -0.05:
            k1, k2, kd = 2000.0, 100.0, 100.0
            k = np.array([-k1, -k2, -k1, -k2])
            tau[foot] += k * offset + kd * (preoffset - offset) * np.ones(4)
            self.preoffset = offset

        # Make sure the first six are zero
        tau[:6] = 0
        return tau
