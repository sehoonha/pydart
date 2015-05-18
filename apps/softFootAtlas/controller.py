import numpy as np
from numpy.linalg import inv


class Controller:
    """ Add damping force to the skeleton """
    def __init__(self, skel, h):
        self.h = h
        self.skel = skel
        ndofs = self.skel.ndofs
        self.qhat = self.skel.q
        self.Kp = np.diagflat([0.0] * 6 + [800.0] * (ndofs - 6))
        self.Kd = np.diagflat([0.0] * 6 + [40.0] * (ndofs - 6))
        self.preoffset = 0.0

    def compute(self):
        skel = self.skel

        invM = inv(skel.M + self.Kd * self.h)
        p = -self.Kp.dot(skel.q + skel.qdot * self.h - self.qhat)
        d = -self.Kd.dot(skel.qdot)
        qddot = invM.dot(-skel.c + p + d + skel.constraint_forces())
        tau = p + d - self.Kd.dot(qddot) * self.h

        # Check the balance
        COP = 0.5 * (skel.body('l_foot').C + skel.body('l_foot').C)
        offset = skel.C[0] - COP[0]
        print self.skel.world.t, offset

        # # Adjust the target pose
        k1 = 200.0 if 0.0 < offset and offset < 0.10 else 2000.0
        k2 = 400.0
        kd = 10.0 if 0.0 < offset and offset < 0.10 else 100.0
        q_delta1 = np.array([-k1, -k2, -k1, -k2]) * offset
        q_delta2 = np.ones(4) * kd * (self.preoffset - offset)
        tau[np.array([13, 21, 14, 22])] -= (q_delta1 + q_delta2)
        self.preoffset = offset

        # Make sure the first six are zero
        tau[:6] = 0
        return tau
