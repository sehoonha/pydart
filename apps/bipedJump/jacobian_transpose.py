import numpy as np


class JTController:
    """
    # Usage
    self.jt = JTController(self.skel)
    tau += self.jt.apply( ["l_hand", "r_hand"], f )
    """
    def __init__(self, _skel):
        self.skel = _skel

    def apply(self, bodynames, f):
        if not isinstance(bodynames, list):
            bodynames = [bodynames]
        f = np.array(f)

        tau = np.zeros(self.skel.ndofs)
        for bodyname in bodynames:
            # J = self.skel.getBodyNodeWorldLinearJacobian(bodyname)
            J = self.skel.body(bodyname).world_linear_jacobian()
            JT = np.transpose(J)
            tau += JT.dot(f)
        return tau
