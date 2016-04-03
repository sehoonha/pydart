import pydart_api as papi
import numpy as np


class FileC3D(object):
    def __init__(self, xyz=None, sign=None, offset=None):
        self.data = None
        if xyz is None:
            self.xyz = np.array([0, 1, 2])
        else:
            self.xyz = np.array(xyz)

        if sign is None:
            self.sign = np.array([1.0, 1.0, 1.0])
        else:
            self.sign = np.array(sign)

        self.T = np.identity(4)
        if offset is not None:
            self.T[:3, 3] = offset
            # self.offset = np.array(offset)

    def load(self, path):
        nbuffer, _ = papi.readC3D(path, 0)
        assert(nbuffer > 0)
        ret, data = papi.readC3D(path, nbuffer)
        assert(ret == 0)
        nf = int(data[0])
        nm = int(data[1])
        self.data = data[2:].reshape((nf, nm * 3))
        # print self.num_frames(), self.num_markers()
        # for i in range(self.num_markers()):
        #     print i, self.marker(0, i)

    def set_rotation_Y(self, angle):
        sth = np.sin(angle)
        cth = np.cos(angle)
        self.T[:3, :3] = [[cth, 0.0, -sth],
                          [0.0, 1.0, 0.0],
                          [sth, 0.0, cth]]

    def set_translation(self, xyz):
        self.T[:3, 3] = xyz

    def num_frames(self):
        return self.data.shape[0]

    def num_markers(self):
        return self.data.shape[1] / 3

    def marker(self, frame, marker):
        x = self.data[frame][marker * 3:marker * 3 + 3]
        x = x[self.xyz]
        x = x * self.sign
        x = self.T.dot(np.concatenate([x, [1.0]]))
        return x[:3]

    def is_marker_visible(self, frame, marker):
        x = self.data[frame][marker * 3:marker * 3 + 3]
        return np.linalg.norm(x) > 1e-4
