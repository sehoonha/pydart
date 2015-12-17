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

        if offset is None:
            self.offset = np.array([0.0, 0.0, 0.0])
        else:
            self.offset = np.array(offset)

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

    def num_frames(self):
        return self.data.shape[0]

    def num_markers(self):
        return self.data.shape[1] / 3

    def marker(self, frame, marker):
        x = self.data[frame][marker * 3:marker * 3 + 3]
        x = x[self.xyz]
        x = x * self.sign
        x = x + self.offset
        return x

    def is_marker_visible(self, frame, marker):
        x = self.marker(frame, marker)
        return np.linalg.norm(x) < 1e-4
