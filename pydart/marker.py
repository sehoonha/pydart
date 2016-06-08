import numpy as np


class Marker(object):
    def __init__(self, _body, _id):
        self.body = _body
        self._id = _id

    @property
    def id(self):
        return self._id

    @property
    def x(self):
        return self.world_pos()

    def local_pos(self):
        return self.body.get_marker_local_pos(self._id)

    def world_pos(self):
        return self.body.get_marker_pos(self._id)

    def is_visible(self):
        return (np.linalg.norm(self.world_pos()) > 1e-4)

    def __repr__(self):
        # This is a quick and dirty method to check the contact list
        return "(M: %s/%d)" % (self.body.name, self.id)
