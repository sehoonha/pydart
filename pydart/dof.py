import pydart_api as papi


class Dof(object):
    def __init__(self, _skel, _id):
        self.skel = _skel
        self.id = _id
        self.name = papi.getSkeletonDofName(self.wid, self.sid, self.id)

    @property
    def wid(self):
        return self.skel.world.id

    @property
    def sid(self):
        return self.skel.id

    def __repr__(self):
        return '<Dof.%s at %d>' % (self.name, self.id)
