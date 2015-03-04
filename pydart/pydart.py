"""
- Rule for properties
1. Shortcuts: q, qdot, tau, t, ...
2. Numbers: ndofs, nframes, ...
"""

import os.path
import pydart_api as papi
import numpy as np


def init():
    papi.init()


def create_world(step, skel_path=None):
    return World(step, skel_path)


class World(object):
    def __init__(self, step, skel_path=None):
        self.skels = []
        self.control_skel = None
        if skel_path is not None:
            self.id = papi.createWorldFromSkel(skel_path)
            nskels = self.num_skeletons()
            for i in range(nskels):
                self.add_skeleton_from_id(i, (i == nskels - 1))
        else:
            self.id = papi.createWorld(step)

    def add_skeleton(self, filename, friction=1.0, control=True):
        self.skels += [Skeleton(self, filename, friction)]
        if control:
            self.control_skel = self.skels[-1]

    def add_skeleton_from_id(self, _skel_id, control=True):
        self.skels += [Skeleton(self, None, None, _skel_id)]
        if control:
            self.control_skel = self.skels[-1]

    def num_skeletons(self):
        return papi.numSkeletons(self.id)

    @property
    def skel(self):
        """ returns the default control skeleton """
        return self.control_skel

    def time(self):
        return papi.getWorldTime(self.id)

    @property
    def t(self):
        return self.time()

    def time_step(self):
        return papi.getWorldTimeStep(self.id)

    @property
    def dt(self):
        return self.time_step()

    def set_time_step(self, _time_step):
        papi.setWorldTimeStep(self.id, _time_step)

    @dt.setter
    def dt(self, _dt):
        self.set_time_step(_dt)

    def num_frames(self):
        return papi.getWorldSimFrames(self.id)

    @property
    def nframes(self):
        return self.num_frames()

    def contacts(self):
        n = papi.getWorldNumContacts(self.id)
        contacts = papi.getWorldContacts(self.id, 7 * n)
        return [contacts[7 * i: 7 * (i + 1)] for i in range(n)]

    def reset(self):
        papi.resetWorld(self.id)

    def step(self):
        papi.stepWorld(self.id)

    def set_frame(self, i):
        papi.setWorldSimFrame(self.id, i)

    def render(self):
        papi.render(self.id)

    def __repr__(self):
        return "<World.%d at %.4f>" % (self.id, self.t)


class Skeleton(object):
    def __init__(self, _world, _filename=None, _friction=None, _id=None):
        self.world = _world
        self.filename = _filename
        self.friction = _friction
        if self.filename is not None:
            self.id = papi.addSkeleton(self.world.id, _filename, _friction)
        else:
            self.id = _id

        # Initialize dofs
        _ndofs = papi.getSkeletonNumDofs(self.world.id, self.id)
        self.dofs = [Dof(self, i) for i in range(_ndofs)]
        self.name_to_dof = {dof.name: dof for dof in self.dofs}

        # Initialize bodies
        _nbodies = papi.getSkeletonNumBodies(self.world.id, self.id)
        self.bodies = [Body(self, i) for i in range(_nbodies)]
        self.name_to_body = {body.name: body for body in self.bodies}

    def set_joint_damping(self, _damping):
        papi.setSkeletonJointDamping(self.world.id, self.id, _damping)

    def num_dofs(self):
        return len(self.dofs)

    @property
    def ndofs(self):
        return self.num_dofs()

    def num_bodies(self):
        return len(self.bodies)

    @property
    def nbodies(self):
        return self.num_bodies()

    def mass(self):
        return papi.getSkeletonMass(self.world.id, self.id)

    @property
    def m(self):
        return self.mass()

    def mass_matrix(self):
        M = np.zeros((self.ndofs, self.ndofs))
        papi.getSkeletonMassMatrix(self.world.id, self.id, M)
        return M

    @property
    def M(self):
        return self.mass_matrix()

    def positions(self):
        return papi.getSkeletonPositions(self.world.id, self.id, self.ndofs)

    @property
    def q(self):
        return self.positions()

    def set_positions(self, _q):
        papi.setSkeletonPositions(self.world.id, self.id, _q)

    @q.setter
    def q(self, _q):
        """ Setter also updates the internal skeleton kinematics """
        self.set_positions(_q)

    def position_lower_limit(self):
        return papi.getSkeletonPositionLowerLimit(self.world.id,
                                                  self.id, self.ndofs)

    def position_upper_limit(self):
        return papi.getSkeletonPositionUpperLimit(self.world.id,
                                                  self.id, self.ndofs)

    @property
    def q_lo(self):
        return self.position_lower_limit()

    @property
    def q_hi(self):
        return self.position_upper_limit()

    def velocities(self):
        return papi.getSkeletonVelocities(self.world.id, self.id, self.ndofs)

    @property
    def qdot(self):
        return self.velocities()

    def set_velocities(self, _qdot):
        papi.setSkeletonVelocities(self.world.id, self.id, _qdot)

    @qdot.setter
    def qdot(self, _qdot):
        """ Setter also updates the internal skeleton kinematics """
        self.set_velocities(_qdot)

    def states(self):
        return np.concatenate((self.positions(), self.velocities()))

    @property
    def x(self):
        return np.concatenate((self.positions(), self.velocities()))

    def set_states(self, _x):
        self.set_positions(_x[:self.ndofs])
        self.set_velocities(_x[self.ndofs:])

    @x.setter
    def x(self, _x):
        self.set_states(_x)

    def coriolis_and_gravity_forces(self):
        return papi.getSkeletonCoriolisAndGravityForces(self.world.id,
                                                        self.id, self.ndofs)

    @property
    def c(self):
        return self.coriolis_and_gravity_forces()

    def constraint_forces(self):
        return papi.getSkeletonConstraintForces(self.world.id,
                                                self.id, self.ndofs)

    def body(self, query):
        if isinstance(query, str):
            return self.name_to_body[query]
        elif isinstance(query, int):
            return self.bodies[query]
        else:
            print 'No find...', query
            return None

    def body_index(self, _name):
        return self.name_to_body[_name].id

    def dof_index(self, _name):
        return self.name_to_dof[_name].id

    def world_com(self):
        return papi.getSkeletonWorldCOM(self.world.id, self.id)

    @property
    def C(self):
        return self.world_com()

    @property
    def COM(self):
        return self.world_com()

    def world_com_velocity(self):
        return papi.getSkeletonWorldCOMVelocity(self.world.id, self.id)

    @property
    def Cdot(self):
        return self.world_com_velocity()

    def linear_momentum(self):
        return self.Cdot * self.m

    @property
    def P(self):
        return self.linear_momentum()

    def forces(self):
        return self._tau

    @property
    def tau(self):
        return self.forces()

    def set_forces(self, _tau):
        self._tau = _tau
        papi.setSkeletonForces(self.world.id, self.id, _tau)

    @tau.setter
    def tau(self, _tau):
        self.set_forces(_tau)

    def force_lower_limit(self):
        return papi.getSkeletonForceLowerLimit(self.world.id,
                                               self.id, self.ndofs)

    def force_upper_limit(self):
        return papi.getSkeletonForceUpperLimit(self.world.id,
                                               self.id, self.ndofs)

    @property
    def tau_lo(self):
        return self.force_lower_limit()

    @property
    def tau_hi(self):
        return self.force_upper_limit()

    def approx_inertia(self, axis):
        """Calculates the point-masses approximated inertia
        with respect to the given axis """
        axis = np.array(axis) / np.linalg.norm(axis)
        I = 0
        C = self.C
        for body in self.bodies:
            d = body.C - C
            # Subtract the distance along the axis
            r_sq = np.linalg.norm(d) ** 2 - np.linalg.norm(d.dot(axis)) ** 2
            I += body.m * r_sq
        return I

    def approx_inertia_x(self):
        return self.approx_inertia([1, 0, 0])

    def approx_inertia_y(self):
        return self.approx_inertia([0, 1, 0])

    def approx_inertia_z(self):
        return self.approx_inertia([0, 0, 1])

    def external_contacts_and_body_id(self):
        cid_cnt = dict()
        contacts = []
        for body in self.bodies:
            for c in body.contacts():
                contacts += [(c, body.id)]
                cid = int(c[6])
                if cid not in cid_cnt:
                    cid_cnt[cid] = 1
                else:
                    cid_cnt[cid] += 1
        return [(c, bid) for (c, bid) in contacts if cid_cnt[int(c[6])] < 2]

    def contacted_bodies(self):
        return [body for body in self.bodies if body.num_contacts() > 0]

    def world_cop(self):
        bodies = self.contacted_bodies()
        if len(bodies) == 0:
            return None
        pos_list = [b.C for b in bodies]
        avg = sum(pos_list) / len(pos_list)
        return avg

    @property
    def COP(self):
        return self.world_cop()

    def contacted_body_names(self):
        return [body.name for body in self.contacted_bodies()]

    def render(self):
        papi.renderSkeleton(self.world.id, self.id)

    def render_with_color(self, r, g, b, a=1.0):
        papi.renderSkeletonWithColor(self.world.id, self.id, r, g, b, a)

    def __repr__(self):
        return '<Skel.%d.%s>' % (self.id, os.path.basename(self.filename))


class Body(object):
    def __init__(self, _skel, _id):
        self.skel = _skel
        self._id = _id
        self.name = papi.getSkeletonBodyName(self.wid, self.sid, self.id)

    @property
    def id(self):
        return self._id

    @property
    def wid(self):
        return self.skel.world.id

    @property
    def sid(self):
        return self.skel.id

    def num_contacts(self):
        return papi.getBodyNodeNumContacts(self.wid, self.sid, self.id)

    def contacts(self):
        n = self.num_contacts()
        contacts = papi.getBodyNodeContacts(self.wid, self.sid, self.id, 7 * n)
        return [contacts[7 * i: 7 * (i + 1)] for i in range(n)]

    def mass(self):
        return papi.getBodyNodeMass(self.wid, self.sid, self.id)

    @property
    def m(self):
        return self.mass()

    def inertia(self):
        return papi.getBodyNodeInertia(self.wid, self.sid, self.id)

    @property
    def I(self):
        return self.inertia()

    def local_com(self):
        return papi.getBodyNodeLocalCOM(self.wid, self.sid, self.id)

    def world_com(self):
        return papi.getBodyNodeWorldCOM(self.wid, self.sid, self.id)

    @property
    def C(self):
        return self.world_com()

    def world_com_velocity(self):
        return papi.getBodyNodeWorldCOMVelocity(self.wid, self.sid, self.id)

    @property
    def Cdot(self):
        return self.world_com_velocity()

    def transformation(self):
        return papi.getBodyNodeTransformation(self.wid, self.sid, self.id)

    @property
    def T(self):
        return self.transformation()

    def world_linear_jacobian(self, offset=None):
        if offset is None:
            offset = np.zeros(3)
        J = np.zeros((3, self.skel.ndofs))
        papi.getBodyNodeWorldLinearJacobian(self.wid, self.sid,
                                            self.id, offset, J)
        return J

    @property
    def J(self):
        return self.world_linear_jacobian()

    def add_ext_force(self, f):
        papi.addBodyNodeExtForce(self.wid, self.sid, self.id, f)

    def add_ext_force_at(self, f, offset):
        papi.addBodyNodeExtForceAt(self.wid, self.sid, self.id, f, offset)

    def __repr__(self):
        return '<Body.%d.%s>' % (self.id, self.name)


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
