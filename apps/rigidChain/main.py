import sys
import pydart
import numpy as np
print('Example: rigidChain')


class DampingController:
    """ Add damping force to the skeleton """
    def __init__(self, skel):
        self.skel = skel

    def compute(self):
        damping = -0.01 * self.skel.qdot
        for i in range(1, self.skel.ndofs, 3):
            damping[i] *= 0.1
        return damping


pydart.init()
print('pydart initialization OK')

data_dir = pydart.misc.example_data_dir(__file__)
print('data_dir = ' + data_dir)

world = pydart.create_world(1.0 / 2000.0, data_dir + '/skel/chain.skel')
print('pydart create_world OK')

skel = world.skels[0]
skel.q = (np.random.rand(skel.ndofs) - 0.5)
print('init pose = %s' % skel.q)
skel.controller = DampingController(skel)

if 'qt' in sys.argv:
    tb = pydart.qtgui.Trackball(phi=-0.0, theta=0.0, zoom=1.0,
                                rot=[-0.02, -0.71, -0.02, 0.71],
                                trans=[0.02, 0.09, -1.0])
    pydart.qtgui.run(title='rigidChain', simulation=world, trackball=tb)
else:
    pydart.glutgui.run(title='rigidChain', simulation=world)
