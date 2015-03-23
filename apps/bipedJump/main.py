import sys
import pydart
import math
import controller
print('Example: bipedJump')


pydart.init()
print('pydart initialization OK')

data_dir = pydart.misc.example_data_dir(__file__)
print('data_dir = ' + data_dir)

world = pydart.create_world(1.0 / 2000.0, data_dir + '/skel/fullbody1.skel')
print('pydart create_world OK')

skel = world.skels[1]

# Initialize the pose
q = skel.q
q[(2, 4, 5)] = [0.02 * math.pi, -0.02, 0]
skel.set_positions(q)
print('skeleton position OK')

# Initialize the controller
skel.controller = controller.Controller(skel, world.dt)
print('create controller OK')


def keyboard_callback(world, key):
    """ Programmable interactions """
    if key == 'S':
        print('save world')
        world.save('test_world.txt')


print("'1'--'4': programmed interaction")

# Run the application
if 'qt' in sys.argv:
    tb = pydart.qtgui.Trackball(phi=-1.4, theta=-6.2, zoom=1.0,
                                rot=[-0.05, 0.07, -0.01, 1.00],
                                trans=[0.02, 0.09, -3.69])
    pydart.qtgui.run(title='bipedStand', simulation=world, trackball=tb,
                     keyboard_callback=keyboard_callback)
else:
    pydart.glutgui.run(title='bipedStand', simulation=world, trans=[0, 0, -3],
                       keyboard_callback=keyboard_callback)
