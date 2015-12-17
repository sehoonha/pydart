import sys
import pydart
import numpy as np
print('Example: bipedStand')


pydart.init()
print('pydart initialization OK')

data_dir = pydart.misc.example_data_dir(__file__)
print('data_dir = ' + data_dir)

world = pydart.create_world(1.0 / 2000.0)
world.add_skeleton(data_dir + '/sdf/atlas/ground.urdf')
world.add_skeleton(data_dir + '/vsk/Yunseong.vsk')
print('pydart create_world OK')

skel = world.skels[1]

# Initialize the pose. q is an instance of SkelVector.
q = skel.q
q[4] = 0.05
q[39] = -1.5
q[55] = -q[39]
skel.set_positions(q)

for i, body in enumerate(skel.bodies):
    print i, body.name, body.num_markers()
for i, dof in enumerate(skel.dofs):
    print i, dof.name
for i, m in enumerate(skel.markers):
    print i, m, m.x
print('skeleton position OK')


def step_callback(world):
    pass


def keyboard_callback(world, key):
    """ Programmable interactions """
    global state
    if key == '1':
        state['Force'][0] = 500
        state['ImpulseDuration'] = 100
        print('push forward')
    elif key == '2':
        state['Force'][0] = -500
        state['ImpulseDuration'] = 100
        print('push backward')
    elif key == '3':
        state['Force'][2] = 500
        state['ImpulseDuration'] = 100
        print('push right')
    elif key == '4':
        state['Force'][2] = -500
        state['ImpulseDuration'] = 100
        print('push left')
    elif key == 's':
        print('save world')
        world.save('test_world.txt')


print("'1'--'4': programmed interaction")

# Run the application
if 'qt' in sys.argv:
    tb = pydart.qtgui.Trackball(phi=-1.4, theta=-6.2, zoom=1.0,
                                rot=[-0.05, 0.07, -0.01, 1.00],
                                trans=[0.02, 0.09, -3.69])
    pydart.qtgui.run(title='bipedStand', simulation=world, trackball=tb,
                     step_callback=step_callback,
                     keyboard_callback=keyboard_callback)
else:
    pydart.glutgui.run(title='bipedStand', simulation=world, trans=[0, 0, -3],
                       step_callback=step_callback,
                       keyboard_callback=keyboard_callback)
