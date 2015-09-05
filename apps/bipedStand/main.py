import sys
import pydart
import numpy as np
import controller
print('Example: bipedStand')

pydart.init()
print('pydart initialization OK')

data_dir = pydart.misc.example_data_dir(__file__)
print('data_dir = ' + data_dir)

world = pydart.create_world(1.0 / 2000.0, data_dir + '/skel/fullbody1.skel')
# world = pydart.create_world(1.0 / 2000.0,
#                             data_dir + '/skel/soft_fullbody.skel')
print('pydart create_world OK')

skel = world.skels[1]

# Initialize the pose. q is an instance of SkelVector.
q = skel.q
q["j_pelvis_pos_y"] = -0.05
q["j_pelvis_rot_y"] = -0.2
q["j_thigh_left_z", "j_shin_left", "j_heel_left_1"] = 0.15, -0.4, 0.25
q["j_thigh_right_z", "j_shin_right", "j_heel_right_1"] = 0.15, -0.4, 0.25
q["j_abdomen_2"] = 0.0
skel.set_positions(q)
print('skeleton position OK')

# Initialize the controller
skel.controller = controller.Controller(skel, world.dt)
print('create controller OK')


# Program interactions
state = {}
state['Force'] = np.zeros(3)
state['ImpulseDuration'] = 0


def step_callback(world):
    """ Apply force to torso if any """
    global state
    if state['ImpulseDuration'] > 0:
        f = state['Force']
        state['ImpulseDuration'] -= 1
        world.skel.body('h_spine').add_ext_force(f)
    else:
        state['Force'] = np.zeros(3)


def keyboard_callback(world, key):
    """ Programmable interactions """
    global state
    if key == '1':
        state['Force'][0] = 50
        state['ImpulseDuration'] = 100
        print('push forward')
    elif key == '2':
        state['Force'][0] = -50
        state['ImpulseDuration'] = 100
        print('push backward')
    elif key == '3':
        state['Force'][2] = 50
        state['ImpulseDuration'] = 100
        print('push right')
    elif key == '4':
        state['Force'][2] = -50
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
