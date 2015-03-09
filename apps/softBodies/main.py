import sys
import pydart
print('Example: softBodies')

pydart.init()
print('pydart initialization OK')

data_dir = pydart.misc.example_data_dir(__file__)
print('data_dir = ' + data_dir)

world = pydart.create_world(1.0 / 2000.0, data_dir + '/skel/softBodies.skel')
print('pydart create_world OK')

if 'qt' in sys.argv:
    tb = pydart.qtgui.Trackball(phi=-1.5, theta=-23.3, zoom=1.0,
                                rot=[-0.20, 0.04, -0.01, 0.98],
                                trans=[0.07, 0.03, -2.00])
    pydart.qtgui.run(title='softBodies', simulation=world, trackball=tb)
else:
    pydart.glutgui.run(title='softBodies', simulation=world,
                       trans=[0, -0.1, -1.4])
