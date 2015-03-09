import sys
import pydart
print('Example: rigidCubes')

pydart.init()
print('pydart initialization OK')

data_dir = pydart.misc.example_data_dir(__file__)
print('data_dir = ' + data_dir)

world = pydart.create_world(1.0 / 2000.0, data_dir + '/skel/cubes.skel')
print('pydart create_world OK')

for i, skel in enumerate(world.skels):
    print('Skeleton %d. nDofs = %d' % (i, len(skel.dofs)))

if 'qt' in sys.argv:
    tb = pydart.qtgui.Trackball(phi=-0.6, theta=-9.3, zoom=1.0,
                                rot=[-0.08, 0.04, -0.00, 1.00],
                                trans=[0.03, 0.21, -1.00])
    pydart.qtgui.run(title='rigidCubes', simulation=world, trackball=tb)
else:
    pydart.glutgui.run(title='rigidCubes', simulation=world)
