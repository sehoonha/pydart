# import sys
# sys.path.append('/home/sehoonha/dev/pydart')
import pydart
print('Hello, PyDART!')

pydart.init()
print('pydart initialization OK')

data_dir = pydart.misc.example_data_dir(__file__)
print('data_dir = ' + data_dir)

world = pydart.create_world(1.0 / 2000.0, data_dir + '/skel/cubes.skel')
print('pydart create_world OK')

for i, skel in enumerate(world.skels):
    print('Skeleton %d. nDofs = %d' % (i, len(skel.dofs)))

pydart.glutgui.run(title='Hello', simulation=world)
