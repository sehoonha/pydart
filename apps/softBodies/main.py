# import sys
# sys.path.append('/home/sehoonha/dev/pydart')
import pydart
print('softBodies!')

pydart.init()
print('pydart initialization OK')

data_dir = pydart.misc.example_data_dir(__file__)
print('data_dir = ' + data_dir)

world = pydart.create_world(1.0 / 2000.0, data_dir + '/skel/softBodies.skel')
print('pydart create_world OK')

pydart.glutgui.run(title='softBodies', simulation=world, trans=[0, -0.1, -1.4])
