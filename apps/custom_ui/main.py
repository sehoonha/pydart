import pydart
from pydart.qtgui import PyDartQtWindow
from pydart.qtgui.trackball import Trackball
print('Example: CustomUi')

pydart.init()
print('pydart initialization OK')

data_dir = pydart.misc.example_data_dir(__file__)
print('data_dir = ' + data_dir)

world = pydart.create_world(1.0 / 2000.0, data_dir + '/skel/cubes.skel')
print('pydart create_world OK')


class CustomWindow(PyDartQtWindow):
    def __init__(self, title, simulation):
        super(CustomWindow, self).__init__(title, simulation)

    def initActions(self):
        PyDartQtWindow.initActions(self)
        self.loadAction = self.createAction('Load', self.printEvent)
        self.saveAction = self.createAction('Save', self.saveEvent)
        self.printAction = self.createAction('Print', self.printEvent)

    def initToolbar(self):
        # Print self.toolbar_actions (initialized from the parent)
        for i, action in enumerate(self.toolbar_actions):
            # If action is none, it will become a separator
            if action is not None:
                print i, action.text()

        # Update self.toolbar_actions
        # "list[x:x] += list2" is Python idiom for add list to the another list
        my_toolbar_actions = [self.printAction, None]
        self.toolbar_actions[4:4] += my_toolbar_actions

        # Call the parent function to initialize the toolbar
        PyDartQtWindow.initToolbar(self)

    def initMenu(self):
        PyDartQtWindow.initMenu(self)
        self.fileMenu.addAction(self.loadAction)
        self.fileMenu.addAction(self.saveAction)

    def loadEvent(self):
        print('load file')

    def saveEvent(self):
        print('save file')

    def printEvent(self):
        print('print world time: %f' % self.sim.t)

    def cam0Event(self):
        """ Change the default camera """
        self.glwidget.tb = Trackball(phi=-0.0, theta=0.0, zoom=1.0,
                                     rot=[-0.02, -0.71, -0.02, 0.71],
                                     trans=[0.02, 0.09, -1.39])

# pydart.qtgui.run(title='rigidChain', simulation=world)
pydart.qtgui.run(title='CustomUI', simulation=world, cls=CustomWindow)
