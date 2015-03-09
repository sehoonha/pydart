import os
import sys
import signal
from PyQt4 import QtGui
from PyQt4 import QtCore
from glwidget import GLWidget
from trackball import Trackball
from OpenGL.GLUT import glutInit


def signal_handler(signal, frame):
    print 'You pressed Ctrl+C! Bye.'
    sys.exit(0)
signal.signal(signal.SIGINT, signal_handler)


class MyQtWindow(QtGui.QMainWindow):
    def __init__(self, _title, _sim, _trackball):
        super(MyQtWindow, self).__init__()

        # Check and create captures directory
        if not os.path.isdir('captures'):
            os.makedirs('captures')

        self.sim = _sim
        self.setWindowTitle(_title)
        self.trackball = _trackball

        self.initUI()
        self.initActions()
        self.initToolbar()
        self.initMenu()

        self.idleTimer = QtCore.QTimer()
        self.idleTimer.timeout.connect(self.idleTimerEvent)
        self.idleTimer.start(0)

        self.renderTimer = QtCore.QTimer()
        self.renderTimer.timeout.connect(self.renderTimerEvent)
        self.renderTimer.start(25)

        self.cam0Event()

    def initUI(self):
        self.setGeometry(0, 0, 1280, 720)
        # self.setWindowTitle('Toolbar')

        self.glwidget = GLWidget(self)
        self.glwidget.setGeometry(0, 30, 1280, 720)
        self.glwidget.sim = self.sim

    def initActions(self):
        # Create actions
        self.resetAction = QtGui.QAction('Reset', self)
        self.resetAction.triggered.connect(self.resetEvent)

        self.playAction = QtGui.QAction('Play', self)
        self.playAction.setCheckable(True)
        self.playAction.setShortcut('Space')

        self.animAction = QtGui.QAction('Anim', self)
        self.animAction.setCheckable(True)

        self.captureAction = QtGui.QAction('Capture', self)
        self.captureAction.setCheckable(True)

        self.movieAction = QtGui.QAction('Movie', self)
        self.movieAction.triggered.connect(self.movieEvent)

        self.screenshotAction = QtGui.QAction('Screenshot', self)
        self.screenshotAction.triggered.connect(self.screenshotEvent)

        # Camera Menu
        self.cam0Action = QtGui.QAction('Camera0', self)
        self.cam0Action.triggered.connect(self.cam0Event)

        self.cam1Action = QtGui.QAction('Camera1', self)
        self.cam1Action.triggered.connect(self.cam1Event)

        self.printCamAction = QtGui.QAction('Print Camera', self)
        self.printCamAction.triggered.connect(self.printCamEvent)

    def initToolbar(self):
        # Create a toolbar
        self.toolbar = self.addToolBar('Control')
        self.toolbar.addAction(self.resetAction)
        self.toolbar.addAction(self.playAction)
        self.toolbar.addAction(self.animAction)
        self.toolbar.addSeparator()
        self.toolbar.addAction(self.screenshotAction)
        self.toolbar.addAction(self.captureAction)
        self.toolbar.addAction(self.movieAction)

        self.rangeSlider = QtGui.QSlider(QtCore.Qt.Horizontal, self)
        self.rangeSlider.valueChanged[int].connect(self.rangeSliderEvent)
        self.toolbar.addWidget(self.rangeSlider)

    def initMenu(self):
        menubar = self.menuBar()
        fileMenu = menubar.addMenu('&File')
        fileMenu.addSeparator()

        # Camera menu
        cameraMenu = menubar.addMenu('&Camera')
        cameraMenu.addAction(self.cam0Action)
        cameraMenu.addAction(self.cam1Action)
        cameraMenu.addSeparator()
        cameraMenu.addAction(self.printCamAction)

        # Recording menu
        recordingMenu = menubar.addMenu('&Recording')
        recordingMenu.addAction(self.screenshotAction)
        recordingMenu.addSeparator()
        recordingMenu.addAction(self.captureAction)
        recordingMenu.addAction(self.movieAction)

    def idleTimerEvent(self):
        doCapture = False
        # Do animation
        if self.animAction.isChecked():
            v = self.rangeSlider.value() + 10
            if v <= self.rangeSlider.maximum():
                self.rangeSlider.setValue(v)
            else:
                self.animAction.setChecked(False)
            doCapture = (v % 10 == 1)
        # Do play
        elif self.playAction.isChecked():
            if self._step_callback is not None:
                self._step_callback(self.sim)
            result = self.sim.step()
            if result:
                self.playAction.setChecked(False)
            doCapture = (self.sim.num_frames() % 4 == 1)

        if self.captureAction.isChecked() and doCapture:
            self.glwidget.capture('capture')

    def renderTimerEvent(self):
        self.glwidget.updateGL()
        self.statusBar().showMessage('# frames = %d' % self.sim.num_frames())
        self.rangeSlider.setRange(0, self.sim.num_frames() - 1)

    def keyPressEvent(self, event):
        if event.key() == QtCore.Qt.Key_Escape:
            print 'Escape key pressed! Bye.'
            self.close()
        if 0 <= event.key() and event.key() < 256:  # If key is ascii
            key = chr(event.key())
            if self._keyboard_callback is not None:
                self._keyboard_callback(self.sim, key)
            if hasattr(self.sim, 'key_pressed'):
                self.sim.key_pressed(key)

    def rangeSliderEvent(self, value):
        self.sim.set_frame(value)

    def screenshotEvent(self):
        self.glwidget.capture()

    def movieEvent(self):
        cmd = 'avconv -r 200 -i ./captures/capture.%%04d.png output.mp4'
        print 'Movie command:', cmd
        os.system(cmd)

    def resetEvent(self):
        self.sim.reset()

    def cam0Event(self):
        if self.trackball is not None:
            self.glwidget.tb = eval(repr(self.trackball))  # Make a deepcopy
            return
        self.glwidget.tb = Trackball(phi=-0.0, theta=0.0, zoom=1.0,
                                     rot=[-0.02, -0.71, -0.02, 0.71],
                                     trans=[0.02, 0.09, -0.69])

    def cam1Event(self):
        self.glwidget.tb = Trackball(phi=-0.0, theta=0.0, zoom=1.0,
                                     rot=[-0.02, -0.71, -0.02, 0.71],
                                     trans=[0.06, 0.26, -1.23])

    def printCamEvent(self):
        print 'printCamEvent'
        print '----'
        print repr(self.glwidget.tb)
        print '----'


def run(title='QT Window', simulation=None, trackball=None,
        step_callback=None, keyboard_callback=None):
    glutInit(())
    app = QtGui.QApplication([title])
    w = MyQtWindow(title, simulation, trackball)
    w._step_callback = step_callback
    w._keyboard_callback = keyboard_callback
    w.show()
    app.exec_()


if __name__ == '__main__':
    run()
