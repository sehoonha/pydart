from OpenGL.GL import *
from OpenGL.GLU import *
from OpenGL.GLUT import *
from PyQt4 import QtGui
from PyQt4 import QtCore
from PyQt4.QtOpenGL import *
import trackball


class GLWidget(QGLWidget):
    def __init__(self, parent=None):
        super(GLWidget, self).__init__(parent)
        self.width = 1280
        self.height = 720

        self.tb = trackball.Trackball()
        self.lastPos = None
        self.zoom = -1.2

        self.sim = None
        self.captureIndex = 0

    def sizeHint(self):
        return QtCore.QSize(self.width, self.height)

    def paintGL(self):
        glEnable(GL_DEPTH_TEST)
        glClearColor(0.95, 0.95, 0.95, 0.0)
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)

        glLoadIdentity()
        # glTranslate(0.0, -0.2, self.zoom)  # Camera
        glTranslate(*self.tb.trans)
        glMultMatrixf(self.tb.matrix)

        self.sim.render()
        if hasattr(self.sim, 'contacts'):
            for c in self.sim.contacts():
                glPushMatrix()
                glColor3d(0.0, 0.0, 1.0)
                glTranslate(*c.p)
                glutSolidSphere(0.01, 10, 10)
                glPopMatrix()
                glLineWidth(2.0)
                glBegin(GL_LINES)
                glVertex3d(*c.p)
                glVertex3d(*(c.p - 0.01 * c.f))
                glEnd()

        # self.enable2D()
        # self.disable2D()

    def resizeGL(self, w, h):
        (self.width, self.height) = (w, h)
        glViewport(0, 0, w, h)
        glMatrixMode(GL_PROJECTION)
        glLoadIdentity()

        gluPerspective(45.0, float(w) / float(h), 0.01, 100.0)
        glMatrixMode(GL_MODELVIEW)
        glLoadIdentity()

    def initializeGL(self):
        glDisable(GL_CULL_FACE)
        glEnable(GL_DEPTH_TEST)

        glDepthFunc(GL_LEQUAL)
        glHint(GL_PERSPECTIVE_CORRECTION_HINT, GL_NICEST)

        glEnable(GL_LINE_SMOOTH)
        glHint(GL_LINE_SMOOTH_HINT, GL_NICEST)

        glEnable(GL_DITHER)
        glShadeModel(GL_SMOOTH)
        glHint(GL_PERSPECTIVE_CORRECTION_HINT, GL_NICEST)
        glHint(GL_POLYGON_SMOOTH_HINT, GL_NICEST)

        glClearColor(1.0, 1.0, 1.0, 1.0)
        glClear(GL_COLOR_BUFFER_BIT)

        ambient = [0.2, 0.2, 0.2, 1.0]
        diffuse = [0.6, 0.6, 0.6, 1.0]
        front_mat_shininess = [60.0]
        front_mat_specular = [0.2, 0.2, 0.2, 1.0]
        front_mat_diffuse = [0.5, 0.28, 0.38, 1.0]
        lmodel_ambient = [0.2, 0.2, 0.2, 1.0]
        lmodel_twoside = [GL_FALSE]

        position = [1.0, 0.0, 0.0, 0.0]
        position1 = [-1.0, 0.0, 0.0, 0.0]

        glEnable(GL_LIGHT0)
        glLightfv(GL_LIGHT0, GL_AMBIENT, ambient)
        glLightfv(GL_LIGHT0, GL_DIFFUSE, diffuse)
        glLightfv(GL_LIGHT0, GL_POSITION, position)

        glLightModelfv(GL_LIGHT_MODEL_AMBIENT, lmodel_ambient)
        glLightModelfv(GL_LIGHT_MODEL_TWO_SIDE, lmodel_twoside)

        glEnable(GL_LIGHT1)
        glLightfv(GL_LIGHT1, GL_DIFFUSE, diffuse)
        glLightfv(GL_LIGHT1, GL_POSITION, position1)
        glEnable(GL_LIGHTING)
        glEnable(GL_COLOR_MATERIAL)

        glMaterialfv(GL_FRONT_AND_BACK, GL_SHININESS, front_mat_shininess)
        glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, front_mat_specular)
        glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, front_mat_diffuse)

        glEnable(GL_DEPTH_TEST)
        glDepthFunc(GL_LEQUAL)
        glDisable(GL_CULL_FACE)
        glEnable(GL_NORMALIZE)

        glColorMaterial(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE)
        glEnable(GL_COLOR_MATERIAL)

        glEnable(GL_BLEND)
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA)

    def enable2D(self):
        w, h = self.width, self.height
        glMatrixMode(GL_PROJECTION)
        glLoadIdentity()
        glDisable(GL_LIGHTING | GL_DEPTH_TEST)
        glDepthMask(0)
        glOrtho(0, w, h, 0, -1, 1)
        glViewport(0, 0, w, h)
        glMatrixMode(GL_MODELVIEW)
        glLoadIdentity()

    def disable2D(self):
        w, h = self.width, self.height
        glMatrixMode(GL_PROJECTION)
        glLoadIdentity()

        glEnable(GL_DEPTH_TEST | GL_LIGHTING)
        glDepthMask(1)
        gluPerspective(45.0, float(w) / float(h), 0.01, 100.0)

        glViewport(0, 0, w, h)
        glMatrixMode(GL_MODELVIEW)
        glLoadIdentity()

    def mousePressEvent(self, event):
        self.lastPos = event.pos()

    def mouseReleaseEvent(self, event):
        self.lastPos = None

    def mouseMoveEvent(self, event):
        # (w, h) = (self.width, self.height)
        x = event.x()
        y = event.y()
        dx = event.x() - self.lastPos.x()
        dy = event.y() - self.lastPos.y()

        modifiers = QtGui.QApplication.keyboardModifiers()
        if modifiers == QtCore.Qt.ShiftModifier:
            self.tb.zoom_to(dx, -dy)
        elif modifiers == QtCore.Qt.ControlModifier:
            self.tb.trans_to(dx, -dy)
        else:
            self.tb.drag_to(x, y, dx, -dy)
        self.lastPos = event.pos()
        self.updateGL()

    def capture(self, name=None):
        img = self.grabFrameBuffer()
        if name is None:
            name = 'frame'
        filename = 'captures/%s.%04d.png' % (name, self.captureIndex)
        img.save(filename)
        print 'Capture to ', filename
        self.captureIndex += 1
