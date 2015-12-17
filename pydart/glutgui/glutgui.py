from OpenGL.GL import *
from OpenGL.GLUT import *
from OpenGL.GLU import *
import sys
import numpy as np

import trackball

# Some api in the chain is translating the keystrokes to this octal string
# so instead of saying: ESCAPE = 27, we use the following.
global window
ESCAPE = '\033'
window = 0
sim = None
mouseLastPos = None
state = {}
state['simulate'] = False
state['play'] = False
state['index'] = 0.0
tb = None
step_callback_func = None
keyboard_callback_func = None
render_callback_func = None


def initGL(w, h):
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


def resizeGL(w, h):
    glViewport(0, 0, w, h)
    glMatrixMode(GL_PROJECTION)
    glLoadIdentity()
    gluPerspective(45.0, float(w) / float(h), 0.1, 100.0)
    glMatrixMode(GL_MODELVIEW)


def drawGL():
    # Clear The Screen And The Depth Buffer
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
    glLoadIdentity()					# Reset The View
    global tb
    glTranslate(*tb.trans)
    glMultMatrixf(tb.matrix)

    global sim
    if sim is not None:
        sim.render()
        if hasattr(sim, 'contacts'):
            for c in sim.contacts():
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

    else:
        glutSolidSphere(0.3, 4, 4)  # Default object
    if render_callback_func is not None:
        render_callback_func(sim)

    glutSwapBuffers()


# The function called whenever a key is pressed.
# Note the use of Python tuples to pass in: (key, x, y)
def keyPressed(*args):
    global sim
    global keyboard_callback_func
    if keyboard_callback_func is not None:
        keyboard_callback_func(sim, args[0])

    n = sim.num_frames()
    if args[0] == ESCAPE:
        glutDestroyWindow(window)
        sys.exit()
    elif args[0] == ' ':
        state['simulate'] = not state['simulate']
    elif args[0] == 'p':
        state['play'] = not state['play']
    elif args[0] == ']':
        if hasattr(sim, 'set_frame') and hasattr(sim, 'num_frames'):
            state['index'] = (state['index'] + 10.0) % n
            sim.set_frame(int(state['index']))
            print('%d / %d' % (state['index'], n))
    elif args[0] == '[':
        if hasattr(sim, 'set_frame') and hasattr(sim, 'num_frames'):
            state['index'] = (state['index'] - 10 + n) % n
            sim.set_frame(int(state['index']))
            print('%d / %d' % (state['index'], n))


def mouseFunc(button, state, x, y):
    global mouseLastPos
    if state == 0:  # Mouse pressed
        mouseLastPos = np.array([x, y])
    elif state == 1:
        mouseLastPos = None


def motionFunc(x, y):
    global mouseLastPos
    global tb
    dx = x - mouseLastPos[0]
    dy = y - mouseLastPos[1]
    tb.drag_to(x, y, dx, -dy)
    mouseLastPos = np.array([x, y])


def idle():
    global sim
    global pd
    global state
    if sim is not None and state['simulate']:
        global step_callback_func
        if step_callback_func is not None:
            step_callback_func(sim)
        sim.step()
    if sim is not None and state['play']:
        n = sim.num_frames()
        state['index'] = (state['index'] + 0.01)
        if int(state['index']) >= n:
            state['index'] = 0
        sim.set_frame(int(state['index']))


def renderTimer(timer):
    glutPostRedisplay()
    glutTimerFunc(20, renderTimer, 1)


def run(title='GLUT Window', simulation=None, trans=None,
        step_callback=None, keyboard_callback=None,
        render_callback=None):
    global sim
    sim = simulation

    print("\n")
    print("space bar: simulation on/off")
    print("'p': playback/stop")
    print("'[' and ']': play one frame backward and forward")

    # Init glut
    global window
    glutInit(())
    glutInitDisplayMode(GLUT_RGBA | GLUT_DOUBLE | GLUT_ALPHA | GLUT_DEPTH)
    glutInitWindowSize(800, 600)
    glutInitWindowPosition(0, 0)
    window = glutCreateWindow(title)

    # Init trackball
    global tb
    if trans is None:
        trans = [0.0, 0.2, -0.9]
    tb = trackball.Trackball(theta=-10.5, trans=trans)

    # Init callback
    global step_callback_func
    global keyboard_callback_func
    global render_callback_func
    step_callback_func = step_callback
    keyboard_callback_func = keyboard_callback
    render_callback_func = render_callback

    # Init functions
    # glutFullScreen()
    glutDisplayFunc(drawGL)
    glutIdleFunc(idle)
    glutReshapeFunc(resizeGL)
    glutKeyboardFunc(keyPressed)
    glutMouseFunc(mouseFunc)
    glutMotionFunc(motionFunc)
    glutTimerFunc(25, renderTimer, 1)
    initGL(800, 600)

    # Run
    glutMainLoop()

    # Print message to console, and kick off the main to get it rolling.
    print "Hit ESC key to quit."
    main()


if __name__ == '__main__':
    run()
