import sys
import pydart
import numpy as np
import threading

print('Example: solveIK')


pydart.init()
print('pydart initialization OK')

data_dir = pydart.misc.example_data_dir(__file__)
print('data_dir = ' + data_dir)

skel_filename = data_dir + '/skel/springStair.skel'
# world = pydart.create_world(1.0 / 1000.0, skel_filename)
world = pydart.create_world(1.0 / 1000.0)
# world = pydart.create_world(1.0 / 2000.0)
# world.add_skeleton(data_dir + '/sdf/atlas/ground.urdf')
# world.add_skeleton(data_dir + '/vsk/Yunseong_copy.vsk')
world.add_skeleton(data_dir + '/vsk/Yunseong_Jan2016.vsk', traditional=True)
# world.add_skeleton(data_dir + '/vsk/JeffHsu_March2016.vsk')
# world.add_skeleton(data_dir + '/vsk/SistaniaM_April2016.vsk')
print('pydart create_world OK')

skel = world.skels[-1]
print skel.body('Head').C
print skel.body('LeftToeBase').C

# Initialize the pose. q is an instance of SkelVector.
q = skel.q
# q[4] = 10.05
# q['Joint-LeftArm_z'] = -1.5
# q['Joint-RightArm_z'] = -q['Joint-LeftArm_z']
# manual pose for frame 403, the most visible frame
skel.set_positions(q)

for i, body in enumerate(skel.bodies):
    print 'Body', i, body.name, body.C, body.m
print 'Body total mass = ', skel.m
for i, dof in enumerate(skel.dofs):
    print 'Dof', i, dof.name
for i, m in enumerate(skel.markers):
    print 'Marker', i, m, m.x
print('skeleton position OK')

# c3d_filename = '/c3d/Control_up5_new_vsk.c3d'
c3d_filename = '/c3d/Level_Ground_10.c3d'
# c3d_filename = '/stair/01 Control Pre/Control_Pre_06.c3d'
# c3d_filename = '/stair/02 Assistive/Assistive_25.c3d'
# c3d_filename = '/stair/03 Control Post/Control_Post_05.c3d'
# c3d_filename = '/stair/Jeff/01 Control Pre/Control_Pre_06.c3d'
# c3d_filename = '/stair/Jeff/02 Assistive/Assistive_27.c3d'
# c3d_filename = '/stair/0413 Data/Camera_relocation_trail_01 (Normal).c3d'
# c3d_filename = '/stair/0413 Data/Camera_relocation_trail_07 (With top Stair).c3d'
c3d_filename = data_dir + c3d_filename
if len(sys.argv) == 2:
    c3d_filename = sys.argv[1]
print('c3d_filename = [%s]' % c3d_filename)
motion_filename = c3d_filename.replace('.c3d', '.c3d.txt')
print('motion_filename = [%s]' % motion_filename)

# filec3d = pydart.FileC3D(xyz=[1, 0, 2],
#                          sign=[1, 1, -1])
# filec3d.load(c3d_filename)
# filec3d.set_rotation_Y(3.14)
# filec3d.set_translation([0.0, -1.2, 0.0])

filec3d = pydart.FileC3D(xyz=[2, 0, 1])
filec3d.load(c3d_filename)

seq = range(len(skel.markers))
seq = [29, 30, 31, 32, 33, 34, 35, 36, 37, 38, 39, 40, 41, 42, 43,
       44, 45, 46, 47, 48, 49, 50, 52, 51, 5, 6, 7, 8, 0, 2, 4, 1,
       3, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20,
       21, 22, 23, 24, 25, 26, 28, 27]

print('Total # markers in c3d = %d' % filec3d.num_markers())
print('Total # markers in skel = %d' % len(skel.markers))

foot_markers = list()
for i in range(11, 15):
    foot_markers.append(seq[i])
for i in range(20, 24):
    foot_markers.append(seq[i])
foot_visibile_count_yes = 0
foot_visibile_count_no = 0


min_visible_index = 10000
for frame in range(filec3d.num_frames()):
    for i in range(filec3d.num_markers()):
        if i in foot_markers:
            if filec3d.is_marker_visible(frame, i):
                foot_visibile_count_yes += 1
            else:
                foot_visibile_count_no += 1

        if i < min_visible_index and filec3d.is_marker_visible(frame, i):
            min_visible_index = i
            print('Marker %d is visible at frame %d' % (i, frame))
            print("%s" % filec3d.marker(frame, i))
print("foot_visibile_count_yes = %d" % foot_visibile_count_yes)
print("foot_visibile_count_no = %d" % foot_visibile_count_no)
y = foot_visibile_count_yes
n = foot_visibile_count_no
r = float(y) / (float(y + n)) * 100.0
print("foot_visibile ratio = %.2f" % r)

offset = min_visible_index
offset = 7
print("offset = %d" % offset)
for i in range(len(seq)):
    seq[i] += offset

mystate = dict()
mystate['skel'] = skel
mystate['frame'] = 0
mystate['seq'] = seq
mystate['solved_motion'] = None
mystate['filec3d'] = filec3d
mystate['eval_counter'] = 0
mystate['kill_thread'] = False


def evaluate():
    global mystate
    filec3d = mystate['filec3d']
    frame = mystate['frame']
    skel = mystate['skel']
    seq = mystate['seq']

    values = list()
    for i, m, in enumerate(skel.markers):
        lhs = m.x
        target_id = seq[i]
        if target_id == -1:
            continue
        rhs = filec3d.marker(frame, target_id)
        if not filec3d.is_marker_visible(frame, target_id):
            continue
        diff = lhs - rhs
        dist = 0.5 * (diff.dot(diff))
        values.append(dist)
        # break

    ret = np.sum(values)
    if mystate['eval_counter'] % 5000 == 0:
        print mystate['eval_counter'], ret

    mystate['eval_counter'] += 1
    return ret


def gradient():
    global mystate
    filec3d = mystate['filec3d']
    frame = mystate['frame']
    skel = mystate['skel']
    seq = mystate['seq']

    grad = np.zeros(skel.ndofs)

    for i, m, in enumerate(skel.markers):
        lhs = m.x

        target_id = seq[i]
        if target_id == -1:
            continue
        rhs = filec3d.marker(frame, target_id)
        if not filec3d.is_marker_visible(frame, target_id):
            continue

        body = m.body
        local_pos = m.local_pos()
        J = body.world_linear_jacobian(local_pos)
        if i == 10:
            print 'G', i, rhs
            print J
        diff = lhs - rhs
        grad += diff.dot(J)
    return grad


def solve():
    import scipy.optimize
    import time
    tic = time.time()
    options = {'maxiter': 3000,
               'ftol': 1e-5,
               'disp': True}

    def f(x):
        global mystate
        skel = mystate['skel']
        skel.q = x
        return evaluate()

    def grad(fun, x, h):
        n = len(x)
        g = np.zeros(n)
        for i in range(n):
            dx = np.zeros(n)
            dx[i] = h
            f1 = fun(x - dx)
            f2 = fun(x + dx)
            g[i] = (0.5 * f2 - 0.5 * f1) / h
        return g

    def g(x):
        # return grad(f, x, 1e-5)

        global mystate
        skel = mystate['skel']
        # skel.q = x
        G = gradient()
        G2 = grad(f, x, 1e-4)

        print 'x:', x
        print 'grad:', G.shape, G
        print 'grad2:', G2.shape, G2
        print np.allclose(G, G2)
        print np.linalg.norm(G - G2)
        exit(0)
        return G

    global mystate
    mystate['eval_counter'] = 0
    skel = mystate['skel']
    x0 = skel.q
    # x0[6] += 1.0
    x0 = skel.q + 0.5 * (np.random.rand(skel.ndofs) - 0.5)
    res = scipy.optimize.minimize(f,
                                  x0,
                                  jac=g,
                                  method='SLSQP',
                                  # method='L-BFGS-B',
                                  options=options)
    # print 'res.x = ', repr(res.x)
    toc = time.time()

    print 'res.fun = ', f(res.x)
    print 'res.message = ', res.message
    print 'res.status = ', res.status
    print 'time = %.8fs' % (toc - tic)
    print 'solve OK'


def find_most_visible_frame(world):
    global mystate
    filec3d = mystate['filec3d']
    n = filec3d.num_markers()
    max_visible = 0
    for i in range(filec3d.num_frames()):
        visible = [filec3d.is_marker_visible(i, j) for j in range(n)]
        num_visible = sum(visible)
        if max_visible < num_visible:
            max_visible = num_visible
            mystate['frame'] = i
            # print 'max_visible', num_visible, 'at', i
    print 'find_most_visible_frame OK'


def match_skel_marker_center(world):
    global mystate
    skel = mystate['skel']
    filec3d = mystate['filec3d']
    frame = mystate['frame']

    # Center of skeleton markers
    lhs = np.zeros(3)
    cnt = 0.0
    for i, m in enumerate(skel.markers):
        lhs += m.x
        cnt += 1.0
    lhs /= cnt

    # Center of frame markers
    rhs = np.zeros(3)
    cnt = 0.0
    for j in range(filec3d.num_markers()):
        if not filec3d.is_marker_visible(frame, j):
            continue
        rhs += filec3d.marker(frame, j)
        cnt += 1.0
    rhs /= cnt

    # Calculate offset and match
    diff = lhs - rhs
    print 'lhs(skel) = ', lhs
    print 'rhs(c3d) = ', rhs
    print 'diff = ', diff
    print 'match_skel_marker_center OK'
    filec3d.offset += diff


def match_nearest(world):
    global mystate
    skel = mystate['skel']
    seq = mystate['seq']
    frame = mystate['frame']

    n = len(seq)
    seq = [-1] * n
    occupied = [False] * n

    for loop in range(n):
        min_dist = None
        min_i, min_j = None, None
        for i, m, in enumerate(skel.markers):
            if seq[i] != -1:
                continue
            lhs = m.x
            for j in range(n):
                if occupied[j]:
                    continue
                if not filec3d.is_marker_visible(frame, j):
                    continue
                rhs = filec3d.marker(frame, j)
                d = np.linalg.norm(lhs - rhs)
                if min_dist is None or d < min_dist:
                    min_dist = d
                    min_i, min_j = i, j
        # Now we get the best pair
        if min_dist is None:
            break
        # print 'min_dist = %.8f (%d, %d)' % (min_dist, min_i, min_j)
        seq[min_i] = min_j
        occupied[min_j] = True
    mystate['seq'] = seq
    print 'match_nearest OK'


def annealing_worker(world):
    """thread worker function"""
    global mystate
    print '=' * 80
    print 'annealing_worker...'
    find_most_visible_frame(world)
    match_skel_marker_center(world)
    # Initial loop without perturbation
    print 'initial solve...'
    match_nearest(world)
    solve()
    # Simulated annealing w/ E-M approach
    T = 1.0
    T_min = 0.01
    alpha = 0.9
    skel = mystate['skel']
    while T > T_min and not mystate['kill_thread']:
        print '=' * 80
        print 'Loop started: T = ', T
        old_cost = evaluate()
        old_pose = np.array(skel.q)
        old_seq = list(mystate['seq'])

        # Random perturbation
        skel.q += 0.2 * (np.random.rand(skel.ndofs) - 0.5) * T
        match_nearest(world)
        solve()
        new_cost = evaluate()
        ap = np.exp(2.0 * (old_cost - new_cost) / (old_cost * T))
        dice = np.random.rand()
        print 'T = ', T
        print 'old_cost = ', old_cost
        print 'new_cost = ', new_cost
        print 'ap = ', ap
        print 'dice = ', dice
        if ap > dice:  # accept
            print 'accepted'
        else:
            print 'rejected'
            skel.q = old_pose
            mystate['seq'] = old_seq

        T *= alpha

    final_cost = evaluate()
    print 'final_cost = ', final_cost
    print 'average_marker_dist = ', final_cost / float(skel.ndofs)
    print 'final_pose = ', repr(skel.q)
    print 'final_seq = ', mystate['seq']
    print 'annealing_worker... OK'


def solve_all_worker(world):
    """thread worker function"""
    global mystate
    print('Solve all!!!')
    import time
    tic = time.time()
    nframes = mystate['filec3d'].num_frames()
    # nframes = 5
    motion = list()
    for frame in range(nframes):
        if mystate['kill_thread']:
            print('break.... kill_thread = True')
            break
        print('>>>')
        print('>>> Frame ID = %04d' % frame)
        print('>>>')
        mystate['frame'] = frame
        solve()
        motion.append(mystate['skel'].q)
        toc = time.time()
        print 'solveall.time = %.8fs' % (toc - tic)

    print('save to motion_file...')
    print('motion_filename = [%s]' % motion_filename)
    with open(motion_filename, 'w+') as fout:
        fout.write("%d\n" % len(motion))
        for q in motion:
            for v in q:
                fout.write("%.8f " % v)
            fout.write("\n")
    print('save to motion.txt... OK')
    mystate['solved_motion'] = motion


def step_callback(world):
    pass


def render_callback(world):
    from OpenGL.GL import *
    from OpenGL.GLUT import *
    from OpenGL.GLU import *
    global mystate
    filec3d = mystate['filec3d']
    frame = mystate['frame']
    skel = mystate['skel']
    seq = mystate['seq']

    for i in range(filec3d.num_markers()):
        # if i not in seq:
        #     continue
        # size = 0.005
        size = 0.02
        glColor3d(1.0, 0.0, 1.0)
        glPushMatrix()
        x = filec3d.marker(frame, i)
        glTranslate(*x)
        glutSolidSphere(size, 10, 10)  # Default object
        glPopMatrix()

    glBegin(GL_LINES)
    for i, m, in enumerate(skel.markers):
        lhs = m.x
        target_id = seq[i]
        if target_id == -1:
            continue
        rhs = filec3d.marker(frame, target_id)
        glVertex3d(*lhs)
        glVertex3d(*rhs)
    glEnd()


def keyboard_callback(world, key):
    """ Programmable interactions """
    global mystate
    nframes = mystate['filec3d'].num_frames()
    if key in 'npNP':
        if key == 'n':
            step = 10
        elif key == 'N':
            step = 1
        elif key == 'p':
            step = -10
        elif key == 'P':
            step = -1
        mystate['frame'] = (mystate['frame'] + step) % nframes
        print('frame = %d' % mystate['frame'])
        if mystate['solved_motion'] is not None:
            frame = mystate['frame']
            motion = mystate['solved_motion']
            if frame < len(motion):
                mystate['skel'].q = motion[frame]
    # elif key == 'n':
    #     mystate['frame'] = (mystate['frame'] + 10) % nframes
    #     if mystate['solved_motion'] is not None:
    #         motion = mystate['solved_motion']
    #         q = motion[mystate['frame']]
    #         mystate['skel'].q = q
    #     print('frame = %d' % mystate['frame'])
    elif key == 'e':
        print('evaluate = %.8f' % evaluate(world))
    elif key == 's':
        solve()
    elif key == 'f':
        find_most_visible_frame(world)
    elif key == 'm':
        match_skel_marker_center(world)
        match_nearest(world)
    elif key == 'q':
        print 'pose = ', repr(mystate['skel'].q)
        print 'seq = ', mystate['seq']
    elif key == 'a':  # All
        mystate['kill_thread'] = False
        t = threading.Thread(target=annealing_worker,
                             args=(world,))
        t.start()
    elif key == 'S':  # solve the entire motion
        mystate['kill_thread'] = False
        t = threading.Thread(target=solve_all_worker,
                             args=(world,))
        t.start()
    elif key == 'k':
        print 'kill_thread = True'
        mystate['kill_thread'] = True

print("--- usage ---")
print("'p': previous frame")
print("'n': next frame")
print("'a': simulate annealing to find the marker seqeunce")
print("  'f': find the most visible frame")
print("  'm': match center and update sequence as nearest")
print("  's': solve the single frame")
print("'S': Solve the entire frame")
print("'k': kill the thread (a/i)")
print("")


# Run the application
pydart.glutgui.run(title='bipedStand', simulation=world, trans=[0.0, 0, -6.0],
                   step_callback=step_callback,
                   keyboard_callback=keyboard_callback,
                   render_callback=render_callback)
