/* RTQL8, Copyright (c) 2011, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Author(s): Sehoon Ha <sha9@gatech.edu>
 * Georgia Tech Graphics Lab
 */

#include "pydart_api.h"
#include <iostream>
#include <string>
#include <vector>
#include <map>
using std::cout;
using std::cerr;
using std::endl;

// Boost headers
#include <boost/algorithm/string.hpp>    

// Dart headers
#include "dart/dart.h"
// #include "dart/renderer/LoadOpengl.h"
// #include "dart/renderer/RenderInterface.h"
// #include "dart/renderer/OpenGLRenderInterface.h"
// #include "dart/simulation/World.h"
// #include "dart/dynamics/Skeleton.h"
// #include "dart/dynamics/BodyNode.h"
// #include "dart/dynamics/Shape.h"
// #include "dart/dynamics/MeshShape.h"
// #include "dart/dynamics/Joint.h"
// #include "dart/dynamics/DegreeOfFreedom.h"
// #include "dart/constraint/ConstraintSolver.h"
// #include "dart/collision/CollisionDetector.h"
// #include "dart/collision/dart/DARTCollisionDetector.h"
// #include "dart/collision/fcl/FCLCollisionDetector.h"
// #include "dart/collision/fcl_mesh/FCLMeshCollisionDetector.h"
// #include "dart/collision/bullet/BulletCollisionDetector.h"
// #include "dart/constraint/ContactConstraint.h"
// // #include "dart/utils/Paths.h"
// #include "dart/math/Geometry.h"
// #include "dart/utils/SkelParser.h"
// #include "dart/utils/sdf/SoftSdfParser.h"
// #include "dart/utils/urdf/DartLoader.h"
// #include "dart/utils/VskParser.h"
// #include "dart/utils/FileInfoWorld.h"

namespace pydart {

////////////////////////////////////////////////////////////
// class Manager
class Manager {
public:
    static void init();
    static void destroy();
    static Manager* getInstance() { return g_manager; }
    static dart::renderer::RenderInterface* getRI() {
        return g_ri;
      
    }

    static dart::simulation::WorldPtr world(int index = 0);
    static dart::dynamics::SkeletonPtr skeleton(int index);
    static dart::dynamics::SkeletonPtr skeleton(int wid, int skid);
    static int createWorld(double timestep);
    static int createWorldFromSkel(const char* const path);
    static void destroyWorld(int id);
    
protected:
    static Manager* g_manager;
    static dart::renderer::RenderInterface* g_ri;

    // std::vector<dart::simulation::WorldPtr> worlds;
    int next_id;
    std::map<int, dart::simulation::WorldPtr> worlds;

};

Manager* Manager::g_manager = NULL;
dart::renderer::RenderInterface* Manager::g_ri = NULL;

void Manager::init() {
    g_manager = new Manager();
    g_ri = new dart::renderer::OpenGLRenderInterface();
    //   g_ri->initialize();
    g_manager->next_id = 0;
    cout << " [pydart_api] Initialize pydart manager OK" << endl;
}

void Manager::destroy() {
    if (g_manager) {
        delete g_manager;
        g_manager = NULL;
    }
    if (g_ri) {
        delete g_ri;
        g_ri = NULL;
    }
    cout << " [pydart_api] Destroy pydart manager OK" << endl;
}

dart::simulation::WorldPtr Manager::world(int index) {
    Manager* manager = getInstance();
    return manager->worlds[index];
}

dart::dynamics::SkeletonPtr Manager::skeleton(int index) {
    return world()->getSkeleton(index);
}

dart::dynamics::SkeletonPtr Manager::skeleton(int wid, int skid) {
    return world(wid)->getSkeleton(skid);
}


int Manager::createWorld(double timestep) {
    Manager* manager = getInstance();

    dart::simulation::WorldPtr w(new dart::simulation::World());
    w->setTimeStep(timestep);
    w->setGravity(Eigen::Vector3d(0.0, -9.81, 0.0));
    // int id = manager->worlds.size();
    // manager->worlds.push_back(w);
    int id = manager->next_id++; 
    manager->worlds[id] = w;
    return id;
}

int Manager::createWorldFromSkel(const char* const path) {
    Manager* manager = getInstance();

    dart::simulation::WorldPtr w(dart::utils::SkelParser::readWorld(path));
    // w->setTimeStep(timestep);
    // w->setGravity(Eigen::Vector3d(0.0, -9.81, 0.0));
    int id = manager->next_id++;
    manager->worlds[id] = w;
    // int id = manager->worlds.size();
    // manager->worlds.push_back(w);
    return id;
}

void Manager::destroyWorld(int id) {
    Manager* manager = getInstance();
    dart::simulation::WorldPtr w = manager->worlds[id];
    manager->worlds.erase(id);
    cout << " [pydart_api] worlds.size = " << manager->worlds.size() << endl;
    // w.reset();
    cout << " [pydart_api] Destroy world OK: " << id << endl;
}

// class Manager
////////////////////////////////////////////////////////////

} // namespace pydart

using namespace pydart;

////////////////////////////////////////////////////////////////////////////////
// Init Functions
void init() {
    if (Manager::getInstance()) {
        Manager::destroy();
    }
    Manager::init();
}

void destroy() {
    Manager::destroy();
}


////////////////////////////////////////////////////////////////////////////////
// Manipulation Functions
int createWorld(double timestep) {
    return Manager::createWorld(timestep);
}

int createWorldFromSkel(const char* const path) {
    int wid = Manager::createWorldFromSkel(path);
    std::cout << " [pydart_api] # Skeletons in " << path << " = " << numSkeletons(wid) << std::endl;
    return wid;
}

void destroyWorld(int wid) {
    Manager::destroyWorld(wid);
}

int saveWorldToFile(int wid, const char* const path) {
    using namespace dart::simulation;
    WorldPtr world = Manager::world(wid);
    dart::utils::FileInfoWorld* file = new dart::utils::FileInfoWorld();
    bool result = file->saveFile(path, world->getRecording());
    return (int)result;
}


int addSkeleton(int wid, const char* const path, double frictionCoeff, int traditionalMode) {
    using namespace dart::simulation;
    using namespace dart::dynamics;
    std::string strpath(path);
    std::string ext = strpath.substr(strpath.length() - 4);
    boost::algorithm::to_lower(ext);

    SkeletonPtr skel = NULL;
    if (ext == ".sdf") {
        cout << " [pydart_api] parse as SDF (ext: " << ext << ")" << endl;
        skel = dart::utils::SdfParser::readSkeleton(path);
    } else if (ext == "urdf") {
        cout << " [pydart_api] parse as URDF (ext: " << ext << ")" << endl;
        dart::utils::DartLoader urdfLoader;
        skel = urdfLoader.parseSkeleton(path);
    } else if (ext == ".vsk") {
        cout << " [pydart_api] parse as VSK (ext: " << ext << ")" << endl;
        skel = dart::utils::VskParser::readSkeleton(path);
    } else {
        cout << " [pydart_api] bad extension (ext: " << ext << ")" << endl;
        return -1;
    }
    // SkeletonPtr skel = urdfLoader.parseSkeleton(path);

    cout << " [pydart_api] skel [" << path << "] : friction = " << frictionCoeff;
    cout << " traditionalMode " << traditionalMode << endl;
    for (size_t i = 0; i < skel->getNumBodyNodes(); ++i) {
        dart::dynamics::BodyNode* bn = skel->getBodyNode(i);
        bn->setFrictionCoeff(frictionCoeff);
    }
    
    WorldPtr world = Manager::world(wid);
    int id = world->getNumSkeletons();
    world->addSkeleton(skel);

    if (traditionalMode == 1) {
        changeRootJointToTransAndEuler(wid, id);
    }

    // Debug
    dart::constraint::ConstraintSolver* solver = world->getConstraintSolver();
    dart::collision::CollisionDetector* detector = solver->getCollisionDetector();
    dart::collision::CollisionDetector* detector2 = new dart::collision::FCLMeshCollisionDetector();
    // dart::collision::CollisionDetector* detector2 = new dart::collision::DARTCollisionDetector();
    solver->setCollisionDetector(detector2);
    detector = detector2;

    // if (dynamic_cast<dart::collision::DARTCollisionDetector*>(detector)) {
    //     std::cout << " [pydart_api] DARTCollisionDetector!" << std::endl;
    // } else if (dynamic_cast<dart::collision::FCLCollisionDetector*>(detector)) {
    //     std::cout << " [pydart_api] FCLCollisionDetector!" << std::endl;
    // } else if (dynamic_cast<dart::collision::FCLMeshCollisionDetector*>(detector)) {
    //     std::cout << " [pydart_api] FCLMeshCollisionDetector!" << std::endl;
    // } else if (dynamic_cast<dart::collision::BulletCollisionDetector*>(detector)) {
    //     std::cout << " [pydart_api] BulletCollisionDetector!" << std::endl;
    // } else {
    //     std::cout << " [pydart_api] Unknown CollisionDetector... (maybe bullet)" << std::endl;
    // }

    dart::constraint::ContactConstraint::setErrorReductionParameter(0);
    // std::cout << " [pydart_api] Zero ERP!!!" << std::endl;
    // std::cout << " [pydart_api] ERP = " << dart::constraint::ContactConstraint::getErrorReductionParameter() << std::endl;
    
    return id;
}

int numSkeletons(int wid) {
    using namespace dart::simulation;
    WorldPtr world = Manager::world(wid);
    return world->getNumSkeletons();
}

void setSkeletonJointDamping(int wid, int skid, double damping) {
    using namespace dart::dynamics;
    SkeletonPtr skel = Manager::skeleton(wid, skid);

    for (size_t i = 1; i < skel->getNumBodyNodes(); ++i) {
        dart::dynamics::Joint* joint = skel->getJoint(i);
        if (joint->getNumDofs() > 0) {
            for (size_t j = 0; j < joint->getNumDofs(); ++j) {
                joint->setDampingCoefficient(j, damping);
            }
        }
    }

}

////////////////////////////////////////////////////////////////////////////////
// Simulation Functions
void setCollisionDetector(int wid, int detector_type) {
    using namespace dart::simulation;
    WorldPtr world = Manager::world(wid);
    dart::constraint::ConstraintSolver* solver = world->getConstraintSolver();
    dart::collision::CollisionDetector* detector = NULL;
    std::cout << " [pydart_api] detector_type = " << detector_type << std::endl;
    if (detector_type == 0) {
        std::cout << " [pydart_api] create DARTCollisionDetector" << std::endl;
        detector = new dart::collision::DARTCollisionDetector();
    } else if (detector_type == 1) {
        std::cout << " [pydart_api] create FCLCollisionDetector" << std::endl;
        detector = new dart::collision::FCLCollisionDetector();
    } else if (detector_type == 2) {
        std::cout << " [pydart_api] create FCLMeshCollisionDetector" << std::endl;
        detector = new dart::collision::FCLMeshCollisionDetector();
    } else if (detector_type == 3) {
        std::cout << " [pydart_api] create BulletCollisionDetector" << std::endl;
        detector = new dart::collision::BulletCollisionDetector();
    } else {
        std::cerr << " [pydart_api] unknown detector_type" << std::endl;
    }

    if (detector != NULL) {
        solver->setCollisionDetector(detector);
    }
    dart::constraint::ContactConstraint::setErrorReductionParameter(0);
    // std::cout << " [pydart_api] Zero ERP!!!" << std::endl;
}

void printCollisionDetector(int wid) {
    using namespace dart::simulation;
    WorldPtr world = Manager::world(wid);
    // Debug
    dart::constraint::ConstraintSolver* solver = world->getConstraintSolver();
    dart::collision::CollisionDetector* detector = solver->getCollisionDetector();

    if (dynamic_cast<dart::collision::DARTCollisionDetector*>(detector)) {
        std::cout << " [pydart_api] DARTCollisionDetector!" << std::endl;
    } else if (dynamic_cast<dart::collision::FCLCollisionDetector*>(detector)) {
        std::cout << " [pydart_api] FCLCollisionDetector!" << std::endl;
    } else if (dynamic_cast<dart::collision::FCLMeshCollisionDetector*>(detector)) {
        std::cout << " [pydart_api] FCLMeshCollisionDetector!" << std::endl;
    } else if (dynamic_cast<dart::collision::BulletCollisionDetector*>(detector)) {
        std::cout << " [pydart_api] BulletCollisionDetector!" << std::endl;
    } else {
        std::cout << " [pydart_api] Unknown CollisionDetector... (maybe bullet)" << std::endl;
    }

    std::cout << " [pydart_api] ERP = " << dart::constraint::ContactConstraint::getErrorReductionParameter() << std::endl;
}

////////////////////////////////////////////////////////////////////////////////
// Simulation Functions
void resetWorld(int wid) {
    using namespace dart::simulation;
    using namespace dart::dynamics;
    WorldPtr world = Manager::world(wid);
    world->reset();
    // Debug
    // std::cout << "reinitialize collision detector" << std::endl;
    // dart::constraint::ConstraintSolver* solver = world->getConstraintSolver();
    // dart::collision::CollisionDetector* detector = solver->getCollisionDetector();
    // std::cout << "reinitialize collision detector... creating new.." << std::endl;
    // dart::collision::CollisionDetector* detector2 = new dart::collision::FCLMeshCollisionDetector();
    // dart::collision::CollisionDetector* detector2 = new dart::collision::BulletCollisionDetector();
    // dart::collision::CollisionDetector* detector2 = new dart::collision::FCLCollisionDetector();
    // solver->setCollisionDetector(detector2);
    // std::cout << " [pydart_api] reinitialize collision detector... deleting old.." << std::endl;
    // delete detector;
    // std::cout << " [pydart_api] OK" << std::endl;
    for (int skid = 0; skid < numSkeletons(wid); skid++) {
        SkeletonPtr skel = Manager::skeleton(wid, skid);

        skel->resetCommands();
        skel->resetPositions();
        skel->resetVelocities();
        skel->resetAccelerations();
        skel->resetGeneralizedForces();
        skel->clearExternalForces();
        skel->clearConstraintImpulses();
    }
    // std::cout << "reinitialize collision detector... done" << std::endl;
}

void stepWorld(int wid) {
    using namespace dart::simulation;
    WorldPtr world = Manager::world(wid);
    world->step();
    world->bake();
}

void checkCollisionWorld(int wid, int _checkAllCollisions) {
    using namespace dart::simulation;
    WorldPtr world = Manager::world(wid);
    bool bCheckAll = (_checkAllCollisions != 0);
    world->checkCollision(bCheckAll);
    dart::collision::CollisionDetector* cd =
        world->getConstraintSolver()->getCollisionDetector();
    // std::cout << "# skels = " <<  world->getNumSkeletons() << std::endl;
    // std::cout << "# collisions = " <<  cd->getNumContacts() << std::endl;
}

void render(int wid) {
    using namespace dart::simulation;
    dart::renderer::RenderInterface* ri = Manager::getRI();
    WorldPtr world = Manager::world(wid);
    
    for (size_t i = 0; i < world->getNumSkeletons(); i++) {
        world->getSkeleton(i)->draw(ri);
    }
}

void renderSkeleton(int wid, int skid) {
    using namespace dart::dynamics;
    dart::renderer::RenderInterface* ri = Manager::getRI();
    SkeletonPtr skel = Manager::skeleton(wid, skid);
    // skel->drawMarkers(ri);
    skel->draw(ri);
}

void renderSkeletonWithColor(int wid, int skid, double r, double g, double b, double a) {
    using namespace dart::dynamics;
    dart::renderer::RenderInterface* ri = Manager::getRI();
    SkeletonPtr skel = Manager::skeleton(wid, skid);
    skel->draw(ri, Eigen::Vector4d(r, g, b, a), false);
}

void renderSkeletonMarkers(int wid, int skid) {
    using namespace dart::dynamics;
    dart::renderer::RenderInterface* ri = Manager::getRI();
    SkeletonPtr skel = Manager::skeleton(wid, skid);
    skel->drawMarkers(ri);
}

////////////////////////////////////////////////////////////////////////////////
// World Functions
double getWorldTime(int wid) {
    using namespace dart::simulation;
    WorldPtr world = Manager::world(wid);
    return world->getTime();
}

double getWorldTimeStep(int wid) {
    using namespace dart::simulation;
    WorldPtr world = Manager::world(wid);
    return world->getTimeStep();
}

void setWorldTimeStep(int wid, double _timeStep) {
    using namespace dart::simulation;
    WorldPtr world = Manager::world(wid);
    world->setTimeStep(_timeStep);
}


int getWorldSimFrames(int wid) {
    using namespace dart::simulation;
    WorldPtr world = Manager::world(wid);
    return world->getSimFrames();
}

void setWorldSimFrame(int wid, int playFrame) {
    using namespace dart::simulation;
    WorldPtr world = Manager::world(wid);
    if (playFrame >= world->getRecording()->getNumFrames()) {
        return;
    }
    
    size_t nSkels = world->getNumSkeletons();
    for (size_t i = 0; i < nSkels; i++) {
        // size_t start = world->getIndex(i);
        // size_t size = world->getSkeleton(i)->getNumDofs();
        world->getSkeleton(i)->setPositions(world->getRecording()->getConfig(playFrame, i));
        world->getSkeleton(i)->computeForwardKinematics(true, true, false);
    }
}

int getWorldNumContacts(int wid) {
    dart::simulation::WorldPtr world = Manager::world(wid);
    dart::collision::CollisionDetector* cd =
        world->getConstraintSolver()->getCollisionDetector();
    return cd->getNumContacts();
}

void getWorldContacts(int wid, double* outv, int len) {
    dart::simulation::WorldPtr world = Manager::world(wid);
    dart::collision::CollisionDetector* cd =
        world->getConstraintSolver()->getCollisionDetector();
    size_t n = cd->getNumContacts();
    if (7 * n != (size_t)len) {
        cerr << "getWorldContacts: 7n is needed for the output vector. n = " << n << ", len =  " << len << endl;
        return;
    }

    // ( v.x, v.y, v.z, p.x, p.y, p.z, id )
    
    int ptr = 0;
    for (size_t i = 0; i < n; i++) {
        Eigen::Vector3d v = cd->getContact(i).point;
        Eigen::Vector3d f = cd->getContact(i).force;
        for (int j = 0; j < 3; j++) {
            outv[ptr++] = v(j);
        }
        for (int j = 0; j < 3; j++) {
            outv[ptr++] = f(j);
        }
        outv[ptr++] = i;

    }    
}

void setWorldCollisionPair(int wid, int skid1, int bid1, int skid2, int bid2, int bEnable) {
    using namespace dart::simulation;
    using namespace dart::dynamics;
    // Get collision detector
    WorldPtr world = Manager::world(wid);
    dart::constraint::ConstraintSolver* solver = world->getConstraintSolver();
    dart::collision::CollisionDetector* detector = solver->getCollisionDetector();

    // Get body1 and body2
    SkeletonPtr skel1 = Manager::skeleton(wid, skid1);
    BodyNode* body1 = skel1->getBodyNode(bid1);
    SkeletonPtr skel2 = Manager::skeleton(wid, skid2);
    BodyNode* body2 = skel2->getBodyNode(bid2);
    // cout << " [pydart_api] set collision pair "
    //      << "(skel " << skid1 << " body " <<  bid1 << ") and "
    //      << "(skel " << skid2 << " body " <<  bid2 << ")"
    //      << " as " << bEnable << endl;
    bool enable = (bEnable != 0);
    if (enable) {
        detector->enablePair(body1, body2);
    } else {
        detector->disablePair(body1, body2);
    }
}

////////////////////////////////////////////////////////////////////////////////
// Skeleton Attribute Functions
const char* getSkeletonName(int wid, int skid) {
    dart::dynamics::SkeletonPtr skel = Manager::skeleton(wid, skid);
    return skel->getName().c_str();
}

double getSkeletonMass(int wid, int skid) {
    using namespace dart::dynamics;
    SkeletonPtr skel = Manager::skeleton(wid, skid);
    return skel->getMass();
}

int getSkeletonNumDofs(int wid, int skid) {
    using namespace dart::dynamics;
    SkeletonPtr skel = Manager::skeleton(wid, skid);
    return skel->getNumDofs();
}

int getSkeletonNumBodies(int wid, int skid) {
    using namespace dart::dynamics;
    SkeletonPtr skel = Manager::skeleton(wid, skid);
    return skel->getNumBodyNodes();
}

const char* getSkeletonBodyName(int wid, int skid, int bodyid) {
    dart::dynamics::SkeletonPtr skel = Manager::skeleton(wid, skid);
    return skel->getBodyNode(bodyid)->getName().c_str();
}

const char* getSkeletonDofName(int wid, int skid, int dofid) {
    dart::dynamics::SkeletonPtr skel = Manager::skeleton(wid, skid);
    // return skel->getGenCoordInfo(dofid).joint->getName().c_str();
    return skel->getDof(dofid)->getName().c_str();
}

int getSkeletonMobile(int wid, int skid) {
    dart::dynamics::SkeletonPtr skel = Manager::skeleton(wid, skid);
    if (skel->isMobile()) {
        return 1;
    } else {
        return 0;
    }
}    

void setSkeletonMobile(int wid, int skid, int mobile) {
    dart::dynamics::SkeletonPtr skel = Manager::skeleton(wid, skid);
    skel->setMobile(mobile != 0);
}

void setSkeletonSelfCollision(int wid, int skid, int bSelfCollision, int bAdjacentBodies) {
    dart::dynamics::SkeletonPtr skel = Manager::skeleton(wid, skid);
    if (bSelfCollision != 0) {
        bool _enableAdjecentBodies = (bAdjacentBodies != 0);
        cout << " [pydart_api] enable skeleton self collision: skid = " << skid << endl;
        cout << " [pydart_api] _enableAdjecentBodies = " << _enableAdjecentBodies << endl;
        skel->enableSelfCollision(_enableAdjecentBodies);
    } else {
        cout << " [pydart_api] disable skeleton self collision: skid = " << skid << endl;
        skel->disableSelfCollision();
    }
}

void changeRootJointToTransAndEuler(int wid, int skid) {
    using namespace dart::dynamics;
    SkeletonPtr skel = Manager::skeleton(wid, skid);
    // change the joint type to euler
    BodyNode* oldRoot = skel->getRootBodyNode();
    oldRoot->changeParentJointType<EulerJoint>();
    oldRoot->getParentJoint()->setName("j_root_r");
    // create a new root
    std::pair<Joint*, BodyNode*> ret = skel->createJointAndBodyNodePair
        <TranslationalJoint, BodyNode>();
    Joint* newJoint = ret.first;
    newJoint->setName("j_root_t");
    BodyNode* newBody = ret.second;
    // rearrange the root joints
    oldRoot->moveTo(newBody);
}

////////////////////////////////////////////////////////////////////////////////
// Skeleton Pose Functions
void getSkeletonPositions(int wid, int skid, double* outpose, int ndofs) {
    using namespace dart::dynamics;
    SkeletonPtr skel = Manager::skeleton(wid, skid);
    
    Eigen::VectorXd q = skel->getPositions();
    for (int i = 0; i < q.size(); i++) {
        outpose[i] = q(i);
    }
}

void getSkeletonVelocities(int wid, int skid, double* outpose, int ndofs) {
    using namespace dart::dynamics;
    SkeletonPtr skel = Manager::skeleton(wid, skid);
    
    Eigen::VectorXd qdot = skel->getVelocities();
    for (int i = 0; i < qdot.size(); i++) {
        outpose[i] = qdot(i);
    }
}

void getSkeletonPositionDifferences(int wid, int skid, double* inpose, int ndofs,
                                    double* inpose2, int ndofs2, double* outpose3, int ndofs3) {
    using namespace dart::dynamics;
    SkeletonPtr skel = Manager::skeleton(wid, skid);
    Eigen::VectorXd q1(ndofs);
    for (int i = 0; i < q1.size(); i++) {
        q1(i) = inpose[i];
    }
    Eigen::VectorXd q2(ndofs2);
    for (int i = 0; i < q2.size(); i++) {
        q2(i) = inpose2[i];
    }
    Eigen::VectorXd q_diff = skel->getPositionDifferences(q1, q2);
    // cout << "q1 = " << q1.transpose() << endl;
    // cout << "q2 = " << q2.transpose() << endl;
    for (int i = 0; i < ndofs3; i++) {
        outpose3[i] = q_diff(i);
    }
    
}


void getSkeletonVelocityDifferences(int wid, int skid, double* inpose, int ndofs,
                                    double* inpose2, int ndofs2, double* outpose3, int ndofs3) {
    using namespace dart::dynamics;
    SkeletonPtr skel = Manager::skeleton(wid, skid);
    Eigen::VectorXd q1(ndofs);
    for (int i = 0; i < q1.size(); i++) {
        q1(i) = inpose[i];
    }
    Eigen::VectorXd q2(ndofs2);
    for (int i = 0; i < q2.size(); i++) {
        q2(i) = inpose2[i];
    }
    Eigen::VectorXd dq_diff = skel->getVelocityDifferences(q1, q2);
    // cout << "q1 = " << q1.transpose() << endl;
    // cout << "q2 = " << q2.transpose() << endl;
    for (int i = 0; i < ndofs3; i++) {
        outpose3[i] = dq_diff(i);
    }
    
}


void getSkeletonMassMatrix(int wid, int skid, double* array2, int nrows, int ncols) {
    using namespace dart::dynamics;
    SkeletonPtr skel = Manager::skeleton(wid, skid);
    const Eigen::MatrixXd& M = skel->getMassMatrix();
    
    int ptr = 0;
    for (int i = 0; i < M.rows(); i++) {
        for (int j = 0; j < M.cols(); j++) {
            array2[ptr++] = M(i, j);
        }
    }
}

void getSkeletonCoriolisAndGravityForces(int wid, int skid, double* outpose, int ndofs) {
    using namespace dart::dynamics;
    SkeletonPtr skel = Manager::skeleton(wid, skid);
    const Eigen::VectorXd& C = skel->getCoriolisAndGravityForces();
    for (int i = 0; i < C.size(); i++) {
        outpose[i] = C(i);
    }
}

void getSkeletonConstraintForces(int wid, int skid, double* outpose, int ndofs) {
    using namespace dart::dynamics;
    SkeletonPtr skel = Manager::skeleton(wid, skid);
    const Eigen::VectorXd& F = skel->getConstraintForces();
    for (int i = 0; i < F.size(); i++) {
        outpose[i] = F(i);
    }
}


void setSkeletonPositions(int wid, int skid, double* inpose, int ndofs) {
    using namespace dart::dynamics;
    SkeletonPtr skel = Manager::skeleton(wid, skid);

    Eigen::VectorXd q(ndofs);
    for (int i = 0; i < q.size(); i++) {
        q(i) = inpose[i];
    }
    skel->setPositions(q);
    // skel->computeForwardKinematics(true, true, false);
}

void setSkeletonVelocities(int wid, int skid, double* inpose, int ndofs) {
    using namespace dart::dynamics;
    SkeletonPtr skel = Manager::skeleton(wid, skid);

    Eigen::VectorXd q(ndofs);
    for (int i = 0; i < q.size(); i++) {
        q(i) = inpose[i];
    }
    skel->setVelocities(q);
    // skel->computeForwardKinematics(true, true, false);
}

void setSkeletonForces(int wid, int skid, double* intorque, int ndofs) {
    using namespace dart::dynamics;
    SkeletonPtr skel = Manager::skeleton(wid, skid);

    Eigen::VectorXd tau(ndofs);
    for (int i = 0; i < tau.size(); i++) {
        tau(i) = intorque[i];
    }
    skel->setForces(tau);
}

////////////////////////////////////////////////////////////////////////////////
// Skeleton Limit Functions
void getSkeletonPositionLowerLimit(int wid, int skid, double* outpose, int ndofs) {
    dart::dynamics::SkeletonPtr skel = Manager::skeleton(wid, skid);
    for (int i = 0; i < ndofs; i++) {
        outpose[i] = skel->getPositionLowerLimit(i);
    }
}

void getSkeletonPositionUpperLimit(int wid, int skid, double* outpose, int ndofs) {
    dart::dynamics::SkeletonPtr skel = Manager::skeleton(wid, skid);
    for (int i = 0; i < ndofs; i++) {
        outpose[i] = skel->getPositionUpperLimit(i);
    }
}

void getSkeletonForceLowerLimit(int wid, int skid, double* outpose, int ndofs) {
    dart::dynamics::SkeletonPtr skel = Manager::skeleton(wid, skid);
    for (int i = 0; i < ndofs; i++) {
        outpose[i] = skel->getForceLowerLimit(i);
    }
}

void getSkeletonForceUpperLimit(int wid, int skid, double* outpose, int ndofs) {
    dart::dynamics::SkeletonPtr skel = Manager::skeleton(wid, skid);
    for (int i = 0; i < ndofs; i++) {
        outpose[i] = skel->getForceUpperLimit(i);
    }
}

////////////////////////////////////////////////////////////////////////////////
// Skeleton Momentum Functions
void getSkeletonWorldCOM(int wid, int skid, double outv3[3]) {
    using namespace dart::dynamics;
    SkeletonPtr skel = Manager::skeleton(wid, skid);
    // Eigen::Vector3d C = skel->getWorldCOM();
    Eigen::Vector3d C = skel->getCOM();
    for (int i = 0; i < C.size(); i++) {
        outv3[i] = C(i);
    }
}

void getSkeletonWorldCOMVelocity(int wid, int skid, double outv3[3]) {
    using namespace dart::dynamics;
    SkeletonPtr skel = Manager::skeleton(wid, skid);
    // Eigen::Vector3d CV = skel->getWorldCOMVelocity();
    Eigen::Vector3d CV = skel->getCOMLinearVelocity();
    for (int i = 0; i < CV.size(); i++) {
        outv3[i] = CV(i);
    }
}

////////////////////////////////////////////////////////////////////////////////
// BodyNode Functions
double getBodyNodeMass(int wid, int skid, int bid) {
    dart::dynamics::SkeletonPtr skel = Manager::skeleton(wid, skid);
    dart::dynamics::BodyNode* bn = skel->getBodyNode(bid);
    return bn->getMass();
}

void getBodyNodeInertia(int wid, int skid, int bid, double outv33[3][3]) {
    dart::dynamics::SkeletonPtr skel = Manager::skeleton(wid, skid);
    dart::dynamics::BodyNode* bn = skel->getBodyNode(bid);
    double Ixx, Iyy, Izz, Ixy, Ixz, Iyz;
    bn->getMomentOfInertia(Ixx, Iyy, Izz, Ixy, Ixz, Iyz);
    outv33[0][0] = Ixx;    outv33[1][1] = Iyy;    outv33[2][2] = Izz;
    outv33[0][1] = Ixy;    outv33[1][0] = Ixy; 
    outv33[0][2] = Ixz;    outv33[2][0] = Ixz; 
    outv33[1][2] = Iyz;    outv33[2][1] = Iyz; 
}

void getBodyNodeLocalCOM(int wid, int skid, int bid, double outv3[3]) {
    dart::dynamics::SkeletonPtr skel = Manager::skeleton(wid, skid);
    dart::dynamics::BodyNode* bn = skel->getBodyNode(bid);
    const Eigen::Vector3d& x = bn->getLocalCOM();
    for (int i = 0; i < x.size(); i++) {
        outv3[i] = x(i);
    }
}

void getBodyNodeShapeBoundingBoxDim(int wid, int skid, int bid, double outv3[3]) {
    dart::dynamics::SkeletonPtr skel = Manager::skeleton(wid, skid);
    dart::dynamics::BodyNode* bn = skel->getBodyNode(bid);
    // cout << "bodynode " << bid << " is fetched" << endl;
    // Assume single shape for now
    // const int num_shapes = bn->getNumCollisionShapes();
    const int num_shapes = bn->getNumShapeNodes();
    if (num_shapes == 0) {
        for (int i = 0; i < 3; i++) {
            outv3[i] = 0.0;
        }
    } else {
        dart::dynamics::ShapeNodePtr shapenode = bn->getShapeNode(0);
        dart::dynamics::ShapePtr shape = shapenode->getShape();
        Eigen::Vector3d scale(1, 1, 1);
        dart::dynamics::MeshShape* mshape = dynamic_cast<dart::dynamics::MeshShape*>(shape.get());
        if (mshape) {
            scale = mshape->getScale();
        }
        // cout << "Scale:" << scale << endl;
        const dart::math::BoundingBox& bbox = shape->getBoundingBox();
        const Eigen::Vector3d& x = bbox.getMax() - bbox.getMin();
        for (int i = 0; i < x.size(); i++) {
            outv3[i] = x(i) * scale(i);
        }
    }
}

void getBodyNodeWorldCOM(int wid, int skid, int bid, double outv3[3]) {
    dart::dynamics::SkeletonPtr skel = Manager::skeleton(wid, skid);
    dart::dynamics::BodyNode* bn = skel->getBodyNode(bid);
    // const Eigen::Vector3d& x = bn->getWorldCOM();
    const Eigen::Vector3d& x = bn->getCOM();
    for (int i = 0; i < x.size(); i++) {
        outv3[i] = x(i);
    }
}

void getBodyNodeWorldCOMVelocity(int wid, int skid, int bid, double outv3[3]) {
    dart::dynamics::SkeletonPtr skel = Manager::skeleton(wid, skid);
    dart::dynamics::BodyNode* bn = skel->getBodyNode(bid);
    // const Eigen::Vector3d& x = bn->getWorldCOMVelocity();
    const Eigen::Vector3d& x = bn->getCOMLinearVelocity();
    for (int i = 0; i < x.size(); i++) {
        outv3[i] = x(i);
    }
}

void getBodyNodeWorldCOMSpatialVelocity(int wid, int skid, int bid, double outv6[6]) {
    dart::dynamics::SkeletonPtr skel = Manager::skeleton(wid, skid);
    dart::dynamics::BodyNode* bn = skel->getBodyNode(bid);
    const Eigen::Vector6d& x = bn->getCOMSpatialVelocity();
    for (int i = 0; i < x.size(); i++) {
        outv6[i] = x(i);
    }
}

void getBodyNodeWorldCOMSpatialAcceleration(int wid, int skid, int bid, double outv6[6]) {
    dart::dynamics::SkeletonPtr skel = Manager::skeleton(wid, skid);
    dart::dynamics::BodyNode* bn = skel->getBodyNode(bid);
    const Eigen::Vector6d& x = bn->getCOMSpatialAcceleration();
    // cout << "C = " << bn->getCOM().transpose() << endl;
    // cout << "x = " << x.transpose() << endl;
    for (int i = 0; i < x.size(); i++) {
        outv6[i] = x(i);
    }
}

void getBodyNodeLocalCOMSpatialVelocity(int wid, int skid, int bid, double outv6[6]) {
    dart::dynamics::SkeletonPtr skel = Manager::skeleton(wid, skid);
    dart::dynamics::BodyNode* bn = skel->getBodyNode(bid);
    const dart::dynamics::Frame* world = dart::dynamics::Frame::World();
    const Eigen::Vector6d& x = bn->getCOMSpatialVelocity(world, bn);
    for (int i = 0; i < x.size(); i++) {
        outv6[i] = x(i);
    }
}

void getBodyNodeLocalCOMSpatialAcceleration(int wid, int skid, int bid, double outv6[6]) {
    dart::dynamics::SkeletonPtr skel = Manager::skeleton(wid, skid);
    dart::dynamics::BodyNode* bn = skel->getBodyNode(bid);
    const dart::dynamics::Frame* world = dart::dynamics::Frame::World();
    const Eigen::Vector6d& x = bn->getCOMSpatialAcceleration(world, bn);
    for (int i = 0; i < x.size(); i++) {
        outv6[i] = x(i);
    }
}

int getBodyNodeNumContacts(int wid, int skid, int bid) {
    dart::dynamics::SkeletonPtr skel = Manager::skeleton(wid, skid);
    // dart::dynamics::BodyNode* bn = skel->getBodyNode(bid);
    dart::dynamics::BodyNodePtr bn = skel->getBodyNode(bid);

    dart::simulation::WorldPtr world = Manager::world(wid);
    dart::collision::CollisionDetector* cd =
        world->getConstraintSolver()->getCollisionDetector();
    size_t n = cd->getNumContacts();
    int cnt = 0;
    for (size_t i = 0; i < n; i++) {
        dart::collision::Contact& c = cd->getContact(i);
        if (c.bodyNode1.lock() == bn || c.bodyNode2.lock() == bn) {
            cnt++;
        }
    }
    return cnt;
}

void getBodyNodeContacts(int wid, int skid, int bid, double* outv, int len) {
    dart::dynamics::SkeletonPtr skel = Manager::skeleton(wid, skid);
    dart::dynamics::BodyNodePtr bn = skel->getBodyNode(bid);

    dart::simulation::WorldPtr world = Manager::world(wid);
    dart::collision::CollisionDetector* cd =
        world->getConstraintSolver()->getCollisionDetector();
    size_t n = cd->getNumContacts();

    int m = 0;
    for (size_t i = 0; i < n; i++) {
        dart::collision::Contact& c = cd->getContact(i);
        if (c.bodyNode1.lock() != bn && c.bodyNode2.lock() != bn) {
            continue;
        }
        m++;
    }

    if (7 * m != len) {
        cerr << "getBodyNodeContacts: 7m is needed for the output vector. m = " << m
             << ", n = " << n << ", len =  " << len << endl;
        return;
    }

    int ptr = 0;
    for (size_t i = 0; i < n; i++) {
        dart::collision::Contact& c = cd->getContact(i);
        if (c.bodyNode1.lock() != bn && c.bodyNode2.lock() != bn) {
            continue;
        }
        Eigen::Vector3d v = cd->getContact(i).point;
        Eigen::Vector3d f = cd->getContact(i).force;
        for (int j = 0; j < 3; j++) {
            outv[ptr++] = v(j);
        }
        for (int j = 0; j < 3; j++) {
            outv[ptr++] = f(j);
        }
        outv[ptr++] = i;

    }
}

void getBodyNodeTransformation(int wid, int skid, int bid, double outv44[4][4]) {
    using namespace dart::dynamics;
    SkeletonPtr skel = Manager::skeleton(wid, skid);
    BodyNode* body = skel->getBodyNode(bid);
    if (!body) {
        cerr << "cannot find the body : " << bid << endl;
    }
    const Eigen::Isometry3d& T = body->getTransform();
    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) {
            outv44[i][j] = T(i, j);
        }
    }
}

void getBodyNodeWorldLinearJacobian(int wid, int skid, int bid, double inv3[3], double* array2, int nrows, int ncols) {
    using namespace dart::dynamics;
    SkeletonPtr skel = Manager::skeleton(wid, skid);
    BodyNode* body = skel->getBodyNode(bid);
    if (!body) {
        cerr << "cannot find the body : " << bid << endl;
    }
    Eigen::Vector3d offset(inv3[0], inv3[1], inv3[2]);
    
    int N = skel->getNumDofs();
    // Eigen::MatrixXd J = body->getWorldLinearJacobian(offset);
    dart::math::LinearJacobian J = body->getLinearJacobian(offset);
    Eigen::MatrixXd JF = Eigen::MatrixXd::Zero(3, N);

    for (int i = 0; i < J.cols(); i++) {
        int j = body->getDependentGenCoordIndex(i);
        JF.col(j) = J.col(i);
    }

    int ptr = 0;
    for (int i = 0; i < JF.rows(); i++) {
        for (int j = 0; j < JF.cols(); j++) {
            array2[ptr++] = JF(i, j);
        }
    }
}

void addBodyNodeExtForce(int wid, int skid, int bid, double inv3[3]) {
    using namespace dart::dynamics;
    SkeletonPtr skel = Manager::skeleton(wid, skid);
    BodyNode* body = skel->getBodyNode(bid);
    if (!body) {
        cerr << "cannot find the body : " << bid << endl;
    }
    Eigen::Vector3d f(inv3[0], inv3[1], inv3[2]);
    body->addExtForce(f);
}

void addBodyNodeExtForceAt(int wid, int skid, int bid, double inv3[3], double inv3_2[3]) {
    using namespace dart::dynamics;
    SkeletonPtr skel = Manager::skeleton(wid, skid);
    BodyNode* body = skel->getBodyNode(bid);
    if (!body) {
        cerr << "cannot find the body : " << bid << endl;
    }
    Eigen::Vector3d f(inv3[0], inv3[1], inv3[2]);
    Eigen::Vector3d offset(inv3_2[0], inv3_2[1], inv3_2[2]);
    body->addExtForce(f, offset);
}

////////////////////////////////////////////////////////////////////////////////
// Marker Functions
int getBodyNodeNumMarkers(int wid, int skid, int bid) {
    using namespace dart::dynamics;
    SkeletonPtr skel = Manager::skeleton(wid, skid);
    BodyNode* body = skel->getBodyNode(bid);
    return body->getNumMarkers();
}

void getMarkerLocalPosition(int wid, int skid, int bid, int mid, double outv3[3]) {
    using namespace dart::dynamics;
    SkeletonPtr skel = Manager::skeleton(wid, skid);
    BodyNode* body = skel->getBodyNode(bid);
    Marker* marker = body->getMarker(mid);
    if (!marker) {
        cerr << "cannot find the marker : " << mid << endl;
    }
    const Eigen::Vector3d& x = marker->getLocalPosition();
    for (int i = 0; i < x.size(); i++) {
        outv3[i] = x(i);
    }
}

void getMarkerPosition(int wid, int skid, int bid, int mid, double outv3[3]) {
    using namespace dart::dynamics;
    SkeletonPtr skel = Manager::skeleton(wid, skid);
    BodyNode* body = skel->getBodyNode(bid);
    Marker* marker = body->getMarker(mid);
    if (!marker) {
        cerr << "cannot find the marker : " << mid << endl;
    }
    const Eigen::Vector3d& x = marker->getWorldPosition();
    for (int i = 0; i < x.size(); i++) {
        outv3[i] = x(i);
    }
}

////////////////////////////////////////////////////////////////////////////////
// C3D Functions
int readC3D(const char* const path, double* outv, int len) {
    dart::utils::FileInfoC3D file;
    file.loadFile(path);
    int nm = file.getNumMarkers();
    int nf = file.getNumFrames();
    if (len == 0) {
        cout << " [pydart_api] # Markers = " << nm << endl;
        cout << " [pydart_api] # Frames = " << nf << endl;
        return 3 * nf * nm + 2; // return buffer size
    } else if (len == 3 * nf * nm + 2) {
        const int DIM = 3;
        int ptr = 0;
        outv[ptr++] = nf;
        outv[ptr++] = nm;
        for (int i = 0; i < nf; i++) {
            for (int j = 0; j < nm; j++) {
                Eigen::Vector3d m = file.getDataAt(i, j);
                for (int k = 0; k < DIM; k++) {
                    outv[ptr++] = m[k];
                    // cout << ptr << " / " << i << " " << j << " " << k << " " << len << endl;
                }
            }
        }
        cout << " [pydart_api] load C3D [" << path << "] OK" << endl;
        return 0;
    } else {
        cerr << "invalid buffer size: " << len << endl;
        return -1;
    }
}
