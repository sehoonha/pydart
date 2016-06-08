/* RTQL8, Copyright (c) 2011, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Author(s): Sehoon Ha <sha9@gatech.edu>
 * Georgia Tech Graphics Lab
 */

#ifndef PYDART_PYDART_API_H
#define PYDART_PYDART_API_H

// Init Functions
void init();
void destroy();

// World Manipulation Functions
int createWorld(double timestep);
int createWorldFromSkel(const char* const path);
void destroyWorld(int wid);
int saveWorldToFile(int wid, const char* const path);
int addSkeleton(int wid, const char* const path, double frictionCoeff = 1.0, int traditionalMode=0);
int numSkeletons(int wid);
void setSkeletonJointDamping(int wid, int skid, double damping);

// Collision Handler Functions
void setCollisionDetector(int wid, int detector_type);
void printCollisionDetector(int wid);

// Simulation Functions
void resetWorld(int wid);
void stepWorld(int wid);
void checkCollisionWorld(int wid, int _checkAllCollisions);
void render(int wid);
void renderSkeleton(int wid, int skid);
void renderSkeletonWithColor(int wid, int skid, double r, double g, double b, double a);
void renderSkeletonMarkers(int wid, int skid);

// World Functions
double getWorldTime(int wid);
double getWorldTimeStep(int wid);
void setWorldTimeStep(int wid, double _timeStep);
int getWorldSimFrames(int wid);
void setWorldSimFrame(int wid, int playFrame);
int getWorldNumContacts(int wid);
void getWorldContacts(int wid, double* outv, int len);
void setWorldCollisionPair(int wid, int skid1, int bid1, int skid2, int bid2, int bEnable);

// Skeleton Attribute Functions
const char* getSkeletonName(int wid, int skid);
double getSkeletonMass(int wid, int skid);
int getSkeletonNumBodies(int wid, int skid);
int getSkeletonNumDofs(int wid, int skid);
const char* getSkeletonBodyName(int wid, int skid, int bodyid);
const char* getSkeletonDofName(int wid, int skid, int dofid);
int getSkeletonMobile(int wid, int skid);
void setSkeletonMobile(int wid, int skid, int mobile);
void setSkeletonSelfCollision(int wid, int skid, int bSelfCollision, int bAdjacentBodies);
void changeRootJointToTransAndEuler(int wid, int skid);

// Skeleton Pose Functions
void getSkeletonPositions(int wid, int skid, double* outpose, int ndofs);
void getSkeletonVelocities(int wid, int skid, double* outpose, int ndofs);
void getSkeletonPositionDifferences(int wid, int skid, double* inpose, int ndofs,
                                    double* inpose2, int ndofs2, double* outpose3, int ndofs3);
void getSkeletonVelocityDifferences(int wid, int skid, double* inpose, int ndofs,
                                    double* inpose2, int ndofs2, double* outpose3, int ndofs3);
void getSkeletonMassMatrix(int wid, int skid, double* array2, int nrows, int ncols);
void getSkeletonCoriolisAndGravityForces(int wid, int skid, double* outpose, int ndofs);
void getSkeletonConstraintForces(int wid, int skid, double* outpose, int ndofs);
void setSkeletonPositions(int wid, int skid, double* inpose, int ndofs);
void setSkeletonVelocities(int wid, int skid, double* inpose, int ndofs);
void setSkeletonForces(int wid, int skid, double* intorque, int ndofs);


// Skeleton Limit Functions
void getSkeletonPositionLowerLimit(int wid, int skid, double* outpose, int ndofs);
void getSkeletonPositionUpperLimit(int wid, int skid, double* outpose, int ndofs);
void getSkeletonForceLowerLimit(int wid, int skid, double* outpose, int ndofs);
void getSkeletonForceUpperLimit(int wid, int skid, double* outpose, int ndofs);

// Skeleton Momentum Functions
void getSkeletonWorldCOM(int wid, int skid, double outv3[3]);
void getSkeletonWorldCOMVelocity(int wid, int skid, double outv3[3]);

// BodyNode Functions
double getBodyNodeMass(int wid, int skid, int bid);
void getBodyNodeInertia(int wid, int skid, int bid, double outv33[3][3]);
void getBodyNodeShapeBoundingBoxDim(int wid, int skid, int bid, double outv3[3]);
void getBodyNodeLocalCOM(int wid, int skid, int bid, double outv3[3]);
void getBodyNodeWorldCOM(int wid, int skid, int bid, double outv3[3]);
void getBodyNodeWorldCOMVelocity(int wid, int skid, int bid, double outv3[3]);
void getBodyNodeWorldCOMSpatialVelocity(int wid, int skid, int bid, double outv6[6]);
void getBodyNodeWorldCOMSpatialAcceleration(int wid, int skid, int bid, double outv6[6]);
void getBodyNodeLocalCOMSpatialVelocity(int wid, int skid, int bid, double outv6[6]);
void getBodyNodeLocalCOMSpatialAcceleration(int wid, int skid, int bid, double outv6[6]);
int getBodyNodeNumContacts(int wid, int skid, int bid);
void getBodyNodeContacts(int wid, int skid, int bid, double* outv, int len);
void getBodyNodeTransformation(int wid, int skid, int bid, double outv44[4][4]);
void getBodyNodeWorldLinearJacobian(int wid, int skid, int bid, double inv3[3], double* array2, int nrows, int ncols);
void addBodyNodeExtForce(int wid, int skid, int bid, double inv3[3]);
void addBodyNodeExtForceAt(int wid, int skid, int bid, double inv3[3], double inv3_2[3]);

// Marker Functions
int getBodyNodeNumMarkers(int wid, int skid, int bid);
void getMarkerLocalPosition(int wid, int skid, int bid, int mid, double outv3[3]);
void getMarkerPosition(int wid, int skid, int bid, int mid, double outv3[3]);

// C3D Functions
int readC3D(const char* const path, double* outv, int len);

#endif // #ifndef PYDART_PYDART_API_H

