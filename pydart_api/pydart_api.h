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

// Manipulation Functions
int createWorld(double timestep);
int createWorldFromSkel(const char* const path);
void destroyWorld(int wid);
int addSkeleton(int wid, const char* const path, double frictionCoeff = 1.0);
int numSkeletons(int wid);
void setSkeletonJointDamping(int wid, int skid, double damping);

// Simulation Functions
void resetWorld(int wid);
void stepWorld(int wid);
void render(int wid);
void renderSkeleton(int wid, int skid);
void renderSkeletonWithColor(int wid, int skid, double r, double g, double b, double a);

// World Functions
double getWorldTime(int wid);
double getWorldTimeStep(int wid);
void setWorldTimeStep(int wid, double _timeStep);
int getWorldSimFrames(int wid);
void setWorldSimFrame(int wid, int playFrame);
int getWorldNumContacts(int wid);
void getWorldContacts(int wid, double* outv, int len);

// Skeleton Attribute Functions
double getSkeletonMass(int wid, int skid);
int getSkeletonNumBodies(int wid, int skid);
int getSkeletonNumDofs(int wid, int skid);
const char* getSkeletonBodyName(int wid, int skid, int bodyid);
const char* getSkeletonDofName(int wid, int skid, int dofid);

// Skeleton Pose Functions
void getSkeletonPositions(int wid, int skid, double* outpose, int ndofs);
void getSkeletonVelocities(int wid, int skid, double* outpose, int ndofs);
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
void getBodyNodeLocalCOM(int wid, int skid, int bid, double outv3[3]);
void getBodyNodeWorldCOM(int wid, int skid, int bid, double outv3[3]);
void getBodyNodeWorldCOMVelocity(int wid, int skid, int bid, double outv3[3]);
int getBodyNodeNumContacts(int wid, int skid, int bid);
void getBodyNodeContacts(int wid, int skid, int bid, double* outv, int len);
void getBodyNodeTransformation(int wid, int skid, int bid, double outv44[4][4]);
void getBodyNodeWorldLinearJacobian(int wid, int skid, int bid, double inv3[3], double* array2, int nrows, int ncols);
void addBodyNodeExtForce(int wid, int skid, int bid, double inv3[3]);
void addBodyNodeExtForceAt(int wid, int skid, int bid, double inv3[3], double inv3_2[3]);


#endif // #ifndef PYDART_PYDART_API_H

