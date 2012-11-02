// Copyright 2012 Tim Shannon. All rights reserved.  
// Use of this source code is governed by the MIT license 
// that can be found in the LICENSE file.  

#ifndef _CALLBACK_H_
#define _CALLBACK_H_

#include <stdlib.h>
//dll for win32?
#include "Newton.h"

extern unsigned goGetTicksCountCB(void);
extern void goBodyLeaveWorldCB(NewtonBody* , int);
extern void goJointIteratorCB(NewtonJoint*, void*);
extern void goBodyIteratorCB(NewtonBody*, void*);
extern dFloat goRayFilterCB(NewtonBody*, dFloat*, int, void*, dFloat); 
extern unsigned goRayPrefilterCB(NewtonBody*, NewtonCollision*, void*);
extern int goOnAABBOverlapCB(NewtonMaterial*,NewtonBody*, NewtonBody*, int);
extern void goContactsProcessCB(NewtonJoint*, dFloat, int);
extern dFloat goCollisionTreeRayCastCallback(NewtonBody*,NewtonCollision*, dFloat,
	dFloat*, int, void*);
extern void goTreeCollisionCallback(NewtonBody*, NewtonBody*, int, int,
	dFloat*, int);
extern void goBodyDestructor(NewtonBody*); 
extern void goTransformCallback(NewtonBody*, dFloat*, int); 
extern void goApplyForceAndTorque(NewtonBody*, dFloat, int);
extern int goBuoyancyPlaneCallback(int, void*, dFloat*, dFloat*);
extern void goConstraintDestructor(NewtonJoint*);
extern void goBallCallback(NewtonJoint*,dFloat); 
extern unsigned goHingeCallback(NewtonJoint*,NewtonHingeSliderUpdateDesc*); 
extern unsigned goSliderCallback(NewtonJoint*,NewtonHingeSliderUpdateDesc*); 
extern unsigned goCorkscrewCallback(NewtonJoint*,NewtonHingeSliderUpdateDesc*); 
extern unsigned goUniversalCallback(NewtonJoint*,NewtonHingeSliderUpdateDesc*); 

void setGetTicksCountCB(NewtonWorld*);
void setBodyLeaveWorldCB(NewtonWorld*);
void setJointIteratorCB(NewtonWorld*, void*);
void setBodyIteratorCB(NewtonWorld*,dFloat*, dFloat*, void*);
void RayCast(NewtonWorld*, dFloat*, dFloat*, void*);
int ConvexCast(NewtonWorld*, dFloat*, dFloat*, NewtonCollision*, dFloat*, void*, 
		NewtonWorldConvexCastReturnInfo*, int, int);
void SetCollisionCB(NewtonWorld*, int, int, void*);
void SetUserRayCastCallback(NewtonCollision*);
void SetStaticCollisionDebugCallback(NewtonCollision*);
void SetBodyDestructor(NewtonBody*); 
void SetTransformCallback(NewtonBody*);
void SetForceAndTorqueCallback(NewtonBody*);
void AddBuoyancyForce(NewtonBody*, dFloat, dFloat, dFloat, dFloat*, void*);
void SetConstraintDestructor(NewtonJoint*);
void BallSetUserCallback(NewtonJoint*);
void HingeSetUserCallback(NewtonJoint*);
void SliderSetUserCallback(NewtonJoint*);
void CorkscrewSetUserCallback(NewtonJoint*);
void UniversalSetUserCallback(NewtonJoint*);



//helpers
void CopyFloatArray(dFloat*, dFloat*, int); 
void CopyShortArray(short*, short*, int); 
#endif //_CALLBACK_H_
