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

void setGetTicksCountCB(NewtonWorld*);
void setBodyLeaveWorldCB(NewtonWorld*);
void setJointIteratorCB(NewtonWorld*, void*);
void setBodyIteratorCB(NewtonWorld*,dFloat*, dFloat*, void*);
void RayCast(NewtonWorld*, dFloat*, dFloat*, void*);
int ConvexCast(NewtonWorld*, dFloat*, dFloat*, NewtonCollision*, dFloat*, void*, 
		NewtonWorldConvexCastReturnInfo*, int, int);
void SetCollisionCB(NewtonWorld*, int, int, void*);





//helpers
void CopyFloatArray(dFloat*, dFloat*, int); 
#endif //_CALLBACK_H_
