// Copyright 2012 Tim Shannon. All rights reserved.  
// Use of this source code is governed by the MIT license 
// that can be found in the LICENSE file.  

#include "callback.h"


void setGetTicksCountCB(NewtonWorld* newtonWorld) {
	NewtonSetPerformanceClock(newtonWorld, goGetTicksCountCB);
}

void setBodyLeaveWorldCB(NewtonWorld* newtonWorld) {
	NewtonSetBodyLeaveWorldEvent(newtonWorld, (NewtonBodyLeaveWorld)goBodyLeaveWorldCB);
}

void setJointIteratorCB(NewtonWorld* newtonWorld, void* userData) {
	NewtonWorldForEachJointDo(newtonWorld, (NewtonJointIterator)goJointIteratorCB, userData);
}

void setBodyIteratorCB(NewtonWorld* newtonWorld, dFloat* p0, dFloat* p1, void* userData) {
	NewtonWorldForEachBodyInAABBDo(newtonWorld, p0, p1, 
			(NewtonBodyIterator)goBodyIteratorCB, userData);
}

void RayCast(NewtonWorld* world, dFloat* p0, dFloat* p1, void* userData) {
	NewtonWorldRayCast(world, p0, p1, (NewtonWorldRayFilterCallback)goRayFilterCB, 
			userData, (NewtonWorldRayPrefilterCallback)goRayPrefilterCB);
}

int ConvexCast(NewtonWorld* world, dFloat* matrix, dFloat* target, NewtonCollision* shape, 
		dFloat* hitParam, void* userData, NewtonWorldConvexCastReturnInfo* info,
		 int maxContactsCount, int threadIndex) {
	NewtonWorldConvexCast(world, matrix, target, shape, hitParam, userData, 
		(NewtonWorldRayPrefilterCallback)goRayPrefilterCB, info, maxContactsCount, threadIndex);

}

void SetCollisionCB(NewtonWorld* world, int id0, int id1, void* userData) {
	NewtonMaterialSetCollisionCallback(world, id0, id1, userData, (NewtonOnAABBOverlap)goOnAABBOverlapCB, 
		(NewtonContactsProcess)goContactsProcessCB);
}

void CopyFloatArray(dFloat* src, dFloat* dest, int len) {
	int i;
	for(i = 0; i < len; i++) 
		dest[i] = src[i];
}
