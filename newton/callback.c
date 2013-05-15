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

void SetUserRayCastCallback(NewtonCollision* treeCollision) {
	NewtonTreeCollisionSetUserRayCastCallback(treeCollision, 
		(NewtonCollisionTreeRayCastCallback)goCollisionTreeRayCastCallback);
}

void SetStaticCollisionDebugCallback(NewtonCollision* staticCollision) {
	NewtonStaticCollisionSetDebugCallback(staticCollision, (NewtonTreeCollisionCallback)goTreeCollisionCallback);
}

void SetBodyDestructor(NewtonBody* body) {
	NewtonBodySetDestructorCallback(body, (NewtonBodyDestructor)goBodyDestructor);
}

void SetTransformCallback(NewtonBody* body) {
	NewtonBodySetTransformCallback(body, (NewtonSetTransform)goTransformCallback);
}

void SetForceAndTorqueCallback(NewtonBody* body) {
	NewtonBodySetForceAndTorqueCallback(body, (NewtonApplyForceAndTorque)goApplyForceAndTorque);
}

void AddBuoyancyForce(NewtonBody* body, dFloat fluidDensity, dFloat fluidLinearViscosity, 
			dFloat fluidAngularViscosity, dFloat* gravityVector , void* context) {
	NewtonBodyAddBuoyancyForce(body, fluidDensity, fluidLinearViscosity, fluidAngularViscosity,
			gravityVector, (NewtonGetBuoyancyPlane)goBuoyancyPlaneCallback, context);
}

void SetConstraintDestructor(NewtonJoint* joint) {
	NewtonJointSetDestructor(joint, (NewtonConstraintDestructor)goConstraintDestructor);
}

void BallSetUserCallback(NewtonJoint* joint) {
	NewtonBallSetUserCallback(joint, (NewtonBallCallback)goBallCallback);
}

void HingeSetUserCallback(NewtonJoint* joint) {
	NewtonHingeSetUserCallback(joint, (NewtonHingeCallback)goHingeCallback);
}

void SliderSetUserCallback(NewtonJoint* joint) {
	NewtonSliderSetUserCallback(joint, (NewtonSliderCallback)goSliderCallback);
}

void CorkscrewSetUserCallback(NewtonJoint* joint) {
	NewtonCorkscrewSetUserCallback(joint, (NewtonCorkscrewCallback)goCorkscrewCallback);
}

void UniversalSetUserCallback(NewtonJoint* joint) {
	NewtonUniversalSetUserCallback(joint, (NewtonUniversalCallback)goUniversalCallback);
}

NewtonMesh* MeshSimplify(NewtonMesh* mesh, int maxVertextCount) {
	return NewtonMeshSimplify(mesh, maxVertextCount, (NewtonReportProgress)goReportProgress);
}
NewtonMesh* MeshApproximateConvexDecomposition(NewtonMesh* mesh, dFloat maxConcavity, dFloat backFaceDistanceFactor, int maxCount, int maxVertexPerHull) {
	return NewtonMeshApproximateConvexDecomposition(mesh, maxConcavity, backFaceDistanceFactor, maxCount, maxVertexPerHull, (NewtonReportProgress)goReportProgress);
}

void setForEachPolygonDo(NewtonCollision* collision, dFloat* matrix, void* userData) {
	NewtonCollisionForEachPolygonDo(collision, matrix, (NewtonCollisionIterator)goNewtonCollisionIterator, userData);
}

NewtonCollision* createCollisionFromSerialization(NewtonWorld* world ,void* handle) {
	return NewtonCreateCollisionFromSerialization(world, (NewtonDeserializeCallback)goNewtonDeserializeCallback, handle);	
}

void serializeCollision(NewtonWorld* world, NewtonCollision* collision, void* handle) {
	NewtonCollisionSerialize(world, collision, (NewtonSerializeCallback)goNewtonSerializeCallback, handle);
}
