// Copyright 2012 Tim Shannon. All rights reserved.  
// Use of this source code is governed by the MIT license 
// that can be found in the LICENSE file.  
package newton

/*
#cgo   linux LDFLAGS: -L/usr/local/lib -lNewton -lstdc++
#include "Newton.h"
#include "callback.h"
#include <stdlib.h>
*/
import "C"
import (
	"unsafe"
)

//A note on callbacks.  Since C callbacks must basically
// be hardcoded to one function, I have to do a look up
// in the function to the proper function pointer based on
// the limited data in that callback.  
// callbackOwners will contain a map of functions and their owners
// so the proper callback can be looked up on the return into Go code
// using only the owners pointer

type owner unsafe.Pointer

//goFloats creates a float32 slice of the given size of the passed in c array pointer
func goFloats(cArray *C.dFloat, size int) []float32 {
	//Copy for now.  Maybe we'll try with using the slice header to repoint the data
	// instead if preformance thrashes with this
	// Maybe use a temp array stored on the callback owner
	slice := make([]float32, size)
	C.CopyFloatArray(cArray, (*C.dFloat)(&slice[0]), C.int(size))
	return slice
}

type GetTicksCountHandler func() uint

var getTicksCount GetTicksCountHandler

//export goGetTicksCountCB
func goGetTicksCountCB() uint {
	return getTicksCount()
}

func (w *World) SetPerformanceClock(f GetTicksCountHandler) {
	getTicksCount = f
	C.setGetTicksCountCB(w.handle)
}

type BodyLeaveWorldHandler func(body *Body, threadIndex int)

var bodyLeaveWorldOwners = make(map[owner]BodyLeaveWorldHandler)

//export goBodyLeaveWorldCB
func goBodyLeaveWorldCB(body *C.NewtonBody, threadIndex C.int) {
	b := &Body{body}

	//owner is world, look up world from body
	// unfortunately needs an additional c call
	bodyLeaveWorldOwners[owner(b.World().handle)](b, int(threadIndex))
}

func (w *World) SetBodyLeaveWorldEvent(f BodyLeaveWorldHandler) {
	bodyLeaveWorldOwners[owner(w.handle)] = f
	C.setBodyLeaveWorldCB(w.handle)
}

type JointIteratorHandler func(joint *Joint, userData interface{})

var jointIterator JointIteratorHandler

//export goJointIteratorCB
func goJointIteratorCB(joint *C.NewtonJoint, userData unsafe.Pointer) {
	j := &Joint{joint}

	jointIterator(j, (interface{})(userData))
}

func (w *World) ForEachJointDo(f JointIteratorHandler, userData interface{}) {
	jointIterator = f
	C.setJointIteratorCB(w.handle, unsafe.Pointer(&userData))
}

type BodyIteratorHandler func(body *Body, userData interface{})

var bodyIterator BodyIteratorHandler

//export goBodyIteratorCB
func goBodyIteratorCB(body *C.NewtonBody, userData unsafe.Pointer) {
	b := &Body{body}
	bodyIterator(b, (interface{})(userData))
}

func (w *World) ForEachBodyInAABBDo(p0, p1 []float32, f BodyIteratorHandler, userData interface{}) {
	bodyIterator = f
	C.setBodyIteratorCB(w.handle, (*C.dFloat)(&p0[0]), (*C.dFloat)(&p1[0]), unsafe.Pointer(&userData))
}

type RayFilterHandler func(body *Body, hitNormal []float32, collisionID int,
	userData interface{}, intersectParam float32) float32

var rayFilter RayFilterHandler

//export goRayFilterCB
func goRayFilterCB(body *C.NewtonBody, hitNormal *C.dFloat, collisionID C.int,
	userData unsafe.Pointer, intersectParam C.dFloat) C.dFloat {
	b := &Body{body}

	return C.dFloat(rayFilter(b, goFloats(hitNormal, 3), int(collisionID),
		(interface{})(userData), float32(intersectParam)))
}

type RayPrefilterHandler func(body *Body, collision *Collision, userData interface{}) uint

var rayPrefilter RayPrefilterHandler

//export goRayPrefilterCB
func goRayPrefilterCB(body *C.NewtonBody, collision *C.NewtonCollision, userData unsafe.Pointer) C.unsigned {
	b := &Body{body}
	gCollision := &Collision{collision}

	return C.unsigned(rayPrefilter(b, gCollision, (interface{})(userData)))
}

func (w *World) RayCast(p0 []float32, p1 []float32, filter RayFilterHandler, userData interface{},
	prefilter RayPrefilterHandler) {
	rayFilter = filter
	rayPrefilter = prefilter
	C.RayCast(w.handle, (*C.dFloat)(&p0[0]), (*C.dFloat)(&p1[0]), unsafe.Pointer(&userData))
}

type ConvexCastReturnInfo struct {
	Point            []float32
	Normal           []float32
	NormalOnHitPoint []float32
	Penetration      float32
	ContactID        int
	HitBody          *Body
}

// NewtonWorldCollide: obsolete

func (w *World) ConvexCast(matrix []float32, target []float32, shape *Collision, hitParam *float32,
	userData interface{}, prefilter RayPrefilterHandler, maxContactsCount int, threadIndex int) []*ConvexCastReturnInfo {

	rayPrefilter = prefilter

	var size int
	//all of this allocation may be a performance issue
	// might have to reapproach this
	cInfo := make([]C.NewtonWorldConvexCastReturnInfo, maxContactsCount)

	size = int(C.ConvexCast(w.handle, (*C.dFloat)(&matrix[0]), (*C.dFloat)(&target[0]), shape.handle,
		(*C.dFloat)(hitParam), unsafe.Pointer(&userData), &cInfo[0], C.int(maxContactsCount),
		C.int(threadIndex)))

	returnInfo := make([]*ConvexCastReturnInfo, size)

	for i := range returnInfo {
		returnInfo[i] = &ConvexCastReturnInfo{
			Point:            goFloats(&cInfo[i].m_point[0], 4),
			Normal:           goFloats(&cInfo[i].m_normal[0], 4),
			NormalOnHitPoint: goFloats(&cInfo[i].m_normalOnHitPoint[0], 4),
			Penetration:      float32(cInfo[i].m_penetration),
			ContactID:        int(cInfo[i].m_contactID),
			HitBody:          &Body{cInfo[i].m_hitBody},
		}

	}

	return returnInfo
}

type OnAABBOverlapHandler func(material *Material, body0, body1 *Body, threadIndex int) int

//Not associated to a world for now, this may be an issue with
// multiple worlds
var onAABBOverlap OnAABBOverlapHandler

//export goOnAABBOverlapCB
func goOnAABBOverlapCB(material *C.NewtonMaterial, body0, body1 *C.NewtonBody, threadIndex C.int) C.int {
	gMaterial := &Material{material}
	b0 := &Body{body0}
	b1 := &Body{body1}

	return C.int(onAABBOverlap(gMaterial, b0, b1, int(threadIndex)))
}

type ContactsProcessHandler func(contact *Joint, timestep float32, threadIndex int)

var contactsProcess ContactsProcessHandler

//export goContactsProcessCB
func goContactsProcessCB(contact *C.NewtonJoint, timestep C.dFloat, threadIndex C.int) {
	j := &Joint{contact}
	contactsProcess(j, float32(timestep), int(threadIndex))
}

func (w *World) SetMaterialCollisionCallback(matid0, matid1 int, userData interface{},
	overlap OnAABBOverlapHandler, contactsProcessor ContactsProcessHandler) {
	onAABBOverlap = overlap
	contactsProcess = contactsProcessor
	C.SetCollisionCB(w.handle, C.int(matid0), C.int(matid1), unsafe.Pointer(&userData))
}

type MeshCollisionCollideDesc struct {
	M_boxP0               []float32
	M_boxP1               []float32
	M_boxDistanceTravel   []float32
	M_threadNumber        int
	M_faceCount           int
	M_vertexStrideInBytes int
	M_skinThickness       float32
	M_userData            interface{}
	M_objBody             *Body
	M_polySoupBody        *Body
	M_objCollision        *Collision
	M_polySoupCollision   *Collision
	M_vertex              []float32
	M_faceIndexCount      *int
	M_faceVertexIndex     *int
}

//Skip user mesh for now
//If implemented...
//Separate New user Mesh creation from some callbacks, so that if a user doesn't want to use
// the callbacks, they aren't tied to the very expensive conversions that would be
// tied to the global go callbacks

//type MeshCollisionCollideHandler func(collideDescData *MeshCollisionCollideDesc)

//export goMeshCollisionCollideCB
//func goMeshCollisionCollideCB(collideDescData *C.NewtonUserMeshCollisionCollideDesc) {
//goCollideDescData := &MeshCollisionCollideDesc{

//}
//gMeshCollisionCollide(
//}

//Skip heightfields for now

type CollisionTreeRayCastCallback func(body *Body, treeCollision *Collision, interception float32,
	normal []float32, faceId int, userData interface{}) float32

var collisionTreeRayOwners = make(map[owner]CollisionTreeRayCastCallback)

//export goCollisionTreeRayCastCallback
func goCollisionTreeRayCastCallback(body *C.NewtonBody, treeCollision *C.NewtonCollision, interception C.dFloat,
	normal *C.dFloat, faceId C.int, userData unsafe.Pointer) C.dFloat {
	b := &Body{body}
	col := &Collision{handle: treeCollision}

	return C.dFloat(collisionTreeRayOwners[owner(treeCollision)](b, col, float32(interception), goFloats(normal, 3),
		int(faceId), (interface{})(userData)))
}

func (c *Collision) SetTreeRayCastCallback(callback CollisionTreeRayCastCallback) {
	collisionTreeRayOwners[owner(c.handle)] = callback

	C.SetUserRayCastCallback(c.handle)
}

type TreeCollisionCallback func(bodyWithTreeCollision, body *Body, faceID, vertexCount int,
	vertex []float32, vertexStrideInBytes int)

var treeCollisionCallback TreeCollisionCallback

//export goTreeCollisionCallback
func goTreeCollisionCallback(bodyWithTreeCollision, body *C.NewtonBody, faceID, vertextCount C.int,
	vertex *C.dFloat, vertexStrideInBytes C.int) {
	bWithTreeCollision := &Body{bodyWithTreeCollision}
	b := &Body{body}
	treeCollisionCallback(bWithTreeCollision, b, int(faceID), int(vertextCount),
		goFloats(vertex, int(vertextCount)), int(vertexStrideInBytes))
}

func StaticCollisionSetDebugCallback(staticCollision *Collision, userCallback TreeCollisionCallback) {
	treeCollisionCallback = userCallback
	C.SetStaticCollisionDebugCallback(staticCollision.handle)
}

type BodyDestructorCallback func(body *Body)

var bodyDestructorCallbackOwners = make(map[owner]BodyDestructorCallback)

//export goBodyDestructor
func goBodyDestructor(body *C.NewtonBody) {
	b := &Body{body}

	bodyDestructorCallbackOwners[owner(body)](b)
}

func (b *Body) SetDestructorCallback(callback BodyDestructorCallback) {
	bodyDestructorCallbackOwners[owner(b.handle)] = callback
	C.SetBodyDestructor(b.handle)
}

func (b *Body) DestructorCallback() BodyDestructorCallback {
	return bodyDestructorCallbackOwners[owner(b.handle)]
}

type TransformCallback func(body *Body, matrix []float32, threadIndex int)

var transformCallbackOwners = make(map[owner]TransformCallback)

//export goTransformCallback
func goTransformCallback(body *C.NewtonBody, matrix *C.dFloat, threadIndex C.int) {
	b := &Body{body}

	transformCallbackOwners[owner(body)](b, goFloats(matrix, 16), int(threadIndex))
}

func (b *Body) SetTransformCallback(callback TransformCallback) {
	transformCallbackOwners[owner(b.handle)] = callback
	C.SetTransformCallback(b.handle)
}

func (b *Body) TransformCallback() TransformCallback {
	return transformCallbackOwners[owner(b.handle)]
}

type ApplyForceAndTorque func(body *Body, timestep float32, threadIndex int)

var applyForceAndTorqueOwners = make(map[owner]ApplyForceAndTorque)

//export goApplyForceAndTorque
func goApplyForceAndTorque(body *C.NewtonBody, timestep C.dFloat, threadIndex C.int) {
	b := &Body{body}

	applyForceAndTorqueOwners[owner(body)](b, float32(timestep), int(threadIndex))
}

func (b *Body) SetForceAndTorqueCallback(callback ApplyForceAndTorque) {
	applyForceAndTorqueOwners[owner(b.handle)] = callback

	C.SetForceAndTorqueCallback(b.handle)
}

func (b *Body) ForceAndTorqueCallback() ApplyForceAndTorque {
	return applyForceAndTorqueOwners[owner(b.handle)]
}

type BuoyancyPlaneHandler func(collisionID int, context interface{}, globalSpaceMatrix, globalSpacePlane []float32) int

var getBuoyancyPlane BuoyancyPlaneHandler

//export goBuoyancyPlaneCallback
func goBuoyancyPlaneCallback(collisionID C.int, context unsafe.Pointer, globalSpaceMatrix,
	globalSpacePlane *C.dFloat) C.int {

	return C.int(getBuoyancyPlane(int(collisionID), (interface{})(context), goFloats(globalSpaceMatrix, 16),
		goFloats(globalSpacePlane, 16)))
}

func (b *Body) AddBuoyancyForce(fluidDensity, fluidLinearViscosity, fluidAngularViscosity float32,
	gravityVector []float32, buoyancyPlane BuoyancyPlaneHandler, context interface{}) {
	getBuoyancyPlane = buoyancyPlane

	C.AddBuoyancyForce(b.handle, C.dFloat(fluidDensity), C.dFloat(fluidLinearViscosity),
		C.dFloat(fluidAngularViscosity), (*C.dFloat)(&gravityVector[0]), unsafe.Pointer(&context))
}
