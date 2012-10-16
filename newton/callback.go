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
// the limited data in that callback.  In order to do this
// I have to track all of the pointers added globally and make
// sure they are cleaned up as well.  It's a little convoluted
// As soon as we can pass a func pointer directly, half of this 
// can go away.

//goFloats creates a float32 slice of the given size of the passed in c array pointer
func goFloats(cArray *C.dFloat, size int) []float32 {
	//Copy for now.  Maybe we'll try with using the slice header to repoint the data
	// instead if preformance thrashes with this
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

//export goBodyLeaveWorldCB
func goBodyLeaveWorldCB(body *C.NewtonBody, threadIndex C.int) {
	b := globalPtr.get(unsafe.Pointer(body)).(*Body)
	b.world.bodyLeaveWorld(b, int(threadIndex))
}

func (w *World) SetBodyLeaveWorldEvent(f BodyLeaveWorldHandler) {
	w.bodyLeaveWorld = f
	C.setBodyLeaveWorldCB(w.handle)
}

type JointIteratorHandler func(joint *Joint, userData interface{})

//export goJointIteratorCB
func goJointIteratorCB(joint *C.NewtonJoint, userData unsafe.Pointer) {
	j := globalPtr.get(unsafe.Pointer(joint)).(*Joint)

	j.body0.world.jointIterator(j, (interface{})(userData))
}

func (w *World) ForEachJointDo(f JointIteratorHandler, userData interface{}) {
	w.jointIterator = f
	C.setJointIteratorCB(w.handle, unsafe.Pointer(&userData))
}

type BodyIteratorHandler func(body *Body, userData interface{})

//export goBodyIteratorCB
func goBodyIteratorCB(body *C.NewtonBody, userData unsafe.Pointer) {
	b := globalPtr.get(unsafe.Pointer(body)).(*Body)
	b.world.bodyIterator(b, (interface{})(userData))
}

func (w *World) ForEachBodyInAABBDo(p0, p1 []float32, f BodyIteratorHandler, userData interface{}) {
	w.bodyIterator = f
	C.setBodyIteratorCB(w.handle, (*C.dFloat)(&p0[0]), (*C.dFloat)(&p1[0]), unsafe.Pointer(&userData))
}

type RayFilterHandler func(body *Body, hitNormal []float32, collisionID int,
	userData interface{}, intersectParam float32) float32

//export goRayFilterCB
func goRayFilterCB(body *C.NewtonBody, hitNormal *C.dFloat, collisionID C.int,
	userData unsafe.Pointer, intersectParam C.dFloat) C.dFloat {
	b := globalPtr.get(unsafe.Pointer(body)).(*Body)

	return C.dFloat(b.world.rayFilter(b, goFloats(hitNormal, 3), int(collisionID),
		(interface{})(userData), float32(intersectParam)))
}

type RayPrefilterHandler func(body *Body, collision *Collision, userData interface{}) uint

//export goRayPrefilterCB
func goRayPrefilterCB(body *C.NewtonBody, collision *C.NewtonCollision, userData unsafe.Pointer) C.unsigned {
	b := globalPtr.get(unsafe.Pointer(body)).(*Body)
	gCollision := &Collision{handle: collision}
	return C.unsigned(b.world.rayPrefilter(b, gCollision, (interface{})(userData)))
}

func (w *World) RayCast(p0 []float32, p1 []float32, filter RayFilterHandler, userData interface{},
	prefilter RayPrefilterHandler) {
	w.rayFilter = filter
	w.rayPrefilter = prefilter
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
	userData *interface{}, prefilter RayPrefilterHandler, maxContactsCount int, threadIndex int) []*ConvexCastReturnInfo {

	w.rayPrefilter = prefilter
	var size int
	//all of this allocation may be a performance issue
	// might have to reapproach this
	cInfo := make([]C.NewtonWorldConvexCastReturnInfo, maxContactsCount)
	size = int(C.ConvexCast(w.handle, (*C.dFloat)(&matrix[0]), (*C.dFloat)(&target[0]), shape.handle,
		(*C.dFloat)(hitParam), unsafe.Pointer(userData), &cInfo[0], C.int(maxContactsCount),
		C.int(threadIndex)))
	returnInfo := make([]*ConvexCastReturnInfo, size)

	for i := range returnInfo {
		returnInfo[i] = &ConvexCastReturnInfo{
			Point:            goFloats(&cInfo[i].m_point[0], 4),
			Normal:           goFloats(&cInfo[i].m_normal[0], 4),
			NormalOnHitPoint: goFloats(&cInfo[i].m_normalOnHitPoint[0], 4),
			Penetration:      float32(cInfo[i].m_penetration),
			ContactID:        int(cInfo[i].m_contactID),
			HitBody:          globalPtr.get(unsafe.Pointer(cInfo[i].m_hitBody)).(*Body),
		}

	}

	return returnInfo
}

type OnAABBOverlapHandler func(material *Material, body0, body1 *Body, threadIndex int) int

//export goOnAABBOverlapCB
func goOnAABBOverlapCB(material *C.NewtonMaterial, body0, body1 *C.NewtonBody, threadIndex C.int) C.int {
	gMaterial := &Material{material}
	b0 := globalPtr.get(unsafe.Pointer(body0)).(*Body)
	b1 := globalPtr.get(unsafe.Pointer(body1)).(*Body)

	return C.int(b0.world.onAABBOverlap(gMaterial, b0, b1, int(threadIndex)))
}

type ContactsProcessHandler func(contact *Joint, timestep float32, threadIndex int)

//export goContactsProcessCB
func goContactsProcessCB(contact *C.NewtonJoint, timestep C.dFloat, threadIndex C.int) {
	j := globalPtr.get(unsafe.Pointer(contact)).(*Joint)
	j.body0.world.contactsProcess(j, float32(timestep), int(threadIndex))
}

func (w *World) SetMaterialCollisionCallback(matid0, matid1 int, userData interface{},
	onAABBOverlap OnAABBOverlapHandler, contactsProcess ContactsProcessHandler) {
	w.onAABBOverlap = onAABBOverlap
	w.contactsProcess = contactsProcess
	C.SetCollisionCB(w.handle, C.int(matid0), C.int(matid1), unsafe.Pointer(&userData))
}

type MeshCollisionCollideDesc struct {
	m_boxP0               []float32
	m_boxP1               []float32
	m_boxDistanceTravel   []float32
	m_threadNumber        int
	m_faceCount           int
	m_vertexStrideInBytes int
	m_skinThickness       float32
	m_userData            interface{}
	m_objBody             *Body
	m_polySoupBody        *Body
	m_objCollision        *Collision
	m_polySoupCollision   *Collision
	m_vertex              []float32
	m_faceIndexCount      *int
	m_faceVertexIndex     *int
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

//export goCollisionTreeRayCastCallback
func goCollisionTreeRayCastCallback(body *C.NewtonBody, treeCollision *C.NewtonCollision, interception C.dFloat,
	normal *C.dFloat, faceId C.int, userData unsafe.Pointer) C.dFloat {
	b := globalPtr.get(unsafe.Pointer(body)).(*Body)
	col := &Collision{handle: treeCollision}
	col = globalPtr.get(col.ptr()).(*Collision)

	return C.dFloat(col.treeRaycastCallback(b, col, float32(interception), goFloats(normal, 3),
		int(faceId), (interface{})(userData)))
}

func (c *Collision) SetTreeRayCastCallback(callback CollisionTreeRayCastCallback) {
	c.treeRaycastCallback = callback

	C.SetUserRayCastCallback(c.handle)
}

type TreeCollisionCallback func(bodyWithTreeCollision, body *Body, faceID, vertexCount int,
	vertex []float32, vertexStrideInBytes int)

var treeCollisionCallback TreeCollisionCallback

//export goTreeCollisionCallback
func goTreeCollisionCallback(bodyWithTreeCollision, body *C.NewtonBody, faceID, vertextCount C.int,
	vertex *C.dFloat, vertexStrideInBytes C.int) {
	bWithTreeCollision := globalPtr.get(unsafe.Pointer(bodyWithTreeCollision)).(*Body)
	b := globalPtr.get(unsafe.Pointer(body)).(*Body)
	treeCollisionCallback(bWithTreeCollision, b, int(faceID), int(vertextCount),
		goFloats(vertex, int(vertextCount)), int(vertexStrideInBytes))
}

func StaticCollisionSetDebugCallback(staticCollision *Collision, userCallback TreeCollisionCallback) {
	treeCollisionCallback = userCallback
	C.SetStaticCollisionDebugCallback(staticCollision.handle)
}
