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

//To simplify the interface and to work around some limitations with cgo based
// callbacks, there will only be one global instance of each callback type
// even though the library could support more.

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

var bodyLeaveWorld BodyLeaveWorldHandler

//export goBodyLeaveWorldCB
func goBodyLeaveWorldCB(body *C.NewtonBody, threadIndex C.int) {
	gBody := &Body{body}
	bodyLeaveWorld(gBody, int(threadIndex))
}

func (w *World) SetBodyLeaveWorldEvent(f BodyLeaveWorldHandler) {
	bodyLeaveWorld = f
	C.setBodyLeaveWorldCB(w.handle)
}

type JointIteratorHandler func(joint *Joint, userData *interface{})

var jointIterator JointIteratorHandler

//export goJointIteratorCB
func goJointIteratorCB(joint *C.NewtonJoint, userData unsafe.Pointer) {
	gJoint := &Joint{joint}
	jointIterator(gJoint, (*interface{})(userData))
}

func (w *World) ForEachJointDo(f JointIteratorHandler, userData *interface{}) {
	jointIterator = f
	C.setJointIteratorCB(w.handle, unsafe.Pointer(userData))
}

type BodyIteratorHandler func(body *Body, userData *interface{})

var bodyIterator BodyIteratorHandler

//export goBodyIteratorCB
func goBodyIteratorCB(body *C.NewtonBody, userData unsafe.Pointer) {
	gBody := &Body{body}
	bodyIterator(gBody, (*interface{})(userData))
}

func (w *World) ForEachBodyInAABBDo(p0, p1 []float32, f BodyIteratorHandler,
	userData *interface{}) {
	bodyIterator = f
	C.setBodyIteratorCB(w.handle, (*C.dFloat)(&p0[0]), (*C.dFloat)(&p1[0]), unsafe.Pointer(userData))
}

type RayFilterHandler func(body *Body, hitNormal []float32, collisionID int,
	userData *interface{}, intersectParam float32) float32

var rayFilter RayFilterHandler

//export goRayFilterCB
func goRayFilterCB(body *C.NewtonBody, hitNormal *C.dFloat, collisionID C.int,
	userData unsafe.Pointer, intersectParam C.dFloat) C.dFloat {
	gBody := &Body{body}

	return C.dFloat(rayFilter(gBody, goFloats(hitNormal, 3), int(collisionID),
		(*interface{})(userData), float32(intersectParam)))
}

type RayPrefilterHandler func(body *Body, collision *Collision, userData *interface{}) uint

var rayPrefilter RayPrefilterHandler

//export goRayPrefilterCB
func goRayPrefilterCB(body *C.NewtonBody, collision *C.NewtonCollision, userData unsafe.Pointer) C.unsigned {
	gBody := &Body{body}
	gCollision := &Collision{collision}
	return C.unsigned(rayPrefilter(gBody, gCollision, (*interface{})(userData)))
}

func SetRayFilterHandler(f RayFilterHandler) {
	rayFilter = f
}

func SetRayPrefilterHandler(f RayPrefilterHandler) {
	rayPrefilter = f
}

func (w *World) RayCast(p0 []float32, p1 []float32, userData *interface{}) {
	C.RayCast(w.handle, (*C.dFloat)(&p0[0]), (*C.dFloat)(&p1[0]), unsafe.Pointer(userData))
}

//goFloats creates a float32 slice of the given size of the passed in c array pointer
func goFloats(cArray *C.dFloat, size int) []float32 {
	//Copy for now.  Maybe we'll try with using the slice header to repoint the data
	// instead if preformance thrashes with this
	slice := make([]float32, size)
	C.CopyFloatArray(cArray, (*C.dFloat)(&slice[0]), C.int(size))
	return slice
}

type ConvexCastReturnInfo struct {
	Point            []float32
	Normal           []float32
	NormalOnHitPoint []float32
	Penetration      float32
	ContactID        int
	HitBody          *Body
}

func (w *World) ConvexCast(matrix []float32, target []float32, shape *Collision, hitParam *float32,
	userData *interface{}, maxContactsCount int, threadIndex int) []*ConvexCastReturnInfo {

	var size int
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
			HitBody:          &Body{cInfo[i].m_hitBody},
		}

	}

	return returnInfo
}

type OnAABBOverlapHandler func(material *Material, body0, body1 *Body, threadIndex int) int

var gOnAABBOverlap OnAABBOverlapHandler

//export goOnAABBOverlapCB
func goOnAABBOverlapCB(material *C.NewtonMaterial, body0, body1 *C.NewtonBody, threadIndex C.int) C.int {
	gMaterial := &Material{material}
	gBody0 := &Body{body0}
	gBody1 := &Body{body1}

	return C.int(gOnAABBOverlap(gMaterial, gBody0, gBody1, int(threadIndex)))
}

type ContactsProcessHandler func(contact *Joint, timestep float32, threadIndex int)

var gContactsProcess ContactsProcessHandler

//export goContactsProcessCB
func goContactsProcessCB(contact *C.NewtonJoint, timestep C.dFloat, threadIndex C.int) {
	gJoint := &Joint{contact}
	gContactsProcess(gJoint, float32(timestep), int(threadIndex))
}

func (w *World) SetMaterialCollisionCallback(matid0, matid1 int, userData *interface{},
	onAABBOverlap OnAABBOverlapHandler, contactsProcess ContactsProcessHandler) {
	gOnAABBOverlap = onAABBOverlap
	gContactsProcess = contactsProcess
	C.SetCollisionCB(w.handle, C.int(matid0), C.int(matid1), unsafe.Pointer(userData))
}
