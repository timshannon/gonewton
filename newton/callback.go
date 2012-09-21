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
	"reflect"
	"unsafe"
)

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

//TODO: Test 
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
	//Test 
	var gHitNormal []float32
	sHead := (*reflect.SliceHeader)((unsafe.Pointer(&gHitNormal)))
	sHead.Cap = 3
	sHead.Len = 3
	sHead.Data = uintptr(unsafe.Pointer(hitNormal))

	return C.dFloat(rayFilter(gBody, gHitNormal, int(collisionID),
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

func (w *World) RayCast(p0 []float32, p1 []float32, filter RayFilterHandler,
	userData *interface{}, prefilter RayPrefilterHandler) {

}
