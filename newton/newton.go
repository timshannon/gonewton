// Copyright 2012 Tim Shannon. All rights reserved.  
// Use of this source code is governed by the MIT license 
// that can be found in the LICENSE file.  
package newton

/*
#cgo   linux LDFLAGS: -L/usr/local/lib -lNewton -lstdc++
#include "Newton.h"
#include <stdlib.h>
*/
import "C"
import "unsafe"

const (
	DynamicBody = iota
	KinematicBody
	DeformableBody
)

//global objects that need to hang around so the callback pointers
// don't get cleaned up by the GC.
// This interface and type assist in managing the translation from
// a C object to the stored Go object
type cObject interface {
	ptr() unsafe.Pointer
}
type ptrManager map[uintptr]cObject

func (p ptrManager) add(object cObject) {
	p[uintptr(object.ptr())] = object
}

func (p ptrManager) remove(object cObject) {
	delete(p, uintptr(object.ptr()))
}

func (p ptrManager) get(ptr unsafe.Pointer) cObject {
	return p[uintptr(ptr)]
}

var globalPtr ptrManager

//used for bools from c interfaces 
// I'm sure there's a better way to do this, but this works for now
var gbool = map[int]bool{0: false, 1: true}
var cint = map[bool]C.int{false: C.int(0), true: C.int(1)}

type World struct {
	handle   *C.NewtonWorld
	bodies   []*Body
	UserData interface{}

	//world unique callbacks
	bodyLeaveWorld  BodyLeaveWorldHandler
	jointIterator   JointIteratorHandler
	bodyIterator    BodyIteratorHandler
	rayFilter       RayFilterHandler
	rayPrefilter    RayPrefilterHandler
	onAABBOverlap   OnAABBOverlapHandler
	contactsProcess ContactsProcessHandler
	//meshCollisionCollide MeshCollisionCollideHandler
}

func (w *World) ptr() unsafe.Pointer {
	return unsafe.Pointer(w.handle)
}

type Body struct {
	handle   *C.NewtonBody
	world    *World
	joints   []*Joint
	UserData interface{}
}

func (b *Body) ptr() unsafe.Pointer { return unsafe.Pointer(b.handle) }

type Joint struct {
	handle   *C.NewtonJoint
	body0    *Body
	body1    *Body
	UserData interface{}
}

func (j *Joint) ptr() unsafe.Pointer { return unsafe.Pointer(j.handle) }

func init() {
	globalPtr = make(ptrManager)
}
func Version() int    { return int(C.NewtonWorldGetVersion()) }
func MemoryUsed() int { return int(C.NewtonGetMemoryUsed()) }

func CreateWorld() *World {
	world := new(World)
	world.handle = C.NewtonCreate()

	globalPtr.add(world)

	return world
}

func (w *World) deleteBodyPointers() {
	for i := range w.bodies {
		w.bodies[i].deleteJointPointers()
		globalPtr.remove(w.bodies[i])

	}
}

func (b *Body) deleteJointPointers() {
	for i := range b.joints {
		globalPtr.remove(b.joints[i])
	}
}

func (w *World) Destroy() {
	C.NewtonDestroy(w.handle)
	w.deleteBodyPointers()
	globalPtr.remove(w)
	w.handle = nil
}

func (w *World) DestroyAllBodies() {
	C.NewtonDestroyAllBodies(w.handle)
	w.deleteBodyPointers()
	w.bodies = []*Body{}
}

func (w *World) CreateDynamicBody(collision *Collision, matrix []float32) *Body {
	body := new(Body)
	body.world = w

	body.handle = C.NewtonCreateDynamicBody(w.handle, collision.handle, (*C.dFloat)(&matrix[0]))

	globalPtr.add(body)
	return body
}

func (w *World) createGoJoint(cObject *C.NewtonJoint, body0, body1 *Body) *Joint {
	joint := new(Joint)
	joint.handle = cObject
	joint.body0 = body0
	joint.body1 = body1

	globalPtr.add(joint)

	return joint
}

func (w *World) InvalidateCache() { C.NewtonInvalidateCache(w.handle) }

//SetSolverModel sets the solver model for the world
// 0 is default with perfect accuracy
// 1 and greater increases iterations. Greater number less peformant but more accurate
func (w *World) SetSolverModel(model int) { C.NewtonSetSolverModel(w.handle, C.int(model)) }

//Skip multithredding for now

func (w *World) ReadPeformanceTicks(performanceEntry uint) uint {
	return uint(C.NewtonReadPerformanceTicks(w.handle, C.unsigned(performanceEntry)))
}

func (w *World) BroadphaseAlgorithm() int {
	return int(C.NewtonGetBroadphaseAlgorithm(w.handle))
}

func (w *World) SetBroadphaseAlgorithm(algorithmType int) {
	C.NewtonSelectBroadphaseAlgorithm(w.handle, C.int(algorithmType))
}

func (w *World) Update(timestep float32) {
	C.NewtonUpdate(w.handle, C.dFloat(timestep))
}

func (w *World) UpdateAsync(timestep float32) {
	C.NewtonUpdateAsync(w.handle, C.dFloat(timestep))
}

func (w *World) WaitForUpdateToFinish() {
	C.NewtonWaitForUpdateToFinish(w.handle)
}

func (w *World) SetFrictionModel(model int) {
	C.NewtonSetFrictionModel(w.handle, C.int(model))
}

func (w *World) SetMinimumFrameRate(frameRate float32) {
	C.NewtonSetMinimumFrameRate(w.handle, C.dFloat(frameRate))
}

func (w *World) BodyCount() int {
	return int(C.NewtonWorldGetBodyCount(w.handle))
}

func (w *World) ConstraintCount() int {
	return int(C.NewtonWorldGetConstraintCount(w.handle))
}

func (w *World) FirstBody() *Body {
	return globalPtr.get(unsafe.Pointer(C.NewtonWorldGetFirstBody(w.handle))).(*Body)
}

func (w *World) NextBody(curBody *Body) *Body {
	return globalPtr.get(unsafe.Pointer(C.NewtonWorldGetNextBody(w.handle, curBody.handle))).(*Body)
}

//Low level standalone collision todo later
//func (w *World) CollisionPointDistance(point []float32, collision *Collision, 

//Not implementing transforms utilty functions, most people use use a go math lib

func CalculateSpringDamperAcceleration(dt, ks, x, kd, s float32) float32 {
	return float32(C.NewtonCalculateSpringDamperAcceleration(C.dFloat(dt), C.dFloat(ks), C.dFloat(x),
		C.dFloat(kd), C.dFloat(s)))
}
