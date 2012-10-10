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

//A note on object user data: Newton's internal user data pointer
// will solely be used for keeping track of the go object pointer
// A Go only empty interface will be used to hold the user's go data
// To an end user of this library, it shouldn't matter and be seemless
// The golang objects need to be kept in a global map so that the GC
// doesn't relaim them.

var nObjects map[uintptr]interface{}

//used for bools from c interfaces 
// I'm sure there's a better way to do this, but this works for now
var Bool = map[int]bool{0: false, 1: true}
var Int = map[bool]C.int{false: C.int(0), true: C.int(1)}

type World struct {
	handle *C.NewtonWorld
	//UserData interface{}
}

type Body struct {
	handle *C.NewtonBody
	//UserData interface{}
}

type Joint struct {
	handle *C.NewtonJoint
	//UserData interface{}
}

func Version() int    { return int(C.NewtonWorldGetVersion()) }
func MemoryUsed() int { return int(C.NewtonGetMemoryUsed()) }

func CreateWorld() *World {
	world := new(World)
	world.handle = C.NewtonCreate()

	nObjects[uintptr(unsafe.Pointer(world))] = world

	return world
}

func (w *World) Destroy()          { C.NewtonDestroy(w.handle) }
func (w *World) DestroyAllBodies() { C.NewtonDestroyAllBodies(w.handle) }
func (w *World) InvalidateCache()  { C.NewtonInvalidateCache(w.handle) }

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
	return &Body{C.NewtonWorldGetFirstBody(w.handle)}
}

func (w *World) NextBody(curBody *Body) *Body {
	return &Body{C.NewtonWorldGetNextBody(w.handle, curBody.handle)}
}
