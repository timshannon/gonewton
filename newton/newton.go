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

type World struct {
	handle *C.NewtonWorld
}

type Body struct {
	handle *C.NewtonBody
}

type Joint struct {
	handle *C.NewtonJoint
}

type Material struct {
	handle *C.NewtonMaterial
}

type Collision struct {
	handle *C.NewtonCollision
}

func Version() int    { return int(C.NewtonWorldGetVersion()) }
func MemoryUsed() int { return int(C.NewtonGetMemoryUsed()) }

func CreateWorld() *World {
	world := new(World)
	world.handle = C.NewtonCreate()

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

func (w *World) SetUserData(userData *interface{}) {
	C.NewtonWorldSetUserData(w.handle, unsafe.Pointer(userData))
}

func (w *World) UserData() *interface{} {
	return (*interface{})(C.NewtonWorldGetUserData(w.handle))
}

func (w *World) BodyCount() int {
	return int(C.NewtonWorldGetBodyCount(w.handle))
}

func (w *World) ConstraintCount() int {
	return int(C.NewtonWorldGetConstraintCount(w.handle))
}

func (w *World) CreateMaterialGroupID() int {
	return int(C.NewtonMaterialCreateGroupID(w.handle))
}

func (w *World) DefaultMaterialGroupID() int {
	return int(C.NewtonMaterialGetDefaultGroupID(w.handle))
}

func (w *World) DestroyAllMaterialGroupID() {
	C.NewtonMaterialDestroyAllGroupID(w.handle)
}

func (w *World) MaterialUserData(matid0, matid1 int) *interface{} {
	return (*interface{})(C.NewtonMaterialGetUserData(w.handle, C.int(matid0), C.int(matid1)))
}

func (w *World) SetMaterialSurfaceThickness(matid0, matid1 int, thickness float32) {
	C.NewtonMaterialSetSurfaceThickness(w.handle, C.int(matid0), C.int(matid1), C.dFloat(thickness))
}
