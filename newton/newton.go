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

//import "unsafe"

const (
	DynamicBody = iota
	KinematicBody
	DeformableBody
)

type World struct{ handle *C.NewtonWorld }

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
type GetTicksCountHandler func() int

//export goGetTicksCountCB
func goGetTicksCountCB() int {
	return getTicksCount()
}

func (w *World) SetPerformanceClock(f GetTicksCountHandler) {
	getTicksCount = f
	//C.NewtonSetPerformanceClock(w.handle, setGetTicksCountCB())
}
