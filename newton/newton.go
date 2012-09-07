package newton

/*
#cgo   linux LDFLAGS: -L/usr/local/lib -pthread -lNewton -lstdc++
#include "Newton.h"
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
