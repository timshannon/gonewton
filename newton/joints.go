package newton

/*
#cgo   linux LDFLAGS: -L/usr/local/lib -lNewton -lstdc++
#include "Newton.h"
#include <stdlib.h>
*/
import "C"
import "unsafe"

type Joint struct {
	handle   *C.NewtonJoint
	body0    *Body
	body1    *Body
	UserData interface{}
}

func (j *Joint) ptr() unsafe.Pointer { return unsafe.Pointer(j.handle) }

func (w *World) createGoJoint(cObject *C.NewtonJoint, body0, body1 *Body) *Joint {
	joint := new(Joint)
	joint.handle = cObject
	joint.body0 = body0
	joint.body1 = body1

	globalPtr.add(joint)

	return joint
}
