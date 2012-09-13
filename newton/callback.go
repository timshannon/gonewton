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
