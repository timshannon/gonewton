// Copyright 2012 Tim Shannon. All rights reserved.  
// Use of this source code is governed by the MIT license 
// that can be found in the LICENSE file.  

#include "callback.h"


void setGetTicksCountCB(NewtonWorld* newtonWorld) {
	NewtonSetPerformanceClock(newtonWorld, goGetTicksCountCB);
}

void setBodyLeaveWorldCB(NewtonWorld* newtonWorld) {
	NewtonSetBodyLeaveWorldEvent(newtonWorld, (NewtonBodyLeaveWorld)goBodyLeaveWorldCB);
}
