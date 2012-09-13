// Copyright 2012 Tim Shannon. All rights reserved.  
// Use of this source code is governed by the MIT license 
// that can be found in the LICENSE file.  

#include "callback.h"


void setGetTicksCountCB(const NewtonWorld* const newtonWorld) {
	NewtonSetPerformanceClock(newtonWorld, goGetTicksCountCB);
}