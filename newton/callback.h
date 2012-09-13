// Copyright 2012 Tim Shannon. All rights reserved.  
// Use of this source code is governed by the MIT license 
// that can be found in the LICENSE file.  

#ifndef _CALLBACK_H_
#define _CALLBACK_H_

#include <stdlib.h>
//dll for win32?
#include "Newton.h"

extern unsigned goGetTicksCountCB(void);
void goBodyLeaveWorldCB(NewtonBody* , int);

void setGetTicksCountCB(NewtonWorld*);
void setBodyLeaveWorldCB(NewtonWorld*);

#endif //_CALLBACK_H_
