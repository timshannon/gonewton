package newton

/*
#cgo   linux  CFLAGS: -I/usr/local/include -pthread 
#cgo   linux LDFLAGS: -L/usr/local/lib -pthread -lNewton -lstdc++
#include "Newton.h"
#include <stdlib.h>
*/
import "C"

func GetVersion() int {
	return int(C.NewtonWorldGetVersion())
}
