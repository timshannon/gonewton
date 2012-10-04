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

type Material struct {
	handle *C.NewtonMaterial
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

func (w *World) SetMaterialDefaultSoftness(matid0, matid1 int, value float32) {
	C.NewtonMaterialSetDefaultSoftness(w.handle, C.int(matid0), C.int(matid1), C.dFloat(value))
}

func (w *World) SetMaterialDefaultElasticity(matid0, matid1 int, elasticCoef float32) {
	C.NewtonMaterialSetDefaultElasticity(w.handle, C.int(matid0), C.int(matid1), C.dFloat(elasticCoef))
}

func (w *World) SetMaterialDefaultCollidable(matid0, matid1, state int) {
	C.NewtonMaterialSetDefaultCollidable(w.handle, C.int(matid0), C.int(matid1), C.int(state))
}

func (w *World) SetMaterialDefaultFriction(matid0, matid1 int, static, kinetic float32) {
	C.NewtonMaterialSetDefaultFriction(w.handle, C.int(matid0), C.int(matid1), C.dFloat(static),
		C.dFloat(kinetic))
}

func (w *World) FirstMaterial() *Material {
	return &Material{C.NewtonWorldGetFirstMaterial(w.handle)}
}

func (w *World) NextMaterial(material *Material) *Material {
	return &Material{C.NewtonWorldGetNextMaterial(w.handle, material.handle)}
}

func (m *Material) UserData() *interface{} {
	return (*interface{})(C.NewtonMaterialGetMaterialPairUserData(m.handle))
}

func (m *Material) ContactFaceAttribute() uint {
	return uint(C.NewtonMaterialGetContactFaceAttribute(m.handle))
}
