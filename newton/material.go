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

func (w *World) MaterialUserData(matid0, matid1 int) interface{} {
	return (interface{})(C.NewtonMaterialGetUserData(w.handle, C.int(matid0), C.int(matid1)))
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

func (m *Material) ContactFaceAttribute() uint {
	return uint(C.NewtonMaterialGetContactFaceAttribute(m.handle))
}

func (m *Material) BodyCollidingShape(body *Body) *Collision {
	collision := &Collision{handle: C.NewtonMaterialGetBodyCollidingShape(m.handle, body.handle)}
	return collision
}

func (m *Material) ContactNormalSpeed() float32 {
	return float32(C.NewtonMaterialGetContactNormalSpeed(m.handle))
}

func (m *Material) ContactForce(body *Body, force []float32) {
	C.NewtonMaterialGetContactForce(m.handle, body.handle, (*C.dFloat)(&force[0]))
}

func (m *Material) ContactPositionAndNormal(body *Body, position, normal []float32) {
	C.NewtonMaterialGetContactPositionAndNormal(m.handle, body.handle, (*C.dFloat)(&position[0]),
		(*C.dFloat)(&normal[0]))
}

func (m *Material) ContactTangentDirections(body *Body, dir0, dir1 []float32) {
	C.NewtonMaterialGetContactTangentDirections(m.handle, body.handle, (*C.dFloat)(&dir0[0]),
		(*C.dFloat)(&dir1[0]))
}

func (m *Material) ContactTangentSpeed(index int) float32 {
	return float32(C.NewtonMaterialGetContactTangentSpeed(m.handle, C.int(index)))
}

func (m *Material) SetContactSoftness(softness float32) {
	C.NewtonMaterialSetContactSoftness(m.handle, C.dFloat(softness))
}

func (m *Material) SetContactElasticity(restitution float32) {
	C.NewtonMaterialSetContactElasticity(m.handle, C.dFloat(restitution))
}

func (m *Material) SetContactFrictionState(state, index int) {
	C.NewtonMaterialSetContactFrictionState(m.handle, C.int(state), C.int(index))
}

func (m *Material) SetContactFrictionCoef(staticCoef, kineticCoef float32, index int) {
	C.NewtonMaterialSetContactFrictionCoef(m.handle, C.dFloat(staticCoef), C.dFloat(kineticCoef),
		C.int(index))
}

func (m *Material) SetContactNormalAcceleration(accel float32) {
	C.NewtonMaterialSetContactNormalAcceleration(m.handle, C.dFloat(accel))
}

func (m *Material) SetContactNormalDirection(directionVector []float32) {
	C.NewtonMaterialSetContactNormalDirection(m.handle, (*C.dFloat)(&directionVector[0]))
}

func (m *Material) SetContactTangentAcceleration(accel float32, index int) {
	C.NewtonMaterialSetContactTangentAcceleration(m.handle, C.dFloat(accel), C.int(index))
}

func (m *Material) ContactRotateTangentDirections(directionVector []float32) {
	C.NewtonMaterialContactRotateTangentDirections(m.handle, (*C.dFloat)(&directionVector[0]))
}
