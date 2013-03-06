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

//used for bools from c interfaces
var gbool = map[int]bool{0: false, 1: true}
var cint = map[bool]C.int{false: C.int(0), true: C.int(1)}

//Holds user data in go code, so it doesn't get collected
var ownerData = make(map[owner]interface{})

type World struct {
	handle *C.NewtonWorld
}

func Version() int    { return int(C.NewtonWorldGetVersion()) }
func MemoryUsed() int { return int(C.NewtonGetMemoryUsed()) }

func CreateWorld() *World {
	return &World{C.NewtonCreate()}
}

func (w *World) Destroy() {
	C.NewtonDestroy(w.handle)
}

func (w *World) DestroyAllBodies() {
	C.NewtonDestroyAllBodies(w.handle)
}

func (w *World) InvalidateCache() { C.NewtonInvalidateCache(w.handle) }

//SetSolverModel sets the solver model for the world
// 0 is default with perfect accuracy
// 1 and greater increases iterations. Greater number less peformant but more accurate
func (w *World) SetSolverModel(model int) { C.NewtonSetSolverModel(w.handle, C.int(model)) }

//TODO: multithreading

func (w *World) ReadPeformanceTicks(performanceEntry uint) uint {
	return uint(C.NewtonReadPerformanceTicks(w.handle, C.unsigned(performanceEntry)))
}

func (w *World) BroadphaseAlgorithm() int {
	return int(C.NewtonGetBroadphaseAlgorithm(w.handle))
}

func (w *World) SetBroadphaseAlgorithm(algorithmType int) {
	C.NewtonSelectBroadphaseAlgorithm(w.handle, C.int(algorithmType))
}

func (w *World) Update(timestep float32) {
	C.NewtonUpdate(w.handle, C.dFloat(timestep))
}

func (w *World) UpdateAsync(timestep float32) {
	C.NewtonUpdateAsync(w.handle, C.dFloat(timestep))
}
func (w *World) WaitForUpdateToFinish() {
	C.NewtonWaitForUpdateToFinish(w.handle)
}

func (w *World) SetFrictionModel(model int) {
	C.NewtonSetFrictionModel(w.handle, C.int(model))
}

func (w *World) SetMinimumFrameRate(frameRate float32) {
	C.NewtonSetMinimumFrameRate(w.handle, C.dFloat(frameRate))
}

func (w *World) BodyCount() int {
	return int(C.NewtonWorldGetBodyCount(w.handle))
}

func (w *World) ConstraintCount() int {
	return int(C.NewtonWorldGetConstraintCount(w.handle))
}

func (w *World) FirstBody() *Body {
	return &Body{C.NewtonWorldGetFirstBody(w.handle)}
}

func (w *World) NextBody(curBody *Body) *Body {
	return &Body{C.NewtonWorldGetNextBody(w.handle, curBody.handle)}
}

//Low level standalone collision todo later
//func (w *World) CollisionPointDistance(point []float32, collision *Collision,

//Not implementing transforms utilty functions

func CalculateSpringDamperAcceleration(dt, ks, x, kd, s float32) float32 {
	return float32(C.NewtonCalculateSpringDamperAcceleration(C.dFloat(dt), C.dFloat(ks), C.dFloat(x),
		C.dFloat(kd), C.dFloat(s)))
}

//Bodies
type Body struct {
	handle *C.NewtonBody
}

const (
	BodyDynamic = iota
	BodyKinematic
	BodyDeformable
)

func (w *World) CreateDynamicBody(collision *Collision, matrix *[16]float32) *Body {
	return &Body{C.NewtonCreateDynamicBody(w.handle, collision.handle, (*C.dFloat)(&matrix[0]))}
}

func (w *World) CreateKinematicBody(collision *Collision, matrix *[16]float32) *Body {
	return &Body{C.NewtonCreateKinematicBody(w.handle, collision.handle, (*C.dFloat)(&matrix[0]))}
}

func (w *World) DestroyBody(body *Body) {
	C.NewtonDestroyBody(w.handle, body.handle)
}

func (w *World) UserData() interface{} {
	//return (interface{})(C.NewtonWorldGetUserData(w.handle))
	return ownerData[owner(w.handle)]
}

func (w *World) SetUserData(userData interface{}) {
	//C.NewtonWorldSetUserData(w.handle, unsafe.Pointer(&userData))
	ownerData[owner(w.handle)] = userData
}

func (b *Body) Type() int {
	return int(C.NewtonBodyGetType(b.handle))
}

func (b *Body) AddForce(force *[3]float32) {
	C.NewtonBodyAddForce(b.handle, (*C.dFloat)(&force[0]))
}

func (b *Body) AddTorque(torque *[3]float32) {
	C.NewtonBodyAddTorque(b.handle, (*C.dFloat)(&torque[0]))
}

func (b *Body) CalculateInverseDynamicsForce(timestep float32, desiredVeloc, forceOut *[3]float32) {
	C.NewtonBodyCalculateInverseDynamicsForce(b.handle, C.dFloat(timestep), (*C.dFloat)(&desiredVeloc[0]),
		(*C.dFloat)(&forceOut[0]))
}

func (b *Body) SetCentreOfMass(relativeOffset *[3]float32) {
	C.NewtonBodySetCentreOfMass(b.handle, (*C.dFloat)(&relativeOffset[0]))
}

func (b *Body) SetMassMatrix(mass, Ixx, Iyy, Izz float32) {
	C.NewtonBodySetMassMatrix___(b.handle, C.dFloat(mass), C.dFloat(Ixx), C.dFloat(Iyy), C.dFloat(Izz))
}

func (b *Body) SetMassProperties(mass float32, collision *Collision) {
	C.NewtonBodySetMassProperties(b.handle, C.dFloat(mass), collision.handle)
}

func (b *Body) SetMatrix(matrix *[16]float32) {
	C.NewtonBodySetMatrix(b.handle, (*C.dFloat)(&matrix[0]))
}

func (b *Body) SetMatrixRecursive(matrix *[16]float32) {
	C.NewtonBodySetMatrixRecursive(b.handle, (*C.dFloat)(&matrix[0]))
}

func (b *Body) SetMaterialGroupID(id int) {
	C.NewtonBodySetMaterialGroupID(b.handle, C.int(id))
}

func (b *Body) SetContinuousCollisionMode(state uint) {
	C.NewtonBodySetContinuousCollisionMode(b.handle, C.uint(state))
}

func (b *Body) SetJointRecursiveCollision(state uint) {
	C.NewtonBodySetJointRecursiveCollision(b.handle, C.uint(state))
}

func (b *Body) SetOmega(omega *[3]float32) {
	C.NewtonBodySetOmega(b.handle, (*C.dFloat)(&omega[0]))
}

func (b *Body) SetVelocity(velocity *[3]float32) {
	C.NewtonBodySetVelocity(b.handle, (*C.dFloat)(&velocity[0]))
}

func (b *Body) SetForce(force *[3]float32) {
	C.NewtonBodySetForce(b.handle, (*C.dFloat)(&force[0]))
}

func (b *Body) SetTorque(torque *[3]float32) {
	C.NewtonBodySetTorque(b.handle, (*C.dFloat)(&torque[0]))
}

func (b *Body) SetLinearDamping(linearDamp float32) {
	C.NewtonBodySetLinearDamping(b.handle, C.dFloat(linearDamp))
}

func (b *Body) SetAngularDamping(angularDamp *[3]float32) {
	C.NewtonBodySetAngularDamping(b.handle, (*C.dFloat)(&angularDamp[0]))
}

func (b *Body) SetCollision(collision *Collision) {
	C.NewtonBodySetCollision(b.handle, collision.handle)
}

func (b *Body) SetCollisionScale(x, y, z float32) {
	C.NewtonBodySetCollisionScale(b.handle, C.dFloat(x), C.dFloat(y), C.dFloat(z))
}

func (b *Body) SleepState() int {
	return int(C.NewtonBodyGetSleepState(b.handle))
}

func (b *Body) SetSleepState(state int) {
	C.NewtonBodySetSleepState(b.handle, C.int(state))
}

func (b *Body) AutoSleep() int {
	return int(C.NewtonBodyGetAutoSleep(b.handle))
}

func (b *Body) SetAutoSleep(state int) {
	C.NewtonBodySetAutoSleep(b.handle, C.int(state))
}

func (b *Body) FreezeState() int {
	return int(C.NewtonBodyGetFreezeState(b.handle))
}

func (b *Body) SetFreezeState(state int) {
	C.NewtonBodySetFreezeState(b.handle, C.int(state))
}

func (b *Body) BodyID() int { return int(C.NewtonBodyGetID(b.handle)) }

func (b *Body) World() *World {
	return &World{C.NewtonBodyGetWorld(b.handle)}
}

func (b *Body) Collision() *Collision {
	return &Collision{handle: C.NewtonBodyGetCollision(b.handle)}
}

func (b *Body) MaterialGroupID() int {
	return int(C.NewtonBodyGetMaterialGroupID(b.handle))
}

func (b *Body) ContinuousCollisionMode() int {
	return int(C.NewtonBodyGetContinuousCollisionMode(b.handle))
}

func (b *Body) JointRecursiveCollision() int {
	return int(C.NewtonBodyGetJointRecursiveCollision(b.handle))
}

func (b *Body) Matrix(matrix *[16]float32) {
	C.NewtonBodyGetMatrix(b.handle, (*C.dFloat)(&matrix[0]))
}

//Rotation gets the Quaternion (4) floats
func (b *Body) Rotation(rotation *[16]float32) {
	C.NewtonBodyGetRotation(b.handle, (*C.dFloat)(&rotation[0]))
}

func (b *Body) MassMatrix(mass, Ixx, Iyy, Izz *float32) {
	C.NewtonBodyGetMassMatrix(b.handle, (*C.dFloat)(mass), (*C.dFloat)(Ixx),
		(*C.dFloat)(Iyy), (*C.dFloat)(Izz))
}

func (b *Body) InvMass(invMass, invIxx, invIyy, invIzz *float32) {
	C.NewtonBodyGetInvMass(b.handle, (*C.dFloat)(invMass), (*C.dFloat)(invIxx),
		(*C.dFloat)(invIyy), (*C.dFloat)(invIzz))
}

func (b *Body) InertiaMatrix(inertiaMatrix *[16]float32) {
	C.NewtonBodyGetInertiaMatrix(b.handle, (*C.dFloat)(&inertiaMatrix[0]))
}

func (b *Body) InvInertiaMatrix(invInertiaMatrix *[16]float32) {
	C.NewtonBodyGetInvInertiaMatrix(b.handle, (*C.dFloat)(&invInertiaMatrix[0]))
}

func (b *Body) Omega(vector *[3]float32) {
	C.NewtonBodyGetOmega(b.handle, (*C.dFloat)(&vector[0]))
}

func (b *Body) Velocity(vector *[3]float32) {
	C.NewtonBodyGetVelocity(b.handle, (*C.dFloat)(&vector[0]))
}

func (b *Body) Force(vector *[3]float32) {
	C.NewtonBodyGetForce(b.handle, (*C.dFloat)(&vector[0]))
}

func (b *Body) Torque(vector *[3]float32) {
	C.NewtonBodyGetTorque(b.handle, (*C.dFloat)(&vector[0]))
}

func (b *Body) ForceAcc(vector *[3]float32) {
	C.NewtonBodyGetForceAcc(b.handle, (*C.dFloat)(&vector[0]))
}

func (b *Body) TorqueAcc(vector *[3]float32) {
	C.NewtonBodyGetTorqueAcc(b.handle, (*C.dFloat)(&vector[0]))
}

func (b *Body) CentreOfMass(com *[3]float32) {
	C.NewtonBodyGetCentreOfMass(b.handle, (*C.dFloat)(&com[0]))
}

func (b *Body) AddImpulse(pointDeltaVeloc, pointPosit *[3]float32) {
	C.NewtonBodyAddImpulse(b.handle, (*C.dFloat)(&pointDeltaVeloc[0]), (*C.dFloat)(&pointPosit[0]))
}

func (b *Body) ApplyImpulsePair(linearImpulse, angularImpulse *[3]float32) {
	C.NewtonBodyApplyImpulsePair(b.handle, (*C.dFloat)(&linearImpulse[0]), (*C.dFloat)(&angularImpulse[0]))
}

func (b *Body) IntegrateVelocity(timestep float32) {
	C.NewtonBodyIntegrateVelocity(b.handle, C.dFloat(timestep))
}

func (b *Body) LinearDamping() float32 {
	return float32(C.NewtonBodyGetLinearDamping(b.handle))
}

func (b *Body) AngularDamping(result *[3]float32) {
	C.NewtonBodyGetAngularDamping(b.handle, (*C.dFloat)(&result[0]))
}

func (b *Body) AABB(p0, p1 *[3]float32) {
	C.NewtonBodyGetAABB(b.handle, (*C.dFloat)(&p0[0]), (*C.dFloat)(&p1[0]))
}

func (b *Body) FirstJoint() *Joint {
	return &Joint{C.NewtonBodyGetFirstJoint(b.handle)}
}

func (b *Body) NextJoint(curJoint *Joint) *Joint {
	return &Joint{C.NewtonBodyGetNextJoint(b.handle, curJoint.handle)}
}

func (b *Body) FirstContactJoint() *Joint {
	return &Joint{C.NewtonBodyGetFirstContactJoint(b.handle)}
}

func (b *Body) NextContactJoint(curJoint *Joint) *Joint {
	return &Joint{C.NewtonBodyGetNextContactJoint(b.handle, curJoint.handle)}
}

func (b *Body) UserData() interface{} {
	return ownerData[owner(b.handle)]
	//return (interface{})(C.NewtonBodyGetUserData(b.handle))
}

func (b *Body) SetUserData(userData interface{}) {
	//C.NewtonBodySetUserData(b.handle, unsafe.Pointer(&userData))
	ownerData[owner(b.handle)] = userData
}
