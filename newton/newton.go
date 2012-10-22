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
import "unsafe"

//global objects that need to hang around so the callback pointers
// don't get cleaned up by the GC.
// This interface and type assist in managing the translation from
// a C object to the stored Go object
type cObject interface {
	ptr() unsafe.Pointer
}
type ptrManager map[uintptr]cObject

func (p ptrManager) add(object cObject) {
	p[uintptr(object.ptr())] = object
}

func (p ptrManager) remove(object cObject) {
	delete(p, uintptr(object.ptr()))
}

func (p ptrManager) get(ptr unsafe.Pointer) cObject {
	return p[uintptr(ptr)]
}

var globalPtr ptrManager

//used for bools from c interfaces 
// I'm sure there's a better way to do this, but this works for now
var gbool = map[int]bool{0: false, 1: true}
var cint = map[bool]C.int{false: C.int(0), true: C.int(1)}

type World struct {
	handle   *C.NewtonWorld
	bodies   []*Body
	UserData interface{}

	//world unique callbacks
	bodyLeaveWorld  BodyLeaveWorldHandler
	jointIterator   JointIteratorHandler
	bodyIterator    BodyIteratorHandler
	rayFilter       RayFilterHandler
	rayPrefilter    RayPrefilterHandler
	onAABBOverlap   OnAABBOverlapHandler
	contactsProcess ContactsProcessHandler
	//meshCollisionCollide MeshCollisionCollideHandler
}

func (w *World) ptr() unsafe.Pointer {
	return unsafe.Pointer(w.handle)
}

func init() {
	globalPtr = make(ptrManager)
}
func Version() int    { return int(C.NewtonWorldGetVersion()) }
func MemoryUsed() int { return int(C.NewtonGetMemoryUsed()) }

func CreateWorld() *World {
	world := new(World)
	world.handle = C.NewtonCreate()

	globalPtr.add(world)

	return world
}

func (w *World) deleteBodyPointers() {
	for i := range w.bodies {
		w.bodies[i].deleteJointPointers()
		globalPtr.remove(w.bodies[i])

	}
}

func (w *World) Destroy() {
	C.NewtonDestroy(w.handle)
	w.deleteBodyPointers()
	globalPtr.remove(w)
	w.handle = nil
}

func (w *World) DestroyAllBodies() {
	C.NewtonDestroyAllBodies(w.handle)
	w.deleteBodyPointers()
	w.bodies = []*Body{}
}

func (w *World) InvalidateCache() { C.NewtonInvalidateCache(w.handle) }

//SetSolverModel sets the solver model for the world
// 0 is default with perfect accuracy
// 1 and greater increases iterations. Greater number less peformant but more accurate
func (w *World) SetSolverModel(model int) { C.NewtonSetSolverModel(w.handle, C.int(model)) }

//Skip multithredding for now

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
	return globalPtr.get(unsafe.Pointer(C.NewtonWorldGetFirstBody(w.handle))).(*Body)
}

func (w *World) NextBody(curBody *Body) *Body {
	return globalPtr.get(unsafe.Pointer(C.NewtonWorldGetNextBody(w.handle, curBody.handle))).(*Body)
}

//Low level standalone collision todo later
//func (w *World) CollisionPointDistance(point []float32, collision *Collision, 

//Not implementing transforms utilty functions, most people use use a go math lib

func CalculateSpringDamperAcceleration(dt, ks, x, kd, s float32) float32 {
	return float32(C.NewtonCalculateSpringDamperAcceleration(C.dFloat(dt), C.dFloat(ks), C.dFloat(x),
		C.dFloat(kd), C.dFloat(s)))
}

//Bodies
type Body struct {
	handle   *C.NewtonBody
	world    *World
	joints   []*Joint
	UserData interface{}

	//callbacks
	destructorCallback  BodyDestructorCallback
	transformCallback   TransformCallback
	applyForceAndTorque ApplyForceAndTorque
	getBuoyancyPlane    BuoyancyPlaneHandler
}

const (
	BodyDynamic = iota
	BodyKinematic
	BodyDeformable
)

func (b *Body) ptr() unsafe.Pointer { return unsafe.Pointer(b.handle) }

func (b *Body) deleteJointPointers() {
	for i := range b.joints {
		globalPtr.remove(b.joints[i])
	}
}

func (w *World) CreateDynamicBody(collision *Collision, matrix []float32) *Body {
	body := new(Body)
	body.world = w

	body.handle = C.NewtonCreateDynamicBody(w.handle, collision.handle, (*C.dFloat)(&matrix[0]))

	globalPtr.add(body)
	return body
}

func (w *World) CreateKinematicBody(collision *Collision, matrix []float32) *Body {
	body := new(Body)
	body.world = w

	body.handle = C.NewtonCreateKinematicBody(w.handle, collision.handle, (*C.dFloat)(&matrix[0]))

	globalPtr.add(body)
	return body
}

func (b *Body) Destroy() {
	C.NewtonDestroyBody(b.world.handle, b.handle)
	b.deleteJointPointers()
	globalPtr.remove(b)
	b.handle = nil
}

func (b *Body) Type() int {
	return int(C.NewtonBodyGetType(b.handle))
}

func (b *Body) AddForce(force []float32) {
	C.NewtonBodyAddForce(b.handle, (*C.dFloat)(&force[0]))
}

func (b *Body) AddTorque(torque []float32) {
	C.NewtonBodyAddTorque(b.handle, (*C.dFloat)(&torque[0]))
}

func (b *Body) CalculateInverseDynamicsForce(timestep float32, desiredVeloc, forceOut []float32) {
	C.NewtonBodyCalculateInverseDynamicsForce(b.handle, C.dFloat(timestep), (*C.dFloat)(&desiredVeloc[0]),
		(*C.dFloat)(&forceOut[0]))
}

func (b *Body) SetCentreOfMass(relativeOffset []float32) {
	C.NewtonBodySetCentreOfMass___(b.handle, (*C.dFloat)(&relativeOffset[0]))
}

func (b *Body) SetMassMatrix(mass, Ixx, Iyy, Izz float32) {
	C.NewtonBodySetMassMatrix___(b.handle, C.dFloat(mass), C.dFloat(Ixx), C.dFloat(Iyy), C.dFloat(Izz))
}

func (b *Body) SetMassProperties(mass float32, collision *Collision) {
	C.NewtonBodySetMassProperties(b.handle, C.dFloat(mass), collision.handle)
}

func (b *Body) SetMatrix(matrix []float32) {
	C.NewtonBodySetMatrix(b.handle, (*C.dFloat)(&matrix[0]))
}

func (b *Body) SetMatrixRecursive(matrix []float32) {
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

func (b *Body) SetOmega(omega []float32) {
	C.NewtonBodySetOmega(b.handle, (*C.dFloat)(&omega[0]))
}

func (b *Body) SetVelocity(velocity []float32) {
	C.NewtonBodySetVelocity(b.handle, (*C.dFloat)(&velocity[0]))
}

func (b *Body) SetForce(force []float32) {
	C.NewtonBodySetForce(b.handle, (*C.dFloat)(&force[0]))
}

func (b *Body) SetTorque(torque []float32) {
	C.NewtonBodySetTorque(b.handle, (*C.dFloat)(&torque[0]))
}

func (b *Body) SetLinearDamping(linearDamp float32) {
	C.NewtonBodySetLinearDamping(b.handle, C.dFloat(linearDamp))
}

func (b *Body) SetAngularDamping(angularDamp []float32) {
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

func (b *Body) World() *World { return b.world }

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

func (b *Body) Matrix(matrix []float32) {
	C.NewtonBodyGetMatrix(b.handle, (*C.dFloat)(&matrix[0]))
}

//Rotation gets the Quaternion (4) floats
func (b *Body) Rotation(rotation []float32) {
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

func (b *Body) InertiaMatrix(inertiaMatrix []float32) {
	C.NewtonBodyGetInertiaMatrix(b.handle, (*C.dFloat)(&inertiaMatrix[0]))
}

func (b *Body) InvInertiaMatrix(invInertiaMatrix []float32) {
	C.NewtonBodyGetInvInertiaMatrix(b.handle, (*C.dFloat)(&invInertiaMatrix[0]))
}

func (b *Body) Omega(vector []float32) {
	C.NewtonBodyGetOmega(b.handle, (*C.dFloat)(&vector[0]))
}

func (b *Body) Velocity(vector []float32) {
	C.NewtonBodyGetVelocity(b.handle, (*C.dFloat)(&vector[0]))
}

func (b *Body) Force(vector []float32) {
	C.NewtonBodyGetForce(b.handle, (*C.dFloat)(&vector[0]))
}

func (b *Body) Torque(vector []float32) {
	C.NewtonBodyGetTorque(b.handle, (*C.dFloat)(&vector[0]))
}

func (b *Body) ForceAcc(vector []float32) {
	C.NewtonBodyGetForceAcc(b.handle, (*C.dFloat)(&vector[0]))
}

func (b *Body) TorqueAcc(vector []float32) {
	C.NewtonBodyGetTorqueAcc(b.handle, (*C.dFloat)(&vector[0]))
}

func (b *Body) CentreOfMass(com []float32) {
	C.NewtonBodyGetCentreOfMass(b.handle, (*C.dFloat)(&com[0]))
}

func (b *Body) AddImpulse(pointDeltaVeloc, pointPosit []float32) {
	C.NewtonBodyAddImpulse(b.handle, (*C.dFloat)(&pointDeltaVeloc[0]), (*C.dFloat)(&pointPosit[0]))
}

func (b *Body) ApplyImpulsePair(linearImpulse, angularImpulse []float32) {
	C.NewtonBodyApplyImpulsePair(b.handle, (*C.dFloat)(&linearImpulse[0]), (*C.dFloat)(&angularImpulse[0]))
}

func (b *Body) IntegrateVelocity(timestep float32) {
	C.NewtonBodyIntegrateVelocity(b.handle, C.dFloat(timestep))
}

func (b *Body) LinearDamping() float32 {
	return float32(C.NewtonBodyGetLinearDamping(b.handle))
}

func (b *Body) AngularDamping(result []float32) {
	C.NewtonBodyGetAngularDamping(b.handle, (*C.dFloat)(&result[0]))
}

func (b *Body) AABB(p0, p1 []float32) {
	C.NewtonBodyGetAABB(b.handle, (*C.dFloat)(&p0[0]), (*C.dFloat)(&p1[0]))
}

func (b *Body) FirstJoint() *Joint {
	return globalPtr.get(unsafe.Pointer(C.NewtonBodyGetFirstJoint(b.handle))).(*Joint)
}

func (b *Body) NextJoint(curJoint *Joint) *Joint {
	return globalPtr.get(unsafe.Pointer(C.NewtonBodyGetNextJoint(b.handle, curJoint.handle))).(*Joint)
}

func (b *Body) FirstContactJoint() *Joint {
	return globalPtr.get(unsafe.Pointer(C.NewtonBodyGetFirstContactJoint(b.handle))).(*Joint)
}

func (b *Body) NextContactJoint(curJoint *Joint) *Joint {
	return globalPtr.get(unsafe.Pointer(C.NewtonBodyGetNextContactJoint(b.handle, curJoint.handle))).(*Joint)
}
