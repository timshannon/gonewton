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
import (
	"reflect"
	"unsafe"
)

//A note on callbacks.  Since C callbacks must basically
// be hardcoded to one function, I have to do a look up
// in the function to the proper function pointer based on
// the limited data in that callback.
// callbackOwners will contain a map of functions and their owners
// so the proper callback can be looked up on the return into Go code
// using only the owners pointer

type owner unsafe.Pointer

func prepSlice(slicePtr unsafe.Pointer, cArrayPtr unsafe.Pointer, length int) {
	slcHead := (*reflect.SliceHeader)(slicePtr)
	slcHead.Cap = length
	slcHead.Len = length
	slcHead.Data = uintptr(cArrayPtr)
}

//goFloats creates a float32 slice of the given size of the passed in c array pointer
func goFloats(cArray *C.dFloat, size int) []float32 {
	var slice []float32

	prepSlice(unsafe.Pointer(&slice), unsafe.Pointer(cArray), size)
	return slice
}

func goBytes(cArray unsafe.Pointer, size int) []byte {
	var slice []byte

	prepSlice(unsafe.Pointer(&slice), cArray, size)
	return slice
}

func go16Floats(cArray *C.dFloat) *[16]float32 {
	gArray := [16]float32{}
	slice := gArray[:]

	prepSlice(unsafe.Pointer(&slice), unsafe.Pointer(cArray), 16)

	return &gArray
}

func go3Floats(cArray *C.dFloat) *[3]float32 {
	gArray := [3]float32{}

	slice := gArray[:]

	prepSlice(unsafe.Pointer(&slice), unsafe.Pointer(cArray), 3)

	return &gArray
}

func go4Floats(cArray *C.dFloat) *[4]float32 {
	gArray := [4]float32{}

	slice := gArray[:]

	prepSlice(unsafe.Pointer(&slice), unsafe.Pointer(cArray), 4)

	return &gArray
}

func goFloat64s(cArray *C.double, size int) []float64 {
	var slice []float64

	prepSlice(unsafe.Pointer(&slice), unsafe.Pointer(cArray), size)
	return slice
}

type GetTicksCountHandler func() uint32

var getTicksCount GetTicksCountHandler

//export goGetTicksCountCB
func goGetTicksCountCB() uint32 {
	return getTicksCount()
}

func (w *World) SetPerformanceClock(f GetTicksCountHandler) {
	getTicksCount = f
	C.setGetTicksCountCB(w.handle)
}

type BodyLeaveWorldHandler func(body *Body, threadIndex int)

var bodyLeaveWorldOwners = make(map[owner]BodyLeaveWorldHandler)

//export goBodyLeaveWorldCB
func goBodyLeaveWorldCB(body *C.NewtonBody, threadIndex C.int) {
	b := &Body{body}

	//owner is world, look up world from body
	// unfortunately needs an additional c call
	bodyLeaveWorldOwners[owner(b.World().handle)](b, int(threadIndex))
}

func (w *World) SetBodyLeaveWorldEvent(f BodyLeaveWorldHandler) {
	bodyLeaveWorldOwners[owner(w.handle)] = f
	C.setBodyLeaveWorldCB(w.handle)
}

type JointIteratorHandler func(joint *Joint, userData interface{})

var jointIterator JointIteratorHandler

//export goJointIteratorCB
func goJointIteratorCB(joint *C.NewtonJoint, userData unsafe.Pointer) {
	j := &Joint{joint}

	jointIterator(j, ownerData[owner(userData)])
}

func (w *World) ForEachJointDo(f JointIteratorHandler, userData interface{}) {
	jointIterator = f
	ownerData[owner(&userData)] = userData
	C.setJointIteratorCB(w.handle, unsafe.Pointer(&userData))
}

type BodyIteratorHandler func(body *Body, userData interface{})

var bodyIterator BodyIteratorHandler

//export goBodyIteratorCB
func goBodyIteratorCB(body *C.NewtonBody, userData unsafe.Pointer) {
	b := &Body{body}
	bodyIterator(b, ownerData[owner(userData)])
}

func (w *World) ForEachBodyInAABBDo(p0, p1 *[3]float32, f BodyIteratorHandler, userData interface{}) {
	bodyIterator = f
	ownerData[owner(&userData)] = userData
	C.setBodyIteratorCB(w.handle, (*C.dFloat)(&p0[0]), (*C.dFloat)(&p1[0]), unsafe.Pointer(&userData))
}

type RayFilterHandler func(body *Body, hitNormal *[3]float32, collisionID int,
	userData interface{}, intersectParam float32) float32

var rayFilter RayFilterHandler

//export goRayFilterCB
func goRayFilterCB(body *C.NewtonBody, hitNormal *C.dFloat, collisionID C.int,
	userData unsafe.Pointer, intersectParam C.dFloat) C.dFloat {
	b := &Body{body}

	return C.dFloat(rayFilter(b, go3Floats(hitNormal), int(collisionID),
		ownerData[owner(userData)], float32(intersectParam)))
}

type RayPrefilterHandler func(body *Body, collision *Collision, userData interface{}) uint

var rayPrefilter RayPrefilterHandler

//export goRayPrefilterCB
func goRayPrefilterCB(body *C.NewtonBody, collision *C.NewtonCollision, userData unsafe.Pointer) C.unsigned {
	b := &Body{body}
	gCollision := &Collision{collision}

	return C.unsigned(rayPrefilter(b, gCollision, ownerData[owner(userData)]))
}

func (w *World) RayCast(p0 *[3]float32, p1 *[3]float32, filter RayFilterHandler, userData interface{},
	prefilter RayPrefilterHandler) {
	rayFilter = filter
	rayPrefilter = prefilter
	ownerData[owner(&userData)] = userData
	C.RayCast(w.handle, (*C.dFloat)(&p0[0]), (*C.dFloat)(&p1[0]), unsafe.Pointer(&userData))
}

type ConvexCastReturnInfo struct {
	Point            *[4]float32
	Normal           *[4]float32
	NormalOnHitPoint *[4]float32
	Penetration      float32
	ContactID        int
	HitBody          *Body
}

// NewtonWorldCollide: obsolete

func (w *World) ConvexCast(matrix *[16]float32, target *[16]float32, shape *Collision, hitParam *float32,
	userData interface{}, prefilter RayPrefilterHandler, maxContactsCount int, threadIndex int) []*ConvexCastReturnInfo {

	rayPrefilter = prefilter

	ownerData[owner(&userData)] = userData

	var size int
	//all of this allocation may be a performance issue
	// might have to reapproach this
	cInfo := make([]C.NewtonWorldConvexCastReturnInfo, maxContactsCount)

	size = int(C.ConvexCast(w.handle, (*C.dFloat)(&matrix[0]), (*C.dFloat)(&target[0]), shape.handle,
		(*C.dFloat)(hitParam), unsafe.Pointer(&userData), &cInfo[0], C.int(maxContactsCount),
		C.int(threadIndex)))

	returnInfo := make([]*ConvexCastReturnInfo, size)

	for i := range returnInfo {
		returnInfo[i] = &ConvexCastReturnInfo{
			Point:            go4Floats(&cInfo[i].m_point[0]),
			Normal:           go4Floats(&cInfo[i].m_normal[0]),
			NormalOnHitPoint: go4Floats(&cInfo[i].m_normalOnHitPoint[0]),
			Penetration:      float32(cInfo[i].m_penetration),
			ContactID:        int(cInfo[i].m_contactID),
			HitBody:          &Body{cInfo[i].m_hitBody},
		}

	}

	return returnInfo
}

type OnAABBOverlapHandler func(material *Material, body0, body1 *Body, threadIndex int) int

//Not associated to a world for now, this may be an issue with
// multiple worlds
var onAABBOverlap OnAABBOverlapHandler

//export goOnAABBOverlapCB
func goOnAABBOverlapCB(material *C.NewtonMaterial, body0, body1 *C.NewtonBody, threadIndex C.int) C.int {
	gMaterial := &Material{material}
	b0 := &Body{body0}
	b1 := &Body{body1}

	return C.int(onAABBOverlap(gMaterial, b0, b1, int(threadIndex)))
}

type ContactsProcessHandler func(contact *Joint, timestep float32, threadIndex int)

var contactsProcess ContactsProcessHandler

//export goContactsProcessCB
func goContactsProcessCB(contact *C.NewtonJoint, timestep C.dFloat, threadIndex C.int) {
	j := &Joint{contact}
	contactsProcess(j, float32(timestep), int(threadIndex))
}

func (w *World) SetMaterialCollisionCallback(matid0, matid1 int, userData interface{},
	overlap OnAABBOverlapHandler, contactsProcessor ContactsProcessHandler) {
	onAABBOverlap = overlap
	ownerData[owner(&userData)] = userData
	contactsProcess = contactsProcessor
	C.SetCollisionCB(w.handle, C.int(matid0), C.int(matid1), unsafe.Pointer(&userData))
}

type MeshCollisionCollideDesc struct {
	M_boxP0               *[4]float32
	M_boxP1               *[4]float32
	M_boxDistanceTravel   *[4]float32
	M_threadNumber        int
	M_faceCount           int
	M_vertexStrideInBytes int
	M_skinThickness       float32
	M_userData            interface{}
	M_objBody             *Body
	M_polySoupBody        *Body
	M_objCollision        *Collision
	M_polySoupCollision   *Collision
	M_vertex              *[3]float32
	M_faceIndexCount      *int
	M_faceVertexIndex     *int
}

//Skip user mesh for now
//If implemented...
//Separate New user Mesh creation from some callbacks, so that if a user doesn't want to use
// the callbacks, they aren't tied to the very expensive conversions that would be
// tied to the global go callbacks

//type MeshCollisionCollideHandler func(collideDescData *MeshCollisionCollideDesc)

//export goMeshCollisionCollideCB
//func goMeshCollisionCollideCB(collideDescData *C.NewtonUserMeshCollisionCollideDesc) {
//goCollideDescData := &MeshCollisionCollideDesc{

//}
//gMeshCollisionCollide(
//}

//Skip heightfields for now

type CollisionTreeRayCastCallback func(body *Body, treeCollision *Collision, interception float32,
	normal *[3]float32, faceId int, userData interface{}) float32

var collisionTreeRayOwners = make(map[owner]CollisionTreeRayCastCallback)

//export goCollisionTreeRayCastCallback
func goCollisionTreeRayCastCallback(body *C.NewtonBody, treeCollision *C.NewtonCollision, interception C.dFloat,
	normal *C.dFloat, faceId C.int, userData unsafe.Pointer) C.dFloat {
	b := &Body{body}
	col := &Collision{handle: treeCollision}

	return C.dFloat(collisionTreeRayOwners[owner(treeCollision)](b, col, float32(interception), go3Floats(normal),
		int(faceId), ownerData[owner(userData)]))
}

func (c *Collision) SetTreeRayCastCallback(callback CollisionTreeRayCastCallback) {
	collisionTreeRayOwners[owner(c.handle)] = callback

	C.SetUserRayCastCallback(c.handle)
}

type TreeCollisionCallback func(bodyWithTreeCollision, body *Body, faceID, vertexCount int,
	vertex []float32, vertexStrideInBytes int)

var treeCollisionCallback TreeCollisionCallback

//export goTreeCollisionCallback
func goTreeCollisionCallback(bodyWithTreeCollision, body *C.NewtonBody, faceID, vertextCount C.int,
	vertex *C.dFloat, vertexStrideInBytes C.int) {
	bWithTreeCollision := &Body{bodyWithTreeCollision}
	b := &Body{body}
	treeCollisionCallback(bWithTreeCollision, b, int(faceID), int(vertextCount),
		goFloats(vertex, int(vertextCount*3)), int(vertexStrideInBytes))
}

func StaticCollisionSetDebugCallback(staticCollision *Collision, userCallback TreeCollisionCallback) {
	treeCollisionCallback = userCallback
	C.SetStaticCollisionDebugCallback(staticCollision.handle)
}

type BodyDestructorCallback func(body *Body)

var bodyDestructorCallbackOwners = make(map[owner]BodyDestructorCallback)

//export goBodyDestructor
func goBodyDestructor(body *C.NewtonBody) {
	b := &Body{body}

	bodyDestructorCallbackOwners[owner(body)](b)
}

func (b *Body) SetDestructorCallback(callback BodyDestructorCallback) {
	bodyDestructorCallbackOwners[owner(b.handle)] = callback
	C.SetBodyDestructor(b.handle)
}

func (b *Body) DestructorCallback() BodyDestructorCallback {
	return bodyDestructorCallbackOwners[owner(b.handle)]
}

type TransformCallback func(body *Body, matrix *[16]float32, threadIndex int)

var transformCallbackOwners = make(map[owner]TransformCallback)

//export goTransformCallback
func goTransformCallback(body *C.NewtonBody, matrix *C.dFloat, threadIndex C.int) {
	b := &Body{body}

	transformCallbackOwners[owner(body)](b, go16Floats(matrix), int(threadIndex))
}

func (b *Body) SetTransformCallback(callback TransformCallback) {
	transformCallbackOwners[owner(b.handle)] = callback
	C.SetTransformCallback(b.handle)
}

func (b *Body) TransformCallback() TransformCallback {
	return transformCallbackOwners[owner(b.handle)]
}

type ApplyForceAndTorque func(body *Body, timestep float32, threadIndex int)

var applyForceAndTorqueOwners = make(map[owner]ApplyForceAndTorque)

//export goApplyForceAndTorque
func goApplyForceAndTorque(body *C.NewtonBody, timestep C.dFloat, threadIndex C.int) {
	b := &Body{body}

	applyForceAndTorqueOwners[owner(body)](b, float32(timestep), int(threadIndex))
}

func (b *Body) SetForceAndTorqueCallback(callback ApplyForceAndTorque) {
	applyForceAndTorqueOwners[owner(b.handle)] = callback

	C.SetForceAndTorqueCallback(b.handle)
}

func (b *Body) ForceAndTorqueCallback() ApplyForceAndTorque {
	return applyForceAndTorqueOwners[owner(b.handle)]
}

type BuoyancyPlaneHandler func(collisionID int, context interface{}, globalSpaceMatrix *[16]float32, globalSpacePlane *[4]float32) int

var getBuoyancyPlane BuoyancyPlaneHandler

//export goBuoyancyPlaneCallback
func goBuoyancyPlaneCallback(collisionID C.int, context unsafe.Pointer, globalSpaceMatrix,
	globalSpacePlane *C.dFloat) C.int {

	return C.int(getBuoyancyPlane(int(collisionID), ownerData[owner(context)], go16Floats(globalSpaceMatrix),
		go4Floats(globalSpacePlane)))
}

func (b *Body) AddBuoyancyForce(fluidDensity, fluidLinearViscosity, fluidAngularViscosity float32,
	gravityVector *[3]float32, buoyancyPlane BuoyancyPlaneHandler, context interface{}) {
	getBuoyancyPlane = buoyancyPlane

	ownerData[owner(&context)] = context

	C.AddBuoyancyForce(b.handle, C.dFloat(fluidDensity), C.dFloat(fluidLinearViscosity),
		C.dFloat(fluidAngularViscosity), (*C.dFloat)(&gravityVector[0]), unsafe.Pointer(&context))
}

//Joint callbacks

type ConstraintDestructor func(me *Joint)

var constraintDestructorOwners = make(map[owner]ConstraintDestructor)

//export goConstraintDestructor
func goConstraintDestructor(me *C.NewtonJoint) {
	joint := &Joint{me}
	constraintDestructorOwners[owner(me)](joint)
}

func (j *Joint) SetDestructor(destructor ConstraintDestructor) {
	constraintDestructorOwners[owner(j.handle)] = destructor
	C.SetConstraintDestructor(j.handle)
}

type BallCallback func(joint *Joint, timestep float32)

var ballCallbackOwners = make(map[owner]BallCallback)

//export goBallCallback
func goBallCallback(joint *C.NewtonJoint, timestep C.dFloat) {
	j := &Joint{joint}
	ballCallbackOwners[owner(joint)](j, float32(timestep))
}

func SetBallCallback(joint *Joint, callback BallCallback) {
	ballCallbackOwners[owner(joint.handle)] = callback
	C.BallSetUserCallback(joint.handle)
}

type HingeCallback func(joint *Joint, desc *HingeSliderUpdateDesc) uint

var HingeCallbackOwners = make(map[owner]HingeCallback)

//export goHingeCallback
func goHingeCallback(joint *C.NewtonJoint, desc *C.NewtonHingeSliderUpdateDesc) C.unsigned {
	j := &Joint{joint}
	gDesc := &HingeSliderUpdateDesc{desc}
	return C.unsigned(HingeCallbackOwners[owner(joint)](j, gDesc))
}

func SetHingeCallback(joint *Joint, callback HingeCallback) {
	HingeCallbackOwners[owner(joint.handle)] = callback
	C.HingeSetUserCallback(joint.handle)
}

type SliderCallback func(joint *Joint, desc *HingeSliderUpdateDesc) uint

var SliderCallbackOwners = make(map[owner]SliderCallback)

//export goSliderCallback
func goSliderCallback(joint *C.NewtonJoint, desc *C.NewtonHingeSliderUpdateDesc) C.unsigned {
	j := &Joint{joint}
	gDesc := &HingeSliderUpdateDesc{desc}
	return C.unsigned(SliderCallbackOwners[owner(joint)](j, gDesc))
}

func SetSliderCallback(joint *Joint, callback SliderCallback) {
	SliderCallbackOwners[owner(joint.handle)] = callback
	C.SliderSetUserCallback(joint.handle)
}

type CorkscrewCallback func(joint *Joint, desc *HingeSliderUpdateDesc) uint

var CorkscrewCallbackOwners = make(map[owner]CorkscrewCallback)

//export goCorkscrewCallback
func goCorkscrewCallback(joint *C.NewtonJoint, desc *C.NewtonHingeSliderUpdateDesc) C.unsigned {
	j := &Joint{joint}
	gDesc := &HingeSliderUpdateDesc{desc}
	return C.unsigned(CorkscrewCallbackOwners[owner(joint)](j, gDesc))
}

func SetCorkscrewCallback(joint *Joint, callback CorkscrewCallback) {
	CorkscrewCallbackOwners[owner(joint.handle)] = callback
	C.CorkscrewSetUserCallback(joint.handle)
}

type UniversalCallback func(joint *Joint, desc *HingeSliderUpdateDesc) uint

var UniversalCallbackOwners = make(map[owner]UniversalCallback)

//export goUniversalCallback
func goUniversalCallback(joint *C.NewtonJoint, desc *C.NewtonHingeSliderUpdateDesc) C.unsigned {
	j := &Joint{joint}
	gDesc := &HingeSliderUpdateDesc{desc}
	return C.unsigned(UniversalCallbackOwners[owner(joint)](j, gDesc))
}

func SetUniversalCallback(joint *Joint, callback UniversalCallback) {
	UniversalCallbackOwners[owner(joint.handle)] = callback
	C.UniversalSetUserCallback(joint.handle)
}

type ReportProgress func(progressPercent float32)

var reportProgress ReportProgress

//export goReportProgress
func goReportProgress(progressPercent C.dFloat) {
	reportProgress(float32(progressPercent))
}

func (m *Mesh) Simplify(maxVertexCount int, reportProgressCallback ReportProgress) *Mesh {
	reportProgress = reportProgressCallback

	return &Mesh{C.MeshSimplify(m.handle, C.int(maxVertexCount))}
}

func (m *Mesh) ApproximateConvexDecomposition(maxConcavity, backFaceDistanceFactor float32,
	maxCount, maxVertexPerHull int, reportProgressCallback ReportProgress) *Mesh {

	reportProgress = reportProgressCallback
	return &Mesh{C.MeshApproximateConvexDecomposition(m.handle, C.dFloat(maxConcavity),
		C.dFloat(backFaceDistanceFactor), C.int(maxCount), C.int(maxVertexPerHull))}

}

type CollisionIterator func(userData interface{}, vertexCount int, faceArray []float32, faceID int)

var newtonCollisionIterator CollisionIterator

//export goNewtonCollisionIterator
func goNewtonCollisionIterator(userData unsafe.Pointer, vertexCount C.int, faceArray *C.dFloat, faceID C.int) {
	newtonCollisionIterator(ownerData[owner(userData)], int(vertexCount), goFloats(faceArray, int(vertexCount*3)),
		int(faceID))
}

func (c *Collision) ForEachPolygonDo(matrix *[16]float32, callback CollisionIterator, userData interface{}) {
	newtonCollisionIterator = callback
	ownerData[owner(&userData)] = userData
	C.setForEachPolygonDo(c.handle, (*C.dFloat)(&matrix[0]), unsafe.Pointer(&userData))
}

type DeserializeCallback func(serializeHandle interface{}, buffer []byte)

var newtonDeserializeCallback DeserializeCallback

//export goNewtonDeserializeCallback
func goNewtonDeserializeCallback(serializeHandle unsafe.Pointer, buffer unsafe.Pointer, size C.int) {
	newtonDeserializeCallback(ownerData[owner(&serializeHandle)], goBytes(buffer, int(size)))
}

func (w *World) CreateCollisionFromSerialization(deserializeFunc DeserializeCallback, serializeHandle interface{}) *Collision {
	ownerData[owner(&serializeHandle)] = serializeHandle
	return &Collision{C.createCollisionFromSerialization(w.handle, unsafe.Pointer(&serializeHandle))}
}

type SerializeCallback func(serializeHandle interface{}, buffer []byte)

var newtonSerializeCallback SerializeCallback

//export goNewtonSerializeCallback
func goNewtonSerializeCallback(serializeHandle unsafe.Pointer, buffer unsafe.Pointer, size C.int) {
	newtonSerializeCallback(ownerData[owner(&serializeHandle)], goBytes(buffer, int(size)))
}

func (w *World) SerializeCollision(collision *Collision, serializeFunc SerializeCallback, serializeHandle interface{}) {
	ownerData[owner(&serializeHandle)] = serializeHandle
	newtonSerializeCallback = serializeFunc
	C.serializeCollision(w.handle, collision.handle, unsafe.Pointer(&serializeHandle))

}
