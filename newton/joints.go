package newton

/*
#cgo   linux LDFLAGS: -L/usr/local/lib -lNewton -lstdc++
#include "Newton.h"
#include <stdlib.h>
*/
import "C"
import "unsafe"

type Joint struct {
	handle   *C.NewtonJoint
	body0    *Body
	body1    *Body
	UserData interface{}
}

func (j *Joint) ptr() unsafe.Pointer { return unsafe.Pointer(j.handle) }

func (w *World) createGoJoint(cObject *C.NewtonJoint, body0, body1 *Body) *Joint {
	joint := new(Joint)
	joint.handle = cObject
	joint.body0 = body0
	joint.body1 = body1

	globalPtr.add(joint)

	return joint
}

type Contact struct {
	handle unsafe.Pointer
}

func (j *Joint) FirstContact() *Contact {
	return &Contact{C.NewtonContactJointGetFirstContact(j.handle)}
}

func (j *Joint) NextContact(curContact *Contact) *Contact {
	return &Contact{C.NewtonContactJointGetNextContact(j.handle, curContact.handle)}
}

func (j *Joint) ContactCount() int {
	return int(C.NewtonContactJointGetContactCount(j.handle))
}

func (j *Joint) RemoveContact(contact *Contact) {
	C.NewtonContactJointRemoveContact(j.handle, contact.handle)
}

func (c *Contact) Material() *Material {
	return &Material{C.NewtonContactGetMaterial(c.handle)}
}

func (j *Joint) Body0() *Body {
	return j.body0
}

func (j *Joint) Body1() *Body {
	return j.body1
}

type JointRecord struct {
	AttachmentMatrix0 [][]float32
	AttachmentMatrix1 [][]float32
	MinLinearDof      []float32
	MaxLinearDof      []float32
	MinAngularDof     []float32
	MaxAngularDof     []float32
	AttachBody0       *Body
	AttachBody1       *Body
	ExtraParameters   []float32
	BodiesCollisionOn int
	DescriptionType   string
}

func (j *Joint) Info() *JointRecord {
	var cInfo *C.NewtonJointRecord
	C.NewtonJointGetInfo(j.handle, cInfo)

	return &JointRecord{
		AttachmentMatrix0: get4x4Float(cInfo.m_attachmenMatrix_0),
		AttachmentMatrix1: get4x4Float(cInfo.m_attachmenMatrix_1),
		MinLinearDof:      goFloats(&cInfo.m_minLinearDof[0], 3),
		MaxLinearDof:      goFloats(&cInfo.m_maxLinearDof[0], 3),
		MinAngularDof:     goFloats(&cInfo.m_minAngularDof[0], 3),
		MaxAngularDof:     goFloats(&cInfo.m_maxAngularDof[0], 3),
		AttachBody0:       globalPtr.get(unsafe.Pointer(cInfo.m_attachBody_0)).(*Body),
		AttachBody1:       globalPtr.get(unsafe.Pointer(cInfo.m_attachBody_1)).(*Body),
		ExtraParameters:   goFloats(&cInfo.m_extraParameters[0], 16),
		BodiesCollisionOn: int(cInfo.m_bodiesCollisionOn),
		DescriptionType:   C.GoString(&cInfo.m_descriptionType[0]),
	}
}

//Untested, not sure why original lib switched to 4x4 arrays from
// [16]arrays for matrices
func get4x4Float(array [4][4]C.dFloat) [][]float32 {
	slice := make([][]float32, 4)

	for i := range slice {
		slice[i] = goFloats(&array[i][0], 4)
	}

	return slice
}

func (j *Joint) CollisionState() int {
	return int(C.NewtonJointGetCollisionState(j.handle))
}

func (j *Joint) SetCollisionState(state int) {
	C.NewtonJointSetCollisionState(j.handle, C.int(state))
}

func (j *Joint) Stiffness() float32 {
	return float32(C.NewtonJointGetStiffness(j.handle))
}

func (j *Joint) SetStiffness(stiffness float32) {
	C.NewtonJointSetStiffness(j.handle, C.dFloat(stiffness))
}
