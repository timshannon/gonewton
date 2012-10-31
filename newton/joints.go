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
import "unsafe"

type Joint struct {
	handle *C.NewtonJoint
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
	return &Body{C.NewtonJointGetBody0(j.handle)}
}

func (j *Joint) Body1() *Body {
	return &Body{C.NewtonJointGetBody1(j.handle)}
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
		AttachBody0:       &Body{cInfo.m_attachBody_0},
		AttachBody1:       &Body{cInfo.m_attachBody_1},
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

func (w *World) DestroyJoint(joint *Joint) {
	C.NewtonDestroyJoint(w.handle, joint.handle)
}

//Particle Systems interface (soft bodies, pressure bodies, and cloth)  

type DeformableMeshSegment struct {
	handle          *C.NewtonDeformableMeshSegment
	collisionParent *C.NewtonCollision
}

func (w *World) CreateDeformableMesh(mesh *Mesh, shapeID int) *Collision {
	return &Collision{C.NewtonCreateDeformableMesh(w.handle, mesh.handle, C.int(shapeID))}
}

func (c *Collision) SetDeformableMeshPlasticity(plasticity float32) {
	C.NewtonDeformableMeshSetPlasticity(c.handle, C.dFloat(plasticity))
}

func (c *Collision) SetDeformableMeshStiffness(stiffness float32) {
	C.NewtonDeformableMeshSetStiffness(c.handle, C.dFloat(stiffness))
}

func (c *Collision) SetDeformableMeshSkinThickness(skinThickness float32) {
	C.NewtonDeformableMeshSetSkinThickness(c.handle, C.dFloat(skinThickness))
}

func (w *World) CreateDeformableBody(deformableMesh *Collision, matrix []float32) *Body {
	return &Body{C.NewtonCreateDeformableBody(w.handle, deformableMesh.handle,
		(*C.dFloat)(&matrix[0]))}
}

func (c *Collision) DeformableMeshUpdateRenderNormals() {
	C.NewtonDeformableMeshUpdateRenderNormals(c.handle)
}

func (c *Collision) DeformableMeshVertexCount() int {
	return int(C.NewtonDeformableMeshGetVertexCount(c.handle))
}

func (c *Collision) DeformableMeshVertexStreams(vertexStrideInByte int, vertex []float32,
	normalStrideInByte int, normal []float32, uvStrideInByte0 int, uv0 []float32,
	uvStrideInByte1 int, uv1 []float32) {
	C.NewtonDeformableMeshGetVertexStreams(c.handle, C.int(vertexStrideInByte), (*C.dFloat)(&vertex[0]),
		C.int(normalStrideInByte), (*C.dFloat)(&normal[0]), C.int(uvStrideInByte0),
		(*C.dFloat)(&uv0[0]), C.int(uvStrideInByte1), (*C.dFloat)(&uv1[0]))
}

func (c *Collision) DeformableMeshFirstSegment() *DeformableMeshSegment {
	return &DeformableMeshSegment{C.NewtonDeformableMeshGetFirstSegment(c.handle), c.handle}
}

func (c *Collision) DeformableMeshNextSegment(curSegment *DeformableMeshSegment) *DeformableMeshSegment {
	return &DeformableMeshSegment{C.NewtonDeformableMeshGetNextSegment(c.handle, curSegment.handle), c.handle}
}

func (segment *DeformableMeshSegment) MaterialID() int {
	return int(C.NewtonDeformableMeshSegmentGetMaterialID(segment.collisionParent, segment.handle))
}

func (segment *DeformableMeshSegment) IndexCount() int {
	return int(C.NewtonDeformableMeshSegmentGetIndexCount(segment.collisionParent, segment.handle))
}

func (segment *DeformableMeshSegment) IndexList(result []int) {
	indexList := C.NewtonDeformableMeshSegmentGetIndexList(segment.collisionParent, segment.handle)
	C.CopyShortArray(indexList, (*C.short)(unsafe.Pointer(&result[0])), C.int(len(result)))
}

//Ball and Socket Joint

func (w *World) CreateBallAndSocket(pivotPoint []float32, child, parent *Body) *Joint {
	return &Joint{C.NewtonConstraintCreateBall(w.handle, (*C.dFloat)(&pivotPoint[0]),
		child.handle, parent.handle)}
}
