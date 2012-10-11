package newton

/*
#cgo   linux LDFLAGS: -L/usr/local/lib -lNewton -lstdc++
#include "Newton.h"
#include <stdlib.h>
*/
import "C"
import "unsafe"

type Collision struct {
	handle *C.NewtonCollision
}

type Mesh struct {
	handle *C.NewtonMesh
}

// *** Primitive Creation Methods
func (w *World) CreateNull() *Collision { return &Collision{C.NewtonCreateNull(w.handle)} }
func (w *World) CreateSphere(radius float32, shapeID int, offsetMatrix []float32) *Collision {
	return &Collision{C.NewtonCreateSphere(w.handle, C.dFloat(radius), C.int(shapeID),
		(*C.dFloat)(&offsetMatrix[0]))}
}

func (w *World) CreateBox(dx, dy, dz float32, shapeID int, offsetMatrix []float32) *Collision {
	return &Collision{C.NewtonCreateBox(w.handle, C.dFloat(dx), C.dFloat(dy), C.dFloat(dz), C.int(shapeID),
		(*C.dFloat)(&offsetMatrix[0]))}
}

func (w *World) CreateCone(radius, height float32, shapeID int, offsetMatrix []float32) *Collision {
	return &Collision{C.NewtonCreateCone(w.handle, C.dFloat(radius), C.dFloat(height), C.int(shapeID),
		(*C.dFloat)(&offsetMatrix[0]))}
}

func (w *World) CreateCapsule(radius, height float32, shapeID int, offsetMatrix []float32) *Collision {
	return &Collision{C.NewtonCreateCapsule(w.handle, C.dFloat(radius), C.dFloat(height), C.int(shapeID),
		(*C.dFloat)(&offsetMatrix[0]))}
}

func (w *World) CreateCylinder(radius, height float32, shapeID int, offsetMatrix []float32) *Collision {
	return &Collision{C.NewtonCreateCylinder(w.handle, C.dFloat(radius), C.dFloat(height), C.int(shapeID),
		(*C.dFloat)(&offsetMatrix[0]))}
}

func (w *World) CreateTaperedCapsule(radio0, radio1, height float32, shapeID int, offsetMatrix []float32) *Collision {
	return &Collision{C.NewtonCreateTaperedCapsule(w.handle, C.dFloat(radio0), C.dFloat(radio1), C.dFloat(height),
		C.int(shapeID), (*C.dFloat)(&offsetMatrix[0]))}

}

func (w *World) CreateTaperedCylinder(radio0, radio1, height float32, shapeID int, offsetMatrix []float32) *Collision {
	return &Collision{C.NewtonCreateTaperedCylinder(w.handle, C.dFloat(radio0), C.dFloat(radio1), C.dFloat(height),
		C.int(shapeID), (*C.dFloat)(&offsetMatrix[0]))}

}

func (w *World) CreateChamferCylinder(radius, height float32, shapeID int, offsetMatrix []float32) *Collision {
	return &Collision{C.NewtonCreateChamferCylinder(w.handle, C.dFloat(radius), C.dFloat(height), C.int(shapeID),
		(*C.dFloat)(&offsetMatrix[0]))}
}

func (w *World) CreateConvexHull(count int, vertexCloud []float32, strideInBytes int, tolerance float32, shapeID int,
	offsetMatrix []float32) *Collision {
	return &Collision{C.NewtonCreateConvexHull(w.handle, C.int(count), (*C.dFloat)(&vertexCloud[0]),
		C.int(strideInBytes), C.dFloat(tolerance), C.int(shapeID), (*C.dFloat)(&offsetMatrix[0]))}
}

func (w *World) CreateConvexHullFromMesh(mesh *Mesh, tolerance float32, shapeID int) *Collision {
	return &Collision{C.NewtonCreateConvexHullFromMesh(w.handle, mesh.handle, C.dFloat(tolerance), C.int(shapeID))}
}

// Primitive typed methods

func (c *Collision) IsTriggerVolume() bool {
	return Bool[int(C.NewtonCollisionIsTriggerVolume(c.handle))]
}

func (c *Collision) SetIsTriggerVolume(value bool) {
	C.NewtonCollisionSetAsTriggerVolume(c.handle, Cint[value])
}

func (c *Collision) FaceIndices(face int, faceIndices []int) int {
	return int(C.NewtonConvexHullGetFaceIndices(c.handle, C.int(face), (*C.int)(&faceIndices[0])))
}

func (c *Collision) CalculateVolume() float32 {
	return float32(C.NewtonConvexCollisionCalculateVolume(c.handle))
}

func (c *Collision) CalculateInertialMatrix(intertia, origin []float32) {
	C.NewtonConvexCollisionCalculateInertialMatrix(c.handle, (*C.dFloat)(&intertia[0]),
		(*C.dFloat)(&origin[0]))
}

type Node struct {
	handle unsafe.Pointer
}

type CompoundCollision struct {
	Collision
}

//Compound Collisions
func (w *World) CreateCompoundCollision(shapeID int) *CompoundCollision {
	return &CompoundCollision{Collision{C.NewtonCreateCompoundCollision(w.handle, C.int(shapeID))}}
}

func (w *World) CreateCompoundCollisionFromMesh(mesh *Mesh, hullTolerance float32, shapeID,
	subShapeID int) *CompoundCollision {
	return &CompoundCollision{Collision{C.NewtonCreateCompoundCollisionFromMesh(w.handle, mesh.handle,
		C.dFloat(hullTolerance), C.int(shapeID), C.int(subShapeID))}}
}

func (c *CompoundCollision) BeginAddRemove() {
	C.NewtonCompoundCollisionBeginAddRemove(c.handle)
}

func (c *CompoundCollision) EndAddRemove() {
	C.NewtonCompoundCollisionEndAddRemove(c.handle)
}

func (c *CompoundCollision) AddSubCollision(subCollision *Collision) *Node {
	return &Node{C.NewtonCompoundCollisionAddSubCollision(c.handle, subCollision.handle)}
}

func (c *CompoundCollision) RemoveSubCollision(collisionNode *Node) {
	C.NewtonCompoundCollisionRemoveSubCollision(c.handle, collisionNode.handle)
}

func (c *CompoundCollision) RemoveSubCollisionByIndex(index int) {
	C.NewtonCompoundCollisionRemoveSubCollisionByIndex(c.handle, C.int(index))
}

func (c *CompoundCollision) SetSubCollisionMatrix(collisionNode *Node, matrix []float32) {
	C.NewtonCompoundCollisionSetSubCollisionMatrix(c.handle, collisionNode.handle,
		(*C.dFloat)(&matrix[0]))
}

func (c *CompoundCollision) FirstNode() *Node {
	return &Node{C.NewtonCompoundCollisionGetFirstNode(c.handle)}
}

func (c *CompoundCollision) NextNode(curNode *Node) *Node {
	return &Node{C.NewtonCompoundCollisionGetNextNode(c.handle, curNode.handle)}
}

func (c *CompoundCollision) NodeByIndex(index int) *Node {
	return &Node{C.NewtonCompoundCollisionGetNodeByIndex(c.handle,
		C.int(index))}
}

func (c *CompoundCollision) NodeIndex(node *Node) int {
	return int(C.NewtonCompoundCollisionGetNodeIndex(c.handle,
		node.handle))
}

func (parent *CompoundCollision) CollisionFromNode(node *Node) *Collision {
	return &Collision{C.NewtonCompoundCollisionGetCollisionFromNode(parent.handle, node.handle)}
}

//SceneCollision
type SceneCollision struct {
	Collision
}

func (w *World) CreateSceneCollision(shapeID int) *SceneCollision {
	return &SceneCollision{Collision{C.NewtonCreateSceneCollision(w.handle, C.int(shapeID))}}
}

func (c *SceneCollision) BeginAddRemove() {
	C.NewtonSceneCollisionBeginAddRemove(c.handle)
}

func (c *SceneCollision) EndAddRemove() {
	C.NewtonSceneCollisionEndAddRemove(c.handle)
}

func (c *SceneCollision) AddSubCollision(subCollision *Collision) *Node {
	return &Node{C.NewtonSceneCollisionAddSubCollision(c.handle, subCollision.handle)}
}

func (c *SceneCollision) SetSubCollisionMatrix(collisionNode *Node, matrix []float32) {
	C.NewtonSceneCollisionSetSubCollisionMatrix(c.handle, collisionNode.handle,
		(*C.dFloat)(&matrix[0]))
}

func (c *SceneCollision) FirstNode() *Node {
	return &Node{C.NewtonSceneCollisionGetFirstNode(c.handle)}
}

func (c *SceneCollision) NextNode(curNode *Node) *Node {
	return &Node{C.NewtonSceneCollisionGetNextNode(c.handle, curNode.handle)}
}

func (parent *SceneCollision) CollisionFromNode(node *Node) *Collision {
	return &Collision{C.NewtonSceneCollisionGetCollisionFromNode(parent.handle, node.handle)}
}
