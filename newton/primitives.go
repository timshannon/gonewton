package newton

/*
#cgo   linux LDFLAGS: -L/usr/local/lib -lNewton -lstdc++
#include "Newton.h"
#include <stdlib.h>
*/
import "C"
import "unsafe"

type Collision struct {
	handle   *C.NewtonCollision
	UserData interface{}
}

//collisions are instances in newton, so the  pointer on the Go side
// may not be the same object on the newton side.  Instead the go pointer
// will be stored in the collsion user data.
func (c *Collision) ptr() unsafe.Pointer {
	return C.NewtonCollisionGetUserData(c.handle)
}

type Mesh struct {
	handle *C.NewtonMesh
}

// *** Primitive Creation Methods
//TODO: Set NewtonUserData on create to point to Go Pointer
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
	return gbool[int(C.NewtonCollisionIsTriggerVolume(c.handle))]
}

func (c *Collision) SetIsTriggerVolume(value bool) {
	C.NewtonCollisionSetAsTriggerVolume(c.handle, cint[value])
}

//func (c *Collision) FaceIndices(face int, faceIndices []int) int {
//	return int(C.NewtonConvexHullGetFaceIndices(c.handle, C.int(face), (*C.int)(&faceIndices[0])))
//}

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

//Compound Collisions
func (w *World) CreateCompoundCollision(shapeID int) *Collision {
	return &Collision{C.NewtonCreateCompoundCollision(w.handle, C.int(shapeID))}
}

func (w *World) CreateCompoundCollisionFromMesh(mesh *Mesh, hullTolerance float32, shapeID,
	subShapeID int) *Collision {
	return &Collision{C.NewtonCreateCompoundCollisionFromMesh(w.handle, mesh.handle,
		C.dFloat(hullTolerance), C.int(shapeID), C.int(subShapeID))}
}

func (c *Collision) CompoundBeginAddRemove() {
	C.NewtonCompoundCollisionBeginAddRemove(c.handle)
}

func (c *Collision) CompoundEndAddRemove() {
	C.NewtonCompoundCollisionEndAddRemove(c.handle)
}

func (c *Collision) CompoundAddSubCollision(subCollision *Collision) *Node {
	return &Node{C.NewtonCompoundCollisionAddSubCollision(c.handle, subCollision.handle)}
}

func (c *Collision) CompoundRemoveSubCollision(collisionNode *Node) {
	C.NewtonCompoundCollisionRemoveSubCollision(c.handle, collisionNode.handle)
}

func (c *Collision) CompoundRemoveSubCollisionByIndex(index int) {
	C.NewtonCompoundCollisionRemoveSubCollisionByIndex(c.handle, C.int(index))
}

func (c *Collision) SetSubCollisionMatrix(collisionNode *Node, matrix []float32) {
	C.NewtonCompoundCollisionSetSubCollisionMatrix(c.handle, collisionNode.handle,
		(*C.dFloat)(&matrix[0]))
}

func (c *Collision) CompoundFirstNode() *Node {
	return &Node{C.NewtonCompoundCollisionGetFirstNode(c.handle)}
}

func (c *Collision) CompoundNextNode(curNode *Node) *Node {
	return &Node{C.NewtonCompoundCollisionGetNextNode(c.handle, curNode.handle)}
}

func (c *Collision) CompoundNodeByIndex(index int) *Node {
	return &Node{C.NewtonCompoundCollisionGetNodeByIndex(c.handle,
		C.int(index))}
}

func (c *Collision) CompoundNodeIndex(node *Node) int {
	return int(C.NewtonCompoundCollisionGetNodeIndex(c.handle,
		node.handle))
}

func (parent *Collision) CompoundCollisionFromNode(node *Node) *Collision {
	return &Collision{C.NewtonCompoundCollisionGetCollisionFromNode(parent.handle, node.handle)}
}

//SceneCollision

func (w *World) CreateSceneCollision(shapeID int) *Collision {
	return &Collision{C.NewtonCreateSceneCollision(w.handle, C.int(shapeID))}
}

func (c *Collision) SceneBeginAddRemove() {
	C.NewtonSceneCollisionBeginAddRemove(c.handle)
}

func (c *Collision) SceneEndAddRemove() {
	C.NewtonSceneCollisionEndAddRemove(c.handle)
}

func (c *Collision) SceneAddSubCollision(subCollision *Collision) *Node {
	return &Node{C.NewtonSceneCollisionAddSubCollision(c.handle, subCollision.handle)}
}

func (c *Collision) SceneSetSubCollisionMatrix(collisionNode *Node, matrix []float32) {
	C.NewtonSceneCollisionSetSubCollisionMatrix(c.handle, collisionNode.handle,
		(*C.dFloat)(&matrix[0]))
}

func (c *Collision) SceneFirstNode() *Node {
	return &Node{C.NewtonSceneCollisionGetFirstNode(c.handle)}
}

func (c *Collision) SceneNextNode(curNode *Node) *Node {
	return &Node{C.NewtonSceneCollisionGetNextNode(c.handle, curNode.handle)}
}

func (parent *Collision) SceneCollisionFromNode(node *Node) *Collision {
	return &Collision{C.NewtonSceneCollisionGetCollisionFromNode(parent.handle, node.handle)}
}

//TreeCollision
func (w *World) CreateTreeCollision(shapeID int) *Collision {
	return &Collision{C.NewtonCreateTreeCollision(w.handle, C.int(shapeID))}
}

func (w *World) CreateTreeCollsionFromMesh(mesh *Mesh, shapeID int) *Collision {
	return &Collision{C.NewtonCreateTreeCollisionFromMesh(w.handle, mesh.handle, C.int(shapeID))}
}
