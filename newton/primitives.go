package newton

/*
#cgo   linux LDFLAGS: -L/usr/local/lib -lNewton -lstdc++
#include "Newton.h"
#include <stdlib.h>
*/
import "C"
import "unsafe"

const (
	CollisionSphere = iota
	CollisionCylinder
	CollisionTaperedCylinder
	CollisionBox
	CollisionCone
	CollisionConvexhull
	CollisionNull
	CollisionCompound
	CollisionTree
	CollisionHeightfield
	CollisionDeformablemesh
	CollisionUsermesh
	CollisionScene
	CollisionCompoundBreakable
)

type Collision struct {
	handle *C.NewtonCollision
}

type Mesh struct {
	handle *C.NewtonMesh
}

// *** Primitive Creation Methods
func (w *World) CreateNull() *Collision {
	collision := &Collision{handle: C.NewtonCreateNull(w.handle)}
	return collision
}
func (w *World) CreateSphere(radius float32, shapeID int, offsetMatrix []float32) *Collision {
	return &Collision{handle: C.NewtonCreateSphere(w.handle, C.dFloat(radius), C.int(shapeID),
		(*C.dFloat)(&offsetMatrix[0]))}
}

func (w *World) CreateBox(dx, dy, dz float32, shapeID int, offsetMatrix []float32) *Collision {
	return &Collision{handle: C.NewtonCreateBox(w.handle, C.dFloat(dx), C.dFloat(dy), C.dFloat(dz), C.int(shapeID),
		(*C.dFloat)(&offsetMatrix[0]))}
}

func (w *World) CreateCone(radius, height float32, shapeID int, offsetMatrix []float32) *Collision {
	return &Collision{handle: C.NewtonCreateCone(w.handle, C.dFloat(radius), C.dFloat(height), C.int(shapeID),
		(*C.dFloat)(&offsetMatrix[0]))}
}

func (w *World) CreateCapsule(radius, height float32, shapeID int, offsetMatrix []float32) *Collision {
	return &Collision{handle: C.NewtonCreateCapsule(w.handle, C.dFloat(radius), C.dFloat(height), C.int(shapeID),
		(*C.dFloat)(&offsetMatrix[0]))}
}

func (w *World) CreateCylinder(radius, height float32, shapeID int, offsetMatrix []float32) *Collision {
	return &Collision{handle: C.NewtonCreateCylinder(w.handle, C.dFloat(radius), C.dFloat(height), C.int(shapeID),
		(*C.dFloat)(&offsetMatrix[0]))}
}

func (w *World) CreateTaperedCapsule(radio0, radio1, height float32, shapeID int, offsetMatrix []float32) *Collision {
	return &Collision{handle: C.NewtonCreateTaperedCapsule(w.handle, C.dFloat(radio0), C.dFloat(radio1),
		C.dFloat(height), C.int(shapeID), (*C.dFloat)(&offsetMatrix[0]))}
}

func (w *World) CreateTaperedCylinder(radio0, radio1, height float32, shapeID int, offsetMatrix []float32) *Collision {
	return &Collision{handle: C.NewtonCreateTaperedCylinder(w.handle, C.dFloat(radio0), C.dFloat(radio1),
		C.dFloat(height), C.int(shapeID), (*C.dFloat)(&offsetMatrix[0]))}
}

func (w *World) CreateChamferCylinder(radius, height float32, shapeID int, offsetMatrix []float32) *Collision {
	return &Collision{handle: C.NewtonCreateChamferCylinder(w.handle, C.dFloat(radius), C.dFloat(height),
		C.int(shapeID), (*C.dFloat)(&offsetMatrix[0]))}
}

func (w *World) CreateConvexHull(count int, vertexCloud []float32, strideInBytes int, tolerance float32, shapeID int,
	offsetMatrix []float32) *Collision {
	return &Collision{handle: C.NewtonCreateConvexHull(w.handle, C.int(count), (*C.dFloat)(&vertexCloud[0]),
		C.int(strideInBytes), C.dFloat(tolerance), C.int(shapeID), (*C.dFloat)(&offsetMatrix[0]))}
}

func (w *World) CreateConvexHullFromMesh(mesh *Mesh, tolerance float32, shapeID int) *Collision {
	return &Collision{handle: C.NewtonCreateConvexHullFromMesh(w.handle, mesh.handle, C.dFloat(tolerance),
		C.int(shapeID))}
}

// Primitive typed methods

func (c *Collision) IsTriggerVolume() bool {
	return gbool[int(C.NewtonCollisionIsTriggerVolume(c.handle))]
}

func (c *Collision) SetIsTriggerVolume(value bool) {
	C.NewtonCollisionSetAsTriggerVolume(c.handle, cint[value])
}

func (c *Collision) FaceIndices(face int, faceIndices []int) int {
	return int(C.NewtonConvexHullGetFaceIndices(c.handle, C.int(face), (*C.int)(unsafe.Pointer(&faceIndices[0]))))
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

//Compound Collisions
func (w *World) CreateCompoundCollision(shapeID int) *Collision {
	return &Collision{handle: C.NewtonCreateCompoundCollision(w.handle, C.int(shapeID))}
}

func (w *World) CreateCompoundCollisionFromMesh(mesh *Mesh, hullTolerance float32, shapeID,
	subShapeID int) *Collision {
	return &Collision{handle: C.NewtonCreateCompoundCollisionFromMesh(w.handle, mesh.handle,
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
	return &Collision{handle: C.NewtonCompoundCollisionGetCollisionFromNode(parent.handle, node.handle)}
}

//SceneCollision

func (w *World) CreateSceneCollision(shapeID int) *Collision {
	return &Collision{handle: C.NewtonCreateSceneCollision(w.handle, C.int(shapeID))}
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
	return &Collision{handle: C.NewtonSceneCollisionGetCollisionFromNode(parent.handle, node.handle)}
}

//TreeCollision
func (w *World) CreateTreeCollision(shapeID int) *Collision {
	return &Collision{handle: C.NewtonCreateTreeCollision(w.handle, C.int(shapeID))}
}

func (w *World) CreateTreeCollsionFromMesh(mesh *Mesh, shapeID int) *Collision {
	return &Collision{handle: C.NewtonCreateTreeCollisionFromMesh(w.handle, mesh.handle, C.int(shapeID))}
}

func (c *Collision) BeginBuild() {
	C.NewtonTreeCollisionBeginBuild(c.handle)
}

func (c *Collision) AddFace(vertexCount int, vertexPtr []float32, strideInBytes, faceAttribute int) {
	C.NewtonTreeCollisionAddFace(c.handle, C.int(vertexCount), (*C.dFloat)(&vertexPtr[0]), C.int(strideInBytes),
		C.int(faceAttribute))
}

//EndBuild ends the building of a Tree collision primative.
// Optimize should be set to true for concave meshes
func (c *Collision) EndBuild(optimize bool) {
	C.NewtonTreeCollisionEndBuild(c.handle, cint[optimize])
}

func (c *Collision) FaceAttribute(faceIndexArray []int) int {
	return int(C.NewtonTreeCollisionGetFaceAtribute(c.handle, (*C.int)(unsafe.Pointer(&faceIndexArray[0]))))
}

func (c *Collision) SetFaceAttribute(faceIndexArray []int, attribute int) {
	C.NewtonTreeCollisionSetFaceAtribute(c.handle, (*C.int)(unsafe.Pointer(&faceIndexArray[0])),
		C.int(attribute))
}

//skip for now
//func (c *Collision) VertexListIndexListInAABB(p0, p1 []float32, vertexAray []int, 

//General Purpose collision library functions

func (c *Collision) CreateInstance() *Collision {
	return &Collision{handle: C.NewtonCollisionCreateInstance(c.handle)}
}

func (c *Collision) Type() int {
	return int(C.NewtonCollisionGetType(c.handle))
}

func (c *Collision) SetUserID(id uint) {
	C.NewtonCollisionSetUserID(c.handle, C.unsigned(id))
}

func (c *Collision) UserID() uint {
	return uint(C.NewtonCollisionGetUserID(c.handle))
}

func (c *Collision) UserData() interface{} {
	//redirection necessary for handling instanced collisions
	return (interface{})(C.NewtonCollisionGetUserData(c.handle))
}

func (c *Collision) SetUserData(data interface{}) {
	C.NewtonCollisionSetUserData(c.handle, unsafe.Pointer(&data))
}

func (c *Collision) SetMatrix(matrix []float32) {
	C.NewtonCollisionSetMatrix(c.handle, (*C.dFloat)(&matrix[0]))
}

func (c *Collision) Matrix(matrix []float32) {
	C.NewtonCollisionGetMatrix(c.handle, (*C.dFloat)(&matrix[0]))
}

func (c *Collision) SetScale(x, y, z float32) {
	C.NewtonCollisionSetScale(c.handle, C.dFloat(x), C.dFloat(y), C.dFloat(z))
}

func (c *Collision) Scale() (x, y, z float32) {
	C.NewtonCollisionGetScale(c.handle, (*C.dFloat)(&x), (*C.dFloat)(&y), (*C.dFloat)(&z))
	return
}

func (c *Collision) Destroy() {
	C.NewtonDestroyCollision(c.handle)
}

//Mesh
func (w *World) CreateMesh() *Mesh {
	return &Mesh{C.NewtonMeshCreate(w.handle)}
}

func (m *Mesh) Duplicate() *Mesh {
	return &Mesh{C.NewtonMeshCreateFromMesh(m.handle)}
}

func (c *Collision) CreateMesh() *Mesh {
	return &Mesh{C.NewtonMeshCreateFromCollision(c.handle)}
}

func (w *World) CreateConvexMesh(pointCount int, vertexCloud []float32, strideInBytes int,
	tolerance float32) *Mesh {
	return &Mesh{C.NewtonMeshCreateConvexHull(w.handle, C.int(pointCount), (*C.dFloat)(&vertexCloud[0]),
		C.int(strideInBytes), C.dFloat(tolerance))}
}

func (w *World) CreateDelaunayTetrahedralizationMesh(pointCount int, vertexCloud []float32, strideInBytes,
	materialID int, textureMatrix []float32) *Mesh {
	return &Mesh{C.NewtonMeshCreateDelaunayTetrahedralization(w.handle, C.int(pointCount),
		(*C.dFloat)(&vertexCloud[0]), C.int(strideInBytes), C.int(materialID),
		(*C.dFloat)(&textureMatrix[0]))}
}

func (w *World) CreateVoronoiConvexDecompositionMesh(pointCount int, vertexCloud []float32, strideInBytes,
	materialID int, textureMatrix []float32, borderConvexSize float32) *Mesh {
	return &Mesh{C.NewtonMeshCreateVoronoiConvexDecomposition(w.handle, C.int(pointCount),
		(*C.dFloat)(&vertexCloud[0]), C.int(strideInBytes), C.int(materialID),
		(*C.dFloat)(&textureMatrix[0]), C.dFloat(borderConvexSize))}
}

func (m *Mesh) Destroy() {
	C.NewtonMeshDestroy(m.handle)
}

func (m *Mesh) ApplyTransform(matrix []float32) {
	C.NewtonMesApplyTransform(m.handle, (*C.dFloat)(&matrix[0]))
}

func (m *Mesh) CalculateOOBB(matrix []float32, x, y, z *float32) {
	C.NewtonMeshCalculateOOBB(m.handle, (*C.dFloat)(&matrix[0]), (*C.dFloat)(x),
		(*C.dFloat)(y), (*C.dFloat)(z))
}

func (m *Mesh) CalculateVertexNormals(angleInRadians float32) {
	C.NewtonMeshCalculateVertexNormals(m.handle, C.dFloat(angleInRadians))
}

func (m *Mesh) ApplySphericalMapping(material int) {
	C.NewtonMeshApplySphericalMapping(m.handle, C.int(material))
}

func (m *Mesh) ApplyBoxMapping(front, side, top int) {
	C.NewtonMeshApplyBoxMapping(m.handle, C.int(front), C.int(side), C.int(top))
}

func (m *Mesh) ApplyCylindricalMapping(cylinderMaterial, capMaterial int) {
	C.NewtonMeshApplyCylindricalMapping(m.handle, C.int(cylinderMaterial), C.int(capMaterial))
}

func (m *Mesh) IsOpenMesh() bool {
	return gbool[int(C.NewtonMeshIsOpenMesh(m.handle))]
}

func (m *Mesh) FixTJoints() {
	C.NewtonMeshFixTJoints(m.handle)
}

func (m *Mesh) Polygonize() {
	C.NewtonMeshPolygonize(m.handle)
}

func (m *Mesh) Triangulate() {
	C.NewtonMeshTriangulate(m.handle)
}

func (m *Mesh) Union(clipper *Mesh, clipperMatrix []float32) {
	C.NewtonMeshUnion(m.handle, clipper.handle, (*C.dFloat)(&clipperMatrix[0]))
}

func (m *Mesh) Difference(clipper *Mesh, clipperMatrix []float32) {
	C.NewtonMeshDifference(m.handle, clipper.handle, (*C.dFloat)(&clipperMatrix[0]))
}

func (m *Mesh) Intersection(clipper *Mesh, clipperMatrix []float32) {
	C.NewtonMeshIntersection(m.handle, clipper.handle, (*C.dFloat)(&clipperMatrix[0]))
}

func (m *Mesh) Clip(clipper *Mesh, clipperMatrix []float32) (topMesh, bottomMesh *Mesh) {
	topMesh = new(Mesh)
	bottomMesh = new(Mesh)
	//fix pointer to pointer
	C.NewtonMeshClip(m.handle, clipper.handle, (*C.dFloat)(&clipperMatrix[0]),
		topMesh.handle, bottomMesh.handle)
}
