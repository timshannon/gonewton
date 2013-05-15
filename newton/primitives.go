package newton

/*
#cgo   linux LDFLAGS: -L/usr/local/lib -lNewton -lstdc++
#include "Newton.h"
#include "callback.h"
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
func (w *World) CreateSphere(radius float32, shapeID int, offsetMatrix *[16]float32) *Collision {
	return &Collision{handle: C.NewtonCreateSphere(w.handle, C.dFloat(radius), C.int(shapeID),
		(*C.dFloat)(&offsetMatrix[0]))}
}

func (w *World) CreateBox(dx, dy, dz float32, shapeID int, offsetMatrix *[16]float32) *Collision {
	return &Collision{handle: C.NewtonCreateBox(w.handle, C.dFloat(dx), C.dFloat(dy), C.dFloat(dz), C.int(shapeID),
		(*C.dFloat)(&offsetMatrix[0]))}
}

func (w *World) CreateCone(radius, height float32, shapeID int, offsetMatrix *[16]float32) *Collision {
	return &Collision{handle: C.NewtonCreateCone(w.handle, C.dFloat(radius), C.dFloat(height), C.int(shapeID),
		(*C.dFloat)(&offsetMatrix[0]))}
}

func (w *World) CreateCapsule(radius, height float32, shapeID int, offsetMatrix *[16]float32) *Collision {
	return &Collision{handle: C.NewtonCreateCapsule(w.handle, C.dFloat(radius), C.dFloat(height), C.int(shapeID),
		(*C.dFloat)(&offsetMatrix[0]))}
}

func (w *World) CreateCylinder(radius, height float32, shapeID int, offsetMatrix *[16]float32) *Collision {
	return &Collision{handle: C.NewtonCreateCylinder(w.handle, C.dFloat(radius), C.dFloat(height), C.int(shapeID),
		(*C.dFloat)(&offsetMatrix[0]))}
}

func (w *World) CreateTaperedCapsule(radio0, radio1, height float32, shapeID int, offsetMatrix *[16]float32) *Collision {
	return &Collision{handle: C.NewtonCreateTaperedCapsule(w.handle, C.dFloat(radio0), C.dFloat(radio1),
		C.dFloat(height), C.int(shapeID), (*C.dFloat)(&offsetMatrix[0]))}
}

func (w *World) CreateTaperedCylinder(radio0, radio1, height float32, shapeID int, offsetMatrix *[16]float32) *Collision {
	return &Collision{handle: C.NewtonCreateTaperedCylinder(w.handle, C.dFloat(radio0), C.dFloat(radio1),
		C.dFloat(height), C.int(shapeID), (*C.dFloat)(&offsetMatrix[0]))}
}

func (w *World) CreateChamferCylinder(radius, height float32, shapeID int, offsetMatrix *[16]float32) *Collision {
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

//func (c *Collision) IsTriggerVolume() bool {
//return gbool[int(C.NewtonCollisionIsTriggerVolume(c.handle))]
//}

//func (c *Collision) SetIsTriggerVolume(value bool) {
//C.NewtonCollisionSetAsTriggerVolume(c.handle, cint[value])
//}

func (c *Collision) FaceIndices(face int, faceIndices []int) int {
	return int(C.NewtonConvexHullGetFaceIndices(c.handle, C.int(face), (*C.int)(unsafe.Pointer(&faceIndices[0]))))
}

func (c *Collision) CalculateVolume() float32 {
	return float32(C.NewtonConvexCollisionCalculateVolume(c.handle))
}

func (c *Collision) CalculateInertialMatrix(intertia, origin *[3]float32) {
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

func (c *Collision) SetSubCollisionMatrix(collisionNode *Node, matrix *[16]float32) {
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

func (c *Collision) SceneSetSubCollisionMatrix(collisionNode *Node, matrix *[16]float32) {
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

func (c *Collision) BeginTreeBuild() {
	C.NewtonTreeCollisionBeginBuild(c.handle)
}

func (c *Collision) AddTreeFace(vertexCount int, vertexPtr []float32, strideInBytes, faceAttribute int) {
	C.NewtonTreeCollisionAddFace(c.handle, C.int(vertexCount), (*C.dFloat)(&vertexPtr[0]), C.int(strideInBytes),
		C.int(faceAttribute))
}

//EndBuild ends the building of a Tree collision primative.
// Optimize should be set to true for concave meshes
func (c *Collision) EndTreeBuild(optimize bool) {
	C.NewtonTreeCollisionEndBuild(c.handle, cint[optimize])
}

func (c *Collision) TreeFaceAttribute(faceIndexArray []int, indexCount int) int {
	return int(C.NewtonTreeCollisionGetFaceAtribute(c.handle, (*C.int)(unsafe.Pointer(&faceIndexArray[0])),
		C.int(indexCount)))
}

func (c *Collision) SetTreeFaceAttribute(faceIndexArray []int, indexCount int, attribute int) {
	C.NewtonTreeCollisionSetFaceAtribute(c.handle, (*C.int)(unsafe.Pointer(&faceIndexArray[0])),
		C.int(indexCount), C.int(attribute))
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
	//return (interface{})(C.NewtonCollisionGetUserData(c.handle))
	return ownerData[owner(c.handle)]
}

func (c *Collision) SetUserData(data interface{}) {
	//C.NewtonCollisionSetUserData(c.handle, unsafe.Pointer(&data))
	ownerData[owner(c.handle)] = data
}

func (c *Collision) SetMatrix(matrix *[16]float32) {
	C.NewtonCollisionSetMatrix(c.handle, (*C.dFloat)(&matrix[0]))
}

func (c *Collision) Matrix(matrix *[16]float32) {
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

func (m *Mesh) ApplyTransform(matrix *[16]float32) {
	C.NewtonMesApplyTransform(m.handle, (*C.dFloat)(&matrix[0]))
}

func (m *Mesh) CalculateOOBB(matrix *[16]float32, x, y, z *float32) {
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

func (m *Mesh) Union(clipper *Mesh, clipperMatrix *[16]float32) {
	C.NewtonMeshUnion(m.handle, clipper.handle, (*C.dFloat)(&clipperMatrix[0]))
}

func (m *Mesh) Difference(clipper *Mesh, clipperMatrix *[16]float32) {
	C.NewtonMeshDifference(m.handle, clipper.handle, (*C.dFloat)(&clipperMatrix[0]))
}

func (m *Mesh) Intersection(clipper *Mesh, clipperMatrix *[16]float32) {
	C.NewtonMeshIntersection(m.handle, clipper.handle, (*C.dFloat)(&clipperMatrix[0]))
}

func (m *Mesh) Clip(clipper *Mesh, clipperMatrix *[16]float32) (topMesh, bottomMesh *Mesh) {
	topMesh = new(Mesh)
	bottomMesh = new(Mesh)

	C.NewtonMeshClip(m.handle, clipper.handle, (*C.dFloat)(&clipperMatrix[0]),
		(**C.NewtonMesh)(unsafe.Pointer(topMesh.handle)),
		(**C.NewtonMesh)(unsafe.Pointer(bottomMesh.handle)))
	return
}

func (m *Mesh) RemoveUnusedVertices(vertexRemapTable []int) {
	C.NewtonRemoveUnusedVertices(m.handle, (*C.int)(unsafe.Pointer(&vertexRemapTable[0])))
}

func (m *Mesh) BeginFace() {
	C.NewtonMeshBeginFace(m.handle)
}

func (m *Mesh) AddFace(vertexCount int, vertex []float32, strideInBytes, materialIndex int) {
	C.NewtonMeshAddFace(m.handle, C.int(vertexCount), (*C.dFloat)(&vertex[0]), C.int(strideInBytes),
		C.int(materialIndex))
}

func (m *Mesh) EndFace() {
	C.NewtonMeshEndFace(m.handle)
}

func (m *Mesh) BuildFromVertexListIndexList(faceCount int, faceIndexCount, faceMaterialIndex []int,
	vertex []float32, vertexStrideInBytes int, vertexIndex []int,
	normal []float32, normalStrideInBytes int, normalIndex []int,
	uv0 []float32, uv0StrideInBytes int, uv0Index []int,
	uv1 []float32, uv1StrideInBytes int, uv1Index []int) {

	C.NewtonMeshBuildFromVertexListIndexList(m.handle, C.int(faceCount),
		(*C.int)(unsafe.Pointer(&faceIndexCount[0])), (*C.int)(unsafe.Pointer(&faceMaterialIndex[0])),
		(*C.dFloat)(&vertex[0]), C.int(vertexStrideInBytes), (*C.int)(unsafe.Pointer(&vertexIndex[0])),
		(*C.dFloat)(&normal[0]), C.int(normalStrideInBytes), (*C.int)(unsafe.Pointer(&normalIndex[0])),
		(*C.dFloat)(&uv0[0]), C.int(uv0StrideInBytes), (*C.int)(unsafe.Pointer(&uv0Index[0])),
		(*C.dFloat)(&uv1[0]), C.int(uv1StrideInBytes), (*C.int)(unsafe.Pointer(&uv1Index[0])))
}

func (m *Mesh) VertexStreams(vertexStrideInByte int, vertex []float32,
	normalStrideInByte int, normal []float32,
	uv0StrideInByte int, uv0 []float32,
	uv1StrideInByte int, uv1 []float32) {

	C.NewtonMeshGetVertexStreams(m.handle, C.int(vertexStrideInByte), (*C.dFloat)(&vertex[0]),
		C.int(normalStrideInByte), (*C.dFloat)(&normal[0]),
		C.int(uv0StrideInByte), (*C.dFloat)(&uv0[0]),
		C.int(uv1StrideInByte), (*C.dFloat)(&uv1[0]))
}

func (m *Mesh) IndirectVertexStreams(vertexStrideInByte int, vertex []float32, vertexIndices []int, vertexCount *int,
	normalStrideInByte int, normal []float32, normalIndices []int, normalCount *int,
	uv0StrideInByte int, uv0 []float32, uv0Indices []int, uv0Count *int,
	uv1StrideInByte int, uv1 []float32, uv1Indices []int, uv1Count *int) {
	C.NewtonMeshGetIndirectVertexStreams(m.handle,
		C.int(vertexStrideInByte), (*C.dFloat)(&vertex[0]), (*C.int)(unsafe.Pointer(&vertexIndices[0])),
		(*C.int)(unsafe.Pointer(vertexCount)),
		C.int(normalStrideInByte), (*C.dFloat)(&normal[0]), (*C.int)(unsafe.Pointer(&normalIndices[0])),
		(*C.int)(unsafe.Pointer(normalCount)),
		C.int(uv0StrideInByte), (*C.dFloat)(&uv0[0]), (*C.int)(unsafe.Pointer(&uv0Indices[0])),
		(*C.int)(unsafe.Pointer(uv0Count)),
		C.int(uv1StrideInByte), (*C.dFloat)(&uv1[0]), (*C.int)(unsafe.Pointer(&uv1Indices[0])),
		(*C.int)(unsafe.Pointer(uv1Count)))

}

type MeshHandle struct {
	handle unsafe.Pointer
}

//I might be able to abstract this a bit more, but I can't wrap my head around
// the difference between a mesh material, and a mesh material material

func (m *Mesh) BeginHandle() *MeshHandle {
	return &MeshHandle{C.NewtonMeshBeginHandle(m.handle)}
}

func (m *Mesh) EndHandle(handle *MeshHandle) {
	C.NewtonMeshEndHandle(m.handle, handle.handle)
}
func (m *Mesh) FirstMaterial(handle *MeshHandle) int {
	return int(C.NewtonMeshFirstMaterial(m.handle, handle.handle))
}

func (m *Mesh) NextMaterial(handle *MeshHandle, materialID int) int {
	return int(C.NewtonMeshNextMaterial(m.handle, handle.handle, C.int(materialID)))
}

func (m *Mesh) MaterialGetMaterial(handle *MeshHandle, materialId int) int {
	return int(C.NewtonMeshMaterialGetMaterial(m.handle, handle.handle, C.int(materialId)))
}

func (m *Mesh) MaterialGetIndexCount(handle *MeshHandle, materialId int) int {
	return int(C.NewtonMeshMaterialGetIndexCount(m.handle, handle.handle, C.int(materialId)))
}

func (m *Mesh) MaterialGetIndexStream(handle *MeshHandle, materialId int, index []int) {
	C.NewtonMeshMaterialGetIndexStream(m.handle, handle.handle, C.int(materialId),
		(*C.int)(unsafe.Pointer(&index[0])))
}

func (m *Mesh) MaterialGetIndexStreamShort(handle *MeshHandle, materialId int, index []int16) {
	C.NewtonMeshMaterialGetIndexStreamShort(m.handle, handle.handle, C.int(materialId),
		(*C.short)(unsafe.Pointer(&index[0])))
}

func (m *Mesh) CreateFirstSingleSegment() *Mesh {
	return &Mesh{C.NewtonMeshCreateFirstSingleSegment(m.handle)}
}

func (m *Mesh) CreateNextSingleSegment(segment *Mesh) *Mesh {
	return &Mesh{C.NewtonMeshCreateNextSingleSegment(m.handle, segment.handle)}
}

func (m *Mesh) CreateFirstLayer() *Mesh {
	return &Mesh{C.NewtonMeshCreateFirstLayer(m.handle)}
}

func (m *Mesh) CreateNextLayer(segment *Mesh) *Mesh {
	return &Mesh{C.NewtonMeshCreateNextLayer(m.handle, segment.handle)}
}

func (m *Mesh) TotalFaceCount() int {
	return int(C.NewtonMeshGetTotalFaceCount(m.handle))
}

func (m *Mesh) TotalIndexCount() int {
	return int(C.NewtonMeshGetTotalIndexCount(m.handle))
}

func (m *Mesh) Faces(faceIndexCount, faceMaterial, faceIndices []int) {
	//untested
	C.NewtonMeshGetFaces(m.handle, (*C.int)(unsafe.Pointer(&faceIndexCount[0])),
		(*C.int)(unsafe.Pointer(&faceMaterial[0])),
		(*unsafe.Pointer)(unsafe.Pointer(&faceIndices[0])))
}

func (m *Mesh) PointCount() int {
	return int(C.NewtonMeshGetPointCount(m.handle))
}

func (m *Mesh) PointStrideInByte() int {
	return int(C.NewtonMeshGetPointStrideInByte(m.handle))
}

func (m *Mesh) PointArray(size int) []float64 {
	cArray := C.NewtonMeshGetPointArray(m.handle)
	return goFloat64s(cArray, size)
}

func (m *Mesh) NormalArray(size int) []float64 {
	cArray := C.NewtonMeshGetNormalArray(m.handle)
	return goFloat64s(cArray, size)
}

func (m *Mesh) UV0Array(size int) []float64 {
	cArray := C.NewtonMeshGetUV0Array(m.handle)
	return goFloat64s(cArray, size)
}

func (m *Mesh) UV1Array(size int) []float64 {
	cArray := C.NewtonMeshGetUV1Array(m.handle)
	return goFloat64s(cArray, size)
}

func (m *Mesh) VertexCount() int {
	return int(C.NewtonMeshGetVertexCount(m.handle))
}

func (m *Mesh) VertexStrideInByte() int {
	return int(C.NewtonMeshGetVertexStrideInByte(m.handle))
}

func (m *Mesh) VertexArray(size int) []float64 {
	cArray := C.NewtonMeshGetVertexArray(m.handle)
	return goFloat64s(cArray, size)
}

type MeshVertex struct {
	handle unsafe.Pointer
}

func (m *Mesh) FirstVertex() *MeshVertex {
	return &MeshVertex{C.NewtonMeshGetFirstVertex(m.handle)}
}

func (m *Mesh) NextVertex(vertex *MeshVertex) *MeshVertex {
	return &MeshVertex{C.NewtonMeshGetNextVertex(m.handle, vertex.handle)}
}

func (m *Mesh) VertexIndex(vertex *MeshVertex) int {
	return int(C.NewtonMeshGetVertexIndex(m.handle, vertex.handle))
}

type MeshPoint struct {
	handle unsafe.Pointer
}

func (m *Mesh) FirstPoint() *MeshPoint {
	return &MeshPoint{C.NewtonMeshGetFirstPoint(m.handle)}
}

func (m *Mesh) NextPoint(Point *MeshPoint) *MeshPoint {
	return &MeshPoint{C.NewtonMeshGetNextPoint(m.handle, Point.handle)}
}

func (m *Mesh) PointIndex(Point *MeshPoint) int {
	return int(C.NewtonMeshGetPointIndex(m.handle, Point.handle))
}

func (m *Mesh) VertexIndexFromPoint(point *MeshPoint) int {
	return int(C.NewtonMeshGetVertexIndexFromPoint(m.handle, point.handle))
}

type MeshEdge struct {
	handle unsafe.Pointer
}

func (m *Mesh) FirstEdge() *MeshEdge {
	return &MeshEdge{C.NewtonMeshGetFirstEdge(m.handle)}
}

func (m *Mesh) NextEdge(Edge *MeshEdge) *MeshEdge {
	return &MeshEdge{C.NewtonMeshGetNextEdge(m.handle, Edge.handle)}
}

func (m *Mesh) EdgeIndices(edge *MeshEdge, v0, v1 []int) {
	C.NewtonMeshGetEdgeIndices(m.handle, edge.handle, (*C.int)(unsafe.Pointer(&v0[0])),
		(*C.int)(unsafe.Pointer(&v1[0])))
}

type MeshFace struct {
	handle unsafe.Pointer
}

func (m *Mesh) FirstFace() *MeshFace {
	return &MeshFace{C.NewtonMeshGetFirstFace(m.handle)}
}

func (m *Mesh) NextFace(Face *MeshFace) *MeshFace {
	return &MeshFace{C.NewtonMeshGetNextFace(m.handle, Face.handle)}
}

func (m *Mesh) IsFaceOpen(face *MeshFace) bool {
	return gbool[int(C.NewtonMeshIsFaceOpen(m.handle, face.handle))]
}

func (m *Mesh) FaceMaterial(face *MeshFace) int {
	return int(C.NewtonMeshGetFaceMaterial(m.handle, face.handle))
}

func (m *Mesh) FaceIndexCount(face *MeshFace) int {
	return int(C.NewtonMeshGetFaceIndexCount(m.handle, face.handle))
}

func (m *Mesh) FaceIndices(Face *MeshFace, indices []int) {
	C.NewtonMeshGetFaceIndices(m.handle, Face.handle, (*C.int)(unsafe.Pointer(&indices[0])))
}

func (m *Mesh) FacePointIndices(Face *MeshFace, indices []int) {
	C.NewtonMeshGetFacePointIndices(m.handle, Face.handle, (*C.int)(unsafe.Pointer(&indices[0])))
}

func (m *Mesh) CalculateFaceNormal(face *MeshFace, normal []float64) {
	C.NewtonMeshCalculateFaceNormal(m.handle, face.handle, (*C.dFloat64)(&normal[0]))
}

func (m *Mesh) SetFaceMaterial(face *MeshFace, matId int) {
	C.NewtonMeshSetFaceMaterial(m.handle, face.handle, C.int(matId))
}
