Convex_Hull :: struct {
    mesh: Half_Edge_Mesh;
    local_geometric_center: Vector3; //I wish I could center the hulls at the origin, but that might be a bad idea at the end.
    worldSpace: Matrix4;
    inv_worldSpace_for_bivectors: Matrix4; //DONT USE IT TO GET world points! this only works for bivectors
    // scale: Vector3;
};

//this datastructure is very powerful, It let you do useful topological stuff.
Half_Edge_Mesh :: struct {
    //old, to be deleted after I figure out how to replicate the compare_he on the linked list
    // #import "Hash_Table"; //:tag_Extra_Import_Here_Maybe_Pull_Out
    // compare_he :: (a: *Hull_Half_Edge, b: *Hull_Half_Edge) -> bool {
    //     c1 := a.next == b.next;
    //     c2 := a.prev == b.prev;
    //     c3 := a.twin == b.twin;
    //     c4 := a.tail == b.tail;
    //     c5 := a.face == b.face;
    //     return c1 && c2 && c3 && c4 && c5;
    // }
    // edges_table: Table(*Hull_Half_Edge, void, given_compare_function = compare_he);
    edges: *Hull_Half_Edge;
    vertices: *Hull_Vertex;
    faces: *Hull_Face;
    //for usage, might be less than allocated
    vertexCount: int;
    edgeCount: int;
    faceCount: int;

    CONVEXITY_EPSILON: float : 0.00005;
    //for allocation
    vertexBuffer: [] Hull_Vertex;
    edgeBuffer: [] Hull_Half_Edge;
    faceBuffer: [] Hull_Face;

    //I use singular instead of plural for grep reasons
    num_allocated_vertex: int;
    num_allocated_edge: int;
    num_allocated_face: int;
    vertex_free_list: *Hull_Vertex;
    edge_free_list: *Hull_Half_Edge;
    face_free_list: *Hull_Face;
    //
};


Hull_Face :: struct
{
    //topology fields
	edge: *Hull_Half_Edge; //any edge
	face_plane_normal: Vector3;
	face_plane_distance: float;
	vertexCount: int;
    //quality of life fields
	linked_list_prev: *Hull_Face;
	linked_list_next: *Hull_Face;
	//fields, used for quickhull,
	conflict_list : [..]*Hull_Vertex;
	visited: bool;
};

Hull_Half_Edge :: struct
{
    //topology fields
	tail: *Hull_Vertex;
	prev: *Hull_Half_Edge;
	next: *Hull_Half_Edge;
	twin: *Hull_Half_Edge;
	face: *Hull_Face; //always on the left of hedge
    //quality of life fields
	linked_list_prev: *Hull_Half_Edge;
	linked_list_next: *Hull_Half_Edge;
};


Hull_Vertex :: struct
{
    //topology fields
	vertex: Vector3;
	referenceCount: int;
    //quality of life fields
	linked_list_prev: *Hull_Vertex;
	linked_list_next: *Hull_Vertex;
    conflict_face: *Hull_Face;

    // Optional fields: to iterate the 1 ring of the vertex (dirk gregorius recommendation) TODO: Figure out how to use it, maybe use it in find_fastest_linear_speed procedure, but that is outside this module layer.
	edge: *Hull_Half_Edge;
    leaving_edge: *Hull_Half_Edge;

};


//for Separating Axis Tests

FaceQuery :: struct
{
	face: *Hull_Face;
	distance: float = FLOAT32_MAX;
	//for pseudo gjk :tagHerePseudoGJK
	world_furthest_point: Vector3;
	world_normal: Vector3;
};

//Toughts0: I think there is not a reasonable reason of why exactly we only take 1 feature for edges, for faces we take 2 features. I will figure it out with experimenting once I finish the core of he physics engine.
//Toughts1: I think the reason might be to have Smooth Normals transition? imagine a cube vx cube on edge, slowly rotating on top of the other cuve, rotating around the edge, the normal is contiguous
EdgeQuery :: struct
{
	edgeA: *Hull_Half_Edge;
	edgeB: *Hull_Half_Edge;
	distance: float = FLOAT32_MAX;

	//maybe capture more than 1 edge?
	num_inersecting_points: int;
	world_furthest_point: Vector3;
    // world_furthest_point_apart: Vector3;
	world_normal: Vector3;
};

SAT_Query :: struct
{
	faceQuery_AB: FaceQuery;
    faceQuery_BA: FaceQuery;
	edgeQuery: EdgeQuery;
};


//for creating contacts after SAT
FaceContact:: struct
{
    query: FaceQuery;
    points: [..]Vector3;
};

EdgeContact :: struct
{
    query: EdgeQuery;
    midpoint: Vector3;
    separationAxis: Vector3;
};


// Shape Primitives

//
// Plane definition: extremely important, using a different one will break everything
//
//The procedures inside this file uses the plane definition as Normal and point belonging to a plane projected on to the normal as distance
//that definition behaves differently than the one that is normal and distance from origin, they differ at the sign of distance if I remember my suffering correctly.
Plane :: struct {
    normal: Vector3;
    distance: float;
}


