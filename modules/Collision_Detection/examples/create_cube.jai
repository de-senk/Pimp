#import,file "../module.jai";
#import "Basic";
#import "Math";
#import "Hash_Table";


STUFF_REQUIRED_FOR_TEST_TO_WORK_NOT_IMPORTANT_TO_SAT :: #string JAI
    //Your body has to have pos and ori
    HASpositionANDorientation :: struct {
        position: Vector3;
        orientation: Quaternion;
    };

    //pos_A and ori_A should be always in world for both l2w and w2l, I need to figure out good names to get rid of this comment.
    //local to world,
    l2w :: (pos_A: Vector3,ori_A: Quaternion, local_point_A: Vector3) -> Vector3 {
        return  pos_A + rotation_matrix(Matrix3, ori_A) * local_point_A;
    }

    //world to local
    w2l :: (pos_A: Vector3, ori_A: Quaternion, world_point: Vector3) -> local_point_A: Vector3 {
        return rotation_matrix(Matrix3, conjugate(ori_A)) * (world_point - pos_A );
    }

     //
    // @Hack for testing, these A, B should be your bodies. position Vec3 in world space, orientation: Quaternion, in world sapce.
    //
    A_stack: HASpositionANDorientation;
    B_stack: HASpositionANDorientation;
    A:= *A_stack;
    B:= *B_stack;
JAI;


main :: () {
    #insert STUFF_REQUIRED_FOR_TEST_TO_WORK_NOT_IMPORTANT_TO_SAT;
    #insert SAME__NOT_IMPORTANT_TO_SAT_BUT__MAYBE_USEFUL_AS_REFERENCE;

    result_contacts: [4]Contact_Info;
    num_contacts: int;

    hull0 := create_cube_hull(position=.{0, 0, 0}, scale=.{1, 1, 1});
    hull1 := create_cube_hull(.{0, .5, 0}, .{1, 1, 1});

    //rightt now some of these are key, otherwise SAT will not work, TODO: recheck what is needed and minimize the tests LOCs or something.
    {
        pos0 := Vector3.{0,0,0}; ori0 := Quaternion.{0,0,0,1};
        pos1 := Vector3.{1.1111,0.4111,1.11111};  ori1 := (Quaternion.{0,0,0,1});
        hull0.worldSpace = make_translation_matrix4(pos0)*rotation_matrix (Matrix4, ori0) * make_scale_matrix4(.{1,1,1});
        hull1.worldSpace  = make_translation_matrix4(pos1)*rotation_matrix (Matrix4, ori1) * make_scale_matrix4(.{3.111,3.111,3.111});
        hull0.scale = .{1,1,1}; //dude, im lucky that the above worked,
        hull1.scale  =  .{2.111,2.111,2.111};

        //we precompute the inverses of A and B, this is still expensive, o(n^2) I think I can convert it to O(n) (not the traversal but the operation, by caching it inside the body
        //idea: check if for this frame is already calculated, if it is, dont execute those. The bottleneck is still  QueryEdgeDirection and its calls
        success:, hull0.inv_worldSpace = inverse(hull0.worldSpace);
        success=, hull1.inv_worldSpace = inverse(hull1.worldSpace);
    }


    //
    // Query wich faces/edges are actually colliding
    //
    query: SAT_Query;
    collision := SAT_dirk_gregorius(hull0, hull1, *query);

    //
    // Given query results, do some heuristic to determine which feature (edge face of A face of B) should we compute contacts from.
    //
    if collision {
        log("query:\t %", query);

		faceDistance0 :=  query.faceQuery_AB.distance ;
		faceDistance1 :=  query.faceQuery_BA.distance ;

        // query.faceQuery_AB.A = A;
        // query.faceQuery_AB.B = B;
        // query.faceQuery_BA.A = A;
        // query.faceQuery_BA.B = B;
		//grep this from https://www.gamedev.net/forums/topic/667499-3d-sat-problem/?page=2
		kLinearSlop :=  0.005 ;
		kRelEdgeTolerance :=  0.90 ;
		kRelFaceTolerance :=  0.98 ;
		kAbsTolerance :=  0.5 * kLinearSlop ;

		if query.edgeQuery.distance > kRelEdgeTolerance * max(faceDistance0, faceDistance1) + kAbsTolerance
		{

			contact : EdgeContact = CreateEdgeContact(query.edgeQuery,hull0,hull1);
			{
			     log("contact %", contact);
                world_normal_A := -contact.separationAxis;
                penetration := contact.query.distance * length_squared(world_normal_A);
                assert(is_normalized(world_normal_A), "normal: % grep:query.edgeQuery.distance > kRelEdgeTolerance * max(faceDistance0, faceDistance1) + kAbsTolerance",world_normal_A);
                if penetration < 0 {
                    world_contact_point := contact.midpoint;
                    world_contact_pointA := world_contact_point + (penetration/2.0)*world_normal_A;
                    world_contact_pointB := world_contact_point - (penetration/2.0)*world_normal_A;
                    result_contacts[0].world_midpoint = world_contact_point;
                    result_contacts[0].local_point_A = w2l(A.position, A.orientation, world_contact_point);
                    result_contacts[0].local_point_B = w2l(B.position, B.orientation, world_contact_point);
                    //NOTE: Seems like I need to cross the contact points to get good stability @Investigate why.
                    // result_contacts[0].local_point_A, result_contacts[0].local_point_B = result_contacts[0].local_point_B, result_contacts[0].local_point_A;

                    result_contacts[0].world_normal_A = world_normal_A;
                    result_contacts[0].penetration = penetration;
                    num_contacts = 1;

                     //@Investigate isthere a better way to do an UID?
                    result_contacts[0].feature.value = query.edgeQuery.edgeA.(u64); //store pointer as uid.
                    result_contacts[0].feature.kind = .EDGE;

                }
			}

		}
		else if faceDistance1 > (kRelFaceTolerance* faceDistance0 + kAbsTolerance)
		{
			    contact :FaceContact= CreateFaceContact(query.faceQuery_BA, hull1,hull0);
			    log("f1contact %", contact);
                {
                    world_normal_A := transform_normal(contact.query.face.facePlane.normal,hull1.inv_worldSpace);
                    penetration := contact.query.distance * length_squared(world_normal_A);
                    assert(is_normalized(world_normal_A), "normal:% if faceDistance1 > (kRelFaceTolerance* faceDistance0 + kAbsTolerance)", world_normal_A);
                    assert(contact.points.count <= 4, "we got more than 4 contact points! contact.points.count == %",contact.points.count);

                    // log("[IMPORTANT] dont factorize  the below code into a procedure until you figure out the right way.");
                    //:tagThisIsTheSameAsitstagTwin_dont_abstract_into_a_procedure
                    if penetration < 0
                    {
                        for point_in_B: contact.points
                        {

                            // add_to_array_of_vector_lines_an_arrow(origin=point_in_B, arrow=world_normal_A, weight=0.5, color=.{0.1,1,0.1,1});
                            midpoint := point_in_B + penetration/2.0 * world_normal_A;
                            world_contact_pointA := point_in_B + penetration * world_normal_A;
                            world_contact_pointB := point_in_B;
                            result_contacts[it_index].world_midpoint = midpoint;
                            // result_contacts[it_index].local_point_A = w2l(A, midpoint);
                            // result_contacts[it_index].local_point_B = w2l(B, midpoint);
                            result_contacts[it_index].local_point_A = w2l(A.position, A.orientation, world_contact_pointA);
                            result_contacts[it_index].local_point_B = w2l(B.position, B.orientation, world_contact_pointB);
                            //crossed points of appllications
                            // result_contacts[0].local_point_A, result_contacts[0].local_point_B = result_contacts[0].local_point_B, result_contacts[0].local_point_A;
                            result_contacts[it_index].world_normal_A = world_normal_A;
                            result_contacts[it_index].penetration = penetration;

                            num_contacts += 1;
                             //@Investigate isthere a better way to do an UID?
                            result_contacts[it_index].feature.value = contact.query.face.(u64);
                            result_contacts[it_index].feature.kind = .FACE;
                            // result_contacts[it_index].feature.value = 72;
                        }

                    }
                }
		}
		else
		{

			contact := CreateFaceContact(query.faceQuery_AB, hull0, hull1);
                {
                    world_normal_A := -transform_normal(contact.query.face.facePlane.normal,hull0.inv_worldSpace);
                    penetration := contact.query.distance * length_squared(world_normal_A);
                    assert(length_squared(world_normal_A) > 0.8, " * length_squared(world_normal_A);");
                    assert(contact.points.count <= 4, "we got more than 4 contact points! AB contact.points.count == %",contact.points.count);
                    //:tagThisIsTheSameAsitstagTwin_dont_abstract_into_a_procedure
                    if penetration < 0
                    {
                        for point_in_A: contact.points
                        {
                            midpoint := point_in_A - penetration/2.0 * world_normal_A;
                            world_contact_pointA := point_in_A;
                            world_contact_pointB := world_contact_pointA - penetration*world_normal_A;
                            result_contacts[it_index].world_midpoint = midpoint;
                            // result_contacts[it_index].local_point_A = w2l(A, midpoint);
                            // result_contacts[it_index].local_point_B = w2l(B, midpoint);
                            result_contacts[it_index].local_point_A = w2l(A.position, A.orientation, world_contact_pointA);
                            result_contacts[it_index].local_point_B = w2l(B.position, B.orientation, world_contact_pointB);
                            //crossed points of appllications
                            // result_contacts[0].local_point_A, result_contacts[0].local_point_B = result_contacts[0].local_point_B, result_contacts[0].local_point_A;
                            result_contacts[it_index].world_normal_A = world_normal_A;
                            result_contacts[it_index].penetration = penetration;

                            num_contacts += 1;
                             //@Investigate isthere a better way to do an UID?
                            result_contacts[it_index].feature.value = contact.query.face.(u64);
                            result_contacts[it_index].feature.kind = .FACE;
                            // result_contacts[it_index].feature.value = 72;
                        }
                    }
                }


		} //end edge or faceBA or faceAB test
    } //end if collision
    else
    {
        log("no collision");
    }

    log("num_contacts %\n", num_contacts);
    for 0..num_contacts - 1 {
        to_print := result_contacts[it];
        log("%", to_print);
    }
}

#if !#exists(is_normalized)
is_normalized :: (a: $T / .[Vector2, Vector3, Vector4], $tolerance:float = 0.000001) -> bool {
    sum: float;
    for a.component {
        sum += it*it;
    }
    return sum < 1+tolerance && sum > 1-tolerance; //Latest Comment: Already tested, its bad we should use 6 or less-> OLD COMMENT: should I use epsilon7? 7 digits are the limits of flaot precisions, but idk if the first 0. counts
}

create_cube_hull :: (position: Vector3 = .{0,0,0}, scale: Vector3 = .{1,1,1}, rotation: Quaternion = .{}) -> Convex_Hull {
    hull: Convex_Hull;
    hull.mesh = create_cube_half_edge_mesh(scale);

    hull.worldSpace = make_translation_matrix4(position)*rotation_matrix (Matrix4, rotation);
    hull.scale = scale;

    return hull;
}


// DANGER DANGER DANGER
//
//  The code below is AI generated, there is a 99% probability that it does NOT work porperly.
//
// DANGER DANGER DANGER
//
//NOTE: the AI code was terrible, I had to heavily modify it, well this is temporary until I implement QuickHull.
// Hardcoded Half-Edge Mesh for a unit cube centered at origin
// Vertices range from -0.5 to 0.5 on each axis
create_cube_half_edge_mesh :: (scale: Vector3) -> Half_Edge_Mesh {
    mesh: Half_Edge_Mesh;

    // Initialize arrays
    array_resize(*mesh.vertexBuffer, 8);
    array_resize(*mesh.edgeBuffer, 24);  // 12 edges * 2 half-edges each
    array_resize(*mesh.faceBuffer, 6);

    // Cube vertices (8 vertices)
    vertices := mesh.vertexBuffer;
    vertices[0] = .{vertex = .{-0.5, -0.5, -0.5}}; // 0: left-bottom-back
    vertices[1] = .{vertex = .{ 0.5, -0.5, -0.5}}; // 1: right-bottom-back
    vertices[2] = .{vertex = .{ 0.5,  0.5, -0.5}}; // 2: right-top-back
    vertices[3] = .{vertex = .{-0.5,  0.5, -0.5}}; // 3: left-top-back
    vertices[4] = .{vertex = .{-0.5, -0.5,  0.5}}; // 4: left-bottom-front
    vertices[5] = .{vertex = .{ 0.5, -0.5,  0.5}}; // 5: right-bottom-front
    vertices[6] = .{vertex = .{ 0.5,  0.5,  0.5}}; // 6: right-top-front
    vertices[7] = .{vertex = .{-0.5,  0.5,  0.5}}; // 7: left-top-front

    // Set up vertex linked list
    for i: 0..7 {
        vertices[i].prev = *vertices[(i + 7) % 8];
        vertices[i].next = *vertices[(i + 1) % 8];
        vertices[i].referenceCount = 3; // Each vertex used by 3 faces
    }
    mesh.vertices = *vertices[0];
    mesh.vertexCount = 8;

    // Half-edges (24 half-edges for 12 edges)
    edges := mesh.edgeBuffer;

    // Face 0: Back face (z = -0.5) - vertices 0,1,2,3 (CCW when viewed from outside)
    edges[0]  = .{tail = *vertices[0]}; // 0->1
    edges[1]  = .{tail = *vertices[1]}; // 1->2
    edges[2]  = .{tail = *vertices[2]}; // 2->3
    edges[3]  = .{tail = *vertices[3]}; // 3->0

    // Face 1: Front face (z = 0.5) - vertices 4,7,6,5 (CCW when viewed from outside)
    edges[4]  = .{tail = *vertices[4]}; // 4->7
    edges[5]  = .{tail = *vertices[7]}; // 7->6
    edges[6]  = .{tail = *vertices[6]}; // 6->5
    edges[7]  = .{tail = *vertices[5]}; // 5->4

    // Face 2: Bottom face (y = -0.5) - vertices 0,4,5,1 (CCW when viewed from outside)
    edges[8]  = .{tail = *vertices[0]}; // 0->4
    edges[9]  = .{tail = *vertices[4]}; // 4->5
    edges[10] = .{tail = *vertices[5]}; // 5->1
    edges[11] = .{tail = *vertices[1]}; // 1->0

    // Face 3: Top face (y = 0.5) - vertices 2,6,7,3 (CCW when viewed from outside)
    edges[12] = .{tail = *vertices[2]}; // 2->6
    edges[13] = .{tail = *vertices[6]}; // 6->7
    edges[14] = .{tail = *vertices[7]}; // 7->3
    edges[15] = .{tail = *vertices[3]}; // 3->2

    // Face 4: Left face (x = -0.5) - vertices 0,3,7,4 (CCW when viewed from outside)
    edges[16] = .{tail = *vertices[0]}; // 0->3
    edges[17] = .{tail = *vertices[3]}; // 3->7
    edges[18] = .{tail = *vertices[7]}; // 7->4
    edges[19] = .{tail = *vertices[4]}; // 4->0

    // Face 5: Right face (x = 0.5) - vertices 1,5,6,2 (CCW when viewed from outside)
    edges[20] = .{tail = *vertices[1]}; // 1->5
    edges[21] = .{tail = *vertices[5]}; // 5->6
    edges[22] = .{tail = *vertices[6]}; // 6->2
    edges[23] = .{tail = *vertices[2]}; // 2->1

    // Set up edge connectivity (prev/next within each face)
    face_edge_groups := [4]int.[
        .[0, 1, 2, 3],   // Back face
        .[4, 5, 6, 7],   // Front face
        .[8, 9, 10, 11], // Bottom face
        .[12, 13, 14, 15], // Top face
        .[16, 17, 18, 19], // Left face
        .[20, 21, 22, 23]  // Right face
    ];

    for group: face_edge_groups {
        for i: 0..3 {
            edges[group[i]].next = *edges[group[(i + 1) % 4]];
            edges[group[i]].prev = *edges[group[(i + 3) % 4]];
        }
    }

    // Set up twin relationships
    twin_pairs := [2]int.[
        .[0, 11],   // 0->1 twins with 1->0
        .[1, 23],   // 1->2 twins with 2->1
        .[2, 15],   // 2->3 twins with 3->2
        .[3, 16],   // 3->0 twins with 0->3
        // .[4, 19],   // 4->7 twins with 7->4
        .[4, 18],   // 4->7 twins with 7->4
        // .[5, 14],   // 7->6 twins with 6->7
        .[5, 13],   // 7->6 twins with 6->7
        // .[6, 22],   // 6->5 twins with 5->6
        .[6, 21],   // 6->5 twins with 5->6
        .[7, 9],    // 5->4 twins with 4->5
        .[8, 19],   // 0->4 (same as 4->0 reverse)
        .[10, 20],  // 5->1 twins with 1->5
        .[12, 22],  // 2->6 (same as 6->2 reverse)
        // .[13, 18]   // 6->7 (same as 7->6 reverse)
        .[14, 17]   // 6->7 (same as 7->6 reverse)
    ];

    for pair: twin_pairs {
        edges[pair[0]].twin = *edges[pair[1]];
        edges[pair[1]].twin = *edges[pair[0]];
    }


    // mesh.edges = *edges[0]; //its a table
    for *edges {
        // log("%| e==ett %==%", it_index ,it, it.twin.twin);
        assert(it == it.twin.twin, "not twinning");
        find_or_add(*mesh.edges, it);
    }

    mesh.edgeCount = 24;

    // Set up faces
    faces := mesh.faceBuffer;
    face_normals := Vector3.[
        .{0, 0, -1},  // Back face
        .{0, 0, 1},   // Front face
        .{0, -1, 0},  // Bottom face
        .{0, 1, 0},   // Top face
        .{-1, 0, 0},  // Left face
        .{1, 0, 0}    // Right face
    ];

    for i: 0..5 {
        faces[i].edge = *edges[face_edge_groups[i][0]];
        faces[i].facePlane.normal = face_normals[i];
        faces[i].facePlane.distance = 0.5; // Distance from origin to face
        faces[i].vertexCount = 4;
        faces[i].prev = *faces[(i + 5) % 6];
        faces[i].next = *faces[(i + 1) % 6];

        // Set face reference for all edges of this face
        for j: 0..3 {
            edges[face_edge_groups[i][j]].face = *faces[i];
        }
    }

    mesh.faces = *faces[0];
    mesh.faceCount = 6;

    // Set vertex edge references (one outgoing edge per vertex)
    vertices[0].edge = *edges[0];   // 0->1
    vertices[1].edge = *edges[1];   // 1->2
    vertices[2].edge = *edges[2];   // 2->3
    vertices[3].edge = *edges[3];   // 3->0
    vertices[4].edge = *edges[4];   // 4->7
    vertices[5].edge = *edges[7];   // 5->4
    vertices[6].edge = *edges[6];   // 6->5
    vertices[7].edge = *edges[5];   // 7->6

    // Set leaving_edge (same as edge for this simple case)
    for i: 0..7 {
        vertices[i].leaving_edge = vertices[i].edge;
    }

    mesh.allocatedVertices = 8;
    mesh.allocatedEdges = 24;
    mesh.allocatedFaces = 6;

    return mesh;
}