
#program_export
simulate :: () {
    CheckInit();
    old_stack_trace := basic.pack_stack_trace(); // 1 of 2 : maybe needs bug fix? already mailed to jon.
    Remap_Context();
    context.stack_trace = *old_stack_trace[1]; //2 of 2: maybe needs bug fix? already mailed to jon.
    using context.solver;

    //
    //cleanup immediate mode cache
    //
    for *bodies
    {
       should_delete := it.latest_frame_of_existence != number_of_steps_executed_so_far;
       if should_delete {
        table_remove(*hashed_bodies, it.hash);
        remove it;
       }
    }

    drawed:bool;
    for *firstbody: bodies
    {
        if firstbody
        { TestCubes(posA=firstbody.position,oriA=firstbody.orientation);
        drawed=true;
        }
        break;
    }
    if !drawed TestCubes();

    for *joint_constraints
    {
       should_delete := it.latest_frame_of_existence != number_of_steps_executed_so_far;
       if should_delete {
        table_remove(*hashed_joint_constraints, it.hash);
        remove it;
       }
    }
    number_of_steps_executed_so_far += 1;

    //
    // ACTUAL ALGORITHM STARTS HERE
    //
    if context.solver.dt < 1.0/1024.0 then return;
    sub_steps := iterations;

    //this is useful for switching solver strategys without changing much code:
    backup_dt := dt;
    dt = dt/sub_steps;
    defer dt = backup_dt;

    for 1..sub_steps
    {
        for *bodies
        {
            //I think I dont even needa mask because stub_world_body.velocity should always be zero because prev_position is always he same as current position for that body.
            mask := (it.inv_mass != 0.0).(float);
            dt_m := dt*mask;

            //
            // Integration      Semi-Implicit Euler  (3 lines of code)
            //
            //linear
            it.velocity += gravity*dt_m;
            it.prev_position = it.position;
            it.position += it.velocity*dt_m;

            //angular usable fields are:  .orientation|.prev_orientation|.angular_velocity|.prev_angular_velocity
            it.prev_orientation = it.orientation;
            dq :=  Quaternion.{it.angular_velocity.x, it.angular_velocity.y, it.angular_velocity.z, 0.0};
            dq = dq*it.orientation; //this is confusing for me
            it.orientation += 0.5 * dq * dt_m; //seriously confusing, the 0.5, why!
            normalize(*it.orientation); //because the above is an approximation since dq is not infty small.


            //Fake floor, temporary.
            if it.position.y<0.0 {
                it.position = it.prev_position;
            }
            basic.assert(is_valid(it.position), "your body exploded");
            basic.assert(is_valid(it.velocity), "your body tried to explode");
        }

        //NOTE IMPORTANT: :tagIm99percentSureThatThisIsWrongBUTitworks TODO: Study iterative convergence.
        for *joint: joint_constraints {
            //solve(it, dt); FLATTENED OUT:
             //these are callbacks of procedures stored inside each joint, I think this strategy is better than a switch statement.
             joint.pre_solve(joint, dt);
             recalculate_world_attach_pos_and_ori(joint);
             joint.solve_position(joint, dt);
             // recalculate_world_attach_pos_and_ori(joint); //maybe I need one of these here?
             joint.solve_orientation(joint, dt);
        }

        //
        // VELOCITIES           update linear and angular velocities
        //
        if dt == 0.0 then continue; //we early return because yes

        for *bodies
        {
            mask := (it.inv_mass != 0.0).(float);
            inv_dt_m := (1.0/dt)*mask;
            //linear
            it.velocity = (it.position - it.prev_position) * inv_dt_m;
            //angualar
            dq := it.orientation * conjugate(it.prev_orientation);
            it.angular_velocity = 2.0* dq.xyz * inv_dt_m;
            if dq.w < 0 then it.angular_velocity *= -1.0;


            push_vector(it.position, it.angular_velocity, Vector4.{.5,.5,.1,1.0});
            if length_squared(it.angular_velocity) > 200 {
                basic.log("MAX ANG VEL REACHED; CLAMPING");
                // clamp_length(*it.angular_velocity, 5);
             }

        }

        //
        // Dampings          Apply velocity dampings, NOTE: I highly doubt that this is the correct way of doing this. BUT this callback strategy allows me to fix it in the future, I hope.
        //
        for *joint: joint_constraints {
             // recalculate_world_attach_pos_and_ori(joint); //maybe I need one of these here?
            joint.apply_linear_damping(joint, dt);
             // recalculate_world_attach_pos_and_ori(joint); //maybe I need one of these here?
            joint.apply_angular_damping(joint, dt);
        }

        //
        // FLOOR            Faked out floor, not perfect! TODO: Collision constraints!
        //
        for *bodies
        {
            next_pos := it.position + it.velocity*backup_dt;
            if dot(gravity, next_pos) > 0 && dot(gravity, it.velocity) > 0
            {
                grav_dir := normalize(gravity); project_upwards := - dot(it.velocity, grav_dir)*grav_dir;
                it.velocity += project_upwards; //to zero it out
                it.velocity += it.factor_energy_loss_on_collision*project_upwards;  //to bounce
            }

        }
    } //for SUBSTEPS

    //actually apply stuff
    for *bodies
    {
       basic.assert(it.end_frame_callback != null);
       it.end_frame_callback(it);
    }

} @dll_export



cubes_initialized: bool;
hull0: Convex_Hull;
hull1: Convex_Hull;
TestCubes :: (posA := Vector3.{1,1,1}, posB := Vector3.{1,.5,1},oriA:=Quaternion.{0,0,0,1},oriB:=Quaternion.{0,0,0,1}) {
    if !cubes_initialized {
        cubes_initialized = true;
        hull3 := create_cube_hull(position=.{0, 0, 0}, scale=.{1, 1, 1});
        hull1 = create_cube_hull(.{0, .5, 0}, .{1, 1, 1});

        hull0 = quickhull_3D(Vector3.[

             .{-0.5, -0.5, -0.5}, // 0: left-bottom-back
             .{ 0.5, -0.5, -0.5}, // 1: right-bottom-back
             .{ 0.5,  0.5, -0.5}, // 2: right-top-back
             .{-0.5,  0.5, -0.5}, // 3: left-top-back
             .{-0.5, -0.5,  0.5}, // 4: left-bottom-front
             .{ 0.5, -0.5,  0.5}, // 5: right-bottom-front
             .{ 0.5,  0.5,  0.5}, // 6: right-top-front
             .{-0.5,  0.5,  0.5}, // 7: left-top-front]);
             ]);
        hull0.worldSpace = hull3.worldSpace;
        hull0.scale = hull3.scale;
        // hull0 = hull3;
    }

    num_contacts             :,
    penetration              :,
    world_normal_towards_A   :,
    world_midpoints          :,
    world_contact_points_on_A:,
    world_contact_points_on_B:=collide_hull_vs_hull(
     position_A=posA, orientation_A=oriA, scale_A=.{1,1,1}, hull0,
     position_B=posB, orientation_B=oriB, scale_B=.{1,1,1}, hull1
    );
    push_debug_hull(posA, oriA, hull0);
    push_debug_hull(posB, oriB, hull1);
    for 0..num_contacts-1 {
        push_arrow(posA, world_normal_towards_A);
        push_X(world_contact_points_on_A[it],.05);
        push_circle(world_contact_points_on_B[it],.05);
    }
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
    v_count := 8;
    he_count := 24;// 12 edges * 2 half-edges each
    f_count := 6;

    mesh.vertexBuffer.count = v_count;
    mesh.edgeBuffer.count = he_count;
    mesh.faceBuffer.count = f_count;

    alloc_size_u8 := v_count*size_of(Hull_Vertex) +
    he_count*size_of(Hull_Half_Edge)+
    f_count*size_of(Hull_Face);
    alldata := alloc(alloc_size_u8);
    memset(alldata, 0, alloc_size_u8);

	assert(alldata!=null, "allocation failed");

    mesh.vertexBuffer.data = alldata;
    mesh.edgeBuffer.data = cast(*Hull_Half_Edge)(mesh.vertexBuffer.data+mesh.vertexBuffer.count);
    mesh.faceBuffer.data = cast(*Hull_Face)(mesh.edgeBuffer.data+mesh.edgeBuffer.count);


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
    assert(vertices.data==vertices[7].next);
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
        faces[i].face_plane_normal = face_normals[i];
        faces[i].face_plane_distance = 0.5; // Distance from origin to face
        faces[i].vertexCount = 4;
        faces[i].prev = *faces[(i + 5) % 6];
        faces[i].next = *faces[(i + 1) % 6];

        // Set face reference for all edges of this face
        for j: 0..3 {
            edges[face_edge_groups[i][j]].face = *faces[i];
        }
    }

    assert(faces.data==faces[5].next);
    mesh.faces = *faces[0];
    mesh.faceCount = 6;


    for it: 0..5 {
        log("% %",it, mesh.faces[it]);
    }

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

    mesh.num_allocated_vertex = 8;
    mesh.num_allocated_edge = 24;
    mesh.num_allocated_face = 6;


    // assert(edges.data==edges[23].next);

    return mesh;
}