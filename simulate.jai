
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
            dq = dq*it.orientation;
            it.orientation += 0.5 * dq * dt_m;
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



            #run TODO("[ERROR]Damping here!");
            it.angular_velocity *= 0.55;
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


collider_switch_face:bool;
cubes_initialized: bool;
hull0: Convex_Hull;
hull1: Convex_Hull;
test1pos: Vector3 = Vector3.{.5,0.7, 2.2};
// test1ori := #run make_quaternion_from_axis_and_angle(.{1,-2.2,0}, 3.141593/4);
test1ori := Quaternion.{-0.291444, -0.490207, -0.050201, 0.819901};
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
        // hull1 = hull0;
        hull0 = hull3;
        hull0.worldSpace = hull3.worldSpace;
        // hull0.scale = hull3.scale;
        // hull0.scale = .{1, 1, 1};
    }

    {
    //@Hack temporary hack, I could not figure out good procedure parameters, maybe I should modularize this procedure.
    //:tagCheckTemporaryHackMovedHere_Maybe_Cache_by_orientation_diff this block was inserted in that tag before, now we do it once per frame maybe, to figure out.
        hull0.worldSpace = make_translation_matrix4(posA)*rotation_matrix (Matrix4, oriA) * make_scale_matrix4(.{1,1,1});
        hull1.worldSpace  = make_translation_matrix4(test1pos)*rotation_matrix (Matrix4, test1ori) * make_scale_matrix4(.{4,4,4});

        //maybe cache per frame/orientation treshold. The bottleneck is still  QueryEdgeDirection and its calls
        success:, hull0.inv_worldSpace = inverse(hull0.worldSpace);
        success=, hull1.inv_worldSpace = inverse(hull1.worldSpace);

        assert(success, "error while inverting some hull world model, are your hulls created?");
        // hull.scale = scale;
    }

    num_contacts             :,
    penetration              :,
    world_normal_towards_A   :,
    world_midpoints          :,
    world_contact_points_on_A:,
    world_contact_points_on_B:= collide_hull_vs_hull(hull0,hull1);


    // push_debug_hull(posA, oriA, hull0);
    // push_debug_hull(test1pos, test1ori, hull1);
    for 0..num_contacts-1 {
        P_START:= world_contact_points_on_B[it];
        P_END := P_START + world_normal_towards_A;

        push_arrow(test1pos, world_normal_towards_A);
        push_X(world_contact_points_on_A[it],.05);
        push_X(P_START,.05);
        push_X(world_midpoints[it],.05);
        push_circle(P_START,.05);
        push_arrow(P_START, world_normal_towards_A, color=.{1,1,1,1});
        doLabel("contact!", WorldToScreen(P_END));
        arm := P_START-test1pos;
        torque := cross_product(arm, -world_normal_towards_A);
        push_arrow(test1pos, torque*4.0, color=.{1,1,1,1});
        // push_line(test1pos, world_contact_points_on_A[it], .{0,1,0,1});

        dq :Quaternion;
        // set_from_axis_and_angle(*dq, torque, length_squared(torque)*context.solver.dt/0.0005);
        dq.xyz = torque *context.solver.dt/(4.0*num_contacts);
        // test1ori = dq* test1ori;
        // test1ori = normalize(test1ori);
    }
}



create_cube_hull :: (position: Vector3 = .{0,0,0}, scale: Vector3 = .{1,1,1}, rotation: Quaternion = .{}) -> Convex_Hull {
    hull: Convex_Hull;
    hull.mesh = create_cube_half_edge_mesh(scale);

    hull.worldSpace = make_translation_matrix4(position)*rotation_matrix (Matrix4, rotation);

    return hull;
}

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


    //
    // We fill the vertex memory with the cloud data and attach them to eachother to form the freelist
    //
    {
        poison_vertex := Vector3.{7777,7777,7777};
        cursor: *Hull_Vertex;
        for 0..v_count - 1
        {
            vert_ptr := *mesh.vertexBuffer[it];
            vert_ptr.vertex = poison_vertex;
            if cursor
            {
                vert_ptr.linked_list_prev = cursor;
                cursor.linked_list_next = vert_ptr;
            }
            cursor = vert_ptr;
        }
        //because this is a special case its safe to do the following line.
        mesh.vertex_free_list = *mesh.vertexBuffer[0];

    }
    //
    // Fill faces free lists
    //
    {
        cursor: *Hull_Face;
        for 0..f_count - 1
        {
            face_ptr := *mesh.faceBuffer[it];
            if cursor
            {
                face_ptr.linked_list_prev = cursor;
                cursor.linked_list_next = face_ptr;
            }
            cursor = face_ptr;
        }
        //because this is a special case its safe to do the following line.
        mesh.face_free_list = *mesh.faceBuffer[0];
    }
    //
    // Fill half edges free lists
    //
    {
        cursor: *Hull_Half_Edge;
        for 0..he_count - 1
        {
            hedge_ptr := *mesh.edgeBuffer[it];
            if cursor
            {
                hedge_ptr.linked_list_prev = cursor;
                cursor.linked_list_next = hedge_ptr;
            }
            cursor = hedge_ptr;
        }
        //because this is a special case its safe to do the following line.
        mesh.edge_free_list = *mesh.edgeBuffer[0];
    }

    // cube (8 vertices)
    //convention  Vector3.{forward, left, up); right haned, LEFT IS POSITIVE, in opengl left is negative or something like that
    v0 := add_vertex(*mesh, vertex=.{-0.5, -0.5, -0.5});//back-right-bottom
    v1 := add_vertex(*mesh, vertex=.{+0.5, -0.5, -0.5});//front-right-bottom
    v2 := add_vertex(*mesh, vertex=.{+0.5, +0.5, -0.5});//front-left-bottom
    v3 := add_vertex(*mesh, vertex=.{-0.5, +0.5, -0.5});//back-left-bottom
    v4 := add_vertex(*mesh, vertex=.{-0.5, -0.5, +0.5});//back-right-top
    v5 := add_vertex(*mesh, vertex=.{+0.5, -0.5, +0.5});//front-right-top
    v6 := add_vertex(*mesh, vertex=.{+0.5, +0.5, +0.5});//front-left-top
    v7 := add_vertex(*mesh, vertex=.{-0.5, +0.5, +0.5});//back-left-top

    //Back face   (X = -0.5): v0, v3, v7, v4 <- TWINS are repeated vertices v4->v0
    //Front face  (X = +0.5): v1, v5, v6, v2 <- TWINS are repeated vertices
    //Left face   (Y = +0.5): v3, v2, v6, v7 <- TWINS are repeated vertices         and so on, im too tired to fill this comment
    //Right face  (Y = -0.5): v0, v4, v5, v1 <- TWINS are repeated vertices v0->v4
    //Bottom face (Z = -0.5): v0, v1, v2, v3 <- TWINS are repeated vertices
    //Top face    (Z = +0.5): v4, v5, v6, v7 <- TWINS are repeated vertices
    face_back, fb_e0, fb_e1, fb_e2, fb_e3           := create_face_from_vertices(*mesh, v0, v4, v7, v3);
    face_front, ff_e0, ff_e1, ff_e2, ff_e3          := create_face_from_vertices(*mesh, v1, v2, v6, v5);
    face_left, fl_e0, fl_e1, fl_e2, fl_e3           := create_face_from_vertices(*mesh, v3, v7, v6, v2);
    face_right, fr_e0, fr_e1, fr_e2, fr_e3          := create_face_from_vertices(*mesh, v0, v1, v5, v4);
    face_bottom, fbot_e0, fbot_e1, fbot_e2, fbot_e3 := create_face_from_vertices(*mesh, v0, v3, v2, v1);
    face_top, ft_e0, ft_e1, ft_e2, ft_e3            := create_face_from_vertices(*mesh, v4, v5, v6, v7);


    {   link_two_faces_by_edge(fb_e0, fr_e3);  // back: v0->v4, right: v4->v0
        link_two_faces_by_edge(fb_e2, fl_e0);  // back: v7->v3, left: v3->v7
        link_two_faces_by_edge(ff_e1, fl_e2);  // front: v2->v6, left: v6->v2
        link_two_faces_by_edge(ff_e3, fr_e1);  // front: v5->v1, right: v1->v5
        link_two_faces_by_edge(fb_e3, fbot_e0);  // back: v3->v0, bottom: v0->v3
        link_two_faces_by_edge(fb_e1, ft_e3);  // back: v4->v7, top: v7->v4
        link_two_faces_by_edge(ff_e0, fbot_e2);  // front: v1->v2, bottom: v2->v1
        link_two_faces_by_edge(ff_e2, ft_e1);  // front: v6->v5, top: v5->v6
        link_two_faces_by_edge(fl_e3, fbot_e1);  // left: v2->v3, bottom: v3->v2
        link_two_faces_by_edge(fl_e1, ft_e2);  // left: v7->v6, top: v6->v7
        link_two_faces_by_edge(fr_e0, fbot_e3);  // right: v0->v1, bottom: v1->v0
        link_two_faces_by_edge(fr_e2, ft_e0);  // right: v5->v4, top: v4->v5
    }



    return mesh;
}