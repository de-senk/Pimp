
simulate :: () {
    CheckInit();
    using context.solver;
    //
    //cleanup immediate mode cache
    //
    for *bodies
    {
       should_delete := it.latest_frame_of_existence != number_of_steps_executed_so_far;
       is_retained_mode := it.latest_frame_of_existence == 0;
       if should_delete && !is_retained_mode {
        table_remove(*hashed_bodies, it.hash);
        remove it;
       }
    }

    for *joint_constraints
    {
       should_delete := it.latest_frame_of_existence != number_of_steps_executed_so_far;
       is_retained_mode := it.latest_frame_of_existence == 0;
       if should_delete && !is_retained_mode {
        table_remove(*hashed_joint_constraints, it.hash);
        remove it;
       }
    }
    number_of_steps_executed_so_far += 1;


                ///////////////////////
                //this block is for debugging purpuses
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
                ///////////////////////



    //
    // ACTUAL ALGORITHM STARTS HERE
    //
    if context.solver.dt < 1.0/1024.0 then return;


    //XPBD for rigidbodies
    backup_dt := dt;
    dt = backup_dt / sub_steps.(float);

    if global_temp_delete {
    // draw_primitive.start_recording();
    }
    defer if global_temp_delete{
     draw_primitive.end_recording();
     global_temp_delete=false;
     }
    for *bodies compute_world_inertia_and_inv_inertia(it);
    for *collision_constraints it.lambda_cache = 0;
    for *joint_constraints it.lambda_cache = 0;

    for 1..sub_steps {
        for *bodies integrate_positions_xpbd(it, dt);
        for 1..iterations {
            for *joint_constraints SolveJointPositions(it, dt);
            for *collision_constraints SolvePositions(it, dt);
        }
        for *bodies integrate_velocities_xpbd(it, dt);
    }
    for *collision_constraints backup_velocities_inside_manifold_constraints(it);
    CollisionDetection();// this is very slow.
    for *collision_constraints SolveVelocities(it, dt);
    for *joint: joint_constraints ApplyDamping(joint, dt);
    dt = backup_dt;


      DO_FAKE_FLOOR :: false;

        #if DO_FAKE_FLOOR
        for *bodies {
            factor_energy_loss_on_collision := 1.0;
            next_pos := it.position + it.velocity*backup_dt;
            if dot(gravity, next_pos) > 0 && dot(gravity, it.velocity) > 0 {
                grav_dir := normalize(gravity);
                project_upwards := - dot(it.prev_velocity, grav_dir)*grav_dir;
                it.velocity += project_upwards; //to zero it out
                it.velocity += factor_energy_loss_on_collision*project_upwards;  //to bounce
            }
        }
    //actually apply stuff
    for *bodies
    {
       basic.assert(it.end_frame_callback != null);
       it.end_frame_callback(it);
    }

}

draw_energies :: (A: *Body) {
    offset :Vector3= {0,0.0,0};
    K := .5*length_squared(A.velocity)/A.inv_mass;
    K_ang := .5*dot(A.angular_velocity, A.I_world*A.angular_velocity);
    U := -dot(context.solver.gravity,A.position+offset);
    draw_primitive.doLabel(basic.tprint("KRotati:   %               .",K_ang), draw_primitive.WorldToScreen(A.position+.{0,.1,0}));
    draw_primitive.doLabel(basic.tprint("KLinear:   %               .",K), draw_primitive.WorldToScreen(A.position+.{0,.2,0}));
    draw_primitive.doLabel(basic.tprint("Upotent:   %               .",U), draw_primitive.WorldToScreen(A.position+.{0,.3,0}));
    draw_primitive.doLabel(basic.tprint("E TOTAL:   %               .",K_ang+ K + U), draw_primitive.WorldToScreen(A.position+.{0,.4,0}));
}

integrate_positions_xpbd :: (body: *Body, dt: float) {
            I_world, inv_I_world := body.I_world, body.inv_I_world;

            mask := (body.inv_mass != 0.0).(float); //maybe unnecesary
            h := dt*mask;
            defer {
                basic.assert(is_valid(body.position), "your body exploded");
                basic.assert(is_valid(body.velocity), "your body tried to explode");
            }

            //
            //page5 of paper
            //
            //LINEAR, 3 steps
            // step 1
            body.prev_position = body.position;
            // step 2
            external_forces := Vector3.{0,0,0}; //Maybe other kinds of pullings
            // acceleration := -dot(body.position,Vector3.{0,1,0})*Vector3.{0,1,0};//external_forces*body.inv_mass + context.solver.gravity;
            acceleration := external_forces*body.inv_mass + context.solver.gravity;
            body.velocity += h * acceleration;
            // step 3
            body.position += h * body.velocity;

            //ANGULAR, 3 steps

            #if ENABLE_ANGULAR_RESOLUTION {
            // step 1
            body.prev_orientation = body.orientation;
            // step 2
            external_torques := Vector3.{0,0,0}; //Maybe hurricanes
            torques := (external_torques - cross_product(body.angular_velocity, I_world*body.angular_velocity));
            angular_acceleration := inv_I_world*torques;
		    body.angular_velocity += h * angular_acceleration;
            // step 3
            dq : Quaternion;
            dq.xyz = body.angular_velocity;
            dq.w = 0; //dual quaternion
            dq = dq*body.orientation;
            body.orientation += h * 0.5 * dq;
            normalize(*body.orientation); //because the above is an approximation since dq is not infty small.
        }

}
backup_velocities_inside_manifold_constraints :: (using colli: *Collision_Constraint) {

            prev_velocity_A = A.velocity;
            prev_velocity_B = B.velocity;
            prev_angular_velocity_A = A.angular_velocity;
            prev_angular_velocity_B = B.angular_velocity;
            // lambda_cache = 0;
            // impulse_cache = 0;
}

#if assume_I_use_symmetric_objects {
compute_world_inertia_and_inv_inertia :: (body: *Body) {
    //  I_world = R * I_local * R^T
    // for diagonal tensors : I_world_ii = sum_j(R_ij^2 * I_local_jj)

    R3D := rotation_matrix(Matrix3, body.orientation);

    // Compute world-space diagonal inertia
    I_world: Vector3;
    I_world.x = body.local_moment_of_inertia.x * (R3D._11 * R3D._11) +
                body.local_moment_of_inertia.y * (R3D._12 * R3D._12) +
                body.local_moment_of_inertia.z * (R3D._13 * R3D._13);

    I_world.y = body.local_moment_of_inertia.x * (R3D._21 * R3D._21) +
                body.local_moment_of_inertia.y * (R3D._22 * R3D._22) +
                body.local_moment_of_inertia.z * (R3D._23 * R3D._23);

    I_world.z = body.local_moment_of_inertia.x * (R3D._31 * R3D._31) +
                body.local_moment_of_inertia.y * (R3D._32 * R3D._32) +
                body.local_moment_of_inertia.z * (R3D._33 * R3D._33);

    // Inverse is simple for diagonal tensors
    inv_I_world := Vector3.{1.0/I_world.x, 1.0/I_world.y, 1.0/I_world.z};

    //TODO: branchless
    if body.inv_mass == 0 {
        inv_I_world = .{0,0,0};
    }

    body.I_world = I_world;
    body.inv_I_world = inv_I_world;

}
} else {
 //TODO: Matrix3
}

// void RigidBody::ProjectVelocities(Real invH)
// {
// 	// Recompute velocities based on dx and dq -- rough approximation
// 	Vec3 const dx = position - prevPosition;
// 	prevLinearVelocity = linearVelocity;
// 	linearVelocity = dx * invH;

// 	Quat const dq = orientation * glm::inverse(prevOrientation);
// 	prevAngularVelocity = angularVelocity;
// 	angularVelocity = Vec3(dq.x, dq.y, dq.z);
// 	if (dq.w < 0.0_r)
// 	{
// 		angularVelocity *= -2.0_r * invH;
// 	}
// 	else
// 	{
// 		angularVelocity *= 2.0_r * invH;
// 	}
// }
ENABLE_ANGULAR_RESOLUTION::true;
integrate_velocities_xpbd :: (body: *Body, dt: float) {

    mask := (body.inv_mass != 0.0).(float);
    inv_h := (1.0/dt)*mask;
    //linear

    body.velocity = inv_h * (body.position - body.prev_position) ;
    //angualar
    #if ENABLE_ANGULAR_RESOLUTION {
    dq := body.orientation * conjugate(body.prev_orientation);
    body.angular_velocity = inv_h * 2.0* dq.xyz ;
    if dq.w < 0 then body.angular_velocity *= -1.0;
    }
}

collider_switch_face:bool;
cubes_initialized: bool;
hull0: Convex_Hull;
hull1: Convex_Hull;
test1pos: Vector3 = Vector3.{.5,0.7, 2.2};
// test1ori := #run make_quaternion_from_axis_and_angle(.{1,-2.2,0}, 3.141593/4);
test1ori := Quaternion.{-0.291444, -0.490207, -0.050201, 0.819901};
TestCubes :: (posA := Vector3.{1,1,1}, posB := Vector3.{1,.5,1},oriA:=Quaternion.{0,0,0,1},oriB:=Quaternion.{0,0,0,1}) {
#if 0{

    if !cubes_initialized {
        cubes_initialized = true;
        hull3 := create_cube_hull(position=.{0, 0, 0}, scale=.{1, 1, 1});
        hull1 = create_cube_hull(.{0, 0, 0}, .{1, 1, 1});

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
        // hull0 = hull3;
        hull0.worldSpace = hull3.worldSpace;
        // hull0.scale = hull3.scale;
        // hull0.scale = .{1, 1, 1};
    }

    {
    //@Hack temporary hack, I could not figure out good procedure parameters, maybe I should modularize this procedure.
    //:tagCheckTemporaryHackMovedHere_Maybe_Cache_by_orientation_diff this block was inserted in that tag before, now we do it once per frame maybe, to figure out.
        hull0.worldSpace = make_translation_matrix4(posA)*rotation_matrix (Matrix4, oriA) * make_scale_matrix4(.{1,1,1});
        hull1.worldSpace  = make_translation_matrix4(test1pos)*rotation_matrix (Matrix4, test1ori) * make_scale_matrix4(.{1,1,1});

        //maybe cache per frame/orientation treshold. The bottleneck is still  QueryEdgeDirection and its calls
        hull0.inv_worldSpace_for_bivectors = calculate_inverse_or_use_transpose(hull0);
        hull1.inv_worldSpace_for_bivectors = calculate_inverse_or_use_transpose(hull1);
    }


#if 0
{
    SPH_RADIUS:= 0.2;
    SPH_HEIGH:= 2.2;
        draw_primitive.circle(posA,SPH_RADIUS);
        draw_primitive.circle(posA,SPH_RADIUS, normal=.{0,0,1});
        capsule_end := posA+ SPH_HEIGH*rotate(Vector3.{0,1,0}, oriA);
        draw_primitive.circle(capsule_end,SPH_RADIUS);
        draw_primitive.circle(capsule_end,SPH_RADIUS, normal=.{0,0,1});
    draw_hull_properties(hull0);
    draw_hull_properties(hull1);
    num_contacts             :,
    penetration              :,
    world_normal_towards_A   :,
    world_midpoints          :,
    world_contact_points_on_A:,
    world_contact_points_on_B:= collide_capsule_vs_hull(posA,capsule_end, SPH_RADIUS,hull1);
    if num_contacts == 1 {
    P_START:= world_contact_points_on_B;
        P_END := P_START + world_normal_towards_A;

        draw_primitive.arrow(test1pos, world_normal_towards_A);
        draw_primitive.arrow(P_START, world_normal_towards_A, color=.{1,1,1,1});
        draw_primitive.doLabel("contact!", draw_primitive.WorldToScreen(P_END));
        arm := P_START-test1pos;
        torque := cross_product(arm, -world_normal_towards_A);

        // draw_primitive.arrow(test1pos, torque*4.0, color=.{1,1,1,1});
        // draw_primitive.doLabel("torque!", draw_primitive.WorldToScreen(test1pos));
        // push_line(test1pos, world_contact_points_on_A[it], .{0,1,0,1});

        dq :Quaternion;
        // set_from_axis_and_angle(*dq, torque, length_squared(torque)*context.solver.dt/0.0005);
        dq.xyz = torque *context.solver.dt/(4.0*num_contacts);
    }

    }

#if 0
    {
    SPH_RADIUS:= 0.2;
        draw_primitive.circle(posA,SPH_RADIUS);
        draw_primitive.circle(posA,SPH_RADIUS, normal=game_coordinates.left);
    draw_hull_properties(hull1);
    num_contacts             :,
    penetration              :,
    world_normal_towards_A   :,
    world_midpoints          :,
    world_contact_points_on_A:,
    world_contact_points_on_B:= collide_sphere_vs_hull(posA, SPH_RADIUS,hull1);
    if num_contacts == 1 {
        P_START := world_contact_points_on_B;
        P_END := P_START + world_normal_towards_A;

        draw_primitive.arrow(test1pos, world_normal_towards_A);
        draw_primitive.arrow(P_START, world_normal_towards_A, color=.{1,1,1,1});
        draw_primitive.doLabel("contact!", draw_primitive.WorldToScreen(P_END));
        arm := P_START-test1pos;
        torque := cross_product(arm, -world_normal_towards_A);

        // draw_primitive.arrow(test1pos, torque*4.0, color=.{1,1,1,1});
        // draw_primitive.doLabel("torque!", draw_primitive.WorldToScreen(test1pos));
        // push_line(test1pos, world_contact_points_on_A[it], .{0,1,0,1});

        dq :Quaternion;
        // set_from_axis_and_angle(*dq, torque, length_squared(torque)*context.solver.dt/0.0005);
        dq.xyz = torque *context.solver.dt/(4.0*num_contacts);
    }

    }

     //HULL STESTS
    #if 1
    {
    num_contacts             :,
    penetration              :,
    world_normal_towards_A   :,
    world_midpoints          :,
    world_contact_points_on_A:,
    world_contact_points_on_B:= collide_hull_vs_hull(hull0,hull1);


    // draw_debug_hull(posA, oriA, hull0);
    draw_primitive.doLabel("A", draw_primitive.WorldToScreen(posA));
    draw_primitive.doLabel("B", draw_primitive.WorldToScreen(test1pos));
    // draw_debug_hull(test1pos, test1ori, hull1);
    for 0..num_contacts-1 {
        P_START:= world_contact_points_on_B[it];
        P_END := P_START + world_normal_towards_A;

        draw_primitive.arrow(test1pos, world_normal_towards_A);
        draw_primitive.circle(P_START,.05);
        draw_primitive.arrow(P_START, world_normal_towards_A, color=.{1,1,1,1});
        draw_primitive.doLabel("contact!", draw_primitive.WorldToScreen(P_END));
        arm := P_START-test1pos;
        torque := cross_product(arm, -world_normal_towards_A);

        draw_primitive.arrow(test1pos, torque*4.0, color=.{1,1,1,1});
        draw_primitive.doLabel("torque!", draw_primitive.WorldToScreen(test1pos+torque*4.0));
        // push_line(test1pos, world_contact_points_on_A[it], .{0,1,0,1});

        dq :Quaternion;
        // set_from_axis_and_angle(*dq, torque, length_squared(torque)*context.solver.dt/0.0005);
        dq.xyz = torque *context.solver.dt/(4.0*num_contacts);
        // test1ori = dq* test1ori;
        // test1ori = normalize(test1ori);
    }

    draw_hull_properties(hull0);
    draw_hull_properties(hull1);
    }
    }
}


create_defult_box_initialized: bool;
create_defult_box_box: Half_Edge_Mesh;
maybe_create_defult_box :: (model: Matrix4 = Matrix4_Identity) -> Convex_Hull {
    hull: Convex_Hull;
    if !create_defult_box_initialized {
        create_defult_box_initialized = true;
        create_defult_box_box = create_cube_half_edge_mesh(.{1,1,1});
    }
    hull.mesh = create_defult_box_box;
    hull.worldSpace = model;

    return hull;
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
    alldata := basic.alloc(alloc_size_u8);
    memset(alldata, 0, alloc_size_u8);

	basic.assert(alldata!=null, "allocation failed");

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