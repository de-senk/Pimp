Joint_Kinds :: enum {
    NONE;
    DISTANCE;
    HINGE;
    SERVO;
    MOTOR;
    BALL;
    PRISMATIC;
    CYLINDER;
    FIXED;
};



Two_Body_Constraint_Shared :: struct {
    A: *Body;
    B: *Body; //I dont know how to make this a default STUB Right here.
    bucket_locator: Bucket_Locator;

    //for immediate mode
    latest_frame_of_existence: u64;
    hash: u64;

    disabled: bool;

    world_position: Vector3; //idk what to do with this yet
    //not all fields are shared, so there is a little bit of wasted memory

    //local - yo
    local_attach_position_A: Vector3;
    local_attach_position_B: Vector3;
    local_attach_orientation_A: Quaternion;
    local_attach_orientation_B: Quaternion;

    //world - calculated every frame, private fields? I dont think these fields are should be here, I think its faster to recalculate for every joint every iteraton.
    world_attach_position_A: Vector3;
    world_attach_position_B: Vector3;
    world_attach_orientation_A: Quaternion;
    world_attach_orientation_B: Quaternion;

    //useful for some cosntraints like hinge, needed because I could not figure out how to avoid it.
    local_ref_A: Vector3 = .{0, 1, 0};
    local_ref_B: Vector3 = .{1, 0, 0};
}

Joint_Constraint :: struct {
    using base: Two_Body_Constraint_Shared;

    //I think these procedure pointers allows for branch predictions, but I don't like doing these assumptions before actually measuring the reuslt.
    pre_solve : (using joint: *Joint_Constraint, dt: float) = mostly_shared_presolve_default_cb;
    solve_position := (using joint: *Joint_Constraint, dt: float){};
    solve_orientation := (using joint: *Joint_Constraint, dt: float){};

    apply_linear_damping := default_linear_damping_cb;
    apply_angular_damping := default_angular_damping_cb;
    // kind: Joint_Kinds;
    // distance
    target_length := 0.0;
    linear_compliance := 0.0;
    distanceMin := -FLOAT32_INFINITY;
    distanceMax := FLOAT32_INFINITY;
    linearDampingCoeff := 1.0;



    // orientation
    swingMin := -FLOAT32_INFINITY;
    swingMax := FLOAT32_INFINITY;
    twistMin := -FLOAT32_INFINITY;
    twistMax := FLOAT32_INFINITY;
    targetAngle := 0.0;
    hasTargetAngle := false;
    angular_compliance := 1.0;
    angularDampingCoeff := 0.0;

    // motor
    velocity := 0.0;

}

recalculate_world_attach_pos_and_ori :: inline (using joint: *Joint_Constraint)
{
    world_attach_position_A = l2w(A.position, A.orientation, local_attach_position_A);
    world_attach_orientation_A = normalize(A.orientation * joint.local_attach_orientation_A);
    world_attach_position_B = l2w(B.position, B.orientation, local_attach_position_B);
    world_attach_orientation_B = normalize(B.orientation * joint.local_attach_orientation_B);
}

#if 0
applyTorque :: (using joint: *Joint_Constraint,dt: float, torque: float)
{
    // recalculate_world_attach_pos_and_ori();

    // assumng x-axis is the hinge axis

    corr := Vector3.{1.0, 0.0, 0.0};
    rotate(*corr, world_attach_orientation_A);
    corr *= torque * dt;

    // joint.body0.applyCorrection(0.0, corr, null, joint.body1, null, true);
    apply_velocity_correction(A,B, dt, 0.0, corr, world_attach_position_A, world_attach_position_B);
}


//UNUSED; I flattened this out to reduce levels of indirections.
solve :: (using joint: *Joint_Constraint, dt: float)
{
    //COMMENTTED OUT BECAUSE UNUSED AND I GOT CONFUSED ONCE
    //these are callbacks of procedures stored inside each joint, I think this strategy is better than a switch statement.
    // joint.pre_solve(joint, dt);
    // recalculate_world_attach_pos_and_ori(joint);
    // joint.solve_orientation(joint, dt);
    // recalculate_world_attach_pos_and_ori(joint);
    // joint.solve_position(joint, dt);
}

mostly_shared_presolve_default_cb :: (using joint: *Joint_Constraint, dt: float) {
        // recalculate_world_attach_pos_and_ori(joint);
}

//there is no solve_orientation for distance constraints, I'm also tempted to get rid of this distance constraint, and use more natural names like steel_rod bungie_rod? gomu_rod?, but thinking again, since I also want to support retained mode, people that have more experience with physics engines might want just this procedire.
solve_position_for_distance_constraint_cb :: (using joint: *Joint_Constraint, dt: float) {
    error := world_attach_position_B - world_attach_position_A;
    distance := length(error);
    if (distance == 0.0) then return; //another option is to select a default direction
    normalize(*error);
    error *= target_length - distance;
    error *= -1.0;
    apply_world_position_correction(A, B, dt, linear_compliance, error, world_attach_position_A, world_attach_position_B);
}



solve_position_for_one_way_distance_constraint_cb :: (using joint: *Joint_Constraint, dt: float) {
    error := world_attach_position_B - world_attach_position_A;
    distance := length(error);
    if (distance == 0.0) then return; //another option is to select a default direction
    if (target_length - distance) >= 0 then return;
    normalize(*error);
    error *= target_length - distance;
    error *= -1.0;
    apply_world_position_correction(A, B, dt, linear_compliance, error, world_attach_position_A, world_attach_position_B);
}

solve_position_for_fixed_constraint_cb :: (using joint: *Joint_Constraint, dt: float) {
    error := world_attach_position_B - world_attach_position_A;
    distance := length(error);
    if  distance == 0.0 then return;  //another option is to select a default direction
    normalize(*error);

    // error *= target_length - distance;
    error *= 0.0 - distance; //0.0 because I cannot figure out how to do it correctl with a stiff rod
    error *= -1.0;
    apply_world_position_correction(A, B, dt, linear_compliance, error, world_attach_position_A, world_attach_position_B);


    //commented out is what I tried to do to make a stiff rod.
    // dq := joint.world_attach_orientation_B * conjugate(joint.world_attach_orientation_A);
    // if dq.w != 0 {
    //     error = .{2.0 * dq.x, 2.0 * dq.y, 2.0 * dq.z};
    //     if dq.w > 0.0
    //         error *= -1.0;
    //         apply_world_position_correction(A, B, dt, 0.0, error, .{}, .{});
    //         // apply_angular_correction(A, B, dt, 0.0, error );

    //     }
}

solve_orientation_for_fixed_constraint_cb :: (using joint: *Joint_Constraint, dt: float) {
    // align orientations
    dq := joint.world_attach_orientation_A * conjugate(joint.world_attach_orientation_B);
    angular_error := Vector3.{2.0 * dq.x, 2.0 * dq.y, 2.0 * dq.z};
    if dq.w > 0.0 then angular_error *= -1.0;
    apply_angular_correction(A, B, dt, 0.0, angular_error );
}


//
// It works, comment these two lines to remember everything :tagLineToUncomment
//
solve_orientation_for_door_constraint_cb :: (using joint: *Joint_Constraint, dt: float)
{
    hardCompliance := 0.0;
    recalculate_world_attach_pos_and_ori(joint); //I think this is not necessary, but we will see, I need to build test with multiple cosntratints attached together and measure.

    //
    // Weld the two Non-Free axis
    //
    world_hinge_axis_A := rotate(joint.local_ref_A, A.orientation);
    world_hinge_axis_B := rotate(joint.local_ref_B, B.orientation);
    angular_error := cross_product(world_hinge_axis_A, world_hinge_axis_B);  //these axis should be aligned! Otherwise too bad, so error is this.
    apply_angular_correction(A, B, dt, hardCompliance, angular_error);

    //
    // Angle limits for free axis
    //
    if (joint.swingMin > -FLOAT32_MAX || joint.swingMax < FLOAT32_MAX)
    {
        //this is messy because I didnt wanted to make new fields in the joints struct. but basically I reconstruct encoded data in the quaternions in useful data to apply angle limits.
        recalculate_world_attach_pos_and_ori(joint);
        world_hinge_axis := rotate(joint.local_ref_A, A.orientation);
        oriA := joint.world_attach_orientation_A;
        oriB := joint.world_attach_orientation_B; //these should be exactly the same.
        //reflection_axis and angle, TODO: explain these.
        // reflection_axis, angle := get_axis_and_angle(oriA); //unnecesarry

        //dont delete, I might need another version what uses local initial conditions.
        // recovered_door_front_axis_A := normalize(rotate(world_hinge_axis, (oriA))); //:tagLineToUncomment
        // recovered_door_front_axis_B := normalize(rotate(world_hinge_axis, (oriB))); //:tagLineToUncomment
        recovered_door_front_axis_A := normalize(rotate(joint.local_ref_A, (oriA)));
        recovered_door_front_axis_B := normalize(rotate(joint.local_ref_B, (oriB)));
        ss := A.position - physics_coordinates.forward+physics_coordinates.up;
        draw_primitive.vector(ss, recovered_door_front_axis_A, .{1,0,0,1}); draw_primitive.doLabel("axis A", draw_primitive.WorldToScreen(ss +recovered_door_front_axis_A));
        draw_primitive.vector(ss, recovered_door_front_axis_B, .{0,1,0,1}); draw_primitive.doLabel("axis B", draw_primitive.WorldToScreen(ss +recovered_door_front_axis_B));
        draw_primitive.vector(ss, world_hinge_axis, .{0,0,1,1}); draw_primitive.doLabel("hinge", draw_primitive.WorldToScreen(ss +world_hinge_axis));
        // axis_normal := get_normal_from_normal(world_hinge_axis);
        axis_normal := normalize(world_hinge_axis);
        // axis_normal := normalize(cross_product(world_hinge_axis, recovered_door_front_axis_A));
        // axis_normal := normalize(recovered_door_front_axis_A);
        // debug_break();

        a0 := cross_product(recovered_door_front_axis_A, world_hinge_axis);
        a1 := cross_product(recovered_door_front_axis_B, world_hinge_axis);

        draw_primitive.vector(ss+physics_coordinates.left, a0, .{0.2,0,1,1}); draw_primitive.doLabel("a0", draw_primitive.WorldToScreen(ss +physics_coordinates.left+a0));
        draw_primitive.vector(ss+physics_coordinates.left, a1, .{0,0.2,1,1}); draw_primitive.doLabel("a1", draw_primitive.WorldToScreen(ss +physics_coordinates.left+a1));

        //extra protection, make sure that a0 and a1 are perpendicular to hinge IMPORTANT!!!!
        a0 = normalize(a0 - dot(a0, axis_normal) * axis_normal);
        a1 = normalize(a1 - dot(a1, axis_normal) * axis_normal);
        error := cross_product(a0, a1);
        draw_primitive.vector(ss+physics_coordinates.left, error, .{1,0.2,0.1,1}); draw_primitive.doLabel("ERROR", draw_primitive.WorldToScreen(ss +physics_coordinates.left+error));

        maybe_apply_angle_limit(joint, dt, axis_normal, a0, a1, joint.swingMin, joint.swingMax, 0.01);
    }
}


get_velocity_at_position :: (body: *Body, position: Vector3) -> Vector3 {
    velocity_at_point: Vector3;
    if body.inv_mass != 0.0
    {
        center_of_mass := body.position;
        lever := position - center_of_mass;
        velocity_at_point = cross_product(lever, body.angular_velocity);
        velocity_at_point = body.velocity - velocity_at_point;
    }
    return velocity_at_point;
}

//
// NOTE IMPORTANT: I think I need to store "contribution_to_A_this_frame" and same for B, so I can damp only if the cosntraint contributed to the bodies. Because right now, I think its wrong.
//
default_linear_damping_cb :: (using joint: *Joint_Constraint, dt: float) {
    dVel := get_velocity_at_position(A, world_attach_position_A);
    dVel -= get_velocity_at_position(B, world_attach_position_B);

    n := normalize(world_attach_position_B - world_attach_position_A);
    projected_velocity := dot(-dVel, n) * n;
    damping_force := basic.min(joint.linearDampingCoeff * dt, 1.0) * projected_velocity;
    apply_world_velocity_correction(A,B,dt,compliance=0.0, damping_force, world_attach_position_A, world_attach_position_B);
}

default_angular_damping_cb :: (using joint: *Joint_Constraint, dt: float)
{
    velocity_diff := A.angular_velocity;
    velocity_diff -= B.angular_velocity;
    damping_force := - basic.min(joint.angularDampingCoeff * dt, 1.0) * velocity_diff;
    apply_angular_velocity_correction(A, B, dt,compliance=0.0, damping_force);

}

max_angular_damping_cb :: (using joint: *Joint_Constraint, dt: float)
{
    velocity_diff := A.angular_velocity;
    velocity_diff -= B.angular_velocity;
    damping_force := -velocity_diff;
    apply_angular_velocity_correction(A, B, dt,compliance=0.0, damping_force);

}

hinge_angular_damping_cb :: (using joint: *Joint_Constraint, dt: float)
{
    velocity_diff := A.angular_velocity;
    velocity_diff -= B.angular_velocity;

    n := Vector3.{1.0, 0.0, 0.0}; //im not sure about this, why not other defautl normal?
    n = rotate(n, world_attach_orientation_A);
    projected_velocity := dot(velocity_diff, n) * n;
    damping_force := - basic.min(joint.angularDampingCoeff * dt, 1.0) * projected_velocity;


    apply_angular_velocity_correction(A, B, dt,compliance=0.0, damping_force);

}


getAngle :: (using joint: *Joint_Constraint, n: Vector3, a: Vector3, b: Vector3) -> float
{
    c := cross_product(a, b);
    h := basic.clamp(dot(c, n), -1.0, 1.0);
    phi := asin(h);
    are_in_opposite_dirs := dot(a, b) < 0.0;
    if are_in_opposite_dirs
        phi = PI - phi;
    if phi > PI
        phi -= 2.0 * PI;

    if phi < -PI
        phi += 2.0 * PI;
    return phi;
}

maybe_apply_angle_limit :: (using joint: *Joint_Constraint, dt:float, n: Vector3, axis_A: Vector3, axis_B: Vector3, minAngle: float, maxAngle: float, compliance: float)
{
    current_angle := getAngle(joint, n, axis_A, axis_B);
    between_allowed_range := current_angle >= minAngle && current_angle <= maxAngle;
    if between_allowed_range return;
    clamped_angle := basic.max(minAngle, basic.min(current_angle, maxAngle));

    //change names of rotated_A and others to better ones.
    dq: Quaternion;
    set_from_axis_and_angle(*dq, n, clamped_angle);
    rotated_A := rotate(axis_A, dq);
    angular_error := cross_product(rotated_A, axis_B);

    apply_angular_correction(A, B, dt, compliance, angular_error);
}