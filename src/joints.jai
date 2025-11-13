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
    B: *Body;
    bucket_locator: Bucket_Locator;

    //for immediate mode
    latest_frame_of_existence: u64;
    hash: u64;

    disabled: bool;

    world_position: Vector3; //idk what to do with this yet
    //not all fields are shared, so there is a little bit of wasted memory

    //local -
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

initHingeJoint :: ( joint: *Joint_Constraint, swingMin: float, swingMax: float, hasTargetAngle: float, targetAngle: float, compliance: float, damping: float)
{
    // joint.kind = .HINGE;
    joint.target_length = 0.0;
    joint.swingMin = swingMin;
    joint.swingMax = swingMax;
    // joint.hasTargetAngle = hasTargetAngle;
    joint.targetAngle = targetAngle;
    joint.angular_compliance = compliance;
    joint.angularDampingCoeff = damping;
}

initServo :: ( joint: *Joint_Constraint, swingMin: float, swingMax: float)
{
    // joint.kind = .SERVO;
    joint.target_length = 0.0;
    joint.swingMin = swingMin;
    joint.swingMax = swingMax;
    joint.hasTargetAngle = true;
    joint.targetAngle = 0.0;
    joint.angular_compliance = 0.0;
}
initMotor :: ( joint: *Joint_Constraint, velocity: Vector3)
{
    // joint.kind = .MOTOR;
    joint.target_length = 0.0;
    // joint.velocity = velocity;
    joint.hasTargetAngle = true;
    joint.targetAngle = 0.0;
    joint.angular_compliance = 0.0;
}

initBallJoint :: ( joint: *Joint_Constraint, swingMax: float, twistMin: float, twistMax: float, damping: float)
{
    // joint.kind = .BALL;
    joint.target_length = 0.0;
    joint.swingMin = 0.0;
    joint.swingMax = swingMax;
    joint.twistMin = twistMin;
    joint.twistMax = twistMax;
    joint.angularDampingCoeff = damping;
}

initPrismaticJoint :: ( joint: *Joint_Constraint, distanceMin: float, distanceMax: float, twistMin: float, twistMax: float, hasTarget: float, target_length: float, targetCompliance: float, damping: float)
{
    // joint.kind = .PRISMATIC;
    joint.distanceMin = distanceMin;
    joint.distanceMax = distanceMax;
    joint.swingMin = 0.0;
    joint.swingMax = 0.0;
    joint.twistMin = twistMin;
    joint.twistMax = twistMax;
    // joint.hasTargetDistance = hasTarget;
    joint.target_length = target_length;
    joint.linear_compliance = targetCompliance;
    joint.linearDampingCoeff = damping;
}

initCylinderJoint :: ( joint: *Joint_Constraint, distanceMin: float, distanceMax: float, twistMin: float, twistMax: float, hasTargetDistance: float, restDistance: float, compliance: float, damping: float)
{
    // joint.kind = .CYLINDER;
    // joint.distaceMin = distanceMin;
    joint.distanceMax = distanceMax;
    joint.swingMin = 0.0;
    joint.swingMax = 0.0;
    joint.twistMin = twistMin;
    joint.twistMax = twistMax;
    joint.linear_compliance = 0.0;
}



recalculate_world_attach_pos_and_ori :: (using joint: *Joint_Constraint)
{
    if A {
        world_attach_position_A = l2w(A.position, A.orientation, local_attach_position_A);
        world_attach_orientation_A = normalize(A.orientation * joint.local_attach_orientation_A);
    }

    if B {
        world_attach_position_B = l2w(B.position, B.orientation, local_attach_position_B);
        world_attach_orientation_B = normalize(B.orientation * joint.local_attach_orientation_B);
    } else {
        world_attach_position_B = local_attach_position_B;
        world_attach_orientation_B = local_attach_orientation_B;
    }
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

// solve_position_for_one_way_distance_constraint_cb :: (using joint: *Joint_Constraint, dt: float) {
//     error := world_attach_position_B - world_attach_position_A;
//     distance := length(error);
//     if (distance == 0.0) then return; //another option is to select a default direction
//     if (target_length - distance) >= 0 then return;
//     normalize(*error);
//     error *= target_length - distance;
//     error *= -1.0;
//     apply_world_position_correction(A, B, dt, linear_compliance, error, world_attach_position_A, world_attach_position_B);
// }
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
    world_hinge_axis_B := rotate(joint.local_ref_B, ifx B B.orientation else .{});
    angular_error := cross_product(world_hinge_axis_A, world_hinge_axis_B);  //these axis should be aligned! Otherwise too bad, so error is this.
    apply_angular_correction(A, B, dt, hardCompliance, angular_error);
    //OK AFTER ITERATING; EVERYTHING ABOVE WORKS!

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
        push_vector(ss, recovered_door_front_axis_A, .{1,0,0,1}); doLabel("axis A", WorldToScreen(ss +recovered_door_front_axis_A));
        push_vector(ss, recovered_door_front_axis_B, .{0,1,0,1}); doLabel("axis B", WorldToScreen(ss +recovered_door_front_axis_B));
        push_vector(ss, world_hinge_axis, .{0,0,1,1}); doLabel("hinge", WorldToScreen(ss +world_hinge_axis));
        // axis_normal := get_normal_from_normal(world_hinge_axis);
        axis_normal := normalize(world_hinge_axis);
        // axis_normal := normalize(cross_product(world_hinge_axis, recovered_door_front_axis_A));
        // axis_normal := normalize(recovered_door_front_axis_A);
        // debug_break();

        a0 := cross_product(recovered_door_front_axis_A, world_hinge_axis);
        a1 := cross_product(recovered_door_front_axis_B, world_hinge_axis);

        push_vector(ss+physics_coordinates.left, a0, .{0.2,0,1,1}); doLabel("a0", WorldToScreen(ss +physics_coordinates.left+a0));
        push_vector(ss+physics_coordinates.left, a1, .{0,0.2,1,1}); doLabel("a1", WorldToScreen(ss +physics_coordinates.left+a1));

        //extra protection, make sure that a0 and a1 are perpendicular to hinge IMPORTANT!!!!
        a0 = normalize(a0 - dot(a0, axis_normal) * axis_normal);
        a1 = normalize(a1 - dot(a1, axis_normal) * axis_normal);
        error := cross_product(a0, a1);
        push_vector(ss+physics_coordinates.left, error, .{1,0.2,0.1,1}); doLabel("ERROR", WorldToScreen(ss +physics_coordinates.left+error));

        maybe_apply_angle_limit(joint, dt, axis_normal, a0, a1, joint.swingMin, joint.swingMax, 0.0);
    }
}

solve_orientation_for_door_constraint_cbBAD :: (using joint: *Joint_Constraint, dt: float)
{
    // oriA := joint.world_attach_orientation_A;
    oriA := joint.world_attach_orientation_A;
    oriB := joint.world_attach_orientation_B;
    hardCompliance := 0.0;
    recalculate_world_attach_pos_and_ori(joint);

    a0 := rotate(physics_coordinates.forward, oriA);
    a1 := rotate(physics_coordinates.forward, oriB);
    error := cross_product(a0, a1);
    apply_angular_correction(A, B, dt, hardCompliance, error );


    //     if (joint.hasTargetAngle)
    //     {
    //         recalculate_world_attach_pos_and_ori(joint);
    //         n.copy(axis0);
    //         n.applyQuaternion(oriA);
    //         a0.copy(axis1);
    //         a0.applyQuaternion(oriA);
    //         a1.copy(axis1);
    //         a1.applyQuaternion(joint.globalRot1);
    //         joint.limitAngle(n, a0, a1, joint.targetAngle, joint.targetAngle, joint.angular_compliance);
    //     }
    if (joint.swingMin > -FLOAT32_MAX || joint.swingMax < FLOAT32_MAX) //because defaults are set to max, but we should not use values beyond 2pi?
    {
        // recalculate_world_attach_pos_and_ori(joint);

        axis_normal := rotate(physics_coordinates.forward, oriA);
        a0 := rotate(physics_coordinates.left, oriA);
        a1 := rotate(physics_coordinates.left, oriB);
        // limitAngle(joint,dt, axis_normal, a0, a1, joint.swingMin, joint.swingMax, hardCompliance);
    }
    // }

}
#if 0
solvePosition :: (using joint: *Joint_Constraint, dt: float)
{
    hardCompliance := 0.0;

    if disabled || kind == .NONE return;


    corr: Vector3;

    // align

    if (kind == .PRISMATIC || kind == .CYLINDER)
    {
        target_length = basic.max(distanceMin, basic.min(target_length, distanceMax));
        hardCompliance := 0.0;

        corr := world_attach_position_B - world_attach_position_A;

        // rotate(*corr, conjugate(world_attach_orientation_A));
        // if (kind == .CYLINDER)
        //     corr.x -= target_length;
        // else if (corr.x > distanceMax)
        //     corr.x -= distanceMax;
        // else if (corr.x < distanceMin)
        //     corr.x -= distanceMin;
        // else
        //     corr.x = 0.0;

        // rotate(*corr, joint.world_attach_orientation_A);
        apply_world_position_correction(A, B, dt, hardCompliance, corr, world_attach_position_A, world_attach_position_B);
    }

    // solve distance

    if (kind != .CYLINDER && hasTargetDistance)
    {
        corr := world_attach_position_B - world_attach_position_A;
        distance := length(corr);
        if (distance == 0.0) then return;
        //another option is to select a default direction
        // if (distance == 0.0)
        // {
        //     corr = .{0.0, 0.0, 1.0};
        //     rotate(*corr, world_attach_orientation_A);
        // }
        // else
            normalize(*corr);

        corr *= target_length - distance;
        corr *= -1.0;
        apply_world_position_correction(A, B, dt, linear_compliance, corr, world_attach_position_A, world_attach_position_B);

    }
}

#if 0
solveOrientation :: (using joint: *Joint_Constraint, dt: float)
{
    // #run basic.log("if joint.disabled || joint.kind == .NONE || joint.kind == .DISTANCE return;");
    // if joint.disabled || joint.kind == .NONE  return;

    // if joint.kind == .MOTOR
    // {
    //     aAngle := basic.min(basic.max(joint.velocity * dt, -1.0), 1.0);
    //     joint.targetAngle += aAngle;
    // }

    hardCompliance := 0.0;
    axis0 := Vector3.{1.0, 0.0, 0.0};
    axis1 := Vector3.{0.0, 1.0, 0.0};
    a0 := Vector3.{};
    a1 := Vector3.{};
    n := Vector3.{};
    corr := Vector3.{};

    // if joint.kind == .HINGE || joint.kind == .SERVO || joint.kind == .MOTOR
    // {
    //     // align axes

    //     recalculate_world_attach_pos_and_ori(joint);

    //     a0.copy(axis0);
    //     a0.applyQuaternion(joint.world_attach_orientation_A);
    //     a1.copy(axis0);
    //     a1.applyQuaternion(joint.globalRot1);
    //     corr.crossVectors(a0, a1);
    //     joint.body0.applyCorrection(hardCompliance, corr, null, joint.body1, null);

    //     if (joint.hasTargetAngle)
    //     {
    //         recalculate_world_attach_pos_and_ori(joint);
    //         n.copy(axis0);
    //         n.applyQuaternion(joint.world_attach_orientation_A);
    //         a0.copy(axis1);
    //         a0.applyQuaternion(joint.world_attach_orientation_A);
    //         a1.copy(axis1);
    //         a1.applyQuaternion(joint.globalRot1);
    //         joint.limitAngle(n, a0, a1, joint.targetAngle, joint.targetAngle, joint.angular_compliance);
    //     }

    //     // joint limits

    //     if (joint.swingMin > -Number.MAX_VALUE || joint.swingMax < Number.MAX_VALUE)
    //     {
    //         recalculate_world_attach_pos_and_ori(joint);

    //         n.copy(axis0);
    //         n.applyQuaternion(joint.world_attach_orientation_A);
    //         a0.copy(axis1);
    //         a0.applyQuaternion(joint.world_attach_orientation_A);
    //         a1.copy(axis1);
    //         a1.applyQuaternion(joint.globalRot1);
    //         joint.limitAngle(n, a0, a1, joint.swingMin, joint.swingMax, hardCompliance);
    //     }
    // }
    // else if (joint.kind == .BALL || joint.kind == .PRISMATIC || joint.kind == .CYLINDER)
    // {
    //     // swing limit

    //     recalculate_world_attach_pos_and_ori(joint);

    //     a0.copy(axis0);
    //     a0.applyQuaternion(joint.world_attach_orientation_A);
    //     a1.copy(axis0);
    //     a1.applyQuaternion(joint.globalRot1);
    //     n.crossVectors(a0, a1);
    //     n.normalize();
    //     joint.limitAngle(n, a0, a1, joint.swingMin, joint.swingMax, hardCompliance);

    //     // twist limit

    //     recalculate_world_attach_pos_and_ori(joint);

    //     a0.copy(axis0);
    //     a0.applyQuaternion(joint.world_attach_orientation_A);
    //     a1.copy(axis0);
    //     a1.applyQuaternion(joint.globalRot1);
    //     n.addVectors(a0, a1);
    //     n.normalize();

    //     a0.copy(axis1);
    //     a0.applyQuaternion(joint.world_attach_orientation_A);
    //     a1.copy(axis1);
    //     a1.applyQuaternion(joint.globalRot1);

    //     a0.addScaledVector(n, -n.dot(a0));
    //     a0.normalize();
    //     a1.addScaledVector(n, -n.dot(a1));
    //     a1.normalize();
    //     joint.limitAngle(n, a0, a1, joint.twistMin, joint.twistMax, hardCompliance);
    // }
    // else
    // if (joint.kind == .FIXED)
    {
        // align orientations


        dq := Quaternion.{};
        // dq.multiplyQuaternions(joint.world_attach_orientation_A, joint.globalRot1.conjugate());
        dq = joint.world_attach_orientation_A * conjugate(joint.world_attach_orientation_B);
        error = .{2.0 * dq.x, 2.0 * dq.y, 2.0 * dq.z};
        if dq.w > 0.0
            error *= -1.0;
        // applyCorrection(A, hardCompliance, error, null, joint.body1, null);
        apply_angular_correction(A, B, dt, hardCompliance, error );
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
    // this.recalculate_world_attach_pos_and_ori();
    dVel := get_velocity_at_position(A, world_attach_position_A);
    if B != null
        dVel -= get_velocity_at_position(B, world_attach_position_B);

    n := normalize(world_attach_position_B - world_attach_position_A);
    projected_velocity := dot(-dVel, n) * n;
    damping_force := basic.min(joint.linearDampingCoeff * dt, 1.0) * projected_velocity;
    apply_world_velocity_correction(A,B,dt,compliance=0.0, damping_force, world_attach_position_A, world_attach_position_B);
}

default_angular_damping_cb :: (using joint: *Joint_Constraint, dt: float)
{
    velocity_diff := A.angular_velocity;
    if B != null then velocity_diff -= B.angular_velocity;
    damping_force := - basic.min(joint.angularDampingCoeff * dt, 1.0) * velocity_diff;
    apply_local_cm_angular_velocity_correction(A, B, dt,compliance=0.0, damping_force);

}

max_angular_damping_cb :: (using joint: *Joint_Constraint, dt: float)
{
    damping_force := A.angular_velocity;
    if B != null damping_force -= B.angular_velocity;
    damping_force = -damping_force;
    apply_local_cm_angular_velocity_correction(A, B, dt,compliance=0.0, damping_force);

}

hinge_angular_damping_cb :: (using joint: *Joint_Constraint, dt: float)
{
    damping_force := A.angular_velocity;
    if B != null
        damping_force -= B.angular_velocity;

    n := Vector3.{1.0, 0.0, 0.0}; //im not sure about this, why not other defautl normal?
    n = rotate(n, world_attach_orientation_A);
    projected_velocity := dot(damping_force, n) * n;
    damping_force = - basic.min(joint.angularDampingCoeff * dt, 1.0) * projected_velocity;


    apply_local_cm_angular_velocity_correction(A, B, dt,compliance=0.0, damping_force);

}


getAngle :: (using joint: *Joint_Constraint, n: Vector3, a: Vector3, b: Vector3) -> float
{
    c := cross_product(a, b);
    h := clamp(dot(c, n), -1.0, 1.0);
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

limitAngle3:: (using joint: *Joint_Constraint, dt:float, n: Vector3, axis_A: Vector3, axis_B: Vector3, minAngle: float, maxAngle: float, compliance: float)
{
    phi := getAngle(joint, n, axis_A, axis_B);
    between_allowed_range := phi >= minAngle && phi <= maxAngle;
    phi = basic.max(minAngle, basic.min(phi, maxAngle));
    sign := 1.5;
    if phi == minAngle sign = -1.0;


    // for drawing
    r := Vector4.{1, 0, 0, 0.9};g := Vector4.{0, 1, 0, 0.9};b := Vector4.{0, 0, 1, 0.9};
    start:= A.position+physics_coordinates.forward+physics_coordinates.left;
    //

    // for rotating
    lq := joint.local_attach_orientation_A;
    // lq.w = -lq.w;
    clq := conjugate(joint.local_attach_orientation_A);
    //

    dq: Quaternion;
    set_from_axis_and_angle(*dq, n, phi);
    rotated_A := rotate(axis_A, dq);
    angular_correction := cross_product(rotated_A, axis_B);

    // Get the reference "left" axis from the LOCAL constraint frame
    local_reference := physics_coordinates.left;

    // Transform to world space using the body's orientation AND the local attachment
    world_reference := rotate(local_reference, lq);

    // Now rotate by the limit angles around the hinge axis
    dqmin: Quaternion;
    set_from_axis_and_angle(*dqmin, n, -minAngle - PI/2);
    dqmax: Quaternion;
    set_from_axis_and_angle(*dqmax, n, -maxAngle - PI/2);

    world_reference1 := rotate(world_reference, Quaternion.{});
    world_reference2 := rotate(world_reference, Quaternion.{});

    limit_normal_min := rotate(world_reference1, dqmin );
    limit_normal_max := rotate(world_reference2, dqmax);
    // limit_normal_min = rotate(world_reference, conjugate(A.orientation));
    // limit_normal_max = rotate(world_reference, conjugate(A.orientation));

    grid_color := ifx between_allowed_range then Vector4.{0.2, 0.2, 0.2, 0.3} else Vector4.{1, 1, 1, 0.9};


    push_vector(start,  world_reference, r);
    push_vector(start,  limit_normal_min, g);
    push_vector(start,  limit_normal_max, g);
    // push_grid(joint.world_attach_position_A, limit_normal_min, min_for=-1, max_for=1, density=4, grid_color);
    // push_grid(joint.world_attach_position_A, limit_normal_max, min_for=-1, max_for=1, density=4, grid_color);

    if between_allowed_range return;
    apply_angular_correction(A, B, dt, compliance, angular_correction);
}

//NOTE for (tr): check milton diagram, this is hard, seems easy because small LOC but it is easy to get wrong.
limitAngleBAD :: (using joint: *Joint_Constraint, dt:float, n: Vector3, axis_A: Vector3, axis_B: Vector3, minAngle: float, maxAngle: float, compliance: float)
{
    phi := getAngle(joint, n, axis_A, axis_B);
    between_allowed_range  := phi >= minAngle && phi <= maxAngle;

    phi = basic.max(minAngle, basic.min(phi, maxAngle));
    sign := 1.5;
    if phi == minAngle sign = -1.0;
    dq: Quaternion; // ---
    set_from_axis_and_angle(*dq, n, phi);
    n_for_grids := n;

    rotated_A := rotate(axis_A, dq);
    angular_correction := cross_product(rotated_A, axis_B);
    nana := get_normal_from_normal(n_for_grids);

    lq := joint.local_attach_orientation_A;
    // lq.w = -lq.w;
    clq := conjugate(joint.local_attach_orientation_A);
    n_for_grids = rotate(n_for_grids, lq);
    nana = rotate(nana, lq);
    a_cross_b := cross_product(n_for_grids, nana);

        // n_for_grids = rotate(a_cross_b1, clq);

    dqmin: Quaternion; // ---
    set_from_axis_and_angle(*dqmin, n, minAngle - PI/7);

    dqmax: Quaternion; // ---
    set_from_axis_and_angle(*dqmax, n, maxAngle - PI/1);

    a_cross_b1 := rotate(a_cross_b, conjugate(dqmin));
    a_cross_b2 := rotate(a_cross_b, conjugate(dqmax));
    // rotate(*a_cross_b1, lq);
    // rotate(*a_cross_b2, lq);
    //these two dont work because then the grid follows the character, and I dont want that.
    // a_cross_b2 = rotate(a_cross_b2, lq);
    grid_color := ifx between_allowed_range then Vector4.{0.2, 0.2, 0.2, 0.3} else Vector4.{1, 1, 1, 0.9};
    r := Vector4.{1, 0, 0, 0.9};g := Vector4.{0, 1, 0, 0.9};b := Vector4.{0, 0, 1, 0.9};
    start:= A.position+physics_coordinates.forward+physics_coordinates.left;

    // push_grid(A.position + axis_A + 0.2*sign*nana, nana, area_length=1, density=4, grid_color);
    // push_grid(A.position, a_cross_b1, min_for=-1, max_for=1, density=4, grid_color);
    // push_grid(A.position, a_cross_b2, min_for=-1, max_for=1, density=4, grid_color);
    push_vector(start,  n_for_grids, r);
    push_vector(start,  nana, g);
    // push_grid(A.position, a_cross_b2, min_for=-1, max_for=1, density=4, grid_color);
    if between_allowed_range return; //early return
    apply_angular_correction(A, B, dt, compliance, angular_correction );
}

maybe_apply_angle_limit :: (using joint: *Joint_Constraint, dt:float, n: Vector3, axis_A: Vector3, axis_B: Vector3, minAngle: float, maxAngle: float, compliance: float)
{
    current_angle := getAngle(joint, n, axis_A, axis_B);
    between_allowed_range := current_angle >= minAngle && current_angle <= maxAngle;
    if between_allowed_range return;
    clamped_angle := basic.max(minAngle, basic.min(current_angle, maxAngle));

    // Create quaternion to rotate axis_A to the clamped position
    dq: Quaternion;
    set_from_axis_and_angle(*dq, n, clamped_angle);
    rotated_A := rotate(axis_A, dq);
    angular_error := cross_product(rotated_A, axis_B);

    // Apply correction
    apply_angular_correction(A, B, dt, compliance, angular_error);
}