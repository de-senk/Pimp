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
    local_attach_position_A: Vector3;
    local_attach_position_B: Vector3;
    local_attach_orientation_A: Quaternion;
    local_attach_orientation_B: Quaternion;

    world_attach_position_A: Vector3;
    world_attach_position_B: Vector3;
    world_attach_orientation_A: Quaternion;
    world_attach_orientation_B: Quaternion;

    local_axis_A: Vector3 = .{0, 1, 0};
    local_axis_B: Vector3 = .{1, 0, 0};
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
        world_attach_orientation_A = A.orientation * joint.local_attach_orientation_A;
    }

    if B {
        world_attach_position_B = l2w(B.position, B.orientation, local_attach_position_B);
        world_attach_orientation_B = B.orientation * joint.local_attach_orientation_B;
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
    //         // apply_local_cm_position_correction(A, B, dt, 0.0, error );

    //     }
}


solve_orientation_for_fixed_constraint_cb :: (using joint: *Joint_Constraint, dt: float) {
    // align orientations
    dq := joint.world_attach_orientation_A * conjugate(joint.world_attach_orientation_B);
    angular_error := Vector3.{2.0 * dq.x, 2.0 * dq.y, 2.0 * dq.z};
    if dq.w > 0.0 then angular_error *= -1.0;
    apply_local_cm_position_correction(A, B, dt, 0.0, angular_error );
}


solve_orientation_for_door_constraint_cb :: (using joint: *Joint_Constraint, dt: float)
{
    hardCompliance := 0.0;
    axis0 := Vector3.{1.0, 0.0, 0.0};
    axis1 := Vector3.{0.0, 1.0, 0.0};
    a0 := Vector3.{};
    a1 := Vector3.{};
    n := Vector3.{};
    error := Vector3.{};
    dq := Quaternion.{};
    // if joint.kind == .HINGE || joint.kind == .SERVO || joint.kind == .MOTOR
    // {
    //     // align axes

    //     recalculate_world_attach_pos_and_ori(joint);

    //     a0.copy(axis0);
    //     a0.applyQuaternion(joint.world_attach_orientation_A);
    //     a1.copy(axis0);
    //     a1.applyQuaternion(joint.globalRot1);
    a0 = joint.world_attach_orientation_A*axis0;
    a1 = joint.world_attach_orientation_B*axis1;
    error = cross_product(a0, a1);
    apply_local_cm_position_correction(A, B, dt, hardCompliance, error );

    //     error.crossVectors(a0, a1);
    //     joint.body0.applyCorrection(hardCompliance, error, null, joint.body1, null);

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
        apply_local_cm_position_correction(A, B, dt, hardCompliance, error );
    }
}


get_velocity_at_position :: (body: *Body, position: Vector3) -> Vector3 {
    velocity_at_point: Vector3;
    if body.inv_mass > 0.0
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
    damping_force := min(joint.linearDampingCoeff * dt, 1.0) * projected_velocity;
    apply_world_velocity_correction(A,B,dt,compliance=0.0, damping_force, world_attach_position_A, world_attach_position_B);
}

default_angular_damping_cb :: (using joint: *Joint_Constraint, dt: float)
{
    velocity_diff := A.angular_velocity;
    if B != null then velocity_diff -= B.angular_velocity;
    damping_force := - min(joint.angularDampingCoeff * dt, 1.0) * velocity_diff;
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
    damping_force = - min(joint.angularDampingCoeff * dt, 1.0) * projected_velocity;


    apply_local_cm_angular_velocity_correction(A, B, dt,compliance=0.0, damping_force);

}


sofarsogood :: #string JAI


getAngle:: (using joint: *Joint_Constraint, n, a, b)
{
    const c = Vector3.{}.crossVectors(a, b);
    let phi = Math.asin(c.dot(n));
    if (a.dot(b) < 0.0)
        phi = Math.PI - phi;
    if (phi > Math.PI)
        phi -= 2.0 * Math.PI;
    if (phi < -Math.PI)
        phi += 2.0 * Math.PI;
    return phi;
}

limitAngle:: (using joint: *Joint_Constraint, n, a, b, minAngle, maxAngle, compliance)
{
    let phi = this.getAngle(n, a, b);

    if (minAngle <= phi && phi <= maxAngle)
        return;
    phi = Math.max(minAngle, Math.min(phi, maxAngle));

    let ra = a.clone();
    ra.applyAxisAngle(n, phi);

    let corr = new THREE.Vector3().crossVectors(ra, b);
    this.body0.applyCorrection(compliance, corr, null, this.body1, null);
}


JAI;