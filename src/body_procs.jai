//
// Maybe the name of this file will be modified in the future
// I try to put in this file all procedures that
// 1) extracts information from bodies
// 2) modifies information of bodies.

//
// Local ---> World
//
//this is the way
#program_export
l2w ::  (position: Vector3,Aori: Quaternion, local_point: Vector3) -> Vector3 {
    // v1 := position + rotate(local_point, Aori);
    // v2 := position + rotation_matrix(Matrix3, Aori) * local_point;
    // if v1!=v2 then log("eeheeh %", v1-v2 );
    return  position + rotation_matrix(Matrix3, Aori) * local_point;
} @dll_export

#program_export
l2w_rotate_only :: inline (_: Vector3, Aori: Quaternion, local_point: Vector3) -> Vector3 {
    return  rotation_matrix(Matrix3, Aori) * local_point;
} @dll_export

//
// World ---> Local
//
#program_export
w2l :: inline (position: Vector3, orientation: Quaternion, world_point: Vector3) -> Vector3 {
    return rotation_matrix(Matrix3, conjugate(orientation)) * (world_point - position);
} @dll_export

#program_export
w2l_rotate_only :: inline (_: Vector3, orientation: Quaternion, world_point: Vector3) -> Vector3 {
    //_ gets ignored, but I write it anyways, the only reason this parameter "_" exist is because I like to iterate on soem stuff without changing arguments.
    return  rotation_matrix(Matrix3, conjugate(orientation)) * (world_point ); //Maybe: replace with rotate() from amth mdouels.
} @dll_export


get_C_and_normal :: inline (error: Vector3) -> C: float, normal: Vector3 {
    dist2 := length_squared(error);
    C := sqrt(dist2);
    normal := normalize(error);
    return C, normal;
}

//
//Lagrange multipliers - TODO: explain lagrange multipliers. NOTE: I know I could take advantage of #bake_arguments but I decided to jsut write more instead.
//

//for max stiffness
get_lambda :: inline (dt: float, w: float,C: float) -> lambda: float, force_magnitude: float {
   dt_squared := dt*dt;
   lambda := -C / (w);
   if w == 0 then lambda = 0.0;
   return lambda, lambda / dt_squared;
}

//most used one, I jsut set compliance to 0 and never use the above one.
get_lambda :: inline (dt: float, w: float,C: float, compliance: float) -> lambda: float, force_magnitude: float {
   dt_squared := dt*dt;
   alpha := compliance / dt_squared;
   lambda := -C / (w + alpha);
   if w == 0 then lambda = 0.0; //SIMD friendly easy blend. But I wish I could avoid masks.
   return lambda, lambda / dt_squared;
}

//With cache of lambda, remember seted lamda to 0 before iterating every frame!
get_lambda :: inline (dt: float, w: float,C: float, compliance: float, old_lambda: float) -> lambda: float, force_magnitude: float {
   dt_squared := dt*dt;
   alpha := compliance / dt_squared;
   lambda := (-C-alpha*old_lambda) / (w + alpha);
   if w == 0 then lambda = 0.0;
   return lambda, lambda / dt_squared;
}


//
// Effective masses is the mass you would believe you are dealing with when you try to move a particle at a distance (arm) in the maybe always normlaized direction (direction), Im not 100% that it is always normalized, but lets assume it for now.
//
get_effective_mass_angular_only :: (using body: Body, world_lever: Vector3) -> w: float
{
    local_lever := rotate(world_lever, conjugate(body.orientation));
    w := local_lever.x * local_lever.x * body.inv_local_moment_of_inertia.x +
         local_lever.y * local_lever.y * body.inv_local_moment_of_inertia.y +
         local_lever.z * local_lever.z * body.inv_local_moment_of_inertia.z;
    return w;
}

get_effective_mass :: (using body: *Body, normalized_world_force: Vector3, world_attach_position: Vector3) -> w: float
{
    w_linear := body.inv_mass;

    //angular effective mass
    center_of_mass := body.position;
    world_lever := world_attach_position - center_of_mass;
    world_angular_force_direction := cross_product(world_lever, normalized_world_force);
    w_angular := get_effective_mass_angular_only(body, world_angular_force_direction);

    //total effective mass
    return w_linear + w_angular;
}

apply_world_position_correction :: (A: *Body, B: *Body, dt: float, compliance: float32, correction: Vector3, attach_position_A: Vector3, attach_position_B: Vector3) -> float
{
    C, normal := get_C_and_normal(correction);

    w := get_effective_mass(A, normal, attach_position_A);
    w += get_effective_mass(B, normal, attach_position_B);

    lambda, force_magnitude := get_lambda(dt, w, C, compliance);

    correction_force := -lambda * normal;

    dx_for_A := get_world_linear_position_correction(A,correction_force, attach_position_A);
    dq_for_A := get_world_angular_position_correction(A,correction_force, attach_position_A);
    A.position += dx_for_A;
    A.orientation = normalize(dq_for_A + A.orientation);

    draw_primitive.line(A.position, attach_position_A,.{x=1, w=1});
    draw_primitive.line(attach_position_A, attach_position_A + correction_force,.{x=0.1, z=1});
    draw_primitive.line(attach_position_A, attach_position_B,.{x=0.1, w=1});

    dx_for_B := get_world_linear_position_correction(B, -correction_force, attach_position_B);
    dq_for_B := get_world_angular_position_correction(B, -correction_force, attach_position_B);
    B.position += dx_for_B;
    B.orientation = normalize(dq_for_B + B.orientation);

    return force_magnitude;
}

apply_world_velocity_correction :: (A: *Body, B: *Body, dt: float, compliance: float32, correction: Vector3, attach_position_A: Vector3, attach_position_B: Vector3) -> float
{
    C, normal := get_C_and_normal(correction);
    w := get_effective_mass(A, normal, attach_position_A);
    w += get_effective_mass(B, normal, attach_position_B);

    lambda, force_magnitude := get_lambda(dt, w, C, 0.0); //NOTE: we hardcode 0 compliance for velocity, I dont know why yet.
    correction_force := - lambda * normal;

    dvel_A := get_world_linear_velocity_correction(A, correction_force);
    dang_A := get_world_angular_velocity_correction(A, correction_force, attach_position_A);
    A.velocity += dvel_A;
    A.angular_velocity += dang_A;

    dvel_B := get_world_linear_velocity_correction(B,-correction_force);
    dang_B := get_world_angular_velocity_correction(B,-correction_force, attach_position_B);
    B.velocity += dvel_B;
    B.angular_velocity += dang_B;
    return force_magnitude;
}


get_world_linear_velocity_correction :: (body: Body, correction_force: Vector3) -> delta_linear_velocity: Vector3 {
    return correction_force * body.inv_mass;
}

get_world_angular_velocity_correction :: (body: Body, correction_force: Vector3, world_point_of_application: Vector3) -> delta_angular_velocity: Vector3 {
    center_of_mass := body.position;
    lever := world_point_of_application - center_of_mass;
    correction_torque := cross_product(lever, correction_force);
    delta_angular_velocity := inline get_delta_angular_from_torque(body, correction_torque);

    return delta_angular_velocity;
}


/*  TODO: SUBJECT[inv_local_moment_of_inertia: Vector3] figure out if we can still use this for NON symetrical objects, or we need to go back to a matrix form of moment of inertia
    this is Angular Force / Angular Mass, is angular acceleration, just like Linear
*/

//maybe delete
// get_delta_angular_from_torque :: inline (inv_local_moment_of_inertia: Vector3, orientation: Quaternion , correction_torque: Vector3) -> delta_angular: Vector3 {
//     delta_angular := correction_torque;
//     delta_angular = rotate(delta_angular, conjugate(orientation));
//     new_ori := orientation;
//     new_ori.x *= inv_local_moment_of_inertia.x ;
//     new_ori.y *= inv_local_moment_of_inertia.y ;
//     new_ori.z *= inv_local_moment_of_inertia.z ;
//     delta_angular = rotate(delta_angular, (new_ori));
//     return delta_angular;
// }

get_delta_angular_from_torque :: inline (inv_local_moment_of_inertia: Vector3, orientation: Quaternion , correction_torque: Vector3) -> delta_angular: Vector3 {
    delta_angular := correction_torque;
    delta_angular = rotate(delta_angular, conjugate(orientation));
    delta_angular *= inv_local_moment_of_inertia;
    delta_angular = rotate(delta_angular, (orientation));
    return delta_angular;
}

get_delta_angular_from_torque :: inline (body: Body, correction_torque: Vector3) -> delta_angular: Vector3 {
    return get_delta_angular_from_torque(body.inv_local_moment_of_inertia, body.orientation, correction_torque);
}

//maybe delete
// get_delta_angular_from_torque :: inline ( w_ang: float,  orientation: Quaternion , correction_torque: Vector3) -> delta_angular: Vector3 {
//     delta_angular := correction_torque;
//     delta_angular = rotate(delta_angular, conjugate(orientation));
//     delta_angular *= w_ang;
//     delta_angular = rotate(delta_angular, (orientation));
//     return delta_angular;
// }

//long name: apply_position_correction_to_center_of_mass_frame_of_reference
apply_angular_correction :: (A: *Body, B: *Body, dt: float, compliance: float32, angular_error: Vector3) -> force_magnitude: float
{
    C, normal := get_C_and_normal(angular_error);
    w := get_effective_mass_angular_only(A, normal);
    w += get_effective_mass_angular_only(B, normal);

    lambda, force_magnitude := get_lambda(dt, w, C, compliance);
    correction_torque := - lambda * normal;

    delta_angular_A     := get_delta_angular_from_torque(A, correction_torque);
    delta_orientation_A := get_stable_delta_orientation(delta_angular_A, A.orientation);
    A.orientation  = normalize(delta_orientation_A + A.orientation);

    delta_angular_B := get_delta_angular_from_torque(B, - correction_torque);
    delta_orientation_B := get_stable_delta_orientation(delta_angular_B, B.orientation);
    B.orientation = normalize(delta_orientation_B + B.orientation);

    return force_magnitude;
}

apply_angular_velocity_correction :: (A: *Body, B: *Body, dt: float, compliance: float32, angular_error: Vector3) -> force_magnitude: float
{
    C, normal := get_C_and_normal(angular_error);
    w := get_effective_mass_angular_only(A, normal);
    w += get_effective_mass_angular_only(B, normal);

    lambda, force_magnitude := get_lambda(dt, w, C, compliance);
    correction_force := - lambda * normal;

    dq_for_A := get_delta_angular_from_torque(A, correction_force);
    A.angular_velocity += dq_for_A;

    dq_for_B := get_delta_angular_from_torque(B,-correction_force);
    B.angular_velocity += dq_for_B;

    return force_magnitude;
}

//I think this smooths the rotation correction too much, and maybe 2.0 instead of 0.5 is the way, will test with 5 to see what happens, 5 goes crazy, I think even 1.0 goes a little bit crazy on big corrections (highly violated onstraints).
get_stable_delta_orientation :: inline (delta_angular: Vector3, orientation: Quaternion) -> delta_orientation: Quaternion {
    return get_small_angle_rotation_to_delta_quaternion_scaled(delta_angular, orientation, 0.5);
}

get_small_angle_rotation_to_delta_quaternion_scaled :: inline (delta_angular: Vector3, orientation: Quaternion, $stabilize_scale := 1.0) -> delta_orientation: Quaternion {
    // max_angle_rad := 0.5;
    // maxRotationPerSubstep := 0.5;
    // angle := length(delta_angular);
    scale := stabilize_scale;
    // if stabilize_scale*angle > maxRotationPerSubstep
    //   then scale = maxRotationPerSubstep / angle;
    delta_orientation := 0.5*Quaternion.{delta_angular.x,delta_angular.y,delta_angular.z,0.0};
    delta_orientation = delta_orientation*orientation;
    return delta_orientation;
}


apply_local_linear_position_correction :: (using A: *Body, correction: Vector3, unused_world_attach_point: Vector3 = .{}) {
    //do nothing because we are at local space! but maybe I need to do some stuff with center of mass. Most likely not.
}

get_world_linear_position_correction :: inline (inv_mass: float, correction_force: Vector3, unused_world_attach_point: Vector3 = .{}) -> translation_correction: Vector3 {
    return correction_force * inv_mass;
}

get_world_linear_position_correction :: inline (body: Body, correction_force: Vector3, unused_world_attach_point: Vector3 = .{}) -> translation_correction: Vector3 {
    return get_world_linear_position_correction(body.inv_mass, correction_force);
}

get_angular_delta :: (body: Body, correction_force: Vector3, point_of_application: Vector3) -> delta_orientation: Quaternion {
    // This wuld be equation 17 but for quaternions as x
    center_of_mass := body.position;
    world_lever := point_of_application - center_of_mass;
    correction_torque := cross_product(world_lever, correction_force);

    delta_angular := correction_torque;
    delta_angular = rotate(delta_angular, conjugate(body.orientation));
    delta_angular = rotate(delta_angular, (body.orientation));

    draw_primitive.line(center_of_mass, center_of_mass+delta_angular*5000,.{x=0, y=.9951, w=1});

    // scale := stabilize_scale;
    delta_orientation := 0.5*Quaternion.{delta_angular.x,delta_angular.y,delta_angular.z,0.0};
    delta_orientation = delta_orientation*body.orientation;

    return delta_orientation;
}

get_world_angular_position_correction :: (body: Body, correction_force: Vector3, point_of_application: Vector3) -> delta_orientation: Quaternion {
    // This wuld be equation 17 but for quaternions as x
    center_of_mass := body.position;
    world_lever := point_of_application - center_of_mass;
    correction_torque := cross_product(world_lever, correction_force);
    //maybe not: 2 lines
    // w_ang := get_effective_mass_angular_only(body, world_lever);
    // delta_angular := get_delta_angular_from_torque(w_ang, body.orientation, correction_torque);
    delta_angular := get_delta_angular_from_torque(body, correction_torque);

    draw_primitive.line(center_of_mass, center_of_mass+delta_angular*5000,.{x=0, y=.9951, w=1});

    delta_orientation := get_stable_delta_orientation(delta_angular, body.orientation);
    return delta_orientation;
}


//this is from my impulse solver, I only have this here as a reference.
#if 0
update_position_impulse :: (using body: *Body, dt: float)
{
        if inv_mass == 0 then return;
        if shape_kind == .PLANE then return;
        /*
        current_position += current_velocity*dt

        */
        //next linear position, l2w uses this updated position, maybe I should flatten l2w here
        body.position += body.velocity*dt;
        //shared stuff:
        CM_world := l2w(body.position, body.orientation, body.local_center_of_mass); //TODO: flatten out l2w
        lever_arm := body.position - CM_world;

        //angular velocity:
        R3D := rotation_matrix(Matrix3, body.orientation);  //NOTE: moment_of_inertia_world_linear := was already applied
        moment_of_inertia_world := R3D * body.inv_local_moment_of_inertia * transpose(R3D);
        success, inv_I_world := inverse_m3x3(moment_of_inertia_world, EPSILON4);
        #if DEBUG assert(success, " % failed to get inverse moment of inertia in warudo space %! %", body.shape_kind,body.inv_mass, inv_I_world);
        omega := body.angular_velocity;
        alpha := inv_I_world*cross_product(omega, moment_of_inertia_world*omega); //alpha is angular acceleration!
        body.angular_velocity += alpha*dt;
        //angular position:

        angle := length(omega) * dt; //@Check, there is other way of doing this, I will use my way for now

        axis := normalize(omega * dt); //the compiler can figure this out. dont confuse the compiler.
        dq := make_quaternion_from_axis_and_angle(axis, angle);
        body.orientation = (dq)*(body.orientation );
        body.orientation = normalize(body.orientation); //because the above is an approximation since dq is not infty small.

        //I dont get this line: //linear position again?:
        body.position = CM_world + rotate(lever_arm, dq);
}

