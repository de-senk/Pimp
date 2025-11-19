
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


