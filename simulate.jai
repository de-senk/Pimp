
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
            if it.inv_mass != 0
            {

                //
                // Semi-Implicit Euler integration (3 lines of code)
                //
                //linear
                it.velocity += gravity*dt;
                it.prev_position = it.position;
                it.position += it.velocity*dt;

                //angular usable fields are:  .orientation|.prev_orientation|.angular_velocity|.prev_angular_velocity
                it.prev_orientation = it.orientation;
                dq : Quaternion = ---;
                dq.w = 0.0;
                dq.xyz = 0.5 * it.angular_velocity * dt;
                dq = dq*it.orientation; //this is confusing for me
                it.orientation += 0.5 * dq * dt;
                normalize(*it.orientation); //because the above is an approximation since dq is not infty small.


                //Fake floor, temporary.
                // if it.position.y<0.0 {
                //     it.position = it.prev_position;
                // }
                basic.assert(is_valid(it.position));
                basic.assert(is_valid(it.velocity));
            }
        }


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
        // update linear and angular velocities
        //
        for *bodies
        {
            if it.inv_mass != 0
            {
                //linear
                it.velocity = (it.position - it.prev_position) / dt;
                //angualar
                dq := it.orientation * conjugate(it.prev_orientation);
                it.angular_velocity = 2.0* dq.xyz / dt;
                if dq.w < 0 then it.angular_velocity *= -1.0;

            }
        }

        //
        // Apply velocity dampings, NOTE: I highly doubt that this is the correct way of doing this. BUT this callback strategy allows me to fix it in the future, I hope.
        //
        for *joint: joint_constraints {
             // recalculate_world_attach_pos_and_ori(joint); //maybe I need one of these here?
            joint.apply_linear_damping(joint, dt);
             // recalculate_world_attach_pos_and_ori(joint); //maybe I need one of these here?
            joint.apply_angular_damping(joint, dt);
        }

        //
        // Faked out floor, not perfect! TODO: Collision constraints!
        //
        for *bodies
        {
            next_pos := it.position + it.velocity*backup_dt;
            if dot(gravity, next_pos) > 0 && dot(gravity, it.velocity) > 0
            {
                project_upwards := - dot(it.velocity, normalize(gravity))*normalize(it.velocity);
                //to zero it out:
                it.velocity += project_upwards;
                //to bounce:
                it.velocity += it.factor_energy_loss_on_collision*project_upwards;
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


