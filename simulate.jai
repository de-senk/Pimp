
#program_export
simulate :: () {
    CheckInit();
    old_stack_trace := basic.pack_stack_trace(); // 1 of 2 : maybe needs bug fix? already mailed to jon.
    Remap_Context();
    context.stack_trace = *old_stack_trace[1]; //2 of 2: maybe needs bug fix? already mailed to jon.
    using context.solver;

    //cleanup cache
    for *bodies
    {
       should_delete := it.latest_frame_of_existence != number_of_steps_executed_so_far;
       if should_delete {
        table_remove(*hashed_bodies, it.hash);
        remove it;
       }
    }
    for *constraints
    {
       should_delete := it.latest_frame_of_existence != number_of_steps_executed_so_far;
       if should_delete {
        table_remove(*hashed_constraints, it.hash);
        remove it;
       }
    }

    number_of_steps_executed_so_far += 1;

    // basic.log("-------------");
    // basic.log(" % ",number_of_steps_executed_so_far);
    //
    // Here is the actual algorithm
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
        {
            it.world_velocity += gravity*dt;
            it.previous_world_position = it.world_position;
            it.world_position += it.world_velocity*dt;
            // basic.log("sub_steps dt % :% % %",dt, it.previous_world_position, it.world_velocity, it.world_position);

            basic.assert(is_valid(it.world_position));
            basic.assert(is_valid(it.world_velocity));
        }
    }

    for 1..1 //1..iterations for other kind of solvers
    for *constraints {
        solve(it, dt);// TODO: world_position += correction;
    }

    //this needs to be converted to a "floor constraint"
    // for *bodies
    // {
    //     next_pos := it.world_position + it.world_velocity*backup_dt;
    //     if dot(gravity, next_pos) > 0 && dot(gravity, it.world_velocity) > 0
    //     {
    //         it.world_velocity = -it.world_velocity*it.factor_energy_loss_on_collision;
    //         basic.log("it.world_velocity %",it.world_velocity);
    //         basic.log("next_pos %",next_pos);
    //         basic.log("it.world_position %",it.world_position);
    //         basic.log("gravity %",gravity);
    //     }

    // }

    for *bodies
    {
        basic.assert(is_valid(it.world_position));
        // basic.log("PV: % % %", it.world_velocity, it.world_position , it.previous_world_position);
        it.world_velocity = (it.world_position - it.previous_world_position) / dt;
        // basic.log("AV: % % %", it.world_velocity, it.world_position , it.previous_world_position);
        basic.assert(is_valid(it.world_position));
        basic.assert(is_valid(it.world_velocity));
    }




    }

        //actually apply stuff
    for *bodies
    {
       basic.assert(it.end_frame_callback != null);
       it.end_frame_callback(it);
    }

} @dll_export