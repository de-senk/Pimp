
#program_export
simulate :: () {
    CheckInit();
    old_stack_trace := basic.pack_stack_trace(); // 1 of 2 : maybe needs bug fix? already mailed to jon.
    Remap_Context();
    context.stack_trace = *old_stack_trace[1]; //2 of 2: maybe needs bug fix? already mailed to jon.
    using context.solver;

    for *soft_bodies
    {
        if !it.is_initialized init(it);
    }
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

            it.velocity += gravity*dt;
            it.prev_position = it.position;
            it.position += it.velocity*dt;
            // basic.log("sub_steps dt % :% % %",dt, it.prev_position, it.velocity, it.position);
            //fake floor?
            if it.position.y<0.0 {
                it.position = it.prev_position;
            }
            basic.assert(is_valid(it.position));
            basic.assert(is_valid(it.velocity));
        }
    }
    if soft_bodies[0].should_try_solve {
    for *soft_bodies preSolve(it,dt, gravity);
    for *soft_bodies solve(it,dt);
    for *soft_bodies postSolve(it,dt);
    }

    for 1..1 //1..iterations for other kind of solvers
    for *constraints {
        solve(it, dt);// TODO: position += correction;
    }

    //this needs to be converted to a "floor constraint"
    // for *bodies
    // {
    //     next_pos := it.position + it.velocity*backup_dt;
    //     if dot(gravity, next_pos) > 0 && dot(gravity, it.velocity) > 0
    //     {
    //         it.velocity = -it.velocity*it.factor_energy_loss_on_collision;
    //         basic.log("it.velocity %",it.velocity);
    //         basic.log("next_pos %",next_pos);
    //         basic.log("it.position %",it.position);
    //         basic.log("gravity %",gravity);
    //     }

    // }

    for *bodies
    {
        if it.inv_mass != 0
        {
        // basic.assert(is_valid(it.position));
        // basic.log("PV: % % %", it.velocity, it.position , it.prev_position);
        it.velocity = (it.position - it.prev_position) / dt;
        // basic.log("AV: % % %", it.velocity, it.position , it.prev_position);
        // basic.assert(is_valid(it.position));
        // basic.assert(is_valid(it.velocity));
        }
    }
    //updateMeshes();




    }

        //actually apply stuff
    for *bodies
    {
       basic.assert(it.end_frame_callback != null);
       it.end_frame_callback(it);
    }

} @dll_export