
#program_export
simulate :: () {
    CheckInit();
    old_stack_trace := basic.pack_stack_trace(); // 1 of 2 : maybe needs bug fix? already mailed to jon.
    Remap_Context();
    context.stack_trace = *old_stack_trace[1]; //2 of 2: maybe needs bug fix? already mailed to jon.
    using context.solver;

    number_of_steps_executed_so_far += 1;
    {
    for *bodies
    {
        if it.world_position.y > 0
        {
            it.world_velocity += gravity*dt;
        }
    }

    for *bodies
    {
        next_pos := it.world_position + it.world_velocity*dt;
        if dot(gravity, next_pos) > 0 && dot(gravity, it.world_velocity) > 0
        {
            it.world_velocity = -it.world_velocity*it.factor_energy_loss_on_collision;
            basic.log("it.world_velocity %",it.world_velocity);
            basic.log("next_pos %",next_pos);
            basic.log("it.world_position %",it.world_position);
            basic.log("gravity %",gravity);
        }

    }

    for *bodies
    {
       it.world_position += it.world_velocity*dt;
    }

    for *bodies
    {
       basic.assert(it.end_frame_callback != null);
       it.end_frame_callback(it);
    }


    }

} @dll_export