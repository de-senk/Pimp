
#program_export
simulate :: () {
    old_stack_trace := b.pack_stack_trace(); // 1 of 2 : maybe needs bug fix? already mailed to jon.
    Remap_Context();
    context.stack_trace = *old_stack_trace[1]; //2 of 2: maybe needs bug fix? already mailed to jon.
    using context.solver;

    number_of_steps_executed_so_far += 1;
    {


    }


} @dll_export