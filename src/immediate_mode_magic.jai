//
// Continuous  Granularity: I focus all my efforts into making this library as granular as possible, without sacrificing performance and code simplicity. It's a hard balance to maintain!
//
KNUTH_GOLDEN_RATIO_64 :: 11400714819323198485;

get_hash :: (loc: Source_Code_Location, identifier: s64) -> u64 {
    Hash :: #import "Hash";
    // Maybe we should hash the filename pointer in order to better prevent collisions.
    name_hash := Hash.djb2_hash(loc.fully_pathed_filename);
    line_hash := Hash.knuth_hash(cast,no_check(u64) loc.line_number);
    identifier_hash := Hash.knuth_hash(cast,no_check(u64) identifier);

    result := combine_hashes(name_hash, combine_hashes(line_hash, identifier_hash));
    return result;
}


combine_hashes :: (a: u64, b: u64) -> u64 #no_aoc {
    return a * KNUTH_GOLDEN_RATIO_64 + b;
}

//
// Layer_0: Highest Exposed level of abstraction procedures goes here, 0 stands for that, is the layer I expect the users to use the most, its the first element they should use when prototyping or releasing game.
//

Linear_Compliance :: struct {
    GOMU_GOMU :: 0.5;
    //maybe dont use values above 0.1? they are really weak springs
    NOT_SO_GOMU :: 0.1;
    TARZAN_VINE :: 0.01; //maybe weak spring is a better name
    WEAK_SPRING :: 0.01; //maybe weak spring is a better name
    MEDIUM_SPRING :: 0.001;
    STRONG_SPRING :: 0.0001;
    STEEL_ROD :: 0.0;
}


doHook :: (body: *Body, world_attach_point: Vector3, target_length: float = -1, linear_compliance := Linear_Compliance.WEAK_SPRING, identifier:s64=0, loc := #caller_location) {
    CheckInit();
    if target_length < 0 then target_length = distance(body.position, world_attach_point);
    hash := get_hash(loc, identifier);
    // hook := find_or_create_hook(hash, body, world_attach_point, target_length);
    hook , did_just_created_a_new_one := find_or_create_joint_constraint(hash, A=body);

    //update hook with relevant data for this new frame
    if did_just_created_a_new_one //just comment out this line
    {
        hook.solve_position = solve_position_for_distance_constraint_cb; //maybe redundant! but whatever
        hook.solve_position = solve_position_for_one_way_distance_constraint_cb; //maybe redundant! but whatever
        hook.target_length = target_length;
        basic.log("world_attach_point %", world_attach_point);
        hook.linear_compliance = basic.clamp(linear_compliance, 0.0, 1.0);

        hook.local_attach_position_A = .{0,1.5,1};
        hook.local_attach_position_B = world_attach_point;

        //cpmmented out debug code to be deleted
        // midgard := 0.5*(world_attach_point + body.position);
        // dir := normalize(midgard - body.position);
        // set_from_axis_and_angle(*hook.local_attach_orientation_A, dir, 0.9);
        // set_from_axis_and_angle(*hook.local_attach_orientation_B, .{1,0,0}, 1.3);


    }
        dir := world_attach_point - body.position;

        if length_squared(dir) > 1 {
            dir_n := normalize(dir);
            // hook.local_attach_position_A = w2l_rotate_only(hook.A.position, hook.A.orientation, dir_n);
        }
        // hook.local_attach_position_B = world_attach_point;
        // hook.local_attach_position_B = world_attach_point - body.position;

    basic.assert(hook!=null);
    hook.latest_frame_of_existence = context.solver.number_of_steps_executed_so_far;
}


//same as doHook but 1 line of code less, maybe I should do a masterHook and make a lot of procedures using #bake_arguments, BUT. that is code complexity...
doHookDontUpdate :: (body: *Body, world_attach_point: Vector3,  target_length: float = -1, linear_compliance := Linear_Compliance.WEAK_SPRING, identifier: s64 = 0, loc := #caller_location) {
    CheckInit();
    if target_length < 0 then target_length = distance(body.position, world_attach_point);
    hash := get_hash(loc, identifier);

    hook, did_just_created_a_new_one := find_or_create_joint_constraint(hash, A=body);

    //update hook with relevant data for this new frame
    if did_just_created_a_new_one
    {
        hook.solve_position = solve_position_for_distance_constraint_cb;
        hook.solve_position = solve_position_for_one_way_distance_constraint_cb;
        hook.target_length = target_length;
        hook.local_attach_position_A = .{};
        hook.local_attach_position_B = world_attach_point;
        basic.log("world_attach_point %", world_attach_point);
        hook.linear_compliance = basic.clamp(linear_compliance, 0.0, 1.0);

            dir := world_attach_point - body.position;
            dir_n := normalize(dir);
            hook.local_attach_position_A = dir_n;
    }


    dir := hook.world_attach_position_B - body.position ;
    if length_squared(dir) > 1 {
        dir_n := normalize(dir);
        hook.local_attach_position_A = w2l_rotate_only(hook.A.position, hook.A.orientation, dir_n);

    }
    basic.assert(hook!=null);
    hook.latest_frame_of_existence = context.solver.number_of_steps_executed_so_far;
}

doWeld :: (body: *Body, world_attach_point: Vector3, weld_relative_axis: Vector3, rest_angle_radians: float = 0, target_length: float = -1, identifier:s64=0, loc := #caller_location) {
    CheckInit();
    if target_length < 0 then target_length = distance(body.position, world_attach_point);
    hash := get_hash(loc, identifier);
    weld := find_or_create_weld(hash, body, world_attach_point, target_length);

    //update weld with relevant data for this new frame
    {
        // constraint.world_attach_position_A = body.position;
        weld.world_attach_position_A = body.position;
        weld.world_attach_position_B = world_attach_point;

        // midgard := 0.5*(world_attach_point + body.position);
        // dir := normalize(midgard - body.position);
        set_from_axis_and_angle(*weld.local_attach_orientation_A, weld_relative_axis, rest_angle_radians);
        // set_from_axis_and_angle(*weld.local_attach_orientation_B, .{1,0,0}, 1.3);
    }

    basic.assert(weld!=null);
    weld.latest_frame_of_existence = context.solver.number_of_steps_executed_so_far;
}

#if 0
doDoor :: (body: *Body,  world_hinge_attach_position: Vector3, world_hinge_axis: Vector3 = physics_coordinates.up, min_angle_radians: float = 0.0*DEG_TO_RAD, max_angle_radians: float = 120.0*DEG_TO_RAD, reverse_opening_direction: bool=false, identifier:s64=0, loc := #caller_location) {
    CheckInit();
    // if target_length < 0 then target_length = distance(body.position, world_attach_point);
    hash := get_hash(loc, identifier);
    weld := find_or_create_weld(hash, body, world_attach_point, target_length);

    //update weld with relevant data for this new frame
    {
     weld.world_attach_position_A = body.position;
     weld.world_attach_position_B = world_attach_point;

     // midgard := 0.5*(world_attach_point + body.position);
     // dir := normalize(midgard - body.position);
     set_from_axis_and_angle(*weld.local_attach_orientation_A, weld_relative_axis, rest_angle_radians);
     // set_from_axis_and_angle(*weld.local_attach_orientation_B, .{1,0,0}, 1.3);
    }

    basic.assert(weld!=null);
    weld.latest_frame_of_existence = context.solver.number_of_steps_executed_so_far;
}

doBall :: (transform: *Matrix4, radius: float32 = 0.1, mass: float32 = 1.1, identifier:s64=0, loc := #caller_location) -> *Body {
    CheckInit();
    ball_position := get_translation(transform);
    hash := get_hash(loc, identifier);
    ball := find_or_create_ball(hash, ball_position, radius, mass);
    basic.assert(ball!=null);
    ball.latest_frame_of_existence = context.solver.number_of_steps_executed_so_far;
    ball.position = ball_position;
    ball.prev_position = ball_position;
    ball.user_data = transform;
    reflect_results_callback :: (body: *Body) {
        transform := cast(*Matrix4)body.user_data;
        previous_t, previous_q, previous_s := koda_decompose_transform(transform);

        delta_pos := body.position - previous_t;

        next_q := body.orientation;
        next_q_minus_previous := next_q*conjugate(previous_q);

        MODEL := Matrix4_Identity;
        MODEL = translate(MODEL, previous_t + delta_pos);
        MODEL = rotate(MODEL, normalize(next_q_minus_previous*previous_q));
        transform.* = MODEL;

    }
    ball.end_frame_callback = reflect_results_callback;
    return ball;
}



//
// Here goes the procedures that the Layer_0 use.
//
//NOTE(to myself): :tagReadTheEnd maybe, I should make a typechecked way of adding stuff here and implementing code in the solver OR do a general abstraction in the code directly. Because, there is a high probability that if I modify or add stuff here that I will forget that in order for it to work, I need to modify other code that should be in sync with code here. :tagReadTheEnd

//
// Layer_1: Retained mode exposed procedures: Im not fully supporting retained mode yet! I prefer to support Layer_0 first. BUT, layer_0 uses layer_1.
//

//these create_[something] are also exposed when the user wants to use a retained mode feature,
//NOTE: It is still so early, I did not implemented all remove() and disable() procedures that the user has to manually call.

//maybe spiderman
create_hook :: (body: *Body, world_attach_point: Vector3, target_length: float) -> *Joint_Constraint {
    constraint := create_joint_constraint(A=body);
    constraint.target_length = target_length;
    constraint.linear_compliance = 0.01;
    constraint.linearDampingCoeff = 0.0;

    constraint.solve_position = solve_position_for_one_way_distance_constraint_cb;
    // constraint.solve_orientation = ; use default callback that does nothing
     constraint.local_attach_position_A = .{};
     // constraint.world_attach_position_A = body.position;
     constraint.local_attach_position_B = world_attach_point;
     // constraint.world_attach_position_B = world_attach_point;
    return constraint;
}

create_steel_rod :: (body: *Body, world_attach_point: Vector3, target_length: float) -> *Joint_Constraint {
    constraint := create_joint_constraint(A=body);
    constraint.target_length = target_length;
    constraint.linear_compliance = 0.0;
    constraint.linearDampingCoeff = 0.0;

    constraint.solve_position = solve_position_for_distance_constraint_cb;
    // constraint.solve_orientation = ; //Maybe add rotation solver? steel "rod" tho. so you might need a triangle of steel rods if you want orientation rigidity (or a quad) TODO(tr): add a cool example usign steel rods.
     constraint.local_attach_position_A = .{};
     // constraint.world_attach_position_A = body.position;
     constraint.local_attach_position_B = world_attach_point;
     // constraint.world_attach_position_B = world_attach_point;
    return constraint;
}

create_weld :: (body: *Body, world_attach_point: Vector3, target_length: float) -> *Joint_Constraint {
    constraint := create_joint_constraint(A=body);
    constraint.target_length = target_length;
    constraint.linear_compliance = 0.0;
    constraint.linearDampingCoeff = 0.0;

    constraint.solve_position = solve_position_for_fixed_constraint_cb;
    constraint.solve_orientation = solve_orientation_for_fixed_constraint_cb;
     constraint.local_attach_position_A = .{};
     // constraint.world_attach_position_A = body.position;
     constraint.local_attach_position_B = world_attach_point;
     // constraint.world_attach_position_B = world_attach_point;
    return constraint;
}


create_ball :: (position: Vector3, radius: float32, mass: float32) -> *Body {
    body := create_body();
    body.shape_kind = .SPHERE;
    body.prev_position = position;
    body.position = position;
    body.radius = radius;
    if mass > 0 {
        body.inv_mass = 1/mass;
        body.inv_local_moment_of_inertia = .{1/mass, 1/mass, 1/mass};
        body.inv_local_moment_of_inertia *= 5.0/2.0; //DENSE SPHERE, its not 2/5 because is inverse of ...
    }
    return body;
}


//
// Layer_1_Auxiliary: create rigidbodys and constraints kinds
//
create_joint_constraint :: (A: *Body, B: *Body = null) -> *Joint_Constraint {
    constraint_ptr, bucket_locator := find_and_occupy_empty_slot(*context.solver.world.joint_constraints);
    constraint_ptr.bucket_locator = bucket_locator;
    constraint_ptr.A = A;
    constraint_ptr.B = B; //if B is null, it means that we attach it to the WORLD itself.
    return constraint_ptr;
}

create_body :: () -> *Body {
    body_ptr, bucket_locator := find_and_occupy_empty_slot(*context.solver.world.bodies);
    body_ptr.bucket_locator = bucket_locator;
    return body_ptr;
}


//
// Layer_0_Auxiliary: Find Or create, if hash not found, create one constraint.
//
//NOTE FOR READER: Im sorry about the triple and quadruple levels of indirections (of code jumpings to get you to code that does something)!
//I know one complicated way of abstracting these :tagComplicatedWayOfReducingLinesOfCode but idk if people would find it easy to follow, will see.
// find_or_create_hook :: (hash: u64, body: *Body, world_attach_point: Vector3, target_length: float ) -> *Joint_Constraint {
find_or_create_hook :: (hash: u64, body: *Body, world_attach_point: Vector3, target_length: float ) -> *Joint_Constraint, created_one_this_frame: bool {
    using context.solver.world;
    did_just_created_a_new_one: bool;
    locator_ptr := table_find_pointer(*hashed_joint_constraints, hash);

    hook_ptr: *Joint_Constraint;
    if locator_ptr {
        // basic.log("Cache detected!");
        hook_ptr = bucket_array_find_pointer(*joint_constraints, locator_ptr.*);
    }

    if !hook_ptr {
        basic.log("creating new Hook cosntraint");
        did_just_created_a_new_one = true;
        hook_ptr = create_hook(body, world_attach_point, target_length);
        hook_ptr.hash = hash;
        table_set(*hashed_joint_constraints, hash, hook_ptr.bucket_locator);
     }
    return hook_ptr, did_just_created_a_new_one;
}


find_or_create_joint_constraint :: (hash: u64, A: *Body, B: *Body = null ) -> *Joint_Constraint, did_just_created_a_new_one: bool {
    using context.solver.world;
    locator_ptr := table_find_pointer(*hashed_joint_constraints, hash);
    joint_ptr: *Joint_Constraint;
    if locator_ptr {
        // basic.log("Cache detected!");
        joint_ptr = bucket_array_find_pointer(*joint_constraints, locator_ptr.*);
    }

    did_just_created_a_new_one: bool;

    if !joint_ptr {
        did_just_created_a_new_one = true;
        basic.log("creating new cosntraint %", hash);
        joint_ptr = create_joint_constraint(A, B);
        joint_ptr.hash = hash;
        table_set(*hashed_joint_constraints, hash, joint_ptr.bucket_locator);
     }
    return joint_ptr, did_just_created_a_new_one;
}

/* //START OF:tagComplicatedWayOfReducingLinesOfCode

the problem is that thse parameters might vary depending on procedure idea
find_or_create_hook :: (hash: u64, body: *Body, world_attach_point: Vector3, target_length: float ) -> *Joint_Constraint {
find_or_create_hook :: (hash: u64, body: *Body, world_attach_point: Vector3, target_length: float ) -> *Joint_Constraint {
find_or_create_door :: (hash: u64, body: *Body, world_attach_point: Vector3, target_length: float ) -> *Joint_Constraint {



*/ //END OF :tagComplicatedWayOfReducingLinesOfCode

find_or_create_weld :: (hash: u64, body: *Body, world_attach_point: Vector3, target_length: float ) -> *Joint_Constraint {
    using context.solver.world;
    locator_ptr := table_find_pointer(*hashed_joint_constraints, hash);

    hook_ptr: *Joint_Constraint;
    if locator_ptr {
        // basic.log("Cache detected!");
        hook_ptr = bucket_array_find_pointer(*joint_constraints, locator_ptr.*);
    }

    if !hook_ptr {
        basic.log("creating new Weld cosntraint");
        hook_ptr = create_weld(body, world_attach_point, target_length);
        hook_ptr.hash = hash;
        table_set(*hashed_joint_constraints, hash, hook_ptr.bucket_locator);
     }
    return hook_ptr;
}



find_or_create_ball :: (hash: u64, ball_position: Vector3, radius: float, mass: float32) -> *Body {
    using context.solver.world;
    locator_ptr := table_find_pointer(*hashed_bodies, hash);

    ball: *Body;
    if locator_ptr {
        // basic.log("Cache detected!");
        ball = bucket_array_find_pointer(*bodies,locator_ptr.*);
    }

    if !ball {
        basic.log("creating new ball");
        ball = create_ball(ball_position, radius, mass);
        ball.hash = hash;
        table_set(*hashed_bodies, hash, ball.bucket_locator);
     }

    return ball;
}
