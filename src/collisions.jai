//depends on Collision_Detection module.

//another name ->  Collision_Constraint :: Collision_Constraint; //manifold is a collection of points (we need only 4 or less points for good stability for all shapes)
Collision_Constraint :: struct {
    A: *Body;
    B: *Body;
    bucket_locator: Bucket_Locator;

    //for immediate mode
    latest_frame_of_existence: u64;
    hash: u64;

    disabled: bool;

    contactsBuffer: [4]Contact_Info;
    contacts_count: int;

    linear_compliance := 0.0;
}

solve_position_for_collision_constraint_cb :: (using colli: *Collision_Constraint, dt: float) {
    //What I will probabli do ater: iterate over contacts, and create 1 correction arrow and 1 angular correction arrow

    for 0..contacts_count-1 {
            //unused for now are 1 tab
        world_point_A := world_point_A[it];
        world_point_B := world_point_B[it];
            local_point_A := local_point_A[it];
            local_point_B := local_point_B[it];
        world_normal_towards_A := world_normal_towards_A[it];

            time_of_impact := time_of_impact[it];
            feature := feature[it];
            world_midpoint := world_midpoint[it];

        error := penetration*world_normal_towards_A;
        if (penetration > -COLLISION_MARGIN) then return;
        correction_arrow := penetration*world_normal_towards_A;
        apply_world_position_correction(A, B, dt, linear_compliance, correction_arrow, world_point_A, world_point_B);

    }

}


AddManifold :: ( A: *Body, B: *Body, contact_stack_lifetime: []Contact_Info) -> *Collision_Constraint {
    using context.solver;
    #if DEBUG basic.assert(contact_stack_lifetime.count != 0, "You tried to create a manifold with 0 contact points, is that ok?");
    locator:, manifold := bucket_array_add(*context.solver.collision_constraints,.{});
    manifold.bucket_locator = locator;

    manifold.A = A;
    manifold.B = B;


    for it_index: 0..3
    {
        // manifold.contactsBuffer[it_index].A = A; //<-this is the hotfix :tagOtherwiseWeNullDerefHere
        // manifold.contactsBuffer[it_index].B = B; //<-this is the hotfix :tagOtherwiseWeNullDerefHere
    }
    UpdateManifold(manifold, contact_stack_lifetime);
    //hmm, well we should never be able to "grab/select" penetration constraints..
    // if A array_add(*manifold.A.local_constraints, manifold);
    // if B array_add(*manifold.B.local_constraints, manifold);
    // // manifold.local_normal_A = 2*normal;


    return manifold;
}


UpdateManifold :: (using manifold: *Collision_Constraint, contact_stack_lifetime: []Contact_Info) {

for contact_stack_lifetime {
        #if DEBUG basic.assert(is_normalized(it.world_normal_towards_A , tolerance=FLOAT32_EPSILON5), "whaat  %", it.world_normal_towards_A );
        manifold.contactsBuffer[it_index].world_contact_point_on_A = it.world_contact_point_on_A;
        manifold.contactsBuffer[it_index].world_contact_point_on_B = it.world_contact_point_on_B;
        manifold.contactsBuffer[it_index].world_normal_towards_A = it.world_normal_towards_A;
    }
    manifold.contacts_count = contact_stack_lifetime.count;
}


broad_phase :: (broad_strategy: enum{SWEEP_AND_PRUNE;}) { // grep BroadPhase ::
    using context.solver;
    basic.array_reset_keeping_memory(*collision_pairs); //we reset here so we can stop time and draw these without problems
    //stragety 1
    SweepAndPrune1D(*collision_pairs);
}


SweepAndPrune1D :: (final_pairs: *[..]Collision_Pair ) {
    using context.solver;
    sorted_bodies: [..]Pseudo_Body;
    sorted_bodies.allocator = basic.temp;
    worst_case_count := bodies.count*2 /* mul by 2 because pairs! */;
    maybe_reserve(*sorted_bodies, worst_case_count);

    //sort the mfs
    {
        axis_to_project_aabbs := #run normalize(Vector3.{1,1,1});
        for * bodies {
            aabb_copy := it.aabb;
            #if DEBUG basic.assert(is_aabb_valid(aabb_copy), "aabb is invalid %", aabb_copy);
            maybe_expand_aabb(*aabb_copy, aabb_copy.min + it.velocity*dt);
            maybe_expand_aabb(*aabb_copy, aabb_copy.max + it.velocity*dt);
            maybe_expand_aabb(*aabb_copy, aabb_copy.min - COLLISION_MARGIN*axis_to_project_aabbs); //I think using the same collision margin for everything will get me in trouble in the future.
            maybe_expand_aabb(*aabb_copy, aabb_copy.max + COLLISION_MARGIN*axis_to_project_aabbs);

            pA := basic.array_add(*sorted_bodies);
            pA.bucket_locator = it.bucket_locator;
            pA.value = dot(aabb_copy.min, axis_to_project_aabbs);
            pA.ismin = true;

            pB := basic.array_add(*sorted_bodies);
            pB.bucket_locator = it.bucket_locator;
            pB.value = dot(aabb_copy.max, axis_to_project_aabbs);
            pB.ismin = false;

            // sorted_bodies[it_index*2 + 0].bucket_locator = it.bucket_locator;
            // sorted_bodies[it_index*2 + 0].value = dot(aabb_copy.min, axis_to_project_aabbs);
            // sorted_bodies[it_index*2 + 0].ismin = true;
            // sorted_bodies[it_index*2 + 1].bucket_locator = it.bucket_locator;
            // sorted_bodies[it_index*2 + 1].value = dot(aabb_copy.max, axis_to_project_aabbs);
            // sorted_bodies[it_index*2 + 1].ismin = false;

        }
        quick_sort(sorted_bodies, CompareSAP);
    }

    //pair the mfs
    {
        basic.array_reset_keeping_memory(final_pairs);
        for a: sorted_bodies
        {
            if !a.ismin then continue;
            pair: Collision_Pair;
            pair.a = a.bucket_locator;

            //TODO: rewrite all this crap and I hate using a b everywhere
            for j: it_index + 1 .. sorted_bodies.count - 1
            {
                b := sorted_bodies[j];
                if b.bucket_locator == a.bucket_locator then break;
                if !b.ismin then continue;
                pair.b = b.bucket_locator;
                basic.array_add(final_pairs, pair);
            }
        }
    }
}

narrow_phase :: ()
{
    already_constrained_pair :: (A: *Body, B: *Body) -> bool
    {
        // add_to_array_of_contact_points((A.position), radii = length(A.size)/2);
        // add_to_array_of_contact_points((B.position), radii = length(B.size)/2);
        for context.solver.collision_constraints
        {
            // if it.kind == .IGNORE_COLLISION then return true;
            are_a_and_b_constrained := (it.A == A && it.B == B) || (it.A == B && it.B == A);
            if are_a_and_b_constrained
            // && it.kind == .COLLISION
            {
             return true;
            }

        }
        return false;
    }

    using context.solver;
    for collision_pairs
    {
        A :*Body= bucket_array_find_pointer(*bodies, it.a);
        B :          = bucket_array_find_pointer(*bodies, it.b);


        #if DEBUG basic.assert(A != null);
        #if DEBUG basic.assert(B != null);

        should_swap_order_by_shape := A.shape_kind > B.shape_kind;

        are_same_shape := A.shape_kind == B.shape_kind;
        if are_same_shape then should_swap_order_by_shape = A.(u64) < B.(u64);
        if should_swap_order_by_shape  A, B = B,A;

        //maybe can be abstracted into a procedure, compressed* into a procedure, the problem is finding a good name for this
        {
            if A.inv_mass == 0 && B.inv_mass == 0 then continue; //otherwise we trigger bad collisions and crash stuff

            old_manifold: *Collision_Constraint; //pointer, why not?
            are_already_constrained: bool;
            {////WRONG ASSUMPTION, what I was thinking? 5+5 == 4+6 == 3 + 7 == 10 ---> WRONG ---> maybe make a hash table pointer A + pointer B as keys, since A + B == B + A, we remove duplicates for free

                //@Slow read the above comments for an idea in how to optimize this
                for *collision_constraints
                {
                    // log("are_already_constrained % -- %", it.A.(u64) + it.B.(u64), A.(u64) + B.(u64));
                    c1 := it.A == A && it.B == B;
                    c2 := it.A == B && it.B == A;
                    if c1 || c2
                    {
                        // log("are_already_constrained");
                        are_already_constrained = true;
                        old_manifold = it;
                        #if DEBUG basic.assert(it.contacts_count != 0, "% how we have allocated a manifold with no contacts? you need to check all manifolds before this and remove the ones with no contacts or something", it.contacts_count);

                    }
                }

            }
            new_contacts_count, new_contacts := compute_contacts(A, B); //@Maybe should we early return false?

            // are_already_constrained := already_constrained_pair(A,B);


            should_try_create_manifold := !are_already_constrained;
            should_compare_with_old    := are_already_constrained;

            //compare to old contacts, and keep the data if they satisfy some conditions
            if should_compare_with_old
            {
                //conditions to hold:
                //1) previous contact pos tangential difference is smaller than threshold
                //2)

                //idk man
                if new_contacts_count != old_manifold.contacts_count
                {
                    //TODO:
                }

                    // if new_contacts_count >= old_manifold.contacts_count
                    {

                    used := bool.[false, false,false,false];
                    for new_idx: 0.. new_contacts_count - 1
                    {
                        if new_contacts[new_idx].feature.value == 0 then basic.log("this is BAD, you are not creating an local UID(feature.value and feature.kind) for contact points! %\n ",old_manifold.contactsBuffer[new_idx].feature );
                        for old_jdx: 0.. old_manifold.contacts_count - 1
                        {
                            if used[old_jdx] == true then continue;
                            is_same_feature := new_contacts[new_idx].feature.value == old_manifold.contactsBuffer[old_jdx].feature.value;
                            if is_same_feature
                            {
                                old_pointA := old_manifold.contactsBuffer[old_jdx].world_contact_point_on_A;
                                old_pointB := old_manifold.contactsBuffer[old_jdx].world_contact_point_on_B;
                                new_pointA := new_contacts[old_jdx].world_contact_point_on_A;
                                new_pointB := new_contacts[old_jdx].world_contact_point_on_B;

                                l1 := length_squared(new_pointA - old_pointA);
                                l2 := length_squared(new_pointB - old_pointB);
                                contacts_are_too_far := l1 > 0.5 || l2 > 0.5;
                                if !contacts_are_too_far
                                {
                                    used[old_jdx] = true;
                                }

                               //TODO(tr): STATICK FRICTION / DYNAMIC FRICTION

                            }
                        }

                    // old_manifold.contacts_count = new_contacts_count;
                    old_manifold.contacts_count = 0;
                    for new_idx: 0.. new_contacts_count - 1
                    {
                        if used[new_idx]
                        {
                            old_manifold.contacts_count += 1;
                        }else {
                            old_manifold.contacts_count += 1;
                          // old_manifold.contactsBuffer[new_idx] = new_contacts[new_idx];
                        }
                    }



                    }

                }
                //@dontdelete
                //sanity @speed but this is super fast anyways @hack too, meh I h should comment this and dont delete the comments and use the tag to grep.
                // for 0..new_contacts_count-1 {
                //             old_manifold.contactsBuffer[idx].A = A; //:tagOtherwiseWeNullDerefHere
                //             old_manifold.contactsBuffer[idx].B = B; //:tagOtherwiseWeNullDerefHere
                // }

                // old_manifold.contactsBuffer = new_contacts;
                {

                contact_stack_lifetime: []Contact_Info = new_contacts;
                    cter := 0;
                     for 0..new_contacts_count-1
                        {

                            contact := new_contacts[it];
                            if !is_normalized(contact.world_normal_towards_A) then continue; //TODO: figure out why the first contact in history is normal 0
                            cter += 1;
                            #if DEBUG basic.assert(is_normalized(contact.world_normal_towards_A , tolerance=FLOAT32_EPSILON5), "weeaat  %", contact.world_normal_towards_A );

                            //for debugging purpusese you can use this: instead of addmanifold
                            // AddPenetration(contact.A, contact.B, contact.local_point_A, contact.local_point_B, contact.world_normal_towards_A);
                        }
                    contact_stack_lifetime.count = cter;
                    UpdateManifold(old_manifold, contact_stack_lifetime);
                }
                for it_index: 0..new_contacts_count -1
    //             #if DEBUG assert(is_normalized(old_manifold.contactsBuffer[it_index].local_normal_A , tolerance=FLOAT32_EPSILON5), "aaa  %", old_manifold.contactsBuffer[it_index].local_normal_A );
                for it_index: 0..new_contacts_count -1 {
                #if DEBUG basic.assert(is_normalized(new_contacts[it_index].world_normal_towards_A , tolerance=FLOAT32_EPSILON5), "ccc  %", new_contacts[it_index].world_normal_towards_A );
    //             #if DEBUG assert(is_normalized(old_manifold.contactsBuffer[it_index].local_normal_A , tolerance=FLOAT32_EPSILON5), "ccc  %", old_manifold.contactsBuffer[it_index].local_normal_A );
                }


            } //else

            if should_try_create_manifold
            {
                found_a_ballistic_contact: bool;
                best_ballistic_contact_index: int;
                best_ballistic_contact_toi: float = FLOAT32_INFINITY;;
                for c_idx:0..new_contacts_count-1
                {
                    contact := new_contacts[c_idx];
                    if contact.time_of_impact > 0.0
                    {
                        // #if DEBUG assert(contact.A != null);
                        // #if DEBUG assert(contact.B != null);

                        found_a_ballistic_contact = true;
                        if contact.time_of_impact < best_ballistic_contact_toi
                        {
                            best_ballistic_contact_toi = contact.time_of_impact;
                            best_ballistic_contact_index = c_idx;

                        }

                    }
                }

                if found_a_ballistic_contact
                {
                    best_contact := new_contacts[best_ballistic_contact_index];
                    basic.array_add(*context.solver.possible_collisions, best_contact); //add to CCD list (ballisitc solver) //maybe use arena instead because this is reseted per frame//so we basically save a backup of the computed data to check if between 0 and dt we have a collision.
                    // log("BALLISIT CONTACT ADDED");
                }
                else
                {
                    contact_stack_lifetime:[]Contact_Info = new_contacts;
                    cter := 0;
                    for 0..new_contacts_count-1
                    {
                        contact := new_contacts[it];
                        if !is_normalized(contact.world_normal_towards_A) then continue; //TODO: figure out why the first contact in history is normal 0
                        cter += 1;
                        //for debugging purpusese you can use this: instead of addmanifold
                        // AddPenetration(contact.A, contact.B, contact.local_point_A, contact.local_point_B, contact.world_normal_towards_A);
                    }
                    contact_stack_lifetime_view: []Contact_Info = contact_stack_lifetime;;
                    contact_stack_lifetime_view.count = cter;
                    //[IMPORTANT] NOTE: when debugging, if you are not removing contacts, and you iterated with 0 contacts, then above booleans will trigger and this code gets executed once, why im writtin this? because I wasted 5 minutes of my life not knowing why contacts suddenly stopped working
                    if cter != 0 {

                        ptr_manifold := AddManifold(A, B, contact_stack_lifetime_view);
                        #if DEBUG basic.assert(ptr_manifold.contacts_count != 0, "GOW?????????");

                        }
                }
            } //if !should_compare_with_old end
        } //block block



    } //for collision pairs


}

/////////////////
//Im not happy about these  but whatever for now.
fill_contact_data_for_wraps :: (
    contact_data: *Contact_Info,
    penetration: float,
    world_normal_towards_A: Vector3,
    world_midpoint: Vector3,
    world_contact_point_on_A: Vector3,
    world_contact_point_on_B: Vector3
) {
    contact_data.penetration = penetration;
    contact_data.world_midpoint = world_midpoint;
    contact_data.world_contact_point_on_A = world_contact_point_on_A;
    contact_data.world_contact_point_on_B = world_contact_point_on_B;
    contact_data.world_normal_towards_A = world_normal_towards_A;

}
wrap_collide_sphere_vs_sphere :: (A: *Body, B: *Body) -> contacts_count:int, contacts_data: [4]Contact_Info{
    num_contacts,
    penetration,
    world_normal_towards_A,
    world_midpoint,
    world_contact_point_on_A,
    world_contact_point_on_B := collide_sphere_vs_sphere(A.position, A.radius, B.position, B.radius);

    //setting up return data
    contacts_count:=num_contacts;
    contacts_data : [4]Contact_Info;
    for 0..contacts_count-1 {
        fill_contact_data_for_wraps(*contacts_data[0],penetration, world_normal_towards_A, world_midpoint, world_contact_point_on_A, world_contact_point_on_B);
    }
    return contacts_count, contacts_data;

}
wrap_collide_sphere_vs_capsule:: (A: *Body, B: *Body) -> contacts_count:int, contacts_data: [4]Contact_Info{
    capsule_start_B := B.position;
    capsule_end_B := capsule_start_B + B.capsule_height*rotate(Vector3.{0,1,0}, B.orientation);

    num_contacts, penetration, world_normal_towards_A, world_midpoint, world_contact_point_on_A, world_contact_point_on_B
    := collide_sphere_vs_capsule(A.position, A.radius, capsule_start_B ,capsule_end_B, B.radius);

    contacts_count:=num_contacts;
    contacts_data: [4]Contact_Info;

    for 0..contacts_count-1 {
    fill_contact_data_for_wraps(*contacts_data[it],penetration, world_normal_towards_A, world_midpoint, world_contact_point_on_A, world_contact_point_on_B);
    }

    return contacts_count, contacts_data;
}
wrap_collide_capsule_vs_capsule:: (A: *Body, B: *Body) -> contacts_count:int, contacts_data: [4]Contact_Info{
    capsule_start_A := A.position;
    capsule_end_A := capsule_start_A + A.capsule_height*rotate(Vector3.{0,1,0}, A.orientation);

    capsule_start_B := B.position;
    capsule_end_B := capsule_start_B + B.capsule_height*rotate(Vector3.{0,1,0}, B.orientation);

    num_contacts, penetration, world_normal_towards_A, world_midpoint, world_contact_point_on_A, world_contact_point_on_B
    := collide_capsule_vs_capsule(capsule_start_A, capsule_end_A, A.radius, capsule_start_B, capsule_end_B, B.radius);

    contacts_count:=num_contacts;
    contacts_data: [4]Contact_Info;

    for 0..contacts_count-1 {
    fill_contact_data_for_wraps(*contacts_data[it],penetration, world_normal_towards_A, world_midpoint, world_contact_point_on_A, world_contact_point_on_B);
    }

    return contacts_count, contacts_data;
}
wrap_collide_sphere_vs_hull:: (A: *Body, B: *Body) -> contacts_count:int, contacts_data: [4]Contact_Info{
    num_contacts, penetration, world_normal_towards_A, world_midpoint, world_contact_point_on_A, world_contact_point_on_B
    := collide_sphere_vs_hull(A.position, A.radius, B.hull_shape);

    contacts_count:=num_contacts;
    contacts_data: [4]Contact_Info;

    for 0..contacts_count-1 {
    fill_contact_data_for_wraps(*contacts_data[it],penetration, world_normal_towards_A, world_midpoint, world_contact_point_on_A, world_contact_point_on_B);
    }

    return contacts_count, contacts_data;
}
wrap_collide_capsule_vs_hull:: (A: *Body, B: *Body) -> contacts_count:int, contacts_data: [4]Contact_Info{
        capsule_start_A := A.position;
    capsule_end_A := capsule_start_A + A.capsule_height*rotate(Vector3.{0,1,0}, A.orientation);

    num_contacts, penetration, world_normal_towards_A, world_midpoint, world_contact_point_on_A, world_contact_point_on_B
    := collide_capsule_vs_hull(capsule_start_A, capsule_end_A, A.radius, B.hull_shape);

    contacts_count:=num_contacts;
    contacts_data: [4]Contact_Info;

    for 0..contacts_count-1 {
    fill_contact_data_for_wraps(*contacts_data[it],penetration, world_normal_towards_A, world_midpoint, world_contact_point_on_A, world_contact_point_on_B);
    }

    return contacts_count, contacts_data;
}
wrap_collide_hull_vs_hull:: (A: *Body, B: *Body) -> contacts_count:int, contacts_data: [4]Contact_Info{

    num_contacts, penetration, world_normal_towards_A, world_midpoint, world_contact_point_on_A, world_contact_point_on_B
    := collide_hull_vs_hull(A.hull_shape ,B.hull_shape);

    contacts_count:=num_contacts;
    contacts_data: [4]Contact_Info;

    for 0..contacts_count-1 {
    fill_contact_data_for_wraps(*contacts_data[it],penetration, world_normal_towards_A, world_midpoint[it], world_contact_point_on_A[it], world_contact_point_on_B[it]);
    }

    return contacts_count, contacts_data;
}

compute_contacts :: (A: *Body, B: *Body) -> contacts_count:int, contacts_data: [4]Contact_Info
{
    // debug_break();
    basic.assert(A.shape_kind <= B.shape_kind, "SHould ordr them before enering here");
    result_contacts: [4]Contact_Info;
    contacts_count: int;
    if A.shape_kind |  B.shape_kind == {
           case .SPHERE | .SPHERE; contacts_count, result_contacts   = wrap_collide_sphere_vs_sphere(A, B);
           case .SPHERE | .CAPSULE; contacts_count, result_contacts  = wrap_collide_sphere_vs_capsule(A, B);
           case .SPHERE | .HULL; contacts_count, result_contacts     = wrap_collide_sphere_vs_hull(A, B);
        // case .SPHERE, .MESH; // contacts_count, result_contactswrap_collide_sphere_vs_mesh(A, B);
           case .CAPSULE | .CAPSULE; contacts_count, result_contacts = wrap_collide_capsule_vs_capsule(A, B);
           case .CAPSULE | .HULL; contacts_count, result_contacts    = wrap_collide_capsule_vs_hull(A, B);
        // case .CAPSULE, .MESH; // contacts_count, result_contactswrap_collide_capsule_vs_mesh(A, B);
           case .HULL | .HULL; contacts_count, result_contacts       = wrap_collide_hull_vs_hull(A, B);
           // case .HULL | .MESH; contacts_count, result_contacts       = wrap_collide_hull_vs_mesh(A, B);
           case; basic.assert(false, "\n[ERROR]:shape vs shape not implemented! or... remember ordering the shapes in the broadphase!-> %", A.shape_kind |  B.shape_kind); }

    //more sanity checks because entropy code
      for 0..contacts_count -1 {
          basic.assert(is_normalized(result_contacts[it].world_normal_towards_A, tolerance=FLOAT32_EPSILON6), "e world_normal_towards_A %", result_contacts[it].world_normal_towards_A);

        if !is_normalized(result_contacts[it].world_normal_towards_A)
        // then continue; //TODO: figure out why the first contact in history is normal 0
        then basic.assert(false); // Debugger Time!
        //these two lines are key to avoid crash at resolve contacts
        //@Important @Check these two lines are key!
        // result_contacts[it].A = A; //:tagAddABtoEachContactPoint
        // result_contacts[it].B = B;
    }


    return contacts_count, result_contacts;
}




//idea, project onto sphere like gauss maps and use angles to.... maybe too slow, angles... sin cos... and I need the axis separation... proejcting onto sphere is information loss.
//wait I think that the idea above might work just needs extra thinking
 Pseudo_Body :: struct {
    bucket_locator: Bucket_Locator; //you can use a pointer or an handle or an id, I use buckets so I use bucketLocator, but I have stable pointers I can use pointers... anyways
    value: float;
    ismin: bool;
}

CompareSAP :: (a: Pseudo_Body, b: Pseudo_Body) -> int {
    if a.value < b.value return -1;
    return 1;
}

Collision_Pair :: struct {
    a,b: Bucket_Locator; //I sohuld use pointers....
}

operator == :: (left: Bucket_Locator, right: Bucket_Locator) -> bool {
    return left.bucket_index==right.bucket_index && left.slot_index == right.slot_index;
}
operator == :: (left: Collision_Pair, right: Collision_Pair) -> bool {
/*
Bucket_Locator :: struct {
    bucket_index: u32;
    slot_index:   s32;  // Signed because we do a trick where we set it to -1.
}
*/
    //it has been 14 hours since I started programming, this code is written after 14hs of work withan extremely tired mind


    return left.a==right.a && left.b == right.b || left.a == right.b && left.b == right.a;
}