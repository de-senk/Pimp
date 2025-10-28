//:tagDecisionsReasonings
#import "Math";
b :: #import "Basic";

#add_context solver: *Solver;
#import,file "math.jai";
#import "SIMD_Math";


//make video of changing this and playing sanity restored from dark souls
forward :: Vector3.{1,0,0}; //I think this is nonsense->if you rotate around X your model should change PITCH (up down) (example, cahracter was running and tripped and started rolling)
left 	:: Vector3.{0,1,0}; //I think this is nonsense->if you rotate around Y your model should change YAW (left right) (example: character falls down to the side)
up 		:: Vector3.{0,0,1}; //I think this is nonsense->if you rotate around Z your model should change ROLL (turning aroud up axis), (example: character turns 180 and now is looking backwards)
back    :: #run -forward; //forward backward? rightward? downward?
right 	:: #run -left;
down 	:: #run -up;


//:tagIrrelevantStuffThatMaybeIShouldMOveOrDelete <- STARTS HERE
//Why I implemented this?
maybe_reserve :: inline (array: *[..] $T, desired_max_size: s64, overshoot_factor: float32 = 1.0) {
    #if DEBUG assert(overshoot_factor>=1.0);
    new_max_size := cast(s64)(desired_max_size.(float32) * overshoot_factor);
    // log("this should not pahhen % % %",new_max_size > array.allocated, new_max_size, array.allocated);


    if new_max_size == 0 then return;
    if new_max_size > array.allocated {
        log("---->maybe_reserve %  desired:% max:%",new_max_size > array.allocated, desired_max_size, new_max_size);
        log("Total bodies=%, squared N*(N-1)/2=%",context.solver.bodies.count, context.solver.bodies.count*(context.solver.bodies.count -1 ) /2);
        // reserve := max(8, 2 * array.count);
        array_reserve(array, new_max_size);
    }
}

#scope_module
#insert #run maybe_insert_DEBUG();
maybe_insert_DEBUG :: () -> string
{
    basic :: #import "Basic";
    cmp :: #import "Compiler";

    current_w := get_current_workspace();
    options := cmp.get_build_options(current_w);
    should_debug_be_true: bool ;
    should_debug_be_true = options.backend == .X64;
    if !should_debug_be_true {
        //try guess if we are in debug from llvm options
        should_debug_be_true = options.llvm_options.bitcode_optimization_setting == .O0;
        should_debug_be_true |= options.llvm_options.machine_code_optimization_setting == .NONE;
        should_debug_be_true |= options.llvm_options.machine_code_optimization_setting == .UNSET;
    }
    //TODO(tr): check if debug already exsts with #if #exists(DEBUG) and... the TODO is to figure out what to do.
    #if #exists(DEBUG) then return ""; //early return

    b: basic.String_Builder;
    basic.print_to_builder(*b, "DEBUG :: %1; //inserted by maybe_insert_DEBUG", should_debug_be_true);
    return basic.builder_to_string(*b);
}
#scope_export

/*
I never liked pitch yaw and roll names.
Im tempted to use others but idk which ones
like, imagine a character in T pose
pitch->arms, so youse the arms direction as the rotational axis
yaw->eyeline, so you use the eyeline as the rotational axis
roll->spine, so you use the spine axis as the rotational axis
I like arms and spine, but I dont like eyeline name. But at least it takes me very little effort to know what axis Im rotating using these naems.
*/

/* copy the two lines
::\s*struct\s*(\{(?:[^{}]|)*?\bWWORDTOFINDWWW\b(?:[^{}]|)*\})


::\s*struct\s*\{([^{}]*(\{[^{}]*\}[^{}]*)*?\bWWORDTOFINDWWW\b[^{}]*)


*/

rotate90aroundx := #run make_quaternion_from_axis_and_angle(.{1,0,0}, 3.141593/2);
rotate90aroundy := #run make_quaternion_from_axis_and_angle(.{0,1,0}, 3.141593/2);
rotate90aroundz := #run make_quaternion_from_axis_and_angle(.{0,0,1}, 3.141593/2);
rotate45aroundx := #run make_quaternion_from_axis_and_angle(.{1,0,0}, 3.141593/4);


g_should_log_this_frame: bool;
deactivate_logger :: () #expand {
    my_logger :: (message: string, data: *void, info: Log_Info) {
    };
    old_logger      := context.logger;
    old_logger_data := context.logger_data;

    default_logger = old_logger;
    default_logger_data = old_logger_data;



    if !g_should_log_this_frame {
        context.logger      = my_logger;
        context.logger_data = null;
        }
    `defer {
        // When we are done with this scope, set the logger back.
        context.logger      = old_logger;
        context.logger_data = old_logger_data;
        g_should_log_this_frame = false;
    }
}

#run b.log("[TODO] more robust contact caching");
//EPSILON float32  0.123456789

#scope_export

log_TODO :: (loc:= #caller_location) {
 //this is for VSCODE |||filepath line:character|||: fk vscode I uninstalled it forever.
  b.log("TODO % %:%", loc.fully_pathed_filename, loc.line_number, loc.character_number);
}



//:tagIrrelevantStuffThatMaybeIShouldMOveOrDelete <- ENDS HERE


#import "Remap_Context"(VERBOSE=false);

//TODO: Remap_context only if @dll_export, and replace remap_context to nothing when used internally?

// #program_export
// set_context_solver_memory :: (solver: *Solver) {
//     context.solver = solver;
//     // context.logger = logger;
// } @dll_export

// #program_export
// create_solver :: () {
//     Remap_Context();
//     initialize_physics_solver();
//     context.solver.world.default_cube_shape = create_cube_hull(.{0, 0, 0}, .{1, 1, 1});
// } @dll_export


//
// Local ---> World
//
//this is the way
#program_export
l2w :: inline (world_position: Vector3,Aori: Quaternion, local_point: Vector3) -> Vector3 {
    return  world_position + rotation_matrix(Matrix3, Aori) * local_point;
} @dll_export

#program_export
l2w_rotate_only :: inline (world_position: Vector3, Aori: Quaternion, local_point: Vector3) -> Vector3 {
    return  rotation_matrix(Matrix3, Aori) * local_point;
} @dll_export

//
// World ---> Local
//
#program_export
w2l :: inline (world_position: Vector3, orientation: Quaternion, world_point: Vector3) -> Vector3 {
    return rotation_matrix(Matrix3, conjugate(orientation)) * (world_point - world_position);
} @dll_export

#program_export
w2l_rotate_only :: inline (_: Vector3, orientation: Quaternion, world_point: Vector3) -> Vector3 {
    //_ gets ignored, but I write it anyways, the only reason this procedure exist is because I like to iterate on soem stuff without changing parameters.
    return  rotation_matrix(Matrix3, conjugate(orientation)) * (world_point ); //Maybe: replace with rotate() from amth mdouels.
} @dll_export




World :: struct {
    // drawing_list: Drawing_List;
    VOLOR :: struct{p: Vector3;radius:float;color:Vector4;}
    counter: u64;

    //this might be pointless, but I just try to makea bucket arrays of the size of a page size of virtual memory
    count_Constraint_Fitted_To_PageSize :: 8*4096 / size_of(Constraint); // size*count=4096*8bit=bucket_size -> count=4096*8;
    count_RigidBody_Fitted_To_PageSize :: 8*4096 / size_of(RigidBody); // size*count=4096*8bit=bucket_size -> count=4096*8;

    bodies     :  Bucket_Array(type=RigidBody , items_per_bucket=count_RigidBody_Fitted_To_PageSize ); // old XAR comment: 1<<(8 +4 -1) 2048
    constraints:  Bucket_Array(type=Constraint, items_per_bucket=count_Constraint_Fitted_To_PageSize); //old XAR comment: 8^4 = 4096


    //for CCD
    // number_of_collisions_this_frame: int;
    // possible_collisions: [..]Contact_Info; // I only use this because I want to quick_sort, and I dont have the stamina to implement quicksort for bucket arrays.
    // collision_pairs: [..]Collision_Pair; //for broadphase, maybe bucket it

    // default_cube_shape: Convex_Hull;

    //unused, maybe delete ring buffer draw
    points_to_draw: [10]VOLOR;
    points_to_draw_cursor: int;
}

Solver :: struct {
    using world: *World;
    dt          := 1.0 / 60.0;
    gravity     := Vector3.{0,0,-10};
    iterations  := 10;
    number_of_steps_executed_so_far : u64;
} @dll_export


Shape_Kind :: enum_flags u64 {
    SPHERE;CAPSULE;CUBE;HULL;PLANE;

    //to be removed, because I found a bug using these but I dont remember how to reproduce
    // but it was like,// try changing to u16 and see
    //combinatorics SPHERE
    SPHEREvsSPHERE :: Shape_Kind.SPHERE | Shape_Kind.SPHERE;
    SPHEREvsCAPSULE :: Shape_Kind.SPHERE | Shape_Kind.CAPSULE;
    SPHEREvsPLANE :: Shape_Kind.SPHERE | Shape_Kind.PLANE;


    //combinatorics CAPSULE
    CAPSULEvsPLANE :: Shape_Kind.CAPSULE | Shape_Kind.PLANE;
}

RigidBody :: struct {
    bucket_locator: Bucket_Locator; //FOR BODIES, NOT CONSTRAINTS! if you want the constraint bucket, use the pointer

}




Constraint :: struct {
    Kind :: enum {
        COLLISION;
        JOINT;
        SPRING;
        IGNORE_COLLISION;
    }
    bucket_locator: Bucket_Locator;
    kind: Kind;
    A, B: *RigidBody;


}

Simplest_Geometric_Shape :: struct {
    a: Vector3;
    b: Vector3;
    c: Vector3;
    d: Vector3;
    count: int;
    #place a;
        points: [4]Vector3 = ---;
    #place count;
        state: enum {POINT;SEGMENT;TRIANGLE;TETRAHEDRON;} = ---;
}

simplex_add :: (simplex: *Simplest_Geometric_Shape, point: Vector3) {
    should_ring_buffer := simplex.count == 4;
	if should_ring_buffer
	{
		simplex.points[0] = simplex.points[1];
		simplex.points[1] = simplex.points[2];
		simplex.points[2] = simplex.points[3];
		#if DEBUG b.assert(simplex.count == 4, "simplex should be count 4 here.");
	} else {
		simplex.count += 1;
		#if DEBUG b.assert(simplex.count <= 4, "simplex count is biger than 4 fix the code.");
	}
	simplex.points[simplex.count - 1] = point;
}

COLLISION_MARGIN :: 0.001;

#load "simulate.jai";


#scope_module

#import "Bucket_Array";

/*
== Shpehe VS Sphere
    Overlap if |C_2 - C_1| -r1 -r2 <= 0;
    max c.ontact point needed = 1;
    normal is C2-C1 normalized
    C.ontact point at the middle of the overlapping surfaces  (.)  (   .   )
    penetration is just our overlap function evaluated |C_2 - C_1| -r1 -r2=d
    on degenerate: 1) invent normal 2) assert

== Shpehe VS Capsule
    Overlap if |C - L| -r_sphere -r_Capsule <= 0;
    max c.ontact point needed = 1;
    normal is L-C1 normalized
    C.ontact point at the middle of the overlapping surfaces  (.)  (   .   )
    penetration is just our overlap function evaluated |C - L| -r_sphere -r_Capsule = d = error = C0
    on degenerate: 1) invent normal orthogonal to L 2) assert: better solution!
== Shpehe VS Hull
    two cases, shallow pnetratio and deep penetration
    GJK/EPA -> when center is Outside the hull (we use the center not the sphere surface hre)
            Overlap if |C - L| -r_sphere <= 0; Its the cilinder case with r_Cylinder 0!
            normal is L-C normalized
            d is just overlap evaluated-> d= |C - L| -r_sphere
    SAT-> when center is inside the hull
        we find the face that minimizes d
        for normal we take the face normal and we project thesphere center on the face


== Capsule VS Capsule : 2 c.ontact points
    Overlap if |L_1 - L_2| -r_1 -r_2 <= 0;
    we put c.ontact point in the middle again!
    degenrate->
    if they are crossing:
        crossproduct of L1 and L2
        if crossproduct is degenerate, bettet to assert!
    if they are parallell (stacking):
        strategy, clip plane (-27:43 of https://dev.gdcvault.com/play/1022193/Physics-for-Game-Programmers-Robust)
== Capsule VS Hull : guess 2 c.ontact points
    we need to take care of the inner segment doesnt clip the hull
    find closest point using GJK using the SEGMENT, if success we build shallow, if fail, we build SAT
    deep face deep edge , etc, etc, more watch the video again

== HUll vs Hull: hard
    Overlap if SAT = false
    test -> face normlas of A
    test -> face normlas of B
    test -> grab the edges of A and the edges of B and test all edge combination cross products.

    SAT :: (manifold: *Manifold, hullA: Hull, hullB: Hull) -> overlap: bool {

        faceQueryA: FaceQuery = QueryFaceDirections(hullA, hullB);
        if faceQueryA.separation > 0.0 then return true;

        faceQueryB: FaceQuery = QueryFaceDirections(hullB, hullA);
        if faceQueryB.separation > 0.0 then return true;

        edgeQuery: EdgeQuery = QueryEdgeDirections(hullA, hullB);
        if edgeQuery.separation > 0.0 then return true;
        /*
            CODE CONTINUES BELOW AFTER SOME EXPLANATION
        */
    }

    we iterate over all planes
    QueryFaceDirections :: (hullA, hullB) -> FaceQuery {
        FaceQuery :: struct {largest_distance: float; face_index: int};
        result: FaceQuery;
        for hullA.FaceCount {
            planeA := get_plane(hullA, it_index);
            vertexB := get_support(hullB, direction = -planeA.normal);
            distance := distance(plane=planeA, point=vertexB); //dot (n, supportpoint - point in plane)
            if distance > result.largest_distance then resul = .{distance, it_index};
        }


        return result;
    }
    for edge we need consistent normal orientation because we need that hull B is behind the plane
    to do that we just check if dot(n,O-Center_B) <0 then n=-n; where O is any point in the plane of B
    QueryEdgeDirections :: (hullA, hullB) -> EdgeQuery {
        EdgeQuery :: struct {largest_distance: float; edge_index: int};
        result: EdgeQuery;
        for *edgeA: hullA.edges {
            for *edgeB: hullB.edges {
            axis := cross_product(get_direction(edgeA), get_direction(edgeB));
            if dot(axis, edgeA.origin - hullA.center) < 0 then axis = -axis;

            planeA := create_plane(axis, edgeA.origin);
            vertexB := get_support(hullB, direction = -planeA.normal /*maybe normalizing is not necessary*/);
            distance := distance(plane=planeA, point=vertexB); //dot (n, supportpoint - point in plane)
            if distance > result.largest_distance then resul = .{distance, it_index};

            }

        }

        return result;

    }

    if we run those test andwe did NOT find a SAT, means taht they are OVERLAPPING then
    we tested 3 fatuers, faces of A faces of B and edge combination
    if minimizing feature is a face, we create face c.ontact
    if minimizing feature is a edge, we create edge c.ontact

    SAT :: (manifold: *Manifold, hullA: Hull, hullB: Hull) -> overlap: bool {
            ... pervious code ...
        /*
            CODE CONTINUES HERE
        */
        blsFaceContactA := faceQueryA.separation > edgeQuery.separation;
        blsFaceContactB := faceQueryB.separation > edgeQuery.separation;
        if blsFaceContactA && blsFaceContactB //shortest way out is prioritized
        then CreateFaceContact(manifold, faceQueryA, hullA, faceQueryB, hullB);
        else CreateEdgeContact(manifold, edgeQuery, hullA, hullB);


    }
    -17:00 need to rewatch

    -9:00 c.ontact points reduction

    50:05 Pair management
    on narrow phase we build contacts
    BuildNewContacts :: () {
        twoConvexShapes := (shape1.type != .MESH) && (shape2.type != .MESH);
        oneConvexShape  := (shape1.type != .MESH) || (shape1.type != .MESH);
        if twoConvexShapes {
            contact:*ConvexContact = create_convex_contact(shape1,shape2);
            array_add(*contacts, contact);
        }

        if oneConvexShape {
            contact:*MeshContact = create_mesh_contact(shape1,shape2);
            array_add(*contacts, contact); //use arena probaably

        }

    }

    collider_argument :: struct {manifold: manifold; transform1; shape1; transform2; *shape2}
    Collide(collider_argument)
    {
        if int.[shape1.kind,shape2.kind]:
           case .[0,0];  collide_sphere_with_sphere(collider_argument);
           case .[0,1];  collide_sphere_with_capsule(collider_argument);
           case .[0,2];  collide_sphere_with_hull(collider_argument);
           case .[0,3];  collide_sphere_with_mesh(collider_argument);
           case .[0,4];  collide_sphere_with_plane(collider_argument);

           case .[1,1];  collide_capsule_with_capsule(collider_argument);
           case .[1,2];  collide_capsule_with_hull(collider_argument);
           case .[1,3];  collide_capsule_with_mesh(collider_argument);
           case .[1,4];  collide_capsule_with_plane(collider_argument);

           case .[2,3];  collide_hull_with_hull(collider_argument);
           case .[2,3];  collide_hull_with_mesh(collider_argument);
           case .[2,4];  collide_hull_with_plane(collider_argument);
           case; assert(false, " not implemented! or... remember ordering the shapes in the broadphase!");
    }

*/



//:tagDecisionsReasonings
assume_I_use_the_context_for_solver :: "I decided to use the context, just to avoid passing a pointer to the solver, this might generate friction if you want to use multiple worlds at the same time, anyways this should be easy to change in less than a day if necessary";

