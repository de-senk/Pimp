//In the beginning you always want Results.
//In the end all you want is Control.
//:tagDecisionsReasonings

physics_coordinates :: struct {
    forward :: Vector3.{1,0,0}; front :: forward;
    left 	:: Vector3.{0,1,0};
    up 	    :: Vector3.{0,0,1};

    back    :: #run -forward; //forward backward? rightward? downward?
    right 	:: #run -left;
    down 	:: #run -up;
}

#add_context solver: *Solver;

#if !#exists(DEBUG)
DEBUG :: false;

//:tagIrrelevantStuffThatMaybeIShouldMOveOrDelete <- STARTS HERE
//Why I implemented this?
maybe_reserve :: inline (array: *[..] $T, desired_max_size: s64, overshoot_factor: float32 = 1.0) {
    #if DEBUG basic.assert(overshoot_factor>=1.0);
    new_max_size := cast(s64)(desired_max_size.(float32) * overshoot_factor);
    // log("this should not pahhen % % %",new_max_size > array.allocated, new_max_size, array.allocated);


    if new_max_size == 0 then return;
    if new_max_size > array.allocated {
        // log("---->maybe_reserve %  desired:% max:%",new_max_size > array.allocated, desired_max_size, new_max_size);
        // log("Total bodies=%, squared N*(N-1)/2=%",context.solver.bodies.count, context.solver.bodies.count*(context.solver.bodies.count -1 ) /2);
        // reserve := max(8, 2 * array.count);
        basic.array_reserve(array, new_max_size);
    }
}

Simplest_Geometric_Shape :: struct {
    a: Vector3;
    b: Vector3;
    c: Vector3;
    d: Vector3;
    count: s64;
    #overlay(a) points: [4]Vector3;
    #overlay(count) state: enum s64 {NONE;POINT;SEGMENT;TRIANGLE;TETRAHEDRON;};
}

simplex_add :: (simplex: *Simplest_Geometric_Shape, point: Vector3) {
    should_ring_buffer := simplex.count == 4;
	if should_ring_buffer
	{
		simplex.points[0] = simplex.points[1];
		simplex.points[1] = simplex.points[2];
		simplex.points[2] = simplex.points[3];
		#if DEBUG basic.assert(simplex.count == 4, "simplex should be count 4 here.");
	} else {
		simplex.count += 1;
		#if DEBUG basic.assert(simplex.count <= 4, "simplex count is biger than 4 fix the code.");
	}
	simplex.points[simplex.count - 1] = point;
}


rotate90aroundx := #run make_quaternion_from_axis_and_angle(.{1,0,0}, 3.141593/2);
rotate90aroundy := #run make_quaternion_from_axis_and_angle(.{0,1,0}, 3.141593/2);
rotate90aroundz := #run make_quaternion_from_axis_and_angle(.{0,0,1}, 3.141593/2);
rotate45aroundx := #run make_quaternion_from_axis_and_angle(.{1,0,0}, 3.141593/4);


DEG_TO_RAD :: 3.141593/180.0; //7 digits bec    ause float32 has a limit of 7 digits
degrees_120_in_radians :: 120.0*DEG_TO_RAD;


#scope_module
#if !#exists(TODO)
TODO :: (msg: string,loc:= #caller_location) {
 //this is for VSCODE |||filepath line:character|||:
  basic.log("\n[TODO->] %, at-> % %:% [<-TODO]", msg, loc.fully_pathed_filename, loc.line_number, loc.character_number);
}

#scope_export
//:tagIrrelevantStuffThatMaybeIShouldMOveOrDelete <- ENDS HERE


#import "Remap_Context"(VERBOSE=false);

create_solver_and_world :: () {
    context.solver = basic.New(Solver);
    context.solver.world = basic.New(World);
}


CheckInit :: () #expand {
    if !context.solver  create_solver_and_world();
    `state := context.solver;

}

World :: struct {
    COLLISION_MARGIN :: 0.001; //

    //Complexity Warning:
        //this might be pointless, but I just try to makea bucket arrays of the size of a page size of virtual memory, I did this years ago and I never measured it, so there is a very high probability that this is pointless:
        count_joint_Constraint_Fitted_To_PageSize :: 8*4096 / size_of(Joint_Constraint); // size*count=4096*8bit=bucket_size -> count=4096*8;
        count_RigidBody_Fitted_To_PageSize :: 8*4096 / size_of(Body); // size*count=4096*8bit=bucket_size -> count=4096*8;
        count_Manifolds_Fitted_To_PageSize :: 8*4096 / size_of(Collision_Constraint); // size*count=4096*8bit=bucket_size -> count=4096*8;

    soft_bodies: [1]Soft_Body;
    stub_world_body: Body; //to avoid null check everywhere on the code and maybe soem easi paralellization without masks and branching.
    bodies     :  Bucket_Array(type=Body , items_per_bucket=count_RigidBody_Fitted_To_PageSize );
    hashed_bodies:  Table(u64, Bucket_Locator);

    joint_constraints:  Bucket_Array(type=Joint_Constraint, items_per_bucket=count_joint_Constraint_Fitted_To_PageSize);
    hashed_joint_constraints:  Table(u64, Bucket_Locator);

    collision_constraints: Bucket_Array(type=Collision_Constraint, items_per_bucket=count_Manifolds_Fitted_To_PageSize);
    collision_pairs: [..]Collision_Pair; //for broadphase, maybe bucket it
    possible_collisions: [..]Contact_Info; // I only use this datatype because I want to quick_sort, and I dont have the stamina to implement quicksort for bucket arrays.

    //for CCD
    // number_of_collisions_this_frame: int;

}

Solver :: struct {
    using world: *World;
    dt          := 1.0 / 60.0;
    gravity     := Vector3.{0,-10.0,0};
    iterations  := 1;
    number_of_steps_executed_so_far : u64; //<- used for immediate mode.
}


Shape_Kind :: enum_flags u64 {
    SPHERE;
    CAPSULE;
    CUBE;
    HULL;
    // PLANE;
    // TRIANGLE_MESH;
}

//For XPBD, not all these fields are used.
Body :: struct {

    aabb: AABB; //local aabb
    hull_shape: Convex_Hull;
    bucket_locator: Bucket_Locator;

    shape_kind: Shape_Kind;
    // shape_castable_data: Basic_Shape; //I dont 100% like this but Is good enough.
    radius: float;
    capsule_height: float;

    scale: Vector3 = .{1,1,1};

    position: Vector3;
    prev_position: Vector3;
    prev_prev_position: Vector3;

    velocity: Vector3;
    prev_velocity: Vector3;

    guess_position:       Vector3; //reduces convergence time (warmstarting)
    guess_orientation:    Quaternion; //reduces convergence time (warmstarting)

    orientation:          Quaternion;
    prev_orientation:     Quaternion;

    angular_velocity:     Vector3;
    prev_angular_velocity: Vector3;

    local_center_of_mass: Vector3;
    inv_mass: float;


    local_moment_of_inertia:   Vector3;
    I_world:   Vector3;

    inv_local_moment_of_inertia:   Vector3;
    inv_I_world:   Vector3;

    // factor_energy_loss_on_collision: float=1.0;
    friction:             float32 = 0.5;
    restitution:             float32 = 1.0;
    elasticity:           float32 = .0; //half means half every collision, you might want .9


    //these fields might change based on orientation and shape kind, we try to cache them if possible.
    inv_world_moment_of_inertia:   Matrix3;


    //caching stuff to make immediate mode work:
    latest_frame_of_existence: u64; //for should_cache check
    end_frame_callback: (body: *Body)->();
    user_data: *void;
    hash: u64; //for immediate mode caching invalidation
}


//Old, can be deleted:
Basic_Shape :: struct {
    _1, _2, _3: float;

    #overlay(_1) using #as v: Vector3;
    #overlay(_1) sphere_radius: float;
    #overlay(_1) capsule_radius: float;
    #overlay(_2) capsule_height: float;
}


Contact_Info :: struct {
    //Contact_Feature will not hold, will change to a better one in the future.
     Contact_Feature :: struct {
            value: u64;
            kind: enum{
                ALREADY_UNIQUE; //this is for spheres vs spheres
                EDGE; //for convex hulls
                FACE; //for convex hulls
                DIRECTION_SIGN; //DIRECTION_SIGN is for capsules, I take capsule A dir an capsule B dir and if its dot product is positive I use value 1 and 7 if it is negative I use vale 7 and 1, why 7? no reason at all., maybe change enum name.
                }
        };
        //these two fields are added just to support CCD, maybe it can be done in a better way later on.
        // A: *Body;
        // B: *Body;


        world_normal_towards_A: Vector3; //same for B but opposite

        // world_contact_point_on_A: Vector3; //no because we need to recalculate it every update! we need local!
        // world_contact_point_on_B: Vector3; //no because we need to recalculate it every update! we need local!
        local_point_on_A: Vector3;
        local_point_on_B: Vector3;

        world_midpoint: Vector3; //unused yet, maybe we also need to use local!



        // position: Vector3;
        penetration: float; //same for B but opposite
        time_of_impact: float; //same for B

        //hacks
        feature: Contact_Feature; //this is some hack to distinghish between frames if the previois contact point is or may be the same as the new computed one.

};


Soft_Body :: struct {
    is_initialized: bool;
    is_graphics_initialized: bool;
    numParticles  : s64     ;
    numTets       : s64     ;
    pos           : []float ;
    prevPos       : []float ;
    vel           : []float ;
    tetIds        : []u32   ;
    edgeIds       : []u32   ;
    triangleList       : []u32   ;
    restVol       : []float ;
    edgeLengths   : []float ;
    invMass       : []float ;
    edgeCompliance: float   ;
    volCompliance : float   ;
    tempp         : []float ;
    grads         : []float ;
    grabId        : s32     ;
    grabInvMass   : float   ;


	volIdOrder: = ([3]u32).[u32.[1,3,2], u32.[0,2,3], u32.[0,3,1], u32.[0,1,2]];
};

#load "src/body_procs.jai";
#load "src/joints.jai";
#load "src/collisions.jai";
#load "src/immediate_mode_magic.jai";

#load "simulate.jai";
// #load "src/spatial_hashing.jai";

#import "Sort";
#import "Math";
#import "Hash_Table";
#import,file "math.jai";


#import "Collision_Detection";
#import "Drawing_Primitives";

basic :: #import "Basic";

#scope_module


FLOAT32_EPSILON7 :float32: 0h33d6bf95; //0.0000001
FLOAT32_EPSILON6 :float32: 0h358637bd; //0.000001
FLOAT32_EPSILON5 :float32: 0h3727c5ac; //0.00001
FLOAT32_EPSILON4 :float32: 0h38d1b717; //0.0001
FLOAT32_EPSILON3 :float32: 0h3a83126f; //0.001
FLOAT32_EPSILON2 :float32: 0h3c23d70a; //0.01
FLOAT32_EPSILON1 :float32: 0h3dcccccd; //0.1
FLOAT32_EPSILON0 :float32: 0h3f800000; //1.0

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
//I like to put my decisions and the reasoning behind them, just to avoid information loss.
assume_I_use_symmetric_objects :: true;
assume_README_PLEASE_Immediate_Mode_Misconceptions :: "Immediate mode IS usable in the release of games!! Immediate mode can be as fast OR even faster than retained mode physics. Same rule applies to Immediate mode GUI's, if an Immediate mode gui recreates everything from scratch when you do doButton or doLabel, then the author of the library is doing something wrong. The trick is to do a lot of caching and caching invalidation and naming stuff, yes, it is hard to do, but that is the way. I hope I provide enough examples to show the power of immediate mode API's. Immedate mode API's reduces user error probability drastically and allows for very fast iterations.";
assume_What_is_this_library_for :: "Iterating gameplay|prototyping games|fearless usage and modification of the code if you need more features or anything you want (MIT + Credit Not Required but appreciated)|1 line of code to do something useful and see results on the screen|have high level primitives that allows you to use them as building blocks to do more complex things|bootstraping your physics engine";
assume_What_is_NOT_for :: "stress testings, Even tho, I will try to make this library very fast, this is not a library to stack 400000 watermelons in a room and expect great performance results. This library is for reasonable games with interesting gameplay elements.";
assume_I_use_simple_code :: "Because for different kind of games you need different kind of solutions, and if the physics code is too complicated, adding simple things like a callback so that on collision you want to play a sound or generate more bodies or whatever you needs are, are NOT straightforward thing to do, complicated code generates friction, decreases productiviy, In contrast, simple code allows for anyone to jump into the code and modify it to satisfy their needs.";
assume_I_care_a_lot_about_performance_and_I_dont_care_about_performance :: "Performance is the currency, the coin, the -transactional unit- we use to BUY features. Its really hard to make decisions and maintain a balance of very simple code, easy to modify code, performant code. But, I will try my best because I want to see more interesting games, and prototyping is key to innovation.";
assume_only_jai_for_now :: "This will not work for interop with other languages, but I think I can build a metaprogram to rewrite all the code in C99 and maintain it, but it would have a slightly different API than jai and will be less simple to use. BUT I defer this feature to near the end only if peole really like this library, if nobody uses this library I will not take that effort.";
assume_I_use_the_context_for_solver :: "I decided to use the context, just to avoid passing a pointer to the solver, this might generate friction if you want to use multiple worlds at the same time, anyways this should be easy to change in less than a day if necessary";
assume_I_dont_support_softbodies_or_clothes_YET :: "Because its so early and Im not sure how to do it performant enough, BUT, XPBD is the best solver to do these kind of stuff. I will support SoftBodies and cloths in the future in a very interesting and useful way like the other procedures, my objective is to be able to mage gameplay features easily with just 1 line of code. So, expect that in the future.";
assume_I_did_not_implemented_multithreading_YET :: "I will, for sure Implement multithreading and all assembly SIMD code for ARM and x86 in the future, and there is a good thing and a bad thing of using scalar single threaded code, that is, measurement, If I started with multicore aproach from default, It would be harder to measure what parts of the code are slow, and the code is simpler, I think that is the complete list of all the advantages. Obviously multicore is a MUST in a game, but IDK what strategy would generate simple code and very flexible code, if you game uses softbody and clothes a lot, you might need a different multithhreading strategy of whatever I choose. So, I dont know YET, maybe I will implement 3 different multicore approaches, and another multicore approach that I've not seen anywhere that is that you use the cores to prefetch the memory and not do anything of value with these extra cores, prefetching the memory will make the singlecore version faster by how CPU's Work. SIMD is a must and I think I know how to do it with metaprogramming so, nobody has to read the SIMD code and only look at the scalar code and modify the scalar code and the metaprogramming can generate the simd, but idk, maye that is a bad idea, time will tell.";

assume_I_only_use_SAT_for_collision_detection :: "because it is 1 solution to solve them all, requieres caching (sad) to be really useful, but same goes for gjkepa, gjkepa would be better for CCD, but I think I can do some trickery to get CCD with SAT";
assume_MAIN_source_of_information_used_to_build_this_library :: "https://matthias-research.github.io/pages/publications/XPBD.pdf and https://www.youtube.com/watch?v=ZQ5_u8Lgvyk";


