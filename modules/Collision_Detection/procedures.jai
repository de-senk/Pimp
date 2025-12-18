QueryFaceDirections :: (hullA: Convex_Hull, hullB: Convex_Hull) -> FaceQuery {
//NOTE: I think we dont need to query for faces because this procedure already give us the most probable faces.
    result: FaceQuery;
    result.face = null;
    result.distance = -FLOAT32_INFINITY;

    for :all_faces current_face: hullA.mesh {
        world_planeA := transform_plane(.{current_face.face_plane_normal, current_face.face_plane_distance}, hullA.worldSpace, hullA.inv_worldSpace_for_bivectors);
        local_direction_in_hull1 := transform_normal(-world_planeA.normal, hullB.worldSpace);
        // draw_primitive.grid(closest_point_to_plane(.{5,5,5}, world_planeA.normal, world_planeA.distance), world_planeA.normal, color=.{1.1,1.1,1.1,1});

        furthest_ptr, furthest_vertex_local, vertex_index := find_furthest_point_along_local_dir(hullB, local_direction_in_hull1);
        furthest_vertex_world := transform_point(furthest_vertex_local, hullB.worldSpace);

        distance := distance_point_to_plane_signed(
            point = furthest_vertex_world,
            plane_normal = world_planeA.normal,
            plane_distance = world_planeA.distance
        );

        // Keep track of largest distance index
        if distance > result.distance {
            result.face = current_face;
            result.distance = distance; //most separating OR if negative: most closest to the penetration separation
            result.world_furthest_point = furthest_vertex_world; //:tagHerePseudoGJK
            result.world_normal = world_planeA.normal; //:tagHerePseudoGJK
            basically_separating := result.distance > FLOAT32_EPSILON4;
            if basically_separating {
                result.distance  = FLOAT32_INFINITY; //this will exit SAT because we found a separating axis
                // draw_primitive.grid(closest_point_to_plane(.{5,5,5}, world_planeA.normal, world_planeA.distance), world_planeA.normal, color=.{1.1,1.1,1.1,1});
                // draw_primitive.grid(furthest_vertex_world, world_planeA.normal,area_length=2, color=.{1.1,1.1,1.1,1});
                // draw_primitive.X(hull_center(hullA));
                // draw_primitive.X(hull_center(hullB));
                // draw_primitive.X(furthest_vertex_world);
                // draw_primitive.doLabel("furthest_vertex", draw_primitive.WorldToScreen(furthest_vertex_world));

                // draw_primitive.doLabel("SEPARATING AXIS FOUND!", draw_primitive.WorldToScreen(furthest_vertex_world));
                return result;
            }
        }


    }

    return result;
}

compute_centroid :: (points: []Vector3) -> Vector3
{
    sum: Vector3;
    for points {
        sum.x += it.x;
        sum.y += it.y;
        sum.z += it.z;
    }
    inv_count := 1.0 / cast(float)points.count;
    return Vector3.{ sum.x * inv_count, sum.y * inv_count, sum.z * inv_count };
}


//NOTE: Quickhull should  make the geometric center at {0,0,0}
project_all_vertices_of_hull_into_axis :: (hull: Convex_Hull, axis_n: Vector3)-> left_interval: float, right_interval:float {
    // left_interval: float = FLOAT32_INFINITY;
    // right_interval: float = -FLOAT32_INFINITY;
    world_center := Vector3.{hull.worldSpace._14, hull.worldSpace._24, hull.worldSpace._34};
    world_geometric_center := world_center; //lets assume geometric center is in the middle, otherwise, user did hull wrong

    center_proj := dot(world_geometric_center, axis_n);

   //so we expand the interval [left, right] from the world hull center because I cannot brain how to do it with ininity's
    left_interval: float = center_proj;
    right_interval: float = center_proj;
    for :all_vertices current_vertex: hull.mesh {
        local_vertex := current_vertex.vertex;
        world_vertex := transform_point(local_vertex, hull.worldSpace);
        // proj := dot(world_vertex, axis_n);
        //I dont use continue here because I want to hint the compiler to make this branchless
        // if proj > right_interval then right_interval = proj;
        // if proj < left_interval  then left_interval = proj;


        proj := dot(world_vertex - world_center, axis_n); // projection relative to center
        left_interval  = min(left_interval, proj);
        right_interval = max(right_interval, proj);
        }
    return left_interval, right_interval;
}

//NOTE: Quickhull should  make the geometric center at {0,0,0}
project_all_vertices_of_hull_into_local_axis :: (hull: Convex_Hull, local_axis: Vector3)-> left_interval: float, right_interval:float {
    // axis_n := normalize(local_axis); //NOTE: DONT NOTMALIZE ;WE NEED THE SCALING FACTOR!!!!!!!!!!!
    axis_n := local_axis; //NOTE: DONT NOTMALIZE ;WE NEED THE SCALING FACTOR!!!!!!!!!!!
    // left_interval: float = FLOAT32_INFINITY;
    // right_interval: float = -FLOAT32_INFINITY;
    translation := Vector3.{hull.worldSpace._14, hull.worldSpace._24, hull.worldSpace._34};
    world_geometric_center := translation; //lets assume geometric center is in the middle, otherwise, user did hull wrong
    center_proj := dot(world_geometric_center, axis_n);

    //so we expand the interval [left, right] from the world hull center because I cannot brain how to do it with ininity's
    left_interval: float = center_proj;
    right_interval: float = center_proj;

    for :all_vertices current_vertex: hull.mesh {
        local_vertex := current_vertex.vertex;
        // world_vertex := hull.worldSpace * local_vertex;
        proj := dot(local_vertex, axis_n);
        //hint branchless
        if proj > right_interval then right_interval = proj;
        if proj < left_interval  then left_interval = proj;
    }
    return left_interval, right_interval;
}

hull_center :: inline (hull: Convex_Hull) -> Vector3 { return Vector3.{hull.worldSpace._14, hull.worldSpace._24, hull.worldSpace._34};}



//this is no gaussmap, n^2, very fast.
QueryEdgeDirections_deSenk :: (hullA: Convex_Hull, hullB: Convex_Hull) -> EdgeQuery
{
    #run TODO("Maybe change the system, so its always faceA+edge points OR faceB +edge points");
    DRAW :: false;

    hullA_center := Vector3.{hullA.worldSpace._14, hullA.worldSpace._24, hullA.worldSpace._34};
    #if DRAW {
        hullB_center := Vector3.{hullB.worldSpace._14, hullB.worldSpace._24, hullB.worldSpace._34};
    }
    dist2_between_points: float = -FLOAT32_INFINITY;
    best_point_1:Vector3;
    best_point_2:Vector3;
    result := EdgeQuery.{ distance = -FLOAT32_INFINITY };
	for :all_edges edgeA, indexA : hullA.mesh
	{
        edgeA_start := hullA.worldSpace * (edgeA.tail.vertex);
        edgeA_end := hullA.worldSpace   * (edgeA.next.tail.vertex);
		for :all_edges edgeB, indexB: hullB.mesh
		{
            edgeB_start := hullB.worldSpace * (edgeB.tail.vertex);
            edgeB_end := hullB.worldSpace   * (edgeB.next.tail.vertex);
            hit, intersection_point, bary_coord, t_slider, this_separation := intersect_segment_triangle_branchless_rename_me(edgeB_start, edgeB_end, hullA_center, edgeA_start, edgeA_end );
            if hit {
                this_separation = -this_separation;
                result.edgeA = edgeA;
                result.edgeB = edgeB;
                result.distance = this_separation;
                //experimental, will not use, but I have this as a reference when I figure out what to do with this data.
                result.world_furthest_point = intersection_point;
                result.num_inersecting_points +=1;
                #if DRAW {
                  // result.world_normal = world_plane_normal;
                  //draw triangle
                  draw_primitive.arrow(hullA_center,   edgeA_start- hullA_center , color=.{1,1,1,1} );
                  draw_primitive.arrow(edgeA_start,   edgeA_end- edgeA_start  , color=.{1,1,1,1});
                  draw_primitive.arrow(edgeA_end,   hullA_center- edgeA_end  , color=.{1,1,1,1});

                  //draw contact points
                  draw_primitive.square(intersection_point,   size=0.1  , color=.{1,0,0,1});
                }
            }

		}

	}


	return result;
}


//this is no gaussmap, n^3, very expensive.
//deSenk version  is way better.
// QueryEdgeDirections_bruteforce :: (hullA: Convex_Hull, hullB: Convex_Hull) -> EdgeQuery
// {

// }


draw_current_edge :: (e: *Hull_Half_Edge, hull: Convex_Hull, color: Vector4= .{1,1,1,1}) {
    draw_primitive.arrow(hull.worldSpace*e.tail.vertex, hull.worldSpace*e.next.tail.vertex-hull.worldSpace*e.tail.vertex,color=color);
}

draw_current_edge_normal :: (e_origin: Vector3, e: *Hull_Half_Edge, inv_world: Matrix4) {
        v0 := e.tail;
        v1 := e.next.tail;
        v2 := e.next.next.tail;
        plane_normal, plane_d := create_plane(v0.vertex, v1.vertex, v2.vertex);
        {
            n := transform_normal(plane_normal, inv_world);
            draw_primitive.arrow(e_origin, n,color=.{abs(n).x,abs(n).y,abs(n).z, 1.0});
    	}
}

draw_hull_properties :: (hull: Convex_Hull){
    // return;
    for :all_edges e: hull.mesh {
        plane_normal :=  transform_normal(e.face.face_plane_normal, hull.inv_worldSpace_for_bivectors);
        off := ( 0.01 )*plane_normal;
        ncol := Vector4.{abs(plane_normal).x,abs(plane_normal).y,abs(plane_normal).z, 1.0};

		e_origin:= hull.worldSpace * ( e.tail.vertex) +off;
        e_end := hull.worldSpace * ( e.twin.tail.vertex)+off;
        draw_primitive.arrow(e_origin,e_end-e_origin,0.05, color=ncol);
        // draw_current_edge_normal(e_origin, e, hull.inv_worldSpace_for_bivectors);

	}
}

//from slide 34, test if arcs a b intersects on sphere.
IsMinkowskyFace :: (A: Vector3, B: Vector3, C: Vector3, D: Vector3) -> bool {
    B_x_A := cross_product(B,A);
    D_x_C := cross_product(D,C);
    CBA := dot(C, B_x_A);
    DBA := dot(D, B_x_A);
    ADC := dot(A, D_x_C);
    BDC := dot(B, D_x_C);
    return CBA*DBA<0.0 && ADC*BDC<0 && CBA*BDC>0;
}

arcs_intersect_on_sphere :: (arcA_start: Vector3, arcA_end: Vector3, arcB_start: Vector3, arcB_end: Vector3) -> bool {
    B_x_A := cross_product(arcA_end,arcA_start); //cross product of face A1 normal with face A2 normal
    D_x_C := cross_product(arcB_end,arcB_start); //no need to normalize
    CBA := dot(arcB_start, B_x_A);
    DBA := dot(arcB_end, B_x_A);
    ADC := dot(arcA_start, D_x_C);
    BDC := dot(arcA_end, D_x_C);
    intersect_test1 := CBA*DBA<0.0;
    intersect_test2 := ADC*BDC<0.0;
    hemisphere_test := CBA*BDC>0.0;
    return intersect_test1 && intersect_test2 && hemisphere_test;
}

//this is gaussian mapping, I think we can use this for faces, I need to do some testings, I will say these keywords so I remember in the future: store array angle prject hull into sphere radius sphere farthest point from center, cornercases arise, store stereoradians in flat array or something. Compare two spheres faces == see if orientaion angle from default (default maybe farthest)  matchs array slots.
//maybe delete comments, this is so hard to udnerstand.
QueryEdgeDirections_gauss :: (hullA: Convex_Hull, hullB: Convex_Hull) -> EdgeQuery
{
    //NOTE: deSenk version seems to be more robust than this, but you need to change contact creation.
    //:tagIshouldREallyDoThis99percentsure  --> I think it is extremely important to precompute the convex hulls per frame (each vertex in world space)...maybe ask dirk gregorius?...
    //this code is from minute -9.34 or minute 24:07, maybe I should delete all the comments once I unedrstand this...
    //ahh I need to rewatch the video on gauss map from dirk gregorius...
    //minute -20:00 in gdc or min 13:58 https://www.gdcvault.com/play/1017646/Physics-for-Game-Programmers-The
    // the faces -> vertices
    // edges -> great arcs
    //if two gauss edges intersects, they form a face on minkowsky sum//diff
    //if not. we skip these edges (IsMinkowskiFace())

    //how we check for edge intersections? minute minus 16:00
    //its separated in 2 tests:
    //      Overlap test:
    //      Hemispehre test: we build plane with any vertex of arc one, same for the other arc
    //      remaining vertices has to be on the same SIDE of that plane(@Investigate what side means)

	result := EdgeQuery.{ distance = -FLOAT32_INFINITY };
    hullA_center := Vector3.{hullA.worldSpace._14, hullA.worldSpace._24, hullA.worldSpace._34};

    bcenter: Vector3;
    acenter: Vector3;
    b1: Vector3;
    b2: Vector3;
    a1: Vector3;
    a2: Vector3;
	for :all_edges edgeA: hullA.mesh
	{
        //we precompute gauss map out of the INNER for loop :tagIshouldREallyDoThis99percentsure
		edgeA_origin:= hullA.worldSpace * ( edgeA.tail.vertex);
    	edgeA_end := hullA.worldSpace   * ( edgeA.twin.tail.vertex);

		 arcA := edgeA_end - edgeA_origin;
		 faceA_vertex := transform_normal(edgeA.face.face_plane_normal, hullA.inv_worldSpace_for_bivectors);//faceA_vertex
		 twin_faceA_vertex := transform_normal(edgeA.twin.face.face_plane_normal, hullA.inv_worldSpace_for_bivectors);;//faceA_vertex_twin?

		for :all_edges edgeB: hullB.mesh
		{
        	//TODO: (inject information as variable names and delete comments)
            edgeB_origin:= hullB.worldSpace * (edgeB.tail.vertex);
        	edgeB_end := hullB.worldSpace   * (edgeB.twin.tail.vertex);
       		arcB := edgeB_end - edgeB_origin;

            //vertex becausse in Gauss world faces are vertex
    		faceB_vertex := transform_normal(edgeB.face.face_plane_normal, hullB.inv_worldSpace_for_bivectors);//faceB_vertex
    		twin_faceB_vertex := transform_normal(edgeB.twin.face.face_plane_normal, hullB.inv_worldSpace_for_bivectors);;//faceB_vertex_twin?

			//minute -15:19 Final Overlap Test: Formulas minute 19, we can avoid cross products. and we dont need to normalize.
          	is_minkowsky_face := arcs_intersect_on_sphere(faceA_vertex, twin_faceA_vertex, -faceB_vertex, -twin_faceB_vertex);

            if is_minkowsky_face
            {
			    this_separation :=  distance_between_arcs(edgeA_origin, arcA, edgeB_origin, arcB, hullA_center);
			    bigger := this_separation >= result.distance;
                if bigger && this_separation<0.0
                {
		           result.edgeA = edgeA;
		           result.edgeB = edgeB;
		           result.distance = this_separation;
                    acenter  =edgeA_origin;
                    bcenter  =edgeB_origin;
                    a1       =faceA_vertex;
                    a2       =twin_faceA_vertex;
                    b1       =faceB_vertex;
                    b2       =twin_faceB_vertex;
                   // draw_primitive.grid(edgeA_origin, faceA_vertex,area_length=1, color=.{1,0,0,1});
                   // draw_primitive.grid(edgeA_origin, twin_faceA_vertex,area_length=1, color=.{0,1,0,1});
                   // draw_primitive.grid(edgeB_origin, faceB_vertex,area_length=1, color=.{0,1,0,1});
                   // draw_primitive.grid(edgeB_origin, twin_faceB_vertex,area_length=1, color=.{0,1,0,1});
			    }
			}
		}
	}

                   // draw_primitive.grid(acenter, a1,density=2.1,area_length=1, color=.{1,0,0,1});
                   // draw_primitive.grid(acenter, a2,density=2.1,area_length=1, color=.{1,0.6,0.6,1});
                   // draw_primitive.grid(bcenter, b1,density=2.1,area_length=1, color=.{0,1,0,1});
                   // draw_primitive.grid(bcenter, b2,density=2.1,area_length=1, color=.{0.6,1,0.6,1});


	return result;
}


CreateFaceContact :: (faceQuery: FaceQuery, referenceHull: Convex_Hull, incidentHull: Convex_Hull)-> FaceContact
{
    result: FaceContact;
    result.query = faceQuery;
    result.points.allocator = temp;
    array_reserve(*result.points, 4);

	reference_face := faceQuery.face;
	world_ref_plane_n, world_ref_plane_d := transform_plane(reference_face.face_plane_normal, reference_face.face_plane_distance, referenceHull.worldSpace, referenceHull.inv_worldSpace_for_bivectors);
	localtoinc_ref_plane_n, localtoinc_ref_plane_d := transform_plane(world_ref_plane_n, world_ref_plane_d, incidentHull.inv_worldSpace_for_bivectors, incidentHull.worldSpace);
    // draw_primitive.grid(closest_point_to_plane(.{5,5,5}, world_ref_plane_n, world_ref_plane_d), world_ref_plane_n, color=.{1,0,0,1});
	incident_face: *Hull_Face;

    /* from slide 91
        Implementation hint: To find the incident face simply iterate all faces on the other hull and compute the
        dot product of each face normal with the normal of the reference face. The face
        with the smallest dot product defines the incident face!

    */
    {
        mostAntiParallel := FLOAT32_MAX;
        for :all_faces incidentHull.mesh.faces {
            // incidentPlaneWorld_normal, incidentPlaneWorld_distance := transform_plane(current_face.face_plane_normal, current_face.face_plane_distance , incidentHull.worldSpace );
            incident_plane_normal := it.face_plane_normal;
            projection := dot(localtoinc_ref_plane_n, incident_plane_normal);
            if projection < mostAntiParallel
            {
                mostAntiParallel = projection;
                incident_face = it;
            }
        }
        assert(mostAntiParallel < 9999.0, "most antiparallel not found");
        assert(incident_face != null, "incident face not found!");
    }

    {
	n, d := transform_plane(incident_face.face_plane_normal, incident_face.face_plane_distance, incidentHull.worldSpace, incidentHull.inv_worldSpace_for_bivectors);
    // draw_primitive.grid(closest_point_to_plane(.{5,5,5}, n, d), n, color=.{.2,.2,1,1});
	}
	//collect incident edge vertices
	incident_world_vertices: [..]Vector3;
    incident_world_vertices.allocator = temp;
    array_reserve(*incident_world_vertices, incident_face.vertexCount); //this is triggering th weirdests bugs ever
    assert(incident_world_vertices.count == 0);

    //fill incident_world_vertices traversing incident face edges
    for :traverse_edge_ring current_edge: incident_face {
			point_in_world := incidentHull.worldSpace * current_edge.tail.vertex;
			array_add(*incident_world_vertices, point_in_world);
            // draw_primitive.square(point_in_world, .2, normal=incident_face.face_plane_normal);
	}


	//slide 92: We clip the incident face polygon against the side planes of the reference face Then we keep all points below the reference face

	for :traverse_edge_ring current_edge: reference_face {
	    c1 := current_edge.face ==  reference_face;
        referenceSideFace := current_edge.face;

        //NOTE: I think this logic makes no sense and I should delete
        if c1 && current_edge.twin
            then referenceSideFace = current_edge.twin.face;

		world_plane_normal, world_plane_dist  := transform_plane(referenceSideFace.face_plane_normal, referenceSideFace.face_plane_distance, referenceHull.worldSpace, referenceHull.inv_worldSpace_for_bivectors);
        // draw_primitive.grid( referenceHull.worldSpace*current_edge.tail.vertex, world_plane_normal,area_length=1, color=.{0,1,0,1});
        clipped_vertices := Sutherland_Hodgman(world_plane_normal,world_plane_dist , incident_world_vertices);

         if clipped_vertices.count == 0 {
            result.points.count = 0;
            print("[seems like you clipped all vertices, early return] maybe one hull is too pointy!");
            return result;
        }

        array_copy(*incident_world_vertices, clipped_vertices);
        incident_world_vertices.count = clipped_vertices.count;
	}

    //NOTE: maybe I should clip using reference face too-

    array_copy(*result.points, incident_world_vertices);
    array_free(incident_world_vertices);


    // for result.points if it.y < 0 then add_to_array_of_contact_points(it, radii = .1, color=.{0,0,1,1});

    reduce_contact_points_to_four_or_less(*result);
    // inc_plane_n,inc_plane_d := transform_plane(incident_face.face_plane_normal, incident_face.face_plane_distance,  incidentHull.worldSpace, incidentHull.inv_worldSpace_for_bivectors);
    assert(result.points.count <= 4, "we aer supposued o get 4 or less contact points here, we got %", result.points.count);

    // draw_primitive.grid(closest_point_to_plane(.{5,5,5}, inc_plane_n, inc_plane_d), inc_plane_n, color=.{.2,1,1,1});
    for *result.points {
        // len := distance_point_to_plane_signed(point=it.*, inc_plane_n, inc_plane_d);
        len := distance_point_to_plane_signed(point=it.*, world_ref_plane_n, world_ref_plane_d*1.0);
        is_separating_basically := len > -FLOAT32_EPSILON3;
        // draw_primitive.doLabel(tprint("is_separating_basically %            < %            == %          ",len, FLOAT32_EPSILON3, is_separating_basically), draw_primitive.WorldToScreen(.{.5,-2.2-it_index.(float),10.0}));
        if is_separating_basically {
        //     draw_primitive.doLabel(tprint("is_separating_basically %            < %            == %          ",len, FLOAT32_EPSILON3, is_separating_basically), draw_primitive.WorldToScreen(.{.5,1.2+it_index.(float),10.0}));
    //         draw_primitive.circle(it.*, 0.1, color=.{1,0,0,0});

            // draw_primitive.circle(it.*, 0.1, color=.{0,0,0,1});
            remove it;
         }
    }
    // for result.points    draw_primitive.circle(it, 0.1, color=.{0,1,0,1});

    // slide 93: Project world vertices that survived clipping onto reference plane
    //  This helps with coherence and (as we will see later) with contact point reduction
    //  where we assume that the contact points are in common plane
    for *result.points {
        it.* = closest_point_to_plane(point=it, world_ref_plane_n, world_ref_plane_d);
    }
    // for result.points   draw_primitive.X(it, 0.3, color=.{1,0,0,1});
  	return result;
}

CreateEdgeContact :: (edgeQuery: EdgeQuery, hullA: Convex_Hull, hullB: Convex_Hull) -> EdgeContact
{
	result: EdgeContact;;
	result.query = edgeQuery;

	//compute closest points between two edges
	edgeA := edgeQuery.edgeA;
	edgeA0 :=  hullA.worldSpace * (edgeA.tail.vertex);
	edgeA1 :=  hullA.worldSpace * (edgeA.next.tail.vertex);
	//variant is half edge
	edgeB := edgeQuery.edgeB;
	edgeB0 := hullB.worldSpace * (edgeB.tail.vertex);
	edgeB1 := hullB.worldSpace * (edgeB.next.tail.vertex);


	L0, L1, s, t, distdist := closest_point_segment_to_segment(edgeA0, edgeA1, edgeB0, edgeB1);

    // draw_primitive.square(edgeA0,.1);
    // draw_primitive.circle(edgeA1,.1);
    // draw_primitive.X(edgeB0,.1);
    // draw_primitive.X(edgeB1,.1);
	//compute center of closest points as contact point
	result.midpoint = (L0 + L1) * 0.5;
	//compute separating axis between two edges
	result.separationAxis = normalize(L0 - L1);
	translation:= Vector3.{hullB.worldSpace._14 , hullB.worldSpace._24, hullB.worldSpace._34 };
    if (dot(result.separationAxis, edgeB0 - translation) > 0.0)
    	result.separationAxis = -result.separationAxis;
	return result;
};

reduce_contact_points_to_four_or_less :: (faceContact: *FaceContact) {
    //we should use the same search direction to incrase the probability of always getting the same contact points in the same array position and order. For CCD you need to set the order from highest penetration to lowest.
    SEARCH_DIR ::  #run normalize(Vector3.{1,1,1});

    manifold_normal := faceContact.query.face.face_plane_normal;
    #run TODO("searchdir is not robust");
    assert(length_squared(cross_product(SEARCH_DIR, manifold_normal)) > FLOAT32_EPSILON3, "Search dir % and face % are pralalell! fix this %",SEARCH_DIR, manifold_normal, FLOAT32_EPSILON3);


    best_points_count := 0;
    //
    // First Point, furthest along search direction, search direction doesnt need to be normalized for this algorithm.
    //
    first_best_point: Vector3;
    first_point_best_distance := -FLOAT32_INFINITY; //first_best_point_best_distance? best best?
    for current_vertex : faceContact.points
	{
		vertex_signed_distance := dot(current_vertex, SEARCH_DIR); //if we get 2+ with same distance, it doesnt matter, just choose 1
        // projection := dot(current_vertex, SEARCH_DIR); //__DO NOT DELETE THIS LINE__ maybe projection is a better name. or just vertex_distance... but is signed! so vertex_signed_disance? too long. I rather never delete this comment.
		if vertex_signed_distance > first_point_best_distance
		{
		    best_points_count = 1;
			first_best_point = current_vertex;
			first_point_best_distance = vertex_signed_distance;
		}
	}

	//dont even think that -SEARCH_DIR will give the second point. It will not.

	//
	// Second point is the furthest away from first point
	//
    //NOTE: [IMPORTANT] we dont need the distances, we are good with the squared of the distances.
    second_best_point: Vector3;
    second_point_best_distance := -FLOAT32_INFINITY;
	if best_points_count == 1
	for vertex : faceContact.points
	{
		current_direction := vertex - first_best_point;
		vertex_squared_dist := length_squared(current_direction);
	    //Notice that, if faceContact.points.count == 1, vertex_squared_dist==0 then we have only 1 contact point that is face with corner
		if vertex_squared_dist > second_point_best_distance
		{
			best_points_count = 2;
			second_best_point = vertex;
			second_point_best_distance = vertex_squared_dist;
		}
	}

    //
    // Third point is the one that obeys ONE condition, 1st, maximises AREA
    //
    third_best_point: Vector3;
    third_point_best_distance := -FLOAT32_INFINITY;
    if best_points_count == 2
	for current_vertex: faceContact.points
	{
       //we call it triangle but in reality is a quad, and we dont need to divide by 1/2 because we dont care, we only care if is bigger than previous
		triangle_area_normal := cross_product(first_best_point - current_vertex, second_best_point - current_vertex);
		triangle_area := length_squared(triangle_area_normal);

		//positive winding is CCW
		if triangle_area > third_point_best_distance
		{
		    best_points_count = 3;
			third_best_point = current_vertex;
			third_point_best_distance = triangle_area;
        }
	}

    //
    // Fourth point is the one that obeys two conditions, 1st, maximises AREA, 2nd, its on the complete opposite side of the 3rd
    //[IMPORTANT] Be aware of an edge case, if it happens that we have a special convex hull with the shape of a D, AND the searh direction coincides with the flat part of the D, we will not find any point that satisfy the following algorithm.
    fourth_best_point: Vector3;
    fourth_point_best_distance := -FLOAT32_INFINITY;
    if best_points_count == 3 {
        //I need to find midpont_first_second
        //OR, we just dot prod, nah, midpoint is better.
        midpoint := (first_best_point+second_best_point)/2;
        from_midpoint_to_third := third_best_point - midpoint;
    	for current_vertex: faceContact.points
    	{
    	   from_midpoint_to_current_vertex := current_vertex - midpoint;
    	   is_on_the_left_of_CAB := dot(from_midpoint_to_current_vertex, from_midpoint_to_third) < 0;
    	   //idk what to do if dot is == 0.
    	   if !is_on_the_left_of_CAB then continue;

           //we dont need the 1/2, and the area fro the third is fixed, so we only need to maximisize this triangle area
           //the sign of normal doesnt matter either.
    	   triangle_area_normal := cross_product(first_best_point - current_vertex, second_best_point - current_vertex);
		   triangle_area := length_squared(triangle_area_normal);

		   if triangle_area > fourth_point_best_distance
    	   {
    		    best_points_count = 4;
    			fourth_best_point = current_vertex;
    			fourth_point_best_distance = triangle_area;
           }
    	}
	}

    //I should just remove the switch and force assign it even if some points are not filled.
    	//lastly, this can be done inside the previous loops actually, but anyways lastly:
    	faceContact.points.count = best_points_count;
    	if best_points_count == {
    	   case 4;
    	       faceContact.points[3] = fourth_best_point;
    	       #through;
    	   case 3;
    	       faceContact.points[2] = third_best_point;
    	       #through;
    	   case 2;
    	       faceContact.points[1] = second_best_point;
    	       #through;
    	   case 1;
    	       faceContact.points[0] = first_best_point;
    	       #through;
    	   case;
    	}


}


//
// Sutherland_Hodgman
//
Sutherland_Hodgman :: (plane_normal: Vector3, plane_distance: float, polygon_vertices: [..]Vector3) -> [..]Vector3 {
    clipped_result: [..]Vector3;
	clipped_result.allocator = temp;
    Vertex1 := polygon_vertices[polygon_vertices.count-1]; //latest vertex is start, im not sure about this.
    Distance1 := distance_point_to_plane_signed(Vertex1, plane_normal, plane_distance);
    for Index: 0..polygon_vertices.count - 1 {
        Vertex2 := polygon_vertices[ Index ]; //[LLVM REPORT]loop not vectorized: call instruction cannot be vectorized
        Distance2 := distance_point_to_plane_signed(Vertex2, plane_normal, plane_distance);
        if  Distance1 <= 0.0 && Distance2 <= 0.0 //[LLVM REPORT]:loop not vectorized: instruction cannot be vectorized
        {
        //     draw_primitive.grid(closest_point_to_plane(Vertex2, plane_normal, plane_distance), plane_normal, color=.{1,0,0,1});
    		// Both vertices are behind the plane - keep vertex2
    		array_add(*clipped_result, Vertex2);
    	}
    	else if  Distance1 <= 0.0 && Distance2 > 0.0
    	{
            // Vertex1 is behind of the plane, vertex2 is in front -> intersection point
    		Fraction := Distance1 / ( Distance1 - Distance2 );
    		IntersectionPoint := Vertex1 + Fraction * ( Vertex2 - Vertex1 );

    		// Keep intersection point
    		array_add(*clipped_result, IntersectionPoint);
        //     draw_primitive.grid(closest_point_to_plane(Vertex2, plane_normal, plane_distance), plane_normal, color=.{0,1,0,1});
    	}
    	else if ( Distance2 <= 0.0 && Distance1 > 0 )
		{
    		// Vertex2 is behind of the plane, vertex1 is in front -> intersection point
    		Fraction := Distance1 / ( Distance1 - Distance2 );
    		IntersectionPoint := Vertex1 + Fraction * ( Vertex2 - Vertex1 );

    		// Keep intersection point
    		array_add(*clipped_result, IntersectionPoint);
        //     draw_primitive.grid(closest_point_to_plane(Vertex2, plane_normal, plane_distance), plane_normal, color=.{0,0,1,1});

    		// And also keep vertex2
    		array_add(*clipped_result, Vertex2); //[LLVM REPORT]loop not vectorized: control flow cannot be substituted for a select
		}

        // draw_primitive.grid(closest_point_to_plane(Vertex2, plane_normal, plane_distance), plane_normal);
		// Keep vertex2 as starting vertex for next edge
		Vertex1 = Vertex2;
		Distance1 = Distance2;
    }
    return clipped_result;
}



dirk_gregorius_rubikon_2019 :: #string CPP
RnArray< RnVector3 > rnClipPolygon( const RnArray< RnVector3 >& Polygon, const RnPlane& Plane, int Edge )
	{
	RN_ASSERT( Polygon.Size() >= 3 );
	RnArray< RnVector3 > Out;

	RnVector3 Vertex1 = Polygon.Back();
	float Distance1 = rnDistance( Plane, Vertex1 );

	for ( int Index = 0; Index < Polygon.Size(); ++Index )
		{
		RnVector3 Vertex2 = Polygon[ Index ];
		float Distance2 = rnDistance( Plane, Vertex2 );

		if ( Distance1 <= 0.0f && Distance2 <= 0.0f )
			{
			// Both vertices are behind the plane - keep vertex2
			Out.PushBack( Vertex2 );
			}
		else if ( Distance1 <= 0.0f && Distance2 > 0.0f )
			{
			// Vertex1 is behind of the plane, vertex2 is in front -> intersection point
			float Fraction = Distance1 / ( Distance1 - Distance2 );
			RnVector3 IntersectionPoint = Vertex1.Position + Fraction * ( Vertex2.Position - Vertex1.Position );

			// Keep intersection point
			Out.PushBack( IntersectionPoint );
			}
		else if ( Distance2 <= 0.0f && Distance1 > 0 )
			{
			// Vertex2 is behind of the plane, vertex1 is in front -> intersection point
			float Fraction = Distance1 / ( Distance1 - Distance2 );
			RnVector3 IntersectionPoint = Vertex1.Position + Fraction * ( Vertex2.Position - Vertex1.Position );

			// Keep intersection point
			Out.PushBack( IntersectionPoint );

			// And also keep vertex2
			Out.PushBack( Vertex2 );
			}

		// Keep vertex2 as starting vertex for next edge
		Vertex1 = Vertex2;
		Distance1 = Distance2;
		}﻿

	return Out;
	}
CPP

dirk_gregorius_rubikon_2015 :: #string CPP
//--------------------------------------------------------------------------------------------------
void rnClipPolygon( rnPolygon& Out, const rnPolygon& Polygon, const rnPlane& Plane )
	{
        RN_ASSERT( Out.Empty() );
	RN_ASSERT( Polygon.Size() >= 3 );

	rnVertex Vertex1 = Polygon.Back();
	float Distance1 = rnDistance( Plane, Vertex1.Position );

	for ( int Index = 0; Index < Polygon.Size(); ++Index )
		{
		rnVertex Vertex2 = Polygon[ Index ];
		float Distance2 = rnDistance( Plane, Vertex2.Position );

		if ( Distance1 <= 0.0f && Distance2 <= 0.0f )
			{
			// Both vertices are behind the plane - keep vertex2
			Out.PushBack( Vertex2 );
			}
		else if ( Distance1 <= 0.0f && Distance2 > 0.0f )
			{
			// Vertex1 is behind the plane, vertex2 is in front -> intersection point
			float Fraction = Distance1 / ( Distance1 - Distance2 );
			rnVector3 Position = Vertex1.Position + Fraction * ( Vertex2.Position - Vertex1.Position );

			// Keep intersection point
			rnVertex Vertex;
			Vertex.Position = Position;
			Out.PushBack( Vertex );
			}
		else if ( Distance2 <= 0.0f && Distance1 > 0 )
			{
			// Vertex2 is behind the plane, vertex1 is in front -> intersection point
			float Fraction = Distance1 / ( Distance1 - Distance2 );
			rnVector3 Position = Vertex1.Position + Fraction * ( Vertex2.Position - Vertex1.Position );

			// Keep intersection point
			rnVertex Vertex;
			Vertex.Position = Position;
			Out.PushBack( Vertex );

			// And also keep vertex2
			Out.PushBack( Vertex2 );
			}

		// Keep vertex2 as starting vertex for next edge
		Vertex1 = Vertex2;
		Distance1 = Distance2;
		}
	}

CPP


//
// Collide Shape X vs Shape Y
//
collide_sphere_vs_sphere :: (sphere_center_A: Vector3, sphere_radius_A: float, sphere_center_B: Vector3, sphere_radius_B: float) ->
    num_contacts: int,
    penetration: float,
    world_normal_towards_A: Vector3,
    world_midpoint: Vector3,
    world_contact_point_on_A: Vector3,
    world_contact_point_on_B: Vector3
{
    num_contacts: int;
    penetration: float;
    world_normal_towards_A: Vector3;
    world_midpoint: Vector3;
    world_contact_point_on_A: Vector3;
    world_contact_point_on_B: Vector3;
    //unpack sphere 1 data
    r1 := sphere_radius_A;
    C1 := sphere_center_A;

    //unpack sphere 2 data
    r2 := sphere_radius_B;
    C2 := sphere_center_A;

    penetration                  = distance(C2, C1) - r1 - r2;
    if penetration < 0 {
        d := penetration; //to write less
        world_normal_towards_A   = normalize(C1 - C2, fallback = .{}); //already pointing towards A center
        assert(length_squared(world_normal_towards_A)>0.001, "TWO SPHERES SPAWNED AT THE SAME POSITION! your problem might be something else, I only have problems with this assert when the world explodes on my test scenes obviusly");
        num_contacts             = 1;
        //TODO: Change C1 to C2 and invert sign on these 3 lines, well it works so maybe dont touch.
        world_midpoint           = C1 - (r1+d/2)*world_normal_towards_A;
        world_contact_point_on_A = C1 - (r1)*world_normal_towards_A;
        world_contact_point_on_B = C1 - (r1+d)*world_normal_towards_A;
    }

    return num_contacts, penetration, world_normal_towards_A, world_midpoint, world_contact_point_on_A, world_contact_point_on_B;
};

collide_sphere_vs_capsule  :: (sphere_center: Vector3, sphere_radius: float, capsule_start_line: Vector3, capsule_end_line: Vector3, capsule_radius: float ) ->
    num_contacts: int,
    penetration: float,
    world_normal_towards_A: Vector3,
    world_midpoint: Vector3,
    world_contact_point_on_A: Vector3,
    world_contact_point_on_B: Vector3
{

    num_contacts: int;
    penetration: float;
    world_normal_towards_A: Vector3;
    world_midpoint: Vector3;
    world_contact_point_on_A: Vector3;
    world_contact_point_on_B: Vector3;

    // unpack sphere data
    //not needed

    // unpack capsule data
    capsule_height := distance(capsule_start_line, capsule_end_line);

    // project sphere position vector into line segment,
    capsule_direction := normalize(capsule_end_line - capsule_start_line);
    to_sphere := sphere_center - capsule_start_line;
    t := clamp(dot(to_sphere, capsule_direction), 0, capsule_height);
    closest_point_on_line := capsule_start_line + t * capsule_direction;
    //

    from_line_to_sphere := sphere_center - closest_point_on_line;
    penetration                  = length(from_line_to_sphere) - sphere_radius - capsule_radius;
    if penetration < 0 {
        d := penetration; //to write less
        world_normal_towards_A   = normalize(from_line_to_sphere, fallback = .{0,0,0});
        assert(length_squared(world_normal_towards_A)>0.001, "sphere spawned inside capsule");
        num_contacts             = 1;
        world_midpoint           = closest_point_on_line +  (capsule_radius  +d/2)*world_normal_towards_A ;
        world_contact_point_on_A = closest_point_on_line + (capsule_radius + d)*world_normal_towards_A ;
        world_contact_point_on_B = closest_point_on_line + (capsule_radius)*world_normal_towards_A;
    }

    return num_contacts, penetration, world_normal_towards_A, world_midpoint, world_contact_point_on_A, world_contact_point_on_B;
};


//another possible name collide_convex_vs_convex
collide_hull_vs_hull :: (hull_A: Convex_Hull, hull_B: Convex_Hull) ->
    num_contacts             : int,
    penetration              : float,
    world_normal_towards_A   : Vector3,
    world_midpoints          : [4]Vector3,
    world_contact_points_on_A: [4]Vector3,
    world_contact_points_on_B: [4]Vector3
{

    num_contacts            : int;
    penetration             : float;
    world_normal_towards_A  : Vector3;
    world_midpoints          : [4]Vector3;
    world_contact_points_on_A: [4]Vector3;
    world_contact_points_on_B: [4]Vector3;


    query: SAT_Query;

    hull0: Convex_Hull;
    hull1: Convex_Hull;
    //:tagCheckTemporaryHackMovedHere_Maybe_Cache_by_orientation_diff
    hull0 = hull_A;
    hull1 = hull_B;

    collision := SAT_dirk_gregorius(hull0, hull1, *query);

    faceDistanceA :=  query.faceQuery_AB.distance ;
	faceDistanceB :=  query.faceQuery_BA.distance ;
	edgeDist := query.edgeQuery.distance;


    if query.edgeQuery.edgeA {
    draw_current_edge(query.edgeQuery.edgeA, hull0,color=.{1,1,1,1});
    draw_current_edge(query.edgeQuery.edgeB, hull1,color=.{1,1,1,1});
    }
    draw_primitive.doLabel(tprint("e %",edgeDist), draw_primitive.WorldToScreen(.{.1,2.0,2.0}));
    draw_primitive.doLabel(tprint("A %",faceDistanceA), draw_primitive.WorldToScreen(.{.1,1.5,2.0}));
    draw_primitive.doLabel(tprint("B %",faceDistanceB), draw_primitive.WorldToScreen(.{.1,1.8,2.0}));
    if collision {
		//grep this from https://www.gamedev.net/forums/topic/667499-3d-sat-problem/?page=2
		//slide 90+: The solution is simple and we apply a bias (or weighting) to prefer face contacts
        //  over edge contacts and one face axis over another
        // kLinearSlop ::  0.005 ;
        // kRelEdgeTolerance ::  0.90 ;
        // kRelFaceTolerance ::  0.98 ;
        // kAbsTolerance ::  0.5 * kLinearSlop ;

		kLinearSlop ::  0.005 ;
		kRelEdgeTolerance ::  0.90 ;
		kRelFaceTolerance ::  0.98 ;
		kAbsTolerance ::  0.5 * kLinearSlop ;

        is_near_edge := query.edgeQuery.distance > kRelEdgeTolerance * max(faceDistanceA, faceDistanceB) + kAbsTolerance;
        is_near_faceA := faceDistanceB > (kRelFaceTolerance* faceDistanceA + kAbsTolerance);
        // is_near_edge = collider_switch_face;
        is_near_faceB := !is_near_faceA;

        // if query.edgeQuery.distance != -FLOAT32_INFINITY
        //   if faceDistanceB > edgeDist || faceDistanceB > edgeDist {
        //    is_near_edge=true;
        //   };

		if is_near_edge
		{

			contact : EdgeContact = CreateEdgeContact(query.edgeQuery,hull0,hull1);
			{
                world_normal_towards_A = -contact.separationAxis;
                // assert(is_normalized(world_normal_towards_A));
                penetration = contact.query.distance;
                if penetration < 0 {
                    num_contacts = 1;
                    world_midpoints[0] = contact.midpoint;
                    world_contact_points_on_A[0] = contact.midpoint + (penetration/2.0)*world_normal_towards_A;
                    world_contact_points_on_B[0] = contact.midpoint - (penetration/2.0)*world_normal_towards_A;
                }
			}

            draw_primitive.doLabel(tprint("is_near_edge % ",penetration), draw_primitive.WorldToScreen(.{0.5,1.2,3.0}));
		}
		else if is_near_faceA
		{
		    contact :FaceContact= CreateFaceContact(query.faceQuery_BA, hull1,hull0); //reference,incindet
            {
                world_normal_towards_A           = transform_normal(contact.query.face.face_plane_normal,hull1.inv_worldSpace_for_bivectors);
                // assert(is_normalized(world_normal_towards_A));
                penetration                      = contact.query.distance;
                assert(contact.points.count <= 4, "we got more than 4 contact points! contact.points.count == %",contact.points.count);
                if penetration < 0
                {
                    num_contacts = contact.points.count;
                    for point_in_B: contact.points
                    {
                        world_midpoints[it_index]           = point_in_B + (penetration/2.0) * world_normal_towards_A;
                        world_contact_points_on_A[it_index] = point_in_B + penetration * world_normal_towards_A;
                        world_contact_points_on_B[it_index] = point_in_B;
                    }
// if num_contacts draw_primitive.grid(contact.points[0]+world_normal_towards_A*0.2, world_normal_towards_A,density=3,area_length=1, color=.{.6,1.1,.6,1});


                }
            }
            draw_primitive.doLabel(tprint("is_near_faceA %",penetration), draw_primitive.WorldToScreen(.{.1,1.2,2.0}));
		}
		else
		{
		    contact := CreateFaceContact(query.faceQuery_AB, hull0, hull1);
            {
                world_normal_towards_A = -transform_normal(contact.query.face.face_plane_normal,hull0.inv_worldSpace_for_bivectors);
                // assert(is_normalized(world_normal_towards_A));
                penetration = contact.query.distance;

                assert(contact.points.count <= 4, "we got more than 4 contact points! AB contact.points.count == %",contact.points.count);
                if penetration < 0
                {
                    num_contacts = contact.points.count;
                    for point_in_A: contact.points
                    {
                        world_midpoints[it_index]           = point_in_A - (penetration/2.0) * world_normal_towards_A;
                        world_contact_points_on_A[it_index] = point_in_A;
                        world_contact_points_on_B[it_index] = point_in_A - penetration*world_normal_towards_A;
                    }
// if num_contacts draw_primitive.grid(contact.points[0]+world_normal_towards_A*0.2, world_normal_towards_A,density=3,area_length=1, color=.{1.1,1.1,1.1,1});

                }
            draw_primitive.doLabel(tprint("is_near_faceB %",penetration), draw_primitive.WorldToScreen(.{.1,1.2,2.0}));
            }


		} //end edge or faceBA or faceAB test

	} //end if collision

    draw_primitive.doLabel(tprint("N % edgedistance  % ",num_contacts, query.edgeQuery.distance ), draw_primitive.WorldToScreen(.{0.5,2.2,2}));
    return num_contacts, penetration, world_normal_towards_A, world_midpoints, world_contact_points_on_A, world_contact_points_on_B ;

};


////////////
// START OF AI-GENERATED CODE
//its not 100% ai, but I dont like these procedures, to be replaced.

collide_sphere_vs_hull :: (
    sphere_center: Vector3, sphere_radius: float,
    hull: Convex_Hull
) ->
    num_contacts: int,
    penetration: float,
    world_normal_towards_A: Vector3,
    world_midpoint: Vector3,
    world_contact_point_on_A: Vector3,
    world_contact_point_on_B: Vector3
{
    num_contacts: int;
    penetration: float;
    world_normal_towards_A: Vector3;
    world_midpoint: Vector3;
    world_contact_point_on_A: Vector3;
    world_contact_point_on_B: Vector3;

    smallest_penetration: float = FLOAT32_INFINITY;
    best_point_on_hull: Vector3;
    best_normal: Vector3;
    best_normal2: Vector3;
    on_hull: Vector3;

    hull_center := Vector3.{hull.worldSpace._14, hull.worldSpace._24, hull.worldSpace._34};
    for :all_edges e, e_idx : hull.mesh
    {

        v0 := hull.worldSpace *e.tail.vertex;
        v1 := hull.worldSpace *e.next.tail.vertex;
        v2 := hull.worldSpace *e.next.next.tail.vertex;

        // Face plane already in local space
        face_normal, face_distance := transform_plane(e.face.face_plane_normal, e.face.face_plane_distance, hull.worldSpace, hull.inv_worldSpace_for_bivectors);
        assert(length_squared(face_normal) > 0.001, "invalid normal %", face_normal);
        // Distance from sphere center to face plane


        // points_towards_A := dot(face_normal, sphere_center - hull_center) < 0.0;
        // if !points_towards_A then face_normal = -face_normal;
        dist_to_plane := distance_point_to_plane_signed(sphere_center, face_normal, face_distance);
        if dist_to_plane > sphere_radius continue;
        // Early out if sphere is too far from this face
        // if dist_to_plane > sphere_radius continue;
        closest_point_on_triangle := closest_point_to_triangle(sphere_center, v0, v1, v2);
        from_triangle_to_sphere := sphere_center - closest_point_on_triangle;
        distance_to_surface := length(from_triangle_to_sphere);
        current_penetration := distance_to_surface - sphere_radius;

        if current_penetration < smallest_penetration {
                best_normal2 = face_normal;
                on_hull = v0;
            smallest_penetration = current_penetration;
            best_point_on_hull = closest_point_on_triangle;
            //@Hack
            if distance_to_surface > 0.0001 {
                best_normal = normalize(from_triangle_to_sphere);
            } else {
                best_normal = face_normal;
            }
        }
    }

    penetration = smallest_penetration;

    if penetration < 0 {
        d := penetration;

        world_normal_towards_A = best_normal;

        points_towards_A := dot(world_normal_towards_A, sphere_center - hull_center) < 0.0;
        if points_towards_A then world_normal_towards_A = -world_normal_towards_A;

        assert(length_squared(world_normal_towards_A) > 0.001, "invalid normal %", world_normal_towards_A);

        num_contacts = 1;
        world_contact_point_on_A = sphere_center - sphere_radius * world_normal_towards_A;
        world_contact_point_on_B = best_point_on_hull;
        world_midpoint = best_point_on_hull + (d / 2.0) * world_normal_towards_A;
        assert(d<0);
    }

    return num_contacts, penetration, world_normal_towards_A, world_midpoint, world_contact_point_on_A, world_contact_point_on_B;
}


collide_capsule_vs_capsule :: (
    capsule_A_start: Vector3, capsule_A_end: Vector3, capsule_A_radius: float,
    capsule_B_start: Vector3, capsule_B_end: Vector3, capsule_B_radius: float
) ->
    num_contacts: int,
    penetration: float,
    world_normal_towards_A: Vector3,
    world_midpoint: Vector3,
    world_contact_point_on_A: Vector3,
    world_contact_point_on_B: Vector3
{
    num_contacts: int;
    penetration: float;
    world_normal_towards_A: Vector3;
    world_midpoint: Vector3;
    world_contact_point_on_A: Vector3;
    world_contact_point_on_B: Vector3;

    // Find closest points between the two line segments
	closest_point_on_A, closest_point_on_B, s, t, distdist := closest_point_segment_to_segment(capsule_A_start, capsule_A_end, capsule_B_start, capsule_B_end);

    // Vector from B's closest point to A's closest point
    from_B_to_A := closest_point_on_A - closest_point_on_B;
    distance_between_axes := length(from_B_to_A);

    penetration = distance_between_axes - capsule_A_radius - capsule_B_radius;

    if penetration < 0 {
        d := penetration;
        world_normal_towards_A = normalize(from_B_to_A, fallback = .{0, 1, 0});
        assert(length_squared(world_normal_towards_A) > 0.001, "capsule axes coincident");
        num_contacts = 1;

        // Contact points on the surface of each capsule
        world_contact_point_on_A = closest_point_on_A - capsule_A_radius * world_normal_towards_A;
        world_contact_point_on_B = closest_point_on_B + capsule_B_radius * world_normal_towards_A;
        world_midpoint = closest_point_on_B + (capsule_B_radius + d / 2.0) * world_normal_towards_A;
    }

    return num_contacts, penetration, world_normal_towards_A, world_midpoint, world_contact_point_on_A, world_contact_point_on_B;
}




collide_capsule_vs_hull :: (
    capsule_start: Vector3, capsule_end: Vector3, capsule_radius: float,
    hull: Convex_Hull
) ->
    num_contacts: int,
    penetration: float,
    world_normal_towards_A: Vector3,
    world_midpoint: Vector3,
    world_contact_point_on_A: Vector3,
    world_contact_point_on_B: Vector3
{
    num_contacts: int;
    penetration: float;
    world_normal_towards_A: Vector3;
    world_midpoint: Vector3;
    world_contact_point_on_A: Vector3;
    world_contact_point_on_B: Vector3;

    // Transform capsule to hull's local space
    local_capsule_start := transform_point(capsule_start, hull.inv_worldSpace_for_bivectors);
    local_capsule_end := transform_point(capsule_end, hull.inv_worldSpace_for_bivectors);

    // Find closest point on hull to capsule axis
    closest_point_on_capsule_local: Vector3;
    closest_point_on_hull_local: Vector3;
    closest_distance_sq := FLOAT32_INFINITY;

    // Check distance from capsule axis to each face
    for :all_faces face, face_idx: hull.mesh {
        // Find closest point on capsule segment to face plane
        capsule_direction := normalize(local_capsule_end - local_capsule_start);
        capsule_length := distance(local_capsule_start, local_capsule_end);

        // Project capsule endpoints onto plane and find closest point on segment
        dist_start := dot(local_capsule_start, face.face_plane_normal) - face.face_plane_distance;
        dist_end := dot(local_capsule_end, face.face_plane_normal) - face.face_plane_distance;

        // Find where segment intersects plane or get closest endpoint
        t: float;
        if abs(dist_start - dist_end) > 0.0001 {
            t = clamp(dist_start / (dist_start - dist_end), 0.0, 1.0);
        } else {
            t = ifx abs(dist_start) < abs(dist_end) then 0.0 else 1.0;
        }

        point_on_capsule := local_capsule_start + t * capsule_length * capsule_direction;
        point_on_plane := point_on_capsule - (dot(point_on_capsule, face.face_plane_normal) - face.face_plane_distance) * face.face_plane_normal;

        // Check if point is inside face
        is_inside := true;
        for :traverse_edge_ring edge: face {
            if dot(point_on_plane, edge.face.face_plane_normal) - edge.face.face_plane_distance > 0.0001 {
                is_inside = false;
                break;
            }
        }

        if is_inside {
            dist_sq := length_squared(point_on_capsule - point_on_plane);
            if dist_sq < closest_distance_sq {
                closest_point_on_capsule_local = point_on_capsule;
                closest_point_on_hull_local = point_on_plane;
                closest_distance_sq = dist_sq;
            }
        }
    }

    // Check distance from capsule to all edges
    for :all_edges edge: hull.mesh {
        v0 := edge.tail.vertex;
        v1 := edge.next.tail.vertex;

    	pt_capsule, pt_edge, s, t, distdist := closest_point_segment_to_segment(local_capsule_start, local_capsule_end, v0, v1);

        dist_sq := length_squared(pt_capsule - pt_edge);
        if dist_sq < closest_distance_sq {
            closest_point_on_capsule_local = pt_capsule;
            closest_point_on_hull_local = pt_edge;
            closest_distance_sq = dist_sq;
        }
    }

    // Transform back to world space
    closest_point_on_capsule := transform_point(closest_point_on_capsule_local, hull.worldSpace);
    closest_point_on_hull := transform_point(closest_point_on_hull_local, hull.worldSpace);

    // Calculate penetration
    from_hull_to_capsule := closest_point_on_capsule - closest_point_on_hull;
    distance_between := length(from_hull_to_capsule);
    penetration = distance_between - capsule_radius;

    if penetration < 0 {
        d := penetration;
        world_normal_towards_A = normalize(from_hull_to_capsule, fallback = .{0, 1, 0});
        assert(length_squared(world_normal_towards_A) > 0.001, "capsule axis intersects hull");
        num_contacts = 1;

        world_contact_point_on_A = closest_point_on_capsule - capsule_radius * world_normal_towards_A;
        world_contact_point_on_B = closest_point_on_hull;
        world_midpoint = closest_point_on_hull + (d / 2.0) * world_normal_towards_A;
    }

    return num_contacts, penetration, world_normal_towards_A, world_midpoint, world_contact_point_on_A, world_contact_point_on_B;
}
// END OF AI-GENERATED CODE
////////////


//  Quick_hull 2D, O(nlogn)
/*
  1- Find 2 most extreme points x,y, we get first_line
  2- find most extrme point from first_line, we get initial_hull
  3- separate remaining points into three cathegories, closest point to each face of initial_hull, three because initial_hull is a triangle in 2D
  4- we can discard internal points on step 3 too. Each face of step 3 holds a conflict_list
  BAD->5- for each face and its own conflict_list, we repeat from step 2, but with 1 difference
 iter
  5-> for the 3 conflict lists, we pick the one that has the largest distance from the hull, we get eye_point
  6-> we discard faces that can be seen fron eye_point, and find two point thta connects from visible to nonvisible , these two vertices are horizon_1, horizon_2
  7-> we create two faces that connects eye_point to horizon_1, and horizon_2
  8-> before doing the discard of faces of step 6, we need to recover its conflict list and redistribute it along the two new ffaces
  9-> repeat with a new eye_point

seems like is not that simple, we need some convexity tests due to floating point stufff
we grab the contigous faces , in 2d a face is 2 vertices (edge) they have a shared vertex and two extreme vertex each belonging to each face, so we grab one face and check if the vertex of the other face is under, we repeat for the other, if both true then its convex
we use fat planes. if point between epsilon, we consider it coplanar, else other cases. s=dot(plane_normal_A, vertex_extreme_B) - plane_distance_A, if s>epsilon then in front,
SO WE USE that result to Merge faces, if they are coplanar we merge the faces, and their remaining conflict lists
we take epsilon to be 2(max_positive x + max_positive y ) * FLT_EPSILON
*/
//  Quick_hull 3D, O(nlogn)
/*
    we do the 2d version to build a triangle
    then we take triangle plane and find furthest point
    we have a tethrahedron as initial_hull

    Horizon in 3d we do a Deepth First Search,
    conflict face S
    cross edge and visit faces until face is not visible, offending edge is part of the orizon

    //I was thinking that everytime I "allocate" an edge, I should allocate a twin, that might be the way.
*/
quickhull_3D :: (/*read only*/cloud_of_points: []Vector3) -> Convex_Hull {
    assert(cloud_of_points.count >=4); //maybe assert !coplanarity too
    new_hull: Convex_Hull;
    using new_hull;
    //
    //allocate entire hull for worst case scenario  (except conflict list)
    //
    worst_num_vertices := cloud_of_points.count;
    _worst_num_edges := 3 * cloud_of_points.count - 6; //underscore to avoid using it accidentally, we use half edge!
    worst_num_faces := 2 * cloud_of_points.count - 4;
    assert(worst_num_vertices-_worst_num_edges+worst_num_faces==2, "euler formula failed");
    worst_num_half_edges := 2 * _worst_num_edges;

    mesh.vertexBuffer.count = worst_num_vertices;
    mesh.edgeBuffer.count = worst_num_half_edges;
    mesh.faceBuffer.count = worst_num_faces;

    alloc_size_u8 :=
                    size_of(Hull_Vertex)   * worst_num_vertices +
                    size_of(Hull_Half_Edge)* worst_num_half_edges +
                    size_of(Hull_Face)     * worst_num_faces;

    alldata := alloc(alloc_size_u8);
	assert(alldata!=null, "allocation failed");
    memset(alldata, 0, alloc_size_u8);


    mesh.vertexBuffer.data = alldata;
    mesh.edgeBuffer.data = cast(*Hull_Half_Edge)(mesh.vertexBuffer.data+mesh.vertexBuffer.count);
    mesh.faceBuffer.data = cast(*Hull_Face)(mesh.edgeBuffer.data+mesh.edgeBuffer.count);

    //
    //we fill the vertex memory with the cloud data and attach them to eachother to form the freelist
    //
    {
        prev: *Hull_Vertex;
        for cloud_of_points
        {
            vert_ptr := *mesh.vertexBuffer[it_index];
            vert_ptr.vertex = it;
            if prev
            {
                vert_ptr.linked_list_prev = null;//<- important null
                prev.linked_list_next = vert_ptr;
            }
            prev = vert_ptr;
        }
        //because this is a special case its safe to do the following line.
        new_hull.mesh.vertex_free_list = *mesh.vertexBuffer[0];


        for mesh.vertexBuffer {
        // log("idx % v %", it_index, it);
        }
    }


    //
    //we also fill edges and faces free lists
    //
    {
        prev: *Hull_Face;
        for 0..worst_num_faces - 1
        {
            face_ptr := *mesh.faceBuffer[it];
            if prev
            {
                face_ptr.linked_list_prev = null; //<- important null
                prev.linked_list_next = face_ptr;
            }
            prev = face_ptr;
        }
        //because this is a special case its safe to do the following line.
        new_hull.mesh.face_free_list = *mesh.faceBuffer[0];
    }
    {
        prev: *Hull_Half_Edge;
        for 0..worst_num_half_edges - 1
        {
            hedge_ptr := *mesh.edgeBuffer[it];
            if prev
            {
                hedge_ptr.linked_list_prev = null;  //<- important null
                prev.linked_list_next = hedge_ptr;
            }
            prev = hedge_ptr;
        }
        //because this is a special case its safe to do the following line.
        new_hull.mesh.edge_free_list = *mesh.edgeBuffer[0];
    }


    //
    //build initial hull: NOTE: I think I should do this using iterators to do this
    //
    //TODO: search four points that maximize their per-pair-distance instead.
    p0,p1, idx0, idx1 := find_furthest_points(cloud_of_points);
    p2, idx2 := find_furthest_away_from_line(cloud_of_points, p0, p1);

    p3: Vector3;
    idx3: int;
    {
        plane_normal, plane_dist := create_plane(p0,p1,p2);
        t1,tmp_idx1 := find_furthest_point_along_dir(cloud_of_points, plane_normal);
        t2,tmp_idx2 := find_furthest_point_along_dir(cloud_of_points, -plane_normal);
        dist1 := distance_point_to_plane(t1, plane_normal, plane_dist);
        dist2 := distance_point_to_plane(t2, plane_normal, plane_dist);
        dist_1_is_farther := dist1 > dist2;
        p3   = ifx dist_1_is_farther then t1 else t2;
        idx3 = ifx dist_1_is_farther then tmp_idx1 else tmp_idx2;
    }

    //new idea, given an horizon, generate a hull!

    {//I think this is ok to be hardcoded, its just to build the first tetrahedron
        using mesh;
        //we remove the 4 vertices from freelist, takind advantage of continuos memory
        v0 := add_vertex_from_slot(*mesh,p0, idx0);
        v1 := add_vertex_from_slot(*mesh,p1, idx1);
        v2 := add_vertex_from_slot(*mesh,p2, idx2);
        v3 := add_vertex_from_slot(*mesh,p3, idx3);

        face_A, e01, e12, e20 := create_face_from_vertices(*mesh, v0, v1, v2);
        face_B, e02, e23, e30 := create_face_from_vertices(*mesh, v0, v2, v3);
        face_C, e31, e10, e03 := create_face_from_vertices(*mesh, v3, v1, v0);
        face_D, e21, e13, e32 := create_face_from_vertices(*mesh, v2, v1, v3);

        link_two_faces_by_edge(e01, e10);
        link_two_faces_by_edge(e02, e20);
        link_two_faces_by_edge(e03, e30);
        link_two_faces_by_edge(e12, e21);
        link_two_faces_by_edge(e23, e32);
        link_two_faces_by_edge(e31, e13);
        #if 0
            {
            //[WARNING] TO FIX THIS YOU SHOULD ALSO MODIFY FACE PLANE NORMALS, SO I WOULD RATHER NOT DO THIS
             //This is just to veryfy that we are in centroid 0 0 0
            pp:[4]Vector3;
            pp[0] = p0;
            pp[1] = p1;
            pp[2] = p2;
            pp[3] = p3;
            centroid := compute_centroid(pp);
            log("pp2 % %", pp,centroid);

            v0.vertex -= centroid;
            v1.vertex -= centroid;
            v2.vertex -= centroid;
            v3.vertex -= centroid;


            pp[0] = v0.vertex;
            pp[1] = v1.vertex;
            pp[2] = v2.vertex;
            pp[3] = v3.vertex;



            centroid = compute_centroid(pp);
            log("pp2 % %", pp,centroid);
            assert(length_squared(centroid) < 0.000001, "%",centroid);

        }
    } //block hardcored initial tetrahedron

    //
    // generate conflic list for initial tetrahedron
    //

    //from triangle to tetrahedron
    while true {
        conflict_vert := NextConflictVertex(*new_hull.mesh);
        if !conflict_vert break;
        AddVertexToHull(*new_hull.mesh, conflict_vert );
    }

    //
    // Expand convex hull or mshtign
    //


    return new_hull;
};


are_coplanar :: (p0: Vector3, p1: Vector3, p2: Vector3, p3: Vector3, error_margin: float = 0.00001) -> bool {
    u := p1 - p0;
    v := p2 - p0;
    w := p3 - p0;

    normal := cross(u, v);
    signed_volume := dot(normal, w);

    return abs(signed_volume) <= error_margin;
}


link_two_faces_by_edge :: (edge1: *Hull_Half_Edge, edge2: *Hull_Half_Edge) {
    assert(edge1.tail == edge2.next.tail, "Edges must form opposite directions of same geometric edge");
    assert(edge1.next.tail == edge2.tail, "Edges must form opposite directions of same geometric edge");
    #run TODO("maybe add a linkedlist of main edges and twin edges separated");
    edge1.twin = edge2;
    edge2.twin = edge1;
}


is_plane_pointing_away_from_center :: (hull: Convex_Hull, n: Vector3, point_on_plane: Vector3) -> bool {
    start := hull_center(hull);
    end :=point_on_plane;
    is_pointing_away := (dot(n,end - start)>0.0);
    return is_pointing_away;

}

is_plane_pointing_away_from_origin :: (n: Vector3, point_on_plane: Vector3) -> bool {
    end :=point_on_plane;
    is_pointing_away := (dot(n,end )>0.0);
    return is_pointing_away;

}

create_face_from_vertices ::  (using mesh: *Half_Edge_Mesh, v0: *Hull_Vertex, v1: *Hull_Vertex, v2: *Hull_Vertex , v3: *Hull_Vertex  ) -> face:*Hull_Face, new_edge0: *Hull_Half_Edge, new_edge1: *Hull_Half_Edge,new_edge2: *Hull_Half_Edge, new_edge3: *Hull_Half_Edge {
    assert(are_coplanar(v0.vertex,v1.vertex,v2.vertex,v3.vertex));
    e0 := pop_free_edge(mesh);
    e1 := pop_free_edge(mesh);
    e2 := pop_free_edge(mesh);
    e3 := pop_free_edge(mesh);

    assert(e0 != null);
    assert(e1 != null);
    assert(e2 != null);
    assert(e3 != null);

    e0.prev = e3;
    e0.next = e1;
    e1.prev = e0;
    e1.next = e2;
    e2.prev = e1;
    e2.next = e3;
    e3.prev = e2;
    e3.next = e0;

    //assign vertex
    e0.tail = v0;
    e1.tail = v1;
    e2.tail = v2;
    e3.tail = v3;


    //add to edge linked list
    {
        e0.linked_list_next = e1;
        e1.linked_list_next = e2;
        e2.linked_list_next = e3;
        e3.linked_list_next = mesh.edges; //:tag_if_list_is_empty_this_is_null_perfect!
        mesh.edges = e0;
        mesh.edgeCount += 4;
    }


    // plane_normal, plane_d := create_plane(v0.vertex, v1.vertex, v2.vertex);
    plane_normal, plane_d := create_plane(v0.vertex, v1.vertex, v2.vertex);
    assert(is_plane_pointing_away_from_origin(plane_normal, v3.vertex), "your vertex ordering is not CCW! % ",v3.vertex);

    face0 := pop_free_face(mesh);
    assert(face0 != null);

    face0.edge =  e0;//because we do this when linking 2 faces throug 1 edge
    face0.face_plane_normal = plane_normal;
    face0.face_plane_distance = plane_d; // Distance from origin to face
    face0.vertexCount = 4;

    e0.face = face0;
    e1.face = face0;
    e2.face = face0;
    e3.face = face0;


    //prepend to faces
    {
        face0.linked_list_next = mesh.faces;  //:tag_if_list_is_empty_this_is_null_perfect!
        mesh.faces = face0;
        mesh.faceCount += 1;
    }
    return face0, e0, e1, e2, e3;
}


create_face_from_vertices ::  (using mesh: *Half_Edge_Mesh, v0: *Hull_Vertex, v1: *Hull_Vertex, v2: *Hull_Vertex) -> f:*Hull_Face, e0: *Hull_Half_Edge, e1: *Hull_Half_Edge,e2: *Hull_Half_Edge {
    // assert(are_coplanar(v0.vertex,v1.vertex,v2.vertex,v3.vertex));
    e0 := pop_free_edge(mesh);
    e1 := pop_free_edge(mesh);
    e2 := pop_free_edge(mesh);

    assert(e0 != null);
    assert(e1 != null);
    assert(e2 != null);

    assert(e0.linked_list_prev == null);
    assert(e1.linked_list_prev == null);
    assert(e2.linked_list_prev == null);
    assert(e0.linked_list_next == null);
    assert(e1.linked_list_next == null);
    assert(e2.linked_list_next == null);

    e0.prev = e2;
    e0.next = e1;
    e1.prev = e0;
    e1.next = e2;
    e2.prev = e1;
    e2.next = e0;

    //assign vertex
    e0.tail = v0;
    e1.tail = v1;
    e2.tail = v2;

    //add to edge linked list
    {
        e0.linked_list_next = e1;
        e1.linked_list_next = e2;
        e2.linked_list_next = mesh.edges;  //:tag_if_list_is_empty_this_is_null_perfect!
        mesh.edges = e0;
        mesh.edgeCount += 3;
    }

    plane_normal, plane_d := create_plane(v0.vertex, v1.vertex, v2.vertex);
    // assert(is_plane_pointing_away_from_origin(plane_normal, v2.vertex), "your vertex ordering is not CCW! % % %, % %, cross:%", v0.vertex, v1.vertex, v2.vertex, v1.vertex-v0.vertex, v2.vertex-v0.vertex, plane_normal);

    face0 := pop_free_face(mesh);
    assert(face0 != null);

    face0.edge =  e0;
    face0.face_plane_normal = plane_normal;
    face0.face_plane_distance = plane_d;
    face0.vertexCount = 3;

    e0.face = face0;
    e1.face = face0;
    e2.face = face0;

    //prepend to faces
    {
        face0.linked_list_next = mesh.faces; //:tag_if_list_is_empty_this_is_null_perfect!
        mesh.faces = face0;
        mesh.faceCount += 1;
    }
    return face0, e0, e1, e2;
}

Ring_Linked_List :: struct {
    linked_list_prev: *#this;
    linked_list_next: *#this;
}

// place_left_before_right :: (to_place: *$T/interface Ring_Linked_List, /*linked_list_node*/NODE: *T) {


//             //bookeeping, hard.
//             in_the_middle := NODE.linked_list_prev && NODE.linked_list_next;
//             in_the_next_extreme := !NODE.linked_list_next;
//             in_the_prev_extreme := !NODE.linked_list_prev;

//             assert((in_the_prev_extreme.(int) + in_the_next_extreme.(int) + in_the_middle.(int)) == 1, "bad free list");
//             if in_the_middle {
//                 NODE.linked_list_prev.linked_list_next = NODE.linked_list_next;
//                 NODE.linked_list_next.linked_list_prev = NODE.linked_list_prev;
//             } else if in_the_prev_extreme {
//                 NODE.linked_list_next.linked_list_prev = null;
//                 mesh.vertex_free_list = NODE.linked_list_next; //<-important here
//             } else if in_the_next_extreme {
//                 NODE.linked_list_prev.linked_list_next = null;
//             } else {
//                 assert(false);
//             }
// }

add_edge_and_twin_and_maybe_face ::  (using mesh: *Half_Edge_Mesh, v0: *Hull_Vertex, v1: *Hull_Vertex, v2: *Hull_Vertex , v3: *Hull_Vertex  ) -> new_edge0: *Hull_Half_Edge, new_edge1: *Hull_Half_Edge,new_edge2: *Hull_Half_Edge, new_edge3: *Hull_Half_Edge {
    //WHY COMMENTED OUT (because I think there is no deterministic wway to attach 1 vertex to some of the multiple available edges imagine cube corner vertex, 3 edges shared that vertex)check if v0 and v1 are already attached to an edge
    // assert(v0.edge == null);
    // assert(v1.edge == null);

    //allocate edges and twins
    e0 := pop_free_edge(mesh); te0 := pop_free_edge(mesh);
    e1 := pop_free_edge(mesh); te1 := pop_free_edge(mesh);
    e2 := pop_free_edge(mesh); te2 := pop_free_edge(mesh);
    e3 := pop_free_edge(mesh); te3 := pop_free_edge(mesh);

    assert(e0 != null);  assert(te0 != null);
    assert(e1 != null);  assert(te1 != null);
    assert(e2 != null);  assert(te2 != null);
    assert(e3 != null);  assert(te3 != null);

    //assign twins
    e0.twin = te0; te0.twin = e0;
    e1.twin = te1; te1.twin = e1;
    e2.twin = te2; te2.twin = e2;
    e3.twin = te3; te3.twin = e3;

    //assign prev next ring (extra care for twins)
    //example, e0.twin.next.twin.tail == e3.tail == e0.next.tail == v3
    e0.prev = e3;  te0.prev = te1;
    e0.next = e1;  te0.next = te3;
    e1.prev = e0;  te1.prev = te2;
    e1.next = e2;  te1.next = te0;
    e2.prev = e1;  te2.prev = te3;
    e2.next = e3;  te2.next = te1;
    e3.prev = e2;  te3.prev = te0;
    e3.next = e0;  te3.next = te2;

    //assign vertex (extra care for twins) ordered by .next
    e0.tail = v0; te0.tail = v1;//te0 = v1            is correct
    e1.tail = v1; te3.tail = v0;//te3 = te0.next = v0 is correct
    e2.tail = v2; te2.tail = v3;//te2 = te3.next = v3 is correct
    e3.tail = v3; te1.tail = v2;//te1 = te2next = v2  is correct


    //because edges are rings, I need another next to traverse nontrwin edges, I sure this will not work for now
    //I try to avoid twins, but this probably is not gonna work
    e0.linked_list_next = e1;
    e1.linked_list_next = e2;
    e2.linked_list_next = e3;

    e3.linked_list_next = mesh.edges;
    mesh.edges = e0;

    edgeCount += 8;

    return e0, e1, e2, e3;
}


add_edge_and_twin_and_maybe_face ::  (using mesh: *Half_Edge_Mesh, v0: *Hull_Vertex, v1: *Hull_Vertex ) -> new_edge0: *Hull_Half_Edge, new_edge1: *Hull_Half_Edge {
    //WHY COMMENTED OUT (because I think there is no deterministic wway to attach 1 vertex to some of the multiple available edges imagine cube corner vertex, 3 edges shared that vertex)check if v0 and v1 are already attached to an edge
    // assert(v0.edge == null);
    // assert(v1.edge == null);
    e0 := pop_free_edge(mesh);
    e1 := pop_free_edge(mesh);
    assert(e0 != null);
    assert(e1 != null);
    e0.twin = e1;
    e1.twin = e0;

    e0.next = e1;
    e1.prev = e0;

    //read WHY COMMENTED OUT above
    // v0.edge = e0;
    // v1.edge = e1;

    e0.tail = v0;
    e1.tail = v1;

    //maybe insert middle tip end
    is_first_edge := edgeCount == 0;
    if is_first_edge {
        // edgeBuff
    } else {
        end_of_list := *edgeBuffer[edgeCount];
        end_of_list.next = e0;
        e1.next = *edgeBuffer[0]; //complete ring buffer.
    }
    //NOTE: I think we never traverse the edge ring directly... we do it by faces or vertices instead.
    edgeCount += 2;

    return e0, e1;
}

all_vertices :: (vertex: *Hull_Vertex, body: Code, flags: For_Flags) #expand {
    `start_vertex := vertex;
    `it := start_vertex;
    `it_index := 0;
     while true {
        defer if it == null || it == start_vertex break;
        defer it = it.linked_list_next;
        defer it_index += 1;
        #insert body;
    }
}

all_vertices :: (mesh: Half_Edge_Mesh, body: Code, flags: For_Flags) #expand {
    `start_vertex := mesh.vertices;
    `it := start_vertex;
    `it_index := 0;
     while true {
        defer if it == null || it == start_vertex break;
        defer it = it.linked_list_next;
        defer it_index += 1;
        #insert body;
    }
}

all_faces :: (face: *Hull_Face, body: Code, flags: For_Flags) #expand {
    `start_face := face;
    `it := start_face;
    `it_index := 0;
     while true {
        defer if it == null || it == start_face break;
        defer it = it.linked_list_next;
        defer it_index += 1;

        #insert body;
    }
}

all_faces :: (mesh: Half_Edge_Mesh, body: Code, flags: For_Flags) #expand {
    `start_face := mesh.faces;
    `it := start_face;
    `it_index := 0;
     while true {
        defer if it == null || it == start_face break;
        defer it = it.linked_list_next;
        defer it_index += 1;
        #insert body;
    }
}

all_edges :: (mesh: Half_Edge_Mesh, body: Code, flags: For_Flags) #expand {
    `start_edge := mesh.faces.edge;
    `it := start_edge;
    `it_index := 0;
     while true {
        defer if it == null ||it == start_edge break;
        defer it = it.linked_list_next;
        defer if it.linked_list_next == null {draw_primitive.doLabel(tprint("NULL, %   ->idx %           ->start%              ->end", it_index, start_edge, it), draw_primitive.WorldToScreen(.{2,2 + it_index.(float)*0.2,2}));};
        defer it_index += 1;
        #insert body;
    }
}

traverse_edge_ring :: (face: Hull_Face, body: Code, flags: For_Flags) #expand {
    `start_edge := face.edge;
    `it := start_edge;
    `it_index := 0;
     while true {
        defer if it == start_edge break;
        defer it = it.next;
        defer it_index += 1;
        #insert body;
    }
}


traverse_edge_ring :: (edge: *Hull_Half_Edge, body: Code, flags: For_Flags) #expand {
    `start_edge := edge;
    `it := start_edge;
    `it_index := 0;
     while true {
        defer if it == start_edge break;
        defer it = it.next;
        defer it_index += 1;
        #insert body;
    }
}




AddVertexToHull :: (mesh:  *Half_Edge_Mesh, conflict_vertex: *Hull_Vertex){
	// Remove conflict_vertex from conflict face
    face := conflict_vertex.conflict_face;
    conflict_vertex.conflict_face = null;
    removed := array_ordered_remove_by_value(*face.conflict_list, conflict_vertex);
    assert(removed);

    //  Add to hull
	new_vert := add_vertex(mesh, vertex=conflict_vertex.vertex);
	new_vert.* = conflict_vertex;

    // Find the horizon edges
    horizon: [..]*Hull_Half_Edge;
    // BuildHorizon(conflict_vertex, conflict_face, *horizon);

    NewFaces: [..]*Hull_Face;
    // BuildNewFaces(mesh, conflict_vertex, *NewFaces, *horizon);
    // MergeFaces(hull, NewFaces);

    // ResolveOrphans(hull, NewFaces);

}

NextConflictVertex :: ( mesh:  *Half_Edge_Mesh) -> *Hull_Vertex
{
    conflict_vertex: *Hull_Vertex = null;
    max_so_far: float = -FLOAT32_INFINITY;

    // start_face := mesh.faces;
    // current_face := start_face;
    for :all_faces current_face: mesh {
    // while true { defer if current_face == start_face break;
        if current_face.conflict_list.count != 0 {

            vertex := current_face.conflict_list[0];
            while true {
                defer if vertex == vertex.linked_list_next then break;
                defer vertex = vertex.linked_list_next;
                assert(vertex.conflict_face == current_face, "seems like the conflic list is wrong");
                this_dist := distance_point_to_plane_signed(vertex.vertex, current_face.face_plane_normal, current_face.face_plane_distance);
                if this_dist > max_so_far {
                    conflict_vertex = vertex;
                    max_so_far = this_dist;
                }
            }
        }
        //while loop face related:
    //     current_face = current_face.linked_list_next;
    }
    return conflict_vertex;
}


pop_from_free_list :: (free_list: **$T/.[Hull_Vertex,Hull_Half_Edge, Hull_Face ], zero_init := true) -> *T {
    assert(free_list != null);
    if free_list.* == null
        return null; //early return;
    popped := free_list.*;
    free_list.* = popped.linked_list_next;
    if free_list.* free_list.*.linked_list_prev = null;//little sanitization
    if zero_init popped.* = .{};
    return popped;
}

pop_free_vertex :: (mesh: *Half_Edge_Mesh, zero_init := true) -> *Hull_Vertex {
    return pop_from_free_list(*mesh.vertex_free_list, zero_init);
}

pop_free_edge :: (mesh: *Half_Edge_Mesh, zero_init := true) -> *Hull_Half_Edge {
    return pop_from_free_list(*mesh.edge_free_list, zero_init);
}

pop_free_face :: (mesh: *Half_Edge_Mesh, zero_init := true) -> *Hull_Face {
    return pop_from_free_list(*mesh.face_free_list, zero_init);
}


//I dont like having this procedure, so I put 2 underscore to it until I figure out a better way to do this
__pop_free_vertex_at :: (mesh: *Half_Edge_Mesh, idx: int) -> *Hull_Vertex {
    ptr_to_remove := *mesh.vertexBuffer[idx];
    popped: *Hull_Vertex;
    previous: *Hull_Vertex;
    for :all_vertices mesh.vertex_free_list {
        should_remove := it == ptr_to_remove;
        if should_remove  {
            popped = it;
            if previous {
                previous.linked_list_next = popped.linked_list_next; //if in middle, squash list
            } else {
                mesh.vertex_free_list = popped.linked_list_next; //if remove first then shove popped next.
            }
            popped.linked_list_next = null; //sanitization

            assert(popped!=null);
            break; //early break;
        }
        previous = it;
    }

    assert(popped!=null);
    return popped;
}

//I dont like having this procedure, so I put 2 underscore to it until I figure out a better way to do this
__pop_free_edge_at :: (mesh: *Half_Edge_Mesh, idx: int) -> *Hull_Half_Edge {
    //
    //NOTE: probably never used
    //
    idx_0 := cast(int)mesh.edgeBuffer.data;
    idx_to_remove := idx_0 + idx;
    popped: *Hull_Half_Edge;
    for :traverse_edge_ring mesh.edge_free_list
    {
        should_remove := it.(int) == idx_to_remove;

        if idx_to_remove
        {
            popped = it;
            //bookeeping, hard.
            in_the_middle := it.prev && it.next;
            in_the_next_extreme := !it.next;
            in_the_prev_extreme := !it.prev;

            assert((in_the_prev_extreme.(int) + in_the_next_extreme.(int) + in_the_middle.(int)) == 1, "bad free list");
            if in_the_middle {
                it.prev.next = it.next;
                it.next.prev = it.prev;
            } else if in_the_prev_extreme {
                it.next.prev = null;
                mesh.edge_free_list = it.next; //<-important here
            } else if in_the_next_extreme {
                it.prev.next = null;
            } else {
                assert(false);
            }

            break;
        }
    }
    return popped;
}


//I dont like having this procedure, so I put 2 underscore to it until I figure out a better way to do this
__pop_free_face_at :: (mesh: *Half_Edge_Mesh, idx: int) -> *Hull_Face {
    //
    //NOTE: probably never used
    //
    idx_0 := cast(int)mesh.faceBuffer.data;
    idx_to_remove := idx_0 + idx;
    popped: *Hull_Face;
    for :all_faces mesh.face_free_list
    {
        should_remove := it.(int) == idx_to_remove;

        if idx_to_remove
        {
            popped = it;
            //bookeeping, hard.
            in_the_middle := it.linked_list_prev && it.linked_list_next;
            in_the_next_extreme := !it.linked_list_next;
            in_the_prev_extreme := !it.linked_list_prev;

            assert((in_the_prev_extreme.(int) + in_the_next_extreme.(int) + in_the_middle.(int)) == 1, "bad free list");
            if in_the_middle {
                it.linked_list_prev.linked_list_next = it.linked_list_next;
                it.linked_list_next.linked_list_prev = it.linked_list_prev;
            } else if in_the_prev_extreme {
                it.linked_list_next.linked_list_prev = null;
                mesh.face_free_list = it.linked_list_next; //<-important here
            } else if in_the_next_extreme {
                it.linked_list_prev.linked_list_next = null;
            } else {
                assert(false);
            }

            break;
        }
    }
    return popped;
}






append_feature :: (using mesh: *Half_Edge_Mesh, new_feat: *$T, to: *T) {
        #if T == {
            case Hull_Vertex;
                vertexCount+=1;
            case Hull_Half_Edge;
                edgeCount+=1;
            case Hull_Face;
                faceCount+=1;
            case; #assert(false);
        }
        feat_to_add: *T;
        if (to)
		{
		    feat_to_add = to;
			backup := feat_to_add.linked_list_prev;
			backup.linked_list_next = new_feat;
			new_feat.linked_list_prev = backup;
		}
		else
			feat_to_add = new_feat;

		new_feat.linked_list_next = feat_to_add;
		feat_to_add.linked_list_prev = new_feat;

}

remove_feature :: (using mesh: *Half_Edge_Mesh, to_remove: *$T, to_iterate_helper: *T) {
    #if T == {
        case Hull_Vertex;
            vertexCount-=1;
        case Hull_Half_Edge;
            edgeCount-=1;
        case Hull_Face;
            faceCount-=1;
        case; #assert(false);
    }

	if to_iterate_helper == to_remove
		to_iterate_helper = to_iterate_helper.next;

	to_remove.prev.next = to_remove.next;
	to_remove.next.prev = to_remove.prev;
}


add_vertex_from_slot :: (using mesh: *Half_Edge_Mesh, vertex: Vector3, free_list_slot_idx: int) -> new_vertex: *Hull_Vertex {
    //this procedure is for when we use free_list also as our cloud of points source of information.
    v0 := __pop_free_vertex_at(mesh, free_list_slot_idx);
    v0.linked_list_prev = null;
    v0.linked_list_next = mesh.vertices;
    assert(v0.vertex == vertex, "idx % vertex not equal % == %",free_list_slot_idx, v0.vertex, vertex);
    mesh.vertices = v0;
    vertexCount += 1;
    return v0;
}

add_vertex :: (using mesh: *Half_Edge_Mesh, vertex: Vector3) -> new_vertex: *Hull_Vertex {
    v0 := pop_free_vertex(mesh);
    v0.linked_list_next = mesh.vertices;
    v0.vertex = vertex;
    mesh.vertices = v0;
    vertexCount += 1;
    return v0;
}

add_edge :: (using mesh: *Half_Edge_Mesh) -> new_edge: *Hull_Half_Edge {
    result := pop_free_edge(mesh);
    if !result  {
        result = *edgeBuffer[edgeCount];
        assert(edgeCount <= num_allocated_edge);
    }
    edgeCount += 1;
    return result;
}

add_face :: (using mesh: *Half_Edge_Mesh ) -> new_face: *Hull_Face {
    result := pop_free_face(mesh);
    if !result {
        result = *faceBuffer[faceCount];
        assert(faceCount <= num_allocated_face);
    }
    faceCount += 1;
    return result;
}

//this is from NotARealCaveman/SAT
// BuildNewFaces :: (mesh: *Half_Edge_Mesh, vertex: *Hull_Vertex, newFaces: *[..]*Hull_Face, horizon: *[..]*Hull_Half_Edge)
// {
//     newEdges: *[..]*Hull_Half_Edge;

// 	//pairs newly created edges with their twins
// 	FindTwin :: (vertex: *Hull_Vertex, edge: *Hull_Half_Edge, newEdges: [..]*Hull_Half_Edge) -> *Hull_Half_Edge
// 	{
// 	twin:  *Hull_Half_Edge;
// 	   for potentialTwin: newEdges {
//             if edge.tail == vertex {
//                 if potentialTwin.tail == edge.toponext.tail {
//                     twin = potentialTwin;
//                     break;
//                 }
//                 if potentialTwin.tail == vertex && potentialTwin.next.tail == edge.tail
//                     twin = potentialTwin;
//                     break;
//             }
// 	   }

// 		if twin == newEdges[newEdges.count-1] return null;

// 		return twin;
// 	};


// 	for edge : horizon.*
// 	{
// 		e0 :=  add_edge(mesh);
// 		e1 :=  edge;
// 		e2 :=  add_edge(mesh);

// 		face := add_face(mesh);

// 		//set inital edge/vertex count
// 		face.vertexCount = 3;
// 		//set pointers
// 		face.edge = e0;
// 		e0.face = face;
// 		e1.face = face;
// 		e2.face = face;
// 		//eye point
// 		e0.prev = e2;
// 		e0.next = e1;
// 		e0.tail = vertex;
// 		//store e2 tail before updating horizon edge
// 		e2.tail = edge.next.tail;
// 		//horizon edge, retains old tail and twin
// 		e1.prev = e0;
// 		e1.next = e2;
// 		//adjoining edge
// 		e2.prev = e1;
// 		e2.next = e0;
// 		//add new edges and faces
// 		array_add(newEdges, e0);
// 		array_add(newEdges, e2);
// 		array_add(newFaces, face);
// 		append_feature(mesh, face, null);
// 		for newEdge: newEdges.* {
// 			if newEdge.twin == FindTwin(vertex, newEdge, newEdges.*) {
// 				newEdge.twin.twin = newEdge;
// 			}
// 		}
// 	}
// }



//
// TODO: a good title for this category.
//
find_furthest_points :: (vertices: []Vector3) -> extreme_1: Vector3, extreme_2: Vector3, idx_1: int, idx_2: int
{
	extreme_1: Vector3;
	extreme_2: Vector3;
	max := -FLOAT32_INFINITY;
	idx_1 :int= -1;
	idx_2 :int= -1;

    for v1, id1: vertices
    {
        for v2, id2: vertices
        {
            current := distance_squared(v1, v2);
    		if current > max
    		{
                extreme_1 = v1;
                extreme_2 = v2;
    			max = current;
    			idx_1 = id1;
    			idx_2 = id2;
    		}
		}
    }

	return  extreme_1, extreme_2, idx_1, idx_2;
}

find_furthest_away_from_line :: (vertices: []Vector3, line_start: Vector3, line_end: Vector3) -> furthest_point: Vector3, idx: int
{
	furthest_point: Vector3;
	max_so_far := -FLOAT32_INFINITY;
	idx :int= -1;

    line_dir_n := normalize(line_end - line_start);

    for point: vertices
    {

       hypothenuse := point - line_start;
       proj := dot(hypothenuse, line_dir_n)*line_dir_n;
       perpendicular_to_line := point-proj;
       current_dist := length_squared(perpendicular_to_line);

        if current_dist > max_so_far {
            max_so_far = current_dist;
            furthest_point = point;
            idx = it_index;
        }
    }

	return  furthest_point, idx;
}

// find_furthest_away_from_plane :: (vertices: []Vector3, plane_normal: Vector3, plane_dist: float) -> furthest_point: Vector3, idx: int
// {
// 	furthest_point: Vector3;
// 	max_so_far := -FLOAT32_INFINITY;
// 	idx :int= -1;

//     line_dir_n := normalize(segment_end - segment_start);

//     for v: vertices
//     {

//        hypothenuse := point - line_start;
//        proj := dot(hypothenuse, line_dir_n)*line_dir_n;
//        perpendicular_to_line := point-proj;
//        current_dist := length_squared(perpendicular_to_line);

//         if current_dist > max_so_far {
//             max_so_far = current_dist;
//             furthest_point = v;
//             idx = it_index;
//         }
//     }

// 	return  furthest_point, idx;
// }

find_furthest_point_along_dir :: (vertices: []Vector3, local_direction: Vector3) -> local_furhtest_point: Vector3, idx: int
{
	local_furhtest_point: Vector3;
	max := -FLOAT32_INFINITY;
	idx :int= -1;

    for localPoint: vertices //loop not vectorized: value that could not be identified as reduction is used outside the loop
    {
        current := dot(localPoint, local_direction); //no point in normalizing local_direction
		if current > max
		{
			local_furhtest_point = localPoint;
			max = current;
			idx = it_index; //[LLVM REPORT]: loop not vectorized: control flow cannot be substituted for a select
		}
    }

	return  local_furhtest_point, idx;
}



find_furthest_point_along_dir :: (hull: Convex_Hull, world_direction: Vector3) -> vertex_ptr: *Hull_Vertex, world_furhtest_point: Vector3, idx: int
{
	world_furhtest_point: Vector3;
	max := -FLOAT32_INFINITY;
	idx :int= -1;
	vertex_ptr: *Hull_Vertex;

    for :all_vertices hull.mesh.vertices {
		localPoint := it.vertex;
		current_dist := dot(hull.worldSpace*localPoint, world_direction);
		if current_dist > max
		{
			idx = it_index;
			world_furhtest_point = hull.worldSpace*localPoint;
			max = current_dist;
			vertex_ptr = it;
		}
    }

	return  vertex_ptr, world_furhtest_point, idx;
}


find_furthest_point_along_local_dir :: (hull: Convex_Hull, local_direction: Vector3) -> vertex_ptr: *Hull_Vertex, local_furhtest_point: Vector3, idx: int
{
	local_furhtest_point: Vector3;
	max := -FLOAT32_INFINITY;
	idx := -1;
	vertex_ptr: *Hull_Vertex;

    for :all_vertices hull.mesh.vertices {
		localPoint := it.vertex;
		current_dist := dot(localPoint, local_direction);
		if current_dist > max
		{
			idx = it_index;
			local_furhtest_point = localPoint;
			max = current_dist;
			vertex_ptr = it;
		}
	}

	return  vertex_ptr, local_furhtest_point, idx;
}
slide43 :: #string CPP
 Distance(edgeA, edgeB, poly)-> float {
dirA := edgeA.end -edgeA.start;
dirB := edgeB.end -edgeB.start;
pointA := edgeA.Head();
pointB := edgeB.Head()
;

}
CPP;
distance_between_arcs :: (pointA: Vector3, arcA: Vector3, pointB: Vector3, arcB: Vector3, world_hull_centroid: Vector3) -> float
{
    //slide 43,
	crossAB := cross_product(arcA, arcB);

	area := length(crossAB);
	are_edges_parallel := area <= FLOAT32_EPSILON3 * sqrt(length_squared(arcA) * length_squared(arcB) ) ;
	if are_edges_parallel return -FLOAT32_INFINITY;

	normal := crossAB / area;

	points_towards_A := dot(normal, pointA - world_hull_centroid) < 0.0;
	if points_towards_A then normal = -normal;

	return dot(normal, pointB - pointA); //no need to compute support points: O(1)
}


transform_point :: (point: Vector3, M: Matrix4) -> Vector3 #symmetric {
    return Vector3.{
         point.x*M._11 + point.y*M._12 + point.z*M._13 + M._14,
         point.x*M._21 + point.y*M._22 + point.z*M._23 + M._24,
         point.x*M._31 + point.y*M._32 + point.z*M._33 + M._34
    };
}

transform_normal :: inline (normal: Vector3, /*use inverse to get result in world -> */M: Matrix4) -> new_normal: Vector3 {
    new_normal: Vector3 = ---;
    new_normal.x  = normal.x * M._11 + normal.y * M._21 +  normal.z * M._31;
    new_normal.y  = normal.x * M._12 + normal.y * M._22 +  normal.z * M._32;
    new_normal.z  = normal.x * M._13 + normal.y * M._23 +  normal.z * M._33;

	return normalize(new_normal);
}

transform_plane :: (plane: Plane, m: Matrix4, m_inv: Matrix4) -> Plane {
    transformed_normal, transformed_distance := transform_plane(plane.normal, plane.distance, m, m_inv);
    return Plane.{transformed_normal, transformed_distance};
}

transform_plane :: (plane_normal: Vector3, plane_distance: float, m: Matrix4, inv_m: Matrix4) -> transformed_normal: Vector3, transformed_distance: float {
    transformed_normal := transform_normal(plane_normal, inv_m);
    point_on_plane := plane_normal * plane_distance;
    transformed_point := transform_point(point_on_plane, m); // transformed_point := m*point_on_plane;
    //dirk way is the same as this way? assert will assure that: //after testing-> I think Dirk way doesnt support scaling inside m, so I deleted it.
    transformed_distance := dot(transformed_point, transformed_normal);
    return transformed_normal, transformed_distance;
}

 calculate_inverse_or_use_transpose :: (hull: Convex_Hull) -> inv_or_t_worldSpace: Matrix4 {
    inv_worldSpace_for_bivectors: Matrix4 = ---;
    //because calculating inverses is super expensive.
    scale_squared := Vector3.{
        length_squared(Vector3.{hull.worldSpace._11, hull.worldSpace._21, hull.worldSpace._31}),
        length_squared(Vector3.{hull.worldSpace._12, hull.worldSpace._22, hull.worldSpace._32}),
        length_squared(Vector3.{hull.worldSpace._13, hull.worldSpace._23, hull.worldSpace._33}),
    };
    should_use_transpose := scale_squared.x == 1.0 && scale_squared.y == 1.0 && scale_squared.z == 1.0;
    if false && should_use_transpose {
        inv_worldSpace_for_bivectors = transpose(hull.worldSpace);
    } else {
        success:, inv_worldSpace_for_bivectors = inverse(hull.worldSpace);
        assert(success, "error calculating inverse of hull");
    }
    return inv_worldSpace_for_bivectors;
}
