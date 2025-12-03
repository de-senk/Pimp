QueryFaceDirections :: (hullA: Convex_Hull, hullB: Convex_Hull) -> FaceQuery {
    early_return := false;
    result: FaceQuery;
    result.face = null;
    result.distance = -FLOAT32_INFINITY;

    start := hullA.mesh.faces;
    current_face := start;
    assert(current_face != null, "[ERROR] hullA.mesh.faces==null... are hullA and hullB created?");

    while true { defer if current_face == start break;
        world_planeA := transform_plane(.{current_face.face_plane_normal, current_face.face_plane_distance}, hullA.worldSpace);

        local_direction_in_hull1 := transform_direction(-world_planeA.normal, hullB.inv_worldSpace);

        furthest_vertex_local, vertex_index := find_furthest_point_along_local_dir(hullB, local_direction_in_hull1);
        furthest_vertex_world := transform_point(furthest_vertex_local, hullB.worldSpace);

        distance := distance_point_to_plane_signed(
            point = furthest_vertex_world,
            plane_normal = world_planeA.normal,
            plane_distance = world_planeA.distance
        );


        if distance > result.distance {
            result.face = current_face;
            result.distance = distance; //most separating OR if negative: most closest to the penetration separation
            basically_separating := result.distance > EPSILON4;
            if basically_separating && early_return {
                result.distance  = FLOAT32_INFINITY;
                return result;
            }
        }

        current_face = current_face.next;

    }

    return result;
}

//NOTE: Quickhull should  make the geometric center at {0,0,0}
project_all_vertices_of_hull_into_axis :: (hull: Convex_Hull, axis: Vector3)-> left_interval: float, right_interval:float {
    axis_n := normalize(axis);
    // left_interval: float = FLOAT32_INFINITY;
    // right_interval: float = -FLOAT32_INFINITY;
    translation := Vector3.{hull.worldSpace._14, hull.worldSpace._24, hull.worldSpace._34};
    world_geometric_center := translation; //lets assume geometric center is in the middle, otherwise, user did hull wrong
    center_proj := dot(world_geometric_center, axis_n);

    //so we expand the interval [left, right] from the world hull center because I cannot brain how to do it with ininity's
    left_interval: float = center_proj;
    right_interval: float = center_proj;

    //THOUGHTS(tr):
    //I think I can embed the dot product inside hull.worlSpace, so I would create a copy of the model matrix and modify it so it also applies dot product
    //even better: we can just recreate the hull in world space, combined with caching per frame, its worth it maybe, double memory tho.
    //but we only use hulls in SAT calculations, so maybe we need to combine all these with the SAT caching logic.
    start := hull.mesh.vertices;
    current := start;
    while true {
        defer current = current.next; //executed second //TODO: replace defers with normal code here.
        defer if current.next == start then break; //executed first
        local_vertex := current.vertex;
        world_vertex := hull.worldSpace * local_vertex;
        proj := dot(world_vertex, axis_n);
        //I dont use continue here because I want to hint the compiler to make this branchless
        if proj > right_interval then right_interval = proj;
        if proj < left_interval  then left_interval = proj;
        }
    return left_interval, right_interval;
}

//this is no gaussmap, n^2 or n^3, very expensive.
QueryEdgeDirections_bruteforce :: (hullA: Convex_Hull, hullB: Convex_Hull) -> EdgeQuery
{
    //ahh I need to rewatch the video on gauss map from dirk gregorius...

	result := EdgeQuery.{ distance = -FLOAT32_INFINITY };
    //are the dot accesors my bottleneck? NOTE: [IMPORTANT] the dot product is not my bottleneck, it was the profiler instrumentation that made the dot product slow, They are fast!
    //we store the edges in the key of the hashtable I think.
	for indexA, edgeA : hullA.mesh.edges //traverse hash table
	{
		assert(edgeA  != null, "edgeA  is null");
		assert(edgeA .twin != null, "edgeA .twin is null % ",hullB.mesh.edges);
		assert(edgeA .twin.twin != null, "edgeA .twin.twin is null");
		assert(edgeA .twin.twin == edgeA , "edge != edge.twin.twin % %", edgeA , edgeA .twin.twin);

		for indexB, edgeB : hullB.mesh.edges //traverse hash table
		{
			assert(edgeB != null, "edgeB is null");
			assert(edgeB.twin != null, "edgeB.twin is null % ",hullB.mesh.edges);
			assert(edgeB.twin.twin != null, "edgeB.twin.twin is null");
			assert(edgeB.twin.twin == edgeB, "edge != edge.twin.twin");


            edgeA_dir: Vector3;
            {
               edgeOrigin := hullA.worldSpace * (hullA.scale * edgeA.tail.vertex);
	           edgeDestination := hullA.worldSpace * (hullA.scale*edgeA.twin.tail.vertex);
	           edgeA_dir = edgeDestination - edgeOrigin;
	        }

            edgeB_dir: Vector3;
            {
               edgeOrigin := hullB.worldSpace * (hullA.scale * edgeB.tail.vertex);
	           edgeDestination := hullB.worldSpace * (hullA.scale*edgeB.twin.tail.vertex);
	           edgeB_dir = edgeDestination - edgeOrigin;
    	    }

            axis := cross_product(edgeA_dir, edgeB_dir);
            leftA, rightA := project_all_vertices_of_hull_into_axis(hullA, axis);
            leftB, rightB := project_all_vertices_of_hull_into_axis(hullB, axis);

            //check your milton diagrams:
            this_separation: float = ---;
            //I probably did a mistake here
            if leftB>leftA && rightB > rightA
                then this_separation  = leftB-rightA;
                else this_separation  = leftA-rightB;
            //if penetrationg then this_separation is negative. but, we want to ignore such cases I guess? idk why, I would use them anyways saving them as penetration distance

            // if this_separation >= result.distance //NOTE: switching this seems important
            if this_separation <= result.distance
            	continue;

            result = .{ edgeA,edgeB , this_separation };
            if result.distance > EPSILON4
            	return result;
		}
	}

	return result;
}

//this is gaussian mapping, I think we can use this for faces, I need to do some testings, I will say these keywords so I remember in the future: store array angle prject hull into sphere radius sphere farthest point from center, cornercases arise, store stereoradians in flat array or something. Compare two spheres faces == see if orientaion angle from default (default maybe farthest)  matchs array slots.
//maybe delete comments, this is so hard to udnerstand.
QueryEdgeDirections :: (hullA: Convex_Hull, hullB: Convex_Hull) -> EdgeQuery
{
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

    //NOTE(tr): @Important We dont need to traverse all edges! because we have duplicated edges! rememebr twin's!! wait? do we? @Investigate figure that out.
    //I will leave the old version commented overlapped with the correct version
	for unused1, edgeA : hullA.mesh.edges //traverse hash table
	{
		assert(edgeA != null, "edgeA is null");
		assert(edgeA.twin != null, "edgeA.twin is null % ",hullB.mesh.edges);
		assert(edgeA.twin.twin != null, "edgeA.twin.twin is null");
		assert(edgeA.twin.twin == edgeA, "edge != edge.twin.twin % %", edgeA, edgeA.twin.twin);
        //for some reason I got a performance boost flattening out BuildMinkowskiFace procedure,but I used an eyemeter...

        //we precompute gauss map out of the INNER for loop :tagIshouldREallyDoThis99percentsure
		edgeA_origin:= hullA.worldSpace * (hullA.scale * edgeA.tail.vertex);
    	edgeA_end := hullA.worldSpace * (hullA.scale*edgeA.twin.tail.vertex);

    	//TODO: (inject information as variable names and delete comments) //how_to: read the naming? to the left edeA_ is what it was, and to the rigght _arc or _vertex is what it becomes after gauss mapping.
		 arcA := edgeA_end - edgeA_origin;
		 faceA_vertex := transform_vector3(edgeA.face.face_plane_normal, hullA.inv_worldSpace);//faceA_vertex
		 twin_faceA_vertex := transform_vector3(edgeA.twin.face.face_plane_normal, hullA.inv_worldSpace);;//faceA_vertex_twin?
        //this works, if you change this make sure to run the collision tests

		for unused2, edgeB : hullB.mesh.edges //traverse hash table
		{
			assert(edgeB != null, "edgeB is null");
			assert(edgeB.twin != null, "edgeB.twin is null % ",hullB.mesh.edges);
			assert(edgeB.twin.twin != null, "edgeB.twin.twin is null");
			assert(edgeB.twin.twin == edgeB, "edge != edge.twin.twin");

        	//TODO: (inject information as variable names and delete comments)
            edgeB_origin:= hullB.worldSpace * (hullB.scale * edgeB.tail.vertex);
        	edgeB_end := hullB.worldSpace * (hullB.scale*edgeB.twin.tail.vertex);
       		arcB := edgeB_end - edgeB_origin;

    		faceB_vertex := transform_vector3(edgeB.face.face_plane_normal, hullB.inv_worldSpace);//faceB_vertex
    		twin_faceB_vertex := transform_vector3(edgeB.twin.face.face_plane_normal, hullB.inv_worldSpace);;//faceB_vertex_twin?

			//minute -15:19 Final Overlap Test: Formulas
			is_minkowsky_face: bool;
			{

			   //minute 19, we can avoid cross products. and we dont need to normalize.
                /*  for this comment: "." is dot product, "x" is cross product
                    //intersection tests: TODO: explanation, I still dont understand what side means in this context, I need diagrams.
                    [c . (bxa)] . [d . (bxa)] < 0
                    [a . (dxc)] . [b . (dxc)] < 0
                    //hemisphere test: are the remaining vertices on the same side
                    [a . (cxb)] . [d . (cxb)] > 0

                    using albegra rules...final minimal test
                    |CBA| . |DBA| < 0 C and D test intersectsBA
                    |ADC| . |BDC| < 0 A and B test intersectsDC
                    |CBA| . |BDC| > 0
                */

            	//plane test using Scalar Triple Products - only need sign
            	//I think the new naming make these more intuitive:
            	CBA := dot(-faceB_vertex,     -arcA); //- SIGN why? because it is minkowski difference! but what about the edgeArcs
            	DBA := dot(-twin_faceB_vertex,-arcA);
            	ADC := dot(faceA_vertex,      -arcB);//- SIGN why?  why?
            	BDC := dot(twin_faceA_vertex, -arcB);//- SIGN why?  why? why?

            	//test for intersections of great arcs - vertices a/b,c/d should be on +/- sides of plane
            	intersectsBA   := CBA * DBA < 0.0;
            	intersectsDC   := ADC * BDC < 0.0;
            	//test both arcs lie on same hemisphere
            	are_in_same_hemisphere := CBA * BDC > 0.0;
                is_minkowsky_face = cast(bool)(intersectsBA & intersectsDC & are_in_same_hemisphere);

			}

            if !is_minkowsky_face then continue; //early skip
            translation := Vector3.{hullA.worldSpace._14, hullA.worldSpace._24, hullA.worldSpace._34};
			this_separation :=  distance_between_arcs(edgeA_origin, arcA, edgeB_origin, arcB, translation);
            // if this_separation <= result.distance continue;  //early skip
            if this_separation >= result.distance continue;  //early skip

			result = .{ edgeA, edgeB, this_separation };
			if result.distance > EPSILON4 then return result;  //early return
		}
	}

	return result;
}


CreateFaceContact :: (faceQuery: FaceQuery, referenceHull: Convex_Hull, incidentHull: Convex_Hull)-> FaceContact
{
    result: FaceContact;
    result.query = faceQuery;
    result.points.allocator = temp;
    array_reserve(*result.points, 4);

	reference_face := faceQuery.face;
	referencePlaneWorld_normal, referencePlaneWorld_distance := transform_plane(reference_face.face_plane_normal, reference_face.face_plane_distance,  referenceHull.worldSpace);
	incident_face: *Hull_Face;

    {
        mostAntiParallel := FLOAT32_MAX;

        start := incidentHull.mesh.faces;
        current_face := start;
        while true
        {
            incidentPlaneWorld_normal, incidentPlaneWorld_dist := transform_plane(current_face.face_plane_normal, current_face.face_plane_distance , incidentHull.worldSpace );
            projection := dot(referencePlaneWorld_normal, incidentPlaneWorld_normal);
            if projection < mostAntiParallel
            {
                mostAntiParallel = projection;
                incident_face = current_face;
            }
            current_face = current_face.next;
            if current_face == start then break;
        }
        assert(mostAntiParallel < 9999.0, "most antiparallel not found");
        assert(incident_face != null, "incident face not found!");
    }

	//collect incident edge vertices
	incident_world_vertices: [..]Vector3;
    incident_world_vertices.allocator = temp;
    array_reserve(*incident_world_vertices, incident_face.vertexCount); //this is triggering th weirdests bugs ever
    assert(incident_world_vertices.count == 0);

    //fill incident_world_vertices traversing incident face edges
	{
		start := incident_face.edge;
		current_edge := start;
		while true
		{
			point_in_local := incidentHull.scale * current_edge.tail.vertex;
            point_in_world := incidentHull.worldSpace * point_in_local;
			array_add(*incident_world_vertices, point_in_world);

			current_edge = current_edge.next;
		    if current_edge == start then break;
		}
	};

	start := reference_face.edge;
	current_edge :=  start;
	while true
	{
		referenceSideFace := ifx current_edge.face !=  reference_face then current_edge.face else current_edge.twin.face;
		world_plane_normal, world_plane_dist  := transform_plane(referenceSideFace.face_plane_normal, referenceSideFace.face_plane_distance, referenceHull.worldSpace);
        clipped_vertices := Sutherland_Hodgman(world_plane_normal,world_plane_dist , incident_world_vertices);


         if clipped_vertices.count == 0 {
            result.points.count = 0;
            print("[seems like you clipped all vertices, early return] original1");
            return result;
        }

        array_copy(*incident_world_vertices, clipped_vertices);
        incident_world_vertices.count = clipped_vertices.count;


		current_edge = current_edge.next;
	    if current_edge == start then break;
	}


    // project world vertices that survived clipping onto reference plane
    for i: 0..incident_world_vertices.count - 1 {
        incident_world_vertices[i] = closest_point_to_plane(point=incident_world_vertices[i], referencePlaneWorld_normal, referencePlaneWorld_distance);
    }

    // result.points = incident_world_vertices;
    array_copy(*result.points, incident_world_vertices);
    array_free(incident_world_vertices);

    // for result.points if it.y < 0 then add_to_array_of_contact_points(it, radii = .1, color=.{0,0,1,1});

    reduce_contact_points_to_four_or_less(incidentHull.scale, *result);
    incident_plane_world_normal,incident_plane_world_distance := transform_plane(incident_face.face_plane_normal, incident_face.face_plane_distance,  incidentHull.worldSpace);
    assert(result.points.count <= 4, "we aer supposued o get 4 or less contact points here, we got %", result.points.count);

    for *result.points {
        len := distance_point_to_plane_signed(point=it.*, incident_plane_world_normal, incident_plane_world_distance);
        is_separating_basically := len > -EPSILON3;
        if is_separating_basically {
            remove it;
         }
    }
    // for result.points  then add_to_array_of_contact_points(it, radii = .2, color=.{1,0,0,1});
  	return result;
}

CreateEdgeContact :: (edgeQuery: EdgeQuery, hullA: Convex_Hull, hullB: Convex_Hull) -> EdgeContact
{
	result: EdgeContact;;
	result.query = edgeQuery;

	//compute closest points between two edges
	edgeA0 :=  hullA.worldSpace * (hullA.scale * edgeQuery.edgeA.tail.vertex);
	edgeA1 :=  hullA.worldSpace * (hullA.scale * edgeQuery.edgeA.twin.tail.vertex);
	//variant is half edge
	edgeB := edgeQuery.edgeB;
	edgeB0 := hullB.worldSpace * (hullB.scale * edgeB.tail.vertex);
	edgeB1 := hullB.worldSpace * (hullB.scale * edgeB.twin.tail.vertex);


	L0, L1, s, t, distdist := closest_point_segment_to_segment(edgeA0, edgeA1, edgeB0, edgeB1);
	//compute center of closest points as contact point
	result.midpoint = (L0 + L1) * 0.5;
	//compute separating axis between two edges
	result.separationAxis = normalize(L0 - L1);
	translation:= Vector3.{hullA.worldSpace._14 , hullA.worldSpace._24, hullA.worldSpace._34 };
    if (dot(result.separationAxis, edgeA0 - translation) < 0.0)
    	result.separationAxis = -result.separationAxis;
	return result;
};

reduce_contact_points_to_four_or_less :: (incidentScale: Vector3, faceContact: *FaceContact) {
    //we should use the same search direction to incrase the probability of always getting the same contact points in the same array position and order. For CCD you need to set the order from highest penetration to lowest.
    SEARCH_DIR ::  #run normalize(Vector3.{1,1,1});

    manifold_normal := faceContact.query.face.face_plane_normal;
    assert(length_squared(cross_product(SEARCH_DIR, manifold_normal)) > EPSILON3, "Search dir % and face % are pralalell! fix this",SEARCH_DIR, manifold_normal);


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
    	}
    	else if ( Distance2 <= 0.0 && Distance1 > 0 )
		{
    		// Vertex2 is behind of the plane, vertex1 is in front -> intersection point
    		Fraction := Distance1 / ( Distance1 - Distance2 );
    		IntersectionPoint := Vertex1 + Fraction * ( Vertex2 - Vertex1 );

    		// Keep intersection point
    		array_add(*clipped_result, IntersectionPoint);

    		// And also keep vertex2
    		array_add(*clipped_result, Vertex2); //[LLVM REPORT]loop not vectorized: control flow cannot be substituted for a select
		}

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
                vert_ptr.prev = prev;
                prev.next = vert_ptr;
            }
            prev = vert_ptr;
        }
        //because this is a special case its safe to do the following line.
        new_hull.mesh.vertex_free_list = *mesh.vertexBuffer[0];
    }


    //
    //build initial hull
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
        p3 = ifx dist_1_is_farther then t1 else t2;
        idx3 = ifx dist_1_is_farther then tmp_idx1 else tmp_idx2;
    }

    {//I think this is ok to be hardcoded, its just to build the first tetrahedron
        using mesh;
        //we remove the 4 vertices from freelist, takind advantage of continuos memory
        v0 := __pop_free_vertex(*mesh, idx0);
        v1 := __pop_free_vertex(*mesh, idx1);
        v2 := __pop_free_vertex(*mesh, idx2);
        v3 := __pop_free_vertex(*mesh, idx3);

        v0.vertex = p0;
        v1.vertex = p1;
        v2.vertex = p2;
        v3.vertex = p3;
        vertexCount += 4;

        v0.prev = v3;
        v0.next = v1;
        v0.referenceCount = 3; // Each vertex used by 3faces

        v1.prev = v0;
        v1.next = v2;
        v1.referenceCount = 3; // Each vertex used by 3faces

        v2.prev = v1;
        v2.next = v3;
        v2.referenceCount = 3; // Each vertex used by 3faces

        v3.prev = v2;
        v3.next = v0;
        v3.referenceCount = 3; // Each vertex used by 3faces
        //
        // I just followed the Image I uploaded on github, initial_tetrahedron.jpg
        //
        //step 0 TWINS assings
        twin_pairs := [2]int.[
        .[0,  8],
        .[1,  9],
        .[2,  3],
        .[4, 11],

        .[5, 6],
        .[7, 10],
        .[8, 0]
        ];

        for pair: twin_pairs {
            edgeBuffer[pair[0]].twin = *edgeBuffer[pair[1]];
            edgeBuffer[pair[1]].twin = *edgeBuffer[pair[0]];
        }

       //step 1 prev next and tail assings
       LittlePacket :: struct {
         tail: *Hull_Vertex;
         prev: int;//*Hull_Half_Edge; TODO: this!
         next: int;//*Hull_Half_Edge; TODO: this!
       }

       edges_tail_prev_next := LittlePacket.[
        //RED: p0,p1, p2 from the image
        .{v0,  2, 1}, //edge[0]
        .{v1,  0, 2}, //edge[1]
        .{v2,  1, 0}, //edge[2]

        //PINK: p3, p0, p2.
        .{v3,  5, 4}, //edge[3]->tail is p3, prev is edge[5], next is edge[4], same logic for all others
        .{v0,  3, 5}, //edge[4]
        .{v2,  4, 3}, //edge[5]

        //GREEN: p0, p3, p1.
        .{v0,  8, 7}, //edge[6]
        .{v3,  6, 8}, //edge[7]
        .{v1,  7, 6}, //edge[8]

        //YELLOW: p2, p1, p3.
        .{v2,  11, 10}, //edge[9]
        .{v1,  9, 11}, //edge[10]
        .{v3,  10, 9}, //edge[11]
        ];

        for edges_tail_prev_next {
            prev_idx := it.prev;
            next_idx := it.next;
            edgeBuffer[it_index].tail = it.tail;
            edgeBuffer[it_index].prev = *edgeBuffer[prev_idx];
            edgeBuffer[it_index].next = *edgeBuffer[next_idx];
            find_or_add(*mesh.edges, *edgeBuffer[it_index]);
        }
        edgeCount = edges_tail_prev_next.count;


        //step 2 FACE assigns, 4 planes

        red_plane_normal, red_plane_dist := create_plane(p0,p1,p2);
        pink_plane_normal, pink_plane_dist := create_plane(p0,p2,p3);
        green_plane_normal, green_plane_dist := create_plane(p0,p3,p1);
        yellow_plane_normal, yellow_plane_dist := create_plane(p1,p3,p2);

        //red
        faceBuffer[0].edge = *edgeBuffer[0];
        faceBuffer[0].face_plane_normal = red_plane_normal;
        faceBuffer[0].face_plane_distance = red_plane_dist;
        faceBuffer[0].vertexCount = 3;
          edgeBuffer[0].face = *faceBuffer[0];
          edgeBuffer[1].face = *faceBuffer[0];
          edgeBuffer[2].face = *faceBuffer[0];
        faceBuffer[0].prev = *faceBuffer[3];
        faceBuffer[0].next = *faceBuffer[1];

        //pink
        faceBuffer[1].edge = *edgeBuffer[3];
        faceBuffer[1].face_plane_normal = pink_plane_normal;
        faceBuffer[1].face_plane_distance = pink_plane_dist;
        faceBuffer[1].vertexCount = 3;
          edgeBuffer[3].face = *faceBuffer[1];
          edgeBuffer[4].face = *faceBuffer[1];
          edgeBuffer[5].face = *faceBuffer[1];
        faceBuffer[1].prev = *faceBuffer[0];
        faceBuffer[1].next = *faceBuffer[2];

        //green
        faceBuffer[2].edge = *edgeBuffer[6];
        faceBuffer[2].face_plane_normal = green_plane_normal;
        faceBuffer[2].face_plane_distance = green_plane_dist;
        faceBuffer[2].vertexCount = 3;
          edgeBuffer[6].face = *faceBuffer[2];
          edgeBuffer[7].face = *faceBuffer[2];
          edgeBuffer[8].face = *faceBuffer[2];
        faceBuffer[2].prev = *faceBuffer[1];
        faceBuffer[2].next = *faceBuffer[3];

        //yellow
        faceBuffer[3].edge = *edgeBuffer[9];
        faceBuffer[3].face_plane_normal = yellow_plane_normal;
        faceBuffer[3].face_plane_distance = yellow_plane_dist;
        faceBuffer[3].vertexCount = 3;
          edgeBuffer[9].face = *faceBuffer[3];
          edgeBuffer[10].face = *faceBuffer[3];
          edgeBuffer[11].face = *faceBuffer[3];
        faceBuffer[3].prev = *faceBuffer[2];
        faceBuffer[3].next = *faceBuffer[0];

        faceCount = 4;

        // NOTE: Im not sure about these
         // Set vertex edge references (one outgoing edge per vertex)
         v0.edge = *edgeBuffer[0];
         v1.edge = *edgeBuffer[1];
         v2.edge = *edgeBuffer[2];
         v3.edge = *edgeBuffer[3];

         // // Set leaving_edge (same as edge for this simple case)
         // for i: 0..2 {
         //     vertexBuffer[i].leaving_edge = vertexBuffer[i].edge;
         // }
        // NOTE: end note.

    #run log("this is hardoced here!");

    mesh.vertices = v0;
    mesh.faces = *faceBuffer[0];
    mesh.edgeCount = 12;
    mesh.faceCount = 4;
    mesh.vertexCount = 4;

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



traverse_vertex :: (vertex: *Hull_Vertex, body: Code, flags: For_Flags) #expand {
    `start_vertex := vertex;
    `it := start_vertex;
    `it_index := 0;
     while true {
        defer if it == start_vertex break;
        defer it = it.next;
        defer it_index += 1;
        #insert body;
    }
}

each_vertex :: (mesh: Half_Edge_Mesh, body: Code, flags: For_Flags) #expand {
    `start_vertex := mesh.vertex;
    `it := start_vertex;
    `it_index := 0;
     while true {
        defer if it == start_vertex break;
        defer it = it.next;
        defer it_index += 1;
        #insert body;
    }
}

each_face :: (mesh: Half_Edge_Mesh, body: Code, flags: For_Flags) #expand {
    `start_face := mesh.faces;
    `it := start_face;
    `it_index := 0;
     while true {
        defer if it == start_face break;
        defer it = it.next;
        defer it_index += 1;
        #insert body;
    }
}

each_edge :: (face: Hull_Face, body: Code, flags: For_Flags) #expand {
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








AddVertexToHull :: (mesh:  *Half_Edge_Mesh, conflict_vertex: *Hull_Vertex){
	// Remove conflict_vertex from conflict face
    face := conflict_vertex.conflict_face;
    conflict_vertex.conflict_face = null;
    removed := array_ordered_remove_by_value(*face.conflict_list, conflict_vertex);
    assert(removed);

    //  Add to hull
	new_vert := add_vertex(mesh);
	new_vert.* = conflict_vertex;

    // Find the horizon edges
    horizon: [..]*Hull_Half_Edge;
    // BuildHorizon(conflict_vertex, conflict_face, *horizon);

    NewFaces: [..]*Hull_Face;
    BuildNewFaces(mesh, conflict_vertex, *NewFaces, *horizon);
    // MergeFaces(hull, NewFaces);

    // ResolveOrphans(hull, NewFaces);

}

NextConflictVertex :: ( mesh:  *Half_Edge_Mesh) -> *Hull_Vertex
{
    conflict_vertex: *Hull_Vertex = null;
    max_so_far: float = -FLOAT32_INFINITY;

    start_face := mesh.faces;
    current_face := start_face;
    while true { defer if current_face == start_face break;
        if current_face.conflict_list.count != 0 {

            vertex := current_face.conflict_list[0];
            while true {
                defer if vertex == vertex.next then break;
                defer vertex = vertex.next;
                assert(vertex.conflict_face == current_face, "seems like the conflic list is wrong");
                this_dist := distance_point_to_plane_signed(vertex.vertex, current_face.face_plane_normal, current_face.face_plane_distance);
                if this_dist > max_so_far {
                    conflict_vertex = vertex;
                    max_so_far = this_dist;
                }
            }
        }
        //while loop face related:
        current_face = current_face.next;
    }
    return conflict_vertex;
}


pop_from_free_list :: (free_list: **$T/.[Hull_Vertex,Hull_Half_Edge, Hull_Face ]) -> *T {
    assert(free_list != null);
    if free_list.* != null
        return null; //early return;
    popped := free_list.*;
    free_list.* = popped.next;
    if free_list.* free_list.*.prev = null;//little sanitization
    return popped;
}

pop_free_vertex :: (mesh: *Half_Edge_Mesh) -> *Hull_Vertex {
    return pop_from_free_list(*mesh.vertex_free_list);
}

pop_free_edge :: (mesh: *Half_Edge_Mesh) -> *Hull_Half_Edge {
    return pop_from_free_list(*mesh.edge_free_list);
}

pop_free_face :: (mesh: *Half_Edge_Mesh) -> *Hull_Face {
    return pop_from_free_list(*mesh.face_free_list);
}


//I dont like having this procedure, so I put 2 underscore to it until I figure out a better way to do this
__pop_free_vertex :: (mesh: *Half_Edge_Mesh, idx: int) -> *Hull_Vertex {
    idx_0 := cast(int)mesh.vertexBuffer.data;
    idx_to_remove := idx_0 + idx;
    popped: *Hull_Vertex;
    for :traverse_vertex mesh.vertex_free_list
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
                mesh.vertex_free_list = it.next; //<-important here
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
			backup := feat_to_add.prev;
			backup.next = new_feat;
			new_feat.prev = backup;
		}
		else
			feat_to_add = new_feat;

		new_feat.next = feat_to_add;
		feat_to_add.prev = new_feat;

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


add_vertex :: (using mesh: *Half_Edge_Mesh ) -> new_vertex: *Hull_Vertex {
    result := pop_free_vertex(mesh);
    if !result {
        result = *vertexBuffer[vertexCount];
        assert(vertexCount <= num_allocated_vertex);
    }

    vertexCount += 1;
    return result;
}

add_edge :: (using mesh: *Half_Edge_Mesh ) -> new_edge: *Hull_Half_Edge {
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
BuildNewFaces :: (mesh: *Half_Edge_Mesh, vertex: *Hull_Vertex, newFaces: *[..]*Hull_Face, horizon: *[..]*Hull_Half_Edge)
{
    newEdges: *[..]*Hull_Half_Edge;

	//pairs newly created edges with their twins
	FindTwin :: (vertex: *Hull_Vertex, edge: *Hull_Half_Edge, newEdges: [..]*Hull_Half_Edge) -> *Hull_Half_Edge
	{
	twin:  *Hull_Half_Edge;
	   for potentialTwin: newEdges {
            if edge.tail == vertex {
                if potentialTwin.tail == edge.next.tail {
                    twin = potentialTwin;
                    break;
                }
                if potentialTwin.tail == vertex && potentialTwin.next.tail == edge.tail
                    twin = potentialTwin;
                    break;
            }
	   }

		if twin == newEdges[newEdges.count-1] return null;

		return twin;
	};


	for edge : horizon.*
	{
		e0 :=  add_edge(mesh);
		e1 :=  edge;
		e2 :=  add_edge(mesh);

		face := add_face(mesh);

		//set inital edge/vertex count
		face.vertexCount = 3;
		//set pointers
		face.edge = e0;
		e0.face = face;
		e1.face = face;
		e2.face = face;
		//eye point
		e0.prev = e2;
		e0.next = e1;
		e0.tail = vertex;
		//store e2 tail before updating horizon edge
		e2.tail = edge.next.tail;
		//horizon edge, retains old tail and twin
		e1.prev = e0;
		e1.next = e2;
		//adjoining edge
		e2.prev = e1;
		e2.next = e0;
		//add new edges and faces
		array_add(newEdges, e0);
		array_add(newEdges, e2);
		array_add(newFaces, face);
		append_feature(mesh, face, null);
		for newEdge: newEdges.* {
			if newEdge.twin == FindTwin(vertex, newEdge, newEdges.*) {
				newEdge.twin.twin = newEdge;
			}
		}
	}
}

//another possible name collide_convex_vs_convex
collide_hull_vs_hull :: (
    position_A: Vector3, orientation_A: Quaternion, scale_A: Vector3, hull_A: Convex_Hull,
    position_B: Vector3, orientation_B: Quaternion, scale_B: Vector3, hull_B: Convex_Hull

    ) ->
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

    //@Hack temporary hack, I could not figure out good procedure parameters, maybe I should modularize this procedure.
    {
        hull0 = hull_A;
        hull1 = hull_B;
        hull0.worldSpace = make_translation_matrix4(position_A)*rotation_matrix (Matrix4, orientation_A) * make_scale_matrix4(scale_A);
        hull1.worldSpace  = make_translation_matrix4(position_B)*rotation_matrix (Matrix4, orientation_B) * make_scale_matrix4(scale_B);
        hull0.scale = scale_A;
        hull1.scale = scale_B;


        //maybe cache per frame/orientation treshold. The bottleneck is still  QueryEdgeDirection and its calls
        success:, hull0.inv_worldSpace = inverse(hull0.worldSpace);
        success=, hull1.inv_worldSpace = inverse(hull1.worldSpace);

        assert(success, "error while inverting some hull world model, are your hulls created?");
        // hull.scale = scale;
    }
    collision := SAT_dirk_gregorius(hull0, hull1, *query);
    if collision {

		faceDistance0 :=  query.faceQuery_AB.distance ;
		faceDistance1 :=  query.faceQuery_BA.distance ;

		//grep this from https://www.gamedev.net/forums/topic/667499-3d-sat-problem/?page=2
		kLinearSlop :=  0.005 ;
		kRelEdgeTolerance :=  0.90 ;
		kRelFaceTolerance :=  0.98 ;
		kAbsTolerance :=  0.5 * kLinearSlop ;

        is_near_edge := query.edgeQuery.distance > kRelEdgeTolerance * max(faceDistance0, faceDistance1) + kAbsTolerance;
        is_near_face0 := faceDistance1 > (kRelFaceTolerance* faceDistance0 + kAbsTolerance);
        is_near_face1 := !is_near_face0;//not used

		if is_near_edge
		{

			contact : EdgeContact = CreateEdgeContact(query.edgeQuery,hull0,hull1);
			{
                world_normal_towards_A = -contact.separationAxis;
                penetration = contact.query.distance * length_squared(world_normal_towards_A);

                if penetration < 0 {
                    num_contacts = 1;
                    world_midpoints[0] = contact.midpoint;
                    world_contact_points_on_A[0] = contact.midpoint + (penetration/2.0)*world_normal_towards_A;
                    world_contact_points_on_B[0] = contact.midpoint - (penetration/2.0)*world_normal_towards_A;
                }
			}

            doLabel(tprint("is_near_edge %",penetration), WorldToScreen(.{0.5,1.2,0}));
		}
		else if is_near_face0
		{
		    contact :FaceContact= CreateFaceContact(query.faceQuery_BA, hull1,hull0); //reference,incindet
            {
                world_normal_towards_A           = transform_direction(contact.query.face.face_plane_normal,hull1.worldSpace);
                penetration                      = contact.query.distance * length_squared(world_normal_towards_A);
                assert(contact.points.count <= 4, "we got more than 4 contact points! contact.points.count == %",contact.points.count);
                if penetration < 0
                {
                    for point_in_B: contact.points
                    {
                        num_contacts += 1;
                        world_midpoints[it_index]           = point_in_B + (penetration/2.0) * world_normal_towards_A;
                        world_contact_points_on_A[it_index] = point_in_B + penetration * world_normal_towards_A;
                        world_contact_points_on_B[it_index] = point_in_B;
                    }

                }
            }
            doLabel(tprint("is_near_face0 %",penetration), WorldToScreen(.{0.5,1.2,0}));
		}
		else
		{
		    contact := CreateFaceContact(query.faceQuery_AB, hull0, hull1);
            {
                world_normal_towards_A = -transform_direction(contact.query.face.face_plane_normal,hull0.worldSpace);
                penetration = contact.query.distance * length_squared(world_normal_towards_A);

                assert(contact.points.count <= 4, "we got more than 4 contact points! AB contact.points.count == %",contact.points.count);
                if penetration < 0
                {
                    for point_in_A: contact.points
                    {
                        num_contacts += 1;
                        world_midpoints[it_index]           = point_in_A - (penetration/2.0) * world_normal_towards_A;
                        world_contact_points_on_A[it_index] = point_in_A;
                        world_contact_points_on_B[it_index] = point_in_A - penetration*world_normal_towards_A;
                    }
                }
            doLabel(tprint("is_near_face1 %",penetration), WorldToScreen(.{0.5,1.2,0}));
            }


		} //end edge or faceBA or faceAB test
    } //end if collision

    return num_contacts, penetration, world_normal_towards_A, world_midpoints, world_contact_points_on_A, world_contact_points_on_B ;

};

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


find_furthest_point_along_local_dir :: (hull: Convex_Hull, local_direction: Vector3) -> local_furhtest_point: Vector3, idx: int
{
	local_furhtest_point: Vector3;
	max := -FLOAT32_INFINITY;
	idx := -1;

	start := hull.mesh.vertices;
	vertex := start;
	currentIndex: int;
	while true
	{
		localPoint := hull.scale * vertex.vertex;
		current_dist := dot(localPoint, local_direction);
		if current_dist > max
		{
			idx = currentIndex;
			local_furhtest_point = localPoint;
			max = current_dist;
		}
		vertex = vertex.next;
		currentIndex += 1;
		if vertex == hull.mesh.vertices then break;
	}

	return  local_furhtest_point, idx;
}

distance_between_arcs :: (edgeA_origin: Vector3, arcA: Vector3, edgeB_origin: Vector3, arcB: Vector3, world_hull_centroid: Vector3) -> float
{
	gauss_face_area := cross_product(arcA, arcB);

	area := length(gauss_face_area);
	are_edges_parallel := area <= EPSILON3 * sqrt(length_squared(arcA) * length_squared(arcB) ) ;
	if are_edges_parallel return -FLOAT32_INFINITY;

	gauss_face_n := gauss_face_area / area;

	n_points_from_B_to_A := dot(edgeA_origin - world_hull_centroid, gauss_face_n) < 0.0;
	if n_points_from_B_to_A then gauss_face_n = -gauss_face_n;

	return dot(edgeB_origin - edgeA_origin, gauss_face_n); //no need to compute support points: O(1)
}

transform_vector3 ::  (in_vector: Vector3,  m: Matrix4) -> vector_transformed: Vector3
{
    vector_transformed: Vector3 = ---;
    vector_transformed.x  = in_vector.x * m.coef[0][0] + in_vector.y * m.coef[1][0] +  in_vector.z * m.coef[2][0];
    vector_transformed.y  = in_vector.x * m.coef[0][1] + in_vector.y * m.coef[1][1] +  in_vector.z * m.coef[2][1];
    vector_transformed.z  = in_vector.x * m.coef[0][2] + in_vector.y * m.coef[1][2] +  in_vector.z * m.coef[2][2];

	return vector_transformed;
}

transform_point :: (v: Vector3, m: Matrix4) -> Vector3 {
    return Vector3.{
        m._11 * v.x + m._12 * v.y + m._13 * v.z + m._14,
        m._21 * v.x + m._22 * v.y + m._23 * v.z + m._24,
        m._31 * v.x + m._32 * v.y + m._33 * v.z + m._34
    };
}


transform_direction :: (direction: Vector3, m: Matrix4) -> Vector3 {
    return normalize(Vector3.{ //A HIDDEN NNORMALIZE!!! @Investigate why I did this? I dont remember.
        m._11 * direction.x + m._12 * direction.y + m._13 * direction.z,
        m._21 * direction.x + m._22 * direction.y + m._23 * direction.z,
        m._31 * direction.x + m._32 * direction.y + m._33 * direction.z
    });
}


transform_plane :: (plane: Plane, m: Matrix4) -> Plane {
    transformed_normal, transformed_distance := transform_plane(plane.normal, plane.distance, m);
    return Plane.{transformed_normal, transformed_distance};
}

transform_plane :: (plane_normal: Vector3, plane_distance: float, m: Matrix4) -> transformed_normal: Vector3, transformed_distance: float {
    //log("https://www.gamedev.net/forums/topic/667499-3d-sat-problem/?page=4");
    transformed_normal := transform_direction(plane_normal, m);
    point_on_plane := plane_normal * plane_distance;
    transformed_point := transform_point(point_on_plane, m);
    //dirk way is the same as this way? assert will assure that:
    //after testing-> I think Dirk way doesnt support scaling inside m, so I deleted it.
    transformed_distance := dot(transformed_point, transformed_normal);
    return transformed_normal, transformed_distance;
}