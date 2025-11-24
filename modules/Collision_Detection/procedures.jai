QueryFaceDirections :: (hullA: Convex_Hull, hullB: Convex_Hull) -> FaceQuery {
    early_return := false;
    result: FaceQuery;
    result.face = null;
    result.distance = -FLOAT32_INFINITY;

    start := hullA.mesh.faces;
    current_face := start;
    assert(current_face != null, "[ERROR] hullA.mesh.faces==null... are hullA and hullB created?");

    while true { defer if current_face == start break;
        world_planeA := transform_plane(.{current_face.facePlane.normal, current_face.facePlane.distance}, hullA.worldSpace);

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


//this is no gaussmap, N^2 kinda or N^3
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

            //iI shouldve deleted this comment -> m trying this trick, but, I dont like "clever" tricks, so TODO(tr): Delete this trick and use a normal block liek the one below
            edgeA_dir := ifx 1 then {

                   edgeOrigin := hullA.worldSpace * (hullA.scale * edgeA.tail.vertex);
    	           edgeDestination := hullA.worldSpace * (hullA.scale*edgeA.twin.tail.vertex);
    	           edgeDestination - edgeOrigin;
    	        }

            edgeB_dir: Vector3 = ---;
            {
               edgeOrigin := hullB.worldSpace * (hullA.scale * edgeB.tail.vertex);
	           edgeDestination := hullB.worldSpace * (hullA.scale*edgeB.twin.tail.vertex);
	           edgeB_dir = edgeDestination - edgeOrigin;
    	    }

            axis := cross_product(edgeA_dir, edgeB_dir);
            leftA, rightA := project_all_vertices_of_hull_into_axis(hullA, axis);
            leftB, rightB := project_all_vertices_of_hull_into_axis(hullB, axis);

            //check your milton diagrams:
            separation: float = ---;
            if leftB>leftA && rightB > rightA
                then separation  = leftB-rightA;
                else separation  = leftA-rightB;
            //if penetrationg then separation is negative. but, we want to ignore such cases I guess? idk why, I would use them anyways saving them as penetration distance

            translation := Vector3.{hullA.worldSpace._14, hullA.worldSpace._24, hullA.worldSpace._34};

            // distance :=  Distance(origin0, BxA, origin1, DxC, translation);
            if separation <= result.distance
            	continue;

            result = .{ edgeA,edgeB , separation };
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
		 faceA_vertex := transform_vector3(edgeA.face.facePlane.normal, hullA.inv_worldSpace);//faceA_vertex
		 twin_faceA_vertex := transform_vector3(edgeA.twin.face.facePlane.normal, hullA.inv_worldSpace);;//faceA_vertex_twin?
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

    		faceB_vertex := transform_vector3(edgeB.face.facePlane.normal, hullB.inv_worldSpace);//faceB_vertex
    		twin_faceB_vertex := transform_vector3(edgeB.twin.face.facePlane.normal, hullB.inv_worldSpace);;//faceB_vertex_twin?

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
			distance :=  distance_between_arcs(edgeA_origin, arcA, edgeB_origin, arcB, translation);
			if distance <= result.distance continue;  //early skip

			result = .{ edgeA, edgeB, distance };
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
	referencePlaneWorld := transform_plane(reference_face.facePlane,  referenceHull.worldSpace);
	incident_face: *Hull_Face;

    {
        mostAntiParallel := FLOAT32_MAX;

        start := incidentHull.mesh.faces;
        current_face := start;
        while true
        {
            incidentPlaneWorld := transform_plane(current_face.facePlane , incidentHull.worldSpace );
            projection := dot(referencePlaneWorld.normal, incidentPlaneWorld.normal);
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
	   if current_edge.face !=  reference_face then log("AAAA");
		referenceSideFace := ifx current_edge.face !=  reference_face then current_edge.face else current_edge.twin.face;
		world_plane := transform_plane(referenceSideFace.facePlane, referenceHull.worldSpace);
        clipped_vertices := Sutherland_Hodgman(world_plane, incident_world_vertices);


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
        incident_world_vertices[i] = closest_point_to_plane(point=incident_world_vertices[i], referencePlaneWorld.normal, referencePlaneWorld.distance);
    }

    // result.points = incident_world_vertices;
    array_copy(*result.points, incident_world_vertices);
    array_free(incident_world_vertices);

    // for result.points if it.y < 0 then add_to_array_of_contact_points(it, radii = .1, color=.{0,0,1,1});

    reduce_contact_points_to_four_or_less(incidentHull.scale, *result);
    incident_plane_world := transform_plane(incident_face.facePlane,  incidentHull.worldSpace);
    assert(result.points.count <= 4, "we aer supposued o get 4 or less contact points here, we got %", result.points.count);

    for *result.points {
        len := distance_point_to_plane_signed(point=it.*, incident_plane_world.normal, incident_plane_world.distance);
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

    manifold_normal := faceContact.query.face.facePlane.normal;
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
Sutherland_Hodgman :: (plane: Plane, polygon_vertices: [..]Vector3) -> [..]Vector3 {
    clipped_result: [..]Vector3;
	clipped_result.allocator = temp;
    Vertex1 := polygon_vertices[polygon_vertices.count-1]; //latest vertex is start, im not sure about this.
    Distance1 := distance_point_to_plane_signed(Vertex1, plane.normal, plane.distance);

    for Index: 0..polygon_vertices.count - 1 {
        Vertex2 := polygon_vertices[ Index ]; //[LLVM REPORT]loop not vectorized: call instruction cannot be vectorized
        Distance2 := distance_point_to_plane_signed(Vertex2, plane.normal, plane.distance);

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
// TODO: a good title for this category.
//

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

//this works? compare with :tagThisWorksPlaneEquation
transform_plane :: (plane: Plane, m: Matrix4) -> Plane {
    //log("https://www.gamedev.net/forums/topic/667499-3d-sat-problem/?page=4");
    transformed_normal := transform_direction(plane.normal, m);
    point_on_plane := plane.normal * plane.distance;
    transformed_point := transform_point(point_on_plane, m);
    //dirk way is the same as this way? assert will assure that:
    // I think Dirk way doesnt support scaling inside m, so I deleted it.
    transformed_distance := dot(transformed_point, transformed_normal);
    return Plane.{transformed_normal, transformed_distance};
}