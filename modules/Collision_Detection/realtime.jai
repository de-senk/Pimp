SHOULD_RUN_TESTS :: false; //Not a single test exists right now. I was prototypyng in how would I add tests in the code.
//some of these procedures are inspired from the book realtime collision detection by Chris Ericson.

// Given three noncollinear points (ordered ccw), compute plane equation
create_plane :: (a: Vector3, b: Vector3, c: Vector3)-> normal: Vector3, d: float {
    plane_normal := normalize(cross_product(b - a, c - a));
    d := dot(plane_normal, a);
    return plane_normal, d;
}

#if SHOULD_RUN_TESTS #run intersect_plane_with_plane(.{}, 0, .{}, 0);
intersect_plane_with_plane :: (plane1_normal: Vector3, plane1_d: float, plane2_normal: Vector3, plane2_d: float)-> (success: bool, intersection_point_on_line: Vector3, line_direction: Vector3)
{
    EPSILON7 :: 0.00000001;
    intersection_point_on_line: Vector3 = ---; //would this be inserted directly into the caller?
    line_direction: Vector3 = ---; //would this be inserted directly into the caller?
    line_direction = cross_product(plane1_normal, plane2_normal); //and this?
    denom := length_squared(line_direction);
    if denom < EPSILON7 return false, .{},.{};
    intersection_point_on_line = cross_product(plane1_d*plane2_normal - plane2_d*plane1_normal, line_direction) / denom;
    return true, intersection_point_on_line, line_direction;
}


#if SHOULD_RUN_TESTS #run intersect_three_planes(.{}, 0, .{}, 0, .{}, 0);
intersect_three_planes:: (plane1_normal: Vector3, plane1_d: float, plane2_normal: Vector3, plane2_d: float, plane3_normal: Vector3, plane3_d: float)-> (success: bool, intersection_point: Vector3)
{
    FILLSTAC:float = 111111111;
    intersection_point: Vector3 = ---;
    EPSILON7 :: 0.00000001;
    line_direction_from_intersec_plane2with3 := cross_product(plane2_normal, plane3_normal);
    denom := dot(plane1_normal, line_direction_from_intersec_plane2with3);
    if abs(denom) < EPSILON7 return false, intersection_point; //---
    intersection_point = (plane1_d * line_direction_from_intersec_plane2with3 + cross_product(plane1_normal, plane3_d * plane2_normal - plane2_d * plane3_normal)) / denom;
    return true, intersection_point;
}


#if SHOULD_RUN_TESTS #run intersect_segment_with_plane(.{}, .{}, .{}, 0.0,cast(*float) *context); //danger, never uncomment.
intersect_segment_with_plane :: (start: Vector3, end: Vector3, plane_normal: Vector3, plane_distance: float, t: *float) -> (success: bool, intersect_point: Vector3)
{
    ab := end - start;
    t.* = (plane_distance - dot(plane_normal, start)) / dot(plane_normal, ab);
    if t.* < 0.0 || t.* > 1.0 return success=false, .{};
    intersect_point := start + (t.*) * ab;
    return success=true, intersect_point=intersect_point;
}


#if SHOULD_RUN_TESTS #run intersect_segment_with_plane(.{}, .{}, .{},.{},.{}, cast(*float) *context); //danger, never uncomment.
intersect_segment_with_plane :: (start: Vector3, end: Vector3, triangle_a: Vector3, triangle_b: Vector3, triangle_c: Vector3, t: *float) -> (success: bool, intersect_point: Vector3)
{
    plane_normal := cross_product(triangle_b - triangle_a, triangle_c - triangle_a);
    plane_distance := dot(plane_normal, triangle_a);
    success:, intersection_point := intersect_segment_with_plane(start, end, plane_normal, plane_distance, t);
    return success, intersection_point;
}


is_quad_convex :: (a: Vector3, b: Vector3, c: Vector3, d: Vector3)-> bool
{
    // Quad is nonconvex if Dot(Cross(bd, ba), Cross(bd, bc)) >= 0
     bda := cross_product(d - b, a - b);
     bdc := cross_product(d - b, c - b);
    if dot(bda, bdc) >= 0.0 return false;
    // Quad is now convex iff Dot(Cross(ac, ad), Cross(ac, ab)) < 0
     acd := cross_product(c - a, d - a);
     acb := cross_product(c - a, b - a);
    return dot(acd, acb) < 0.0;
}

distance_point_to_line_prenormalized:: inline (point: Vector3, line_start: Vector3, normalized_line_dir: Vector3)-> float
{
    hypothenuse := point - line_start;
    proj := dot(hypothenuse, normalized_line_dir)*normalized_line_dir;
    orth_diff := point-proj; //cant figure out a good name but orth_diff is perpendicualr to line
    return length(orth_diff);
}

DistanceFromLine :: distance_point_to_line;
distance_point_to_line:: inline (point: Vector3, line_start: Vector3, line_end: Vector3)-> float
{
    line_dir_n := normalize(line_start - line_end);
    hypothenuse := point - line_start;
    proj := dot(hypothenuse, line_dir_n)*line_dir_n;
    orth_diff := point-proj; //cant figure out a good name but orth_diff is perpendicualr to line
    return length(orth_diff);
}

distance_point_to_plane :: inline (point: Vector3, plane_normal: Vector3, plane_distance: float)-> float
{
    return abs((dot(plane_normal, point) - plane_distance));
}

distance_point_to_plane_signed :: inline (point: Vector3, plane_normal: Vector3, plane_distance: float)-> float
{
    return (dot(plane_normal, point) - plane_distance);
}

closest_point_to_plane :: (point: Vector3, plane_normal: Vector3, plane_distance: float) -> Vector3
{
    assert(length_squared(plane_normal) != 0);
    return point - distance_point_to_plane_signed(point, plane_normal, plane_distance) * plane_normal;
}

closest_point_to_on_segment ::( start: Vector3, end: Vector3, Point: Vector3)-> Vector3
{
    AB := end - start;
    t:  = dot(Point - start, AB) / dot(AB, AB);
    return start + min(max(t,0),1) * AB; // saturate(t) can be written as: min((max(t, 0), 1)
}

intersect_ray_with_plane :: (ray_origin: Vector3, ray_dir: Vector3, plane: Plane)-> bool, point_on_plane: Vector3
{
	point_on_plane: Vector3 = ---;
	fp := distance_point_to_plane_signed(point=ray_origin, plane.normal, plane.distance);
	fv := distance_point_to_plane_signed(point=ray_dir, plane.normal, plane.distance);
	t_slider := -1.0 * (fp / fv);
	if t_slider >= -FLOAT32_INFINITY && abs(t_slider) != FLOAT32_INFINITY
	{
		point_on_plane = ray_origin +  t_slider * ray_dir;
		return true, point_on_plane;
	}

	return false, point_on_plane;
}

// Also returns t for the position of d, d(t) = a + t*(b - a)
closest_point_to_on_segmentbad :: inline (point: Vector3, segment_start: Vector3, segment_end: Vector3, t: *float, d: *Vector3)
{
    ab := segment_end - segment_start;
    // Project c onto ab, computing parameterized position d(t) = a + t*(b – a)
    t.* = dot(point- segment_start, ab) / dot(ab, ab);
    if t.* < 0.0 t.* = 0.0;
    if t.* > 1.0 t.* = 1.0;
    d.* = segment_start + t.* * ab;


    //optimization, avoid division, (I dont care, this is year 2000 premature optimization)
    // // Project c onto ab, but deferring divide by Dot(ab, ab)
    // t = Dot(c – a, ab);
    // if (t <= 0.0) {
    // // c projects outside the [a,b] interval, on the a side; clamp to a
    // t = 0.0;
    // d = a;
    // } else {
    // float denom = Dot(ab, ab);
    // // Always nonnegative since denom = ||ab||∧2
    // if (t >= denom) {
    // // c projects outside the [a,b] interval, on the b side; clamp to b
    // t = 1.0;
    // d = b;
    // } else {
    // // c projects inside the [a,b] interval; must do deferred divide now
    // t = t / denom;
    // d = a + t * ab;
    // }
    // }
    // }
}

//source https://math.stackexchange.com/a/2812513 and the realtime book
closest_point_segment_to_segment :: inline (
 segmentA_start: Vector3, segmentA_end: Vector3,
 segmentB_start: Vector3, segmentB_end: Vector3
 )-> closest_point_on_A: Vector3, closest_point_on_B: Vector3, sss:float, slider_t: float, distance_distance: float
{
    closest_point_on_A: Vector3;
    closest_point_on_B: Vector3;
    slider_t: float = ---;
    sss: float = ---;

	dirA := segmentA_end - segmentA_start;
	dirB := segmentB_end - segmentB_start;
	verticalDirection := segmentA_start - segmentB_start;
	dirA_dirA := dot(dirA, dirA);
	dirB_dirB := dot(dirB, dirB);

	projected_center_B := dot(dirB, verticalDirection);


	A_segment_degenerate_into_point := dirA_dirA <= EPSILON5;
	B_segment_degenerate_into_point := dirB_dirB <= EPSILON5;
	if A_segment_degenerate_into_point && B_segment_degenerate_into_point
	{
		sss = 0;
		slider_t = 0;
    	closest_point_on_A = segmentA_start;
    	closest_point_on_B = segmentB_start;
    	distance_distance := dot(closest_point_on_A - closest_point_on_B, closest_point_on_A - closest_point_on_B);
        return closest_point_on_A, closest_point_on_B, sss, slider_t, distance_distance;
        // --------------- early return ---------- to be fixed, make it return at the end.
        // --------------- early return ---------- to be fixed, make it return at the end.
        // --------------- early return ---------- to be fixed, make it return at the end.
	}

	if A_segment_degenerate_into_point
	{
		sss = 0.0;
		slider_t = projected_center_B / dirA_dirA;
		slider_t = clamp(slider_t, 0.0, 1.0);
	}
	else
	{
		center_projected_A := dot(dirA, verticalDirection);
		if B_segment_degenerate_into_point
		{

			slider_t = 0.0;
			sss = clamp(-center_projected_A / dirA_dirA, 0.0, 1.0);
		}
		else
		{
			b: float = dot(dirA, dirB);
			denom: float = dirA_dirA * dirB_dirB - b * b;

			if denom != 0.0
				sss = clamp((b * projected_center_B - center_projected_A * dirB_dirB) / denom, 0.0, 1.0);
			else
				sss = 0.0;

			slider_t = (b * sss + projected_center_B) / dirB_dirB;

			if slider_t < 0.0
			{
				slider_t = 0.0;
				sss = clamp(-center_projected_A / dirA_dirA, 0.0, 1.0);
			}
			else if slider_t > 1.0
			{
				slider_t = 1.0;
				sss = clamp((b - center_projected_A) / dirA_dirA, 0.0, 1.0);
			}
		}
	}


	closest_point_on_A = segmentA_start + dirA * sss;
	closest_point_on_B = segmentB_start + dirB * slider_t;
	distance_distance := dot(closest_point_on_A - closest_point_on_B, closest_point_on_A - closest_point_on_B);

    return closest_point_on_A, closest_point_on_B, sss, slider_t, distance_distance;
}

distance_point_to_segment_squared :: (point: Vector3, segment_start: Vector3, segment_end: Vector3) -> float
{
    //I dont like using ab ac bc e iu js ie oa oep  or ,
    //those abreviations are unneccesary mental overhead,
    // Only works  when your brain have the correct cache lines in your frontal lobe.
    ab := segment_end - segment_start;
    ac := point - segment_start;
    bc := point - segment_end;
    e := dot(ac, ab);
    // Handle cases where c projects outside ab
    if e <= 0.0 return dot(ac, ac);
    f := dot(ab, ab);
    if e >= f return dot(bc, bc);
    // Handle cases where c projects onto ab
    return dot(ac, ac) - e * e / f;
}

// ClosestPtPointAABB

AABB :: struct {
     min: Vector3;
     max: Vector3;
     #place max;
        extents: Vector3 = ---;

     #place min;
      center: Vector3 = ---;
      half_extents: Vector3 = ---;

};


//I dont like using the name intersect because I was reserving that name to check if collision is true AND retriving collision points.
// intersect_aabb_with_aabb :: TestAABBAABB; // @Report: why this forces Southerland_Hodgmand to be a declaration?
// are_colliding ::  TestAABBAABB; // @Report: //I should use procedure overloading and semantic compression isntead.

is_aabb_valid :: (using aabb: AABB) -> bool {
  return min != max && length_squared(min) + length_squared(max) > COLLISION_MARGIN*COLLISION_MARGIN;
}

//used mostry for velocity, we get a point by extrapolating by velocity and dt and we expand the AABB  using that point
maybe_expand_aabb :: inline (using aabb: *AABB, point: Vector3) {
    if point.x < min.x then min.x = point.x;
    if point.y < min.y then min.y = point.y;
    if point.z < min.z then min.z = point.z;

    if point.x > max.x then max.x = point.x;
    if point.y > max.y then max.y = point.y;
    if point.z > max.z then max.z = point.z;
}

maybe_expand_aabb :: (using aabb: *AABB, ref_aabb: AABB) {
    maybe_expand_aabb(aabb, ref_aabb.min);
    maybe_expand_aabb(aabb, ref_aabb.max);
}

calculate_aabb_from_points :: (using aabb: *AABB, points: []Vector3) {
    for points {
        maybe_expand_aabb(aabb, it);
    }
}

//I dont support OBB because code is too long
OBB :: struct { //confusing data structure
    center: Vector3;
    local_axes: Matrix3; //you use it by rows local_axes.v[0] is x- axis, will see
    half_extents: Vector3;
};

#if SHOULD_RUN_TESTS #run TestOBBPlane(.{}, .{}, 0);
TestOBBPlane :: (obb: OBB,  plane_normal: Vector3, plane_distance: float)-> bool
{
    // Compute the projection interval radius of b onto L(t) = b.c + t * plane_normal
    r := obb.half_extents.component[0]*abs(dot(plane_normal, obb.local_axes.v[0])) +
         obb.half_extents.component[1]*abs(dot(plane_normal, obb.local_axes.v[1])) +
         obb.half_extents.component[2]*abs(dot(plane_normal, obb.local_axes.v[2]));
    // Compute distance of box center from plane
    s := dot(plane_normal, obb.center) - plane_distance;
    // Intersection occurs when distance s falls within [-r,+r] interval
    return abs(s) <= r;
}


// #if SHOULD_RUN_TESTS #run TestAABBAABB(.{},.{}, .CENTER_RADIUS);
TestAABBAABB :: (a: AABB, b: AABB, $mode: enum{MIN_MAX;MIN_EXTENT;CENTER_RADIUS;} = .MIN_MAX)-> bool
{
    // Exit with no intersection if separated along an axis
    #if mode == .MIN_MAX {
        if a.max.x < b.min.x || a.min.x > b.max.x return false;
        if a.max.y < b.min.y || a.min.y > b.max.y return false;
        if a.max.z < b.min.z || a.min.z > b.max.z return false;
    }
    #if mode == .MIN_EXTENT {
        t: float;
        if (a.min.x - b.min.x) > b.extents.x || -(a.min.x - b.min.x) > a.extents.x return false;
        if (a.min.y - b.min.y) > b.extents.y || -(a.min.y - b.min.y) > a.extents.y return false;
        if (a.min.z - b.min.z) > b.extents.z || -(a.min.z - b.min.z) > a.extents.z return false;
    }
    #if mode == .CENTER_RADIUS {
        if abs(a.center.x - b.center.x) > a.half_extents.x + b.half_extents.x return false;
        if abs(a.center.y - b.center.y) > a.half_extents.y + b.half_extents.y return false;
        if abs(a.center.z - b.center.z) > a.half_extents.z + b.half_extents.z return false;
    }
    // Overlapping on all axes means AABBs are intersecting
    return true;
};


// Given point p, return the point q on or in AABB b that is closest to p
#if SHOULD_RUN_TESTS #run closest_point_to_aabb(.{},.{});
closest_point_to_aabb :: ( point: Vector3, b: AABB)->Vector3
{
    closest: Vector3 = ---;
    for point.component
    {
        val := it;
        if val < b.min.component[it_index] then val = b.min.component[it_index]; // val = max(val, b.min[it_index])
        if val > b.max.component[it_index] then val = b.max.component[it_index]; // val = min(val, b.max[it_index])
        closest.component[it_index] = val;
    }
    return closest;
}


#if SHOULD_RUN_TESTS #run distance_point_to_aabb_squared(.{},.{});
distance_point_to_aabb_squared :: (point: Vector3, aabb: AABB) -> float
{
    sqDist := 0.0;
    for point.component
    {
        v := it;
        if v < aabb.min.component[it_index]
        then sqDist += (aabb.min.component[it_index] - v) * (aabb.min.component[it_index] - v);
        if v > aabb.max.component[it_index]
        then sqDist += (v - aabb.max.component[it_index]) * (v - aabb.max.component[it_index]);
    }
    return sqDist;
}


Capsule :: struct {
    start: Vector3;
    end: Vector3;

    radius: float;
};

Lozenge :: struct { //This is cheaper than OBB when objects are NEAR each other
    origin: Vector3;
    edge1: Vector3;
    edge2: Vector3;
    radius: float;
};


Rect :: struct {
     center: Vector3;
     axis1: Vector3;
     axis2: Vector3;
     half_extent: [2]float; //maybe I should use vector2 here.
     #place axis1;
        u: [2]Vector3 = ---;
};


// I hope this crap is useful later on.
ClosestPtPointRect :: (point: Vector3, rect: Rect) -> (closest: Vector3)
{
    closest: Vector3 = ---;
    d := point - rect.center;
    closest = rect.center;
    for 0..1
    {
        dist := dot(d, rect.u[it]);
        if dist > rect.half_extent[it] then dist = rect.half_extent[it];
        if dist < -rect.half_extent[it] then dist = -rect.half_extent[it];
        closest += dist * rect.u[it];
    }
    return closest;
}
// TestSphereCapsule :: (Sphere s, Capsule capsule) -> bool
// // Compute (squared) distance between sphere center and capsule line segment
// float dist2 = SqDistPointSegment(capsule.a, capsule.b, s.c);
// // If (squared) distance smaller than (squared) sum of radii, they collide
// float radius = s.r + capsule.r;
// return dist2 <= radius * radius;
// }

// TestCapsuleCapsule :: (Capsule capsule1, Capsule capsule2) -> bool
// // Compute (squared) distance between the inner structures of the capsules
// float s, t;
// Point c1, c2;
// float dist2 = ClosestPtSegmentSegment(capsule1.a, capsule1.b,
// capsule2.a, capsule2.b, s, t, c1, c2);
// // If (squared) distance smaller than (squared) sum of radii, they collide
// float radius = capsule1.r + capsule2.r;
// return dist2 <= radius * radius;
// }


// Point ClosestPtPointTriangle(Point p, Point a, Point b, Point c)
// closest_point_to_triangle :: (point: Vector3, a: Vector3, b: Vector3, c: Vector3) -> Vector3
// {
//     ab := b - a;
//     ac := c - a;
//     bc := c - b;
//     // Compute parametric position s for projection P’ of P on AB,
//     // P’ = A + s*AB, s = snom/(snom+sdenom)
//     snom := dot(point - a, ab);
//     sdenom := dot(point - b, a - b);
//     // Compute parametric position t for projection P’ of P on AC,
//     // P’ = A + t*AC, s = tnom/(tnom+tdenom)
//     tnom := dot(point - a, ac);
//     tdenom := dot(point - c, a - c);
//     if (snom <= 0.0 && tnom <= 0.0)
//     then return a;
//     // Vertex region early out
//     // Compute parametric position u for projection P’ of P on BC,
//     // P’ = B + u*BC, u = unom/(unom+udenom)
//     unom := dot(point - b, bc);
//     udenom := dot(point - c, b - c);
//     if sdenom <= 0.0 && unom <= 0.0
//     then return b;
//     if tdenom <= 0.0 && udenom <= 0.0
//     then return c;// Vertex region early out
//     // Vertex region early out
//     // P is outside (or on) AB if the triple scalar product [N PA PB] <= 0
//     n := cross_product(b - a, c - a);
//     vc := dot(n, cross_product(a - point, b - point));
//     // If P outside AB and within feature region of AB,
//     // return projection of P onto AB
//     if (vc <= 0.0 && snom >= 0.0 && sdenom >= 0.0)
//     then return a + snom / (snom + sdenom) * ab;
//     // P is outside (or on) BC if the triple scalar product [N PB PC] <= 0
//     va := dot(n, cross_product(b - point, c - point));
//     // If P outside BC and within feature region of BC,
//     // return projection of P onto BC
//     if (va <= 0.0 && unom >= 0.0 && udenom >= 0.0)
//     then return b + unom / (unom + udenom) * bc;
//     // P is outside (or on) CA if the triple scalar product [N PC PA] <= 0
//     vb := dot(n, cross_product(c - point, a - point));
//     // If P outside CA and within feature region of CA,
//     // return projection of P onto CA
//     if (vb <= 0.0 && tnom >= 0.0 && tdenom >= 0.0)
//     then return a + tnom / (tnom + tdenom) * ac;
//     // P must project inside face region. Compute Q using barycentric coordinates
//     u := va / (va + vb + vc);
//     v := vb / (va + vb + vc);
//     w := 1.0 - u - v;
//     // = vc / (va + vb + vc)
//     return u * a + v * b + w * c;
// }


#if SHOULD_RUN_TESTS #run () => {result := closest_point_to_triangle(.{1.1,1,0},.{0,0,0},.{1,0,0},.{1,1,0}); assert(result==.{1,1,0}, tprint("%",result));}();
closest_point_to_triangle :: (point: Vector3, a: Vector3, b: Vector3, c: Vector3) -> Vector3
{
    ab := b - a;
    ac := c - a;
    ap := point - a;

    // Check if P in vertex region outside A
    d1 := dot(ab, ap);
    d2 := dot(ac, ap);
    if d1 <= 0.0 && d2 <= 0.0 return a; // barycentric coordinates (1,0,0)

    // Check if P in vertex region outside B
    bp := point - b;
    d3 := dot(ab, bp);
    d4 := dot(ac, bp);
    if d3 >= 0.0 && d4 <= d3  return b; // barycentric coordinates (0,1,0)

    // Check if P in edge region of AB, if so return projection of P onto AB
    vc := d1*d4 - d3*d2;
    if vc <= 0.0 && d1 >= 0.0 && d3 <= 0.0 {
    v := d1 / (d1 - d3);
    return a + v * ab; // barycentric coordinates (1-v,v,0)
    }
    // Check if P in vertex region outside C
    cp := point - c;
    d5 := dot(ab, cp);
    d6 := dot(ac, cp);
    if (d6 >= 0.0 && d5 <= d6) return c; // barycentric coordinates (0,0,1)

    // Check if P in edge region of AC, if so return projection of P onto AC
    vb := d5*d2 - d1*d6;
    if vb <= 0.0 && d2 >= 0.0 && d6 <= 0.0 {
    w := d2 / (d2 - d6);
    return a + w * ac; // barycentric coordinates (1-w,0,w)
    }
    // Check if P in edge region of BC, if so return projection of P onto BC
    va := d3*d6 - d5*d4;
    if va <= 0.0 && (d4 - d3) >= 0.0 && (d5 - d6) >= 0.0 {
    w := (d4 - d3) / ((d4 - d3) + (d5 - d6));
    return b + w * (c - b); // barycentric coordinates (0,1-w,w)
    }
    // P inside face region. Compute Q through its barycentric coordinates (u,v,w)
    denom := 1.0 / (va + vb + vc);
    v := vb * denom;
    w := vc * denom;
    return a + ab * v + ac * w; // = u*a + v*b + w*c, u = va * denom = 1.0f - v - w
}

is_point_outside_plane :: inline (point: Vector3, a: Vector3, b: Vector3, c: Vector3) -> bool
{
    return dot(point - a, cross_product(b - a, c - a)) >= 0.0; // [AP AB AC] >= 0
}

is_point_outside_plane :: inline (point: Vector3, a: Vector3, b: Vector3, c: Vector3, d: Vector3) -> bool
{
    signp := dot(point - a, cross_product(b - a, c - a)); // [AP AB AC]
    signd := dot(d - a, cross_product(b - a, c - a)); // [AD AB AC]
    return signp * signd < 0.0; // Points on opposite sides if expression signs are opposite
}


#if SHOULD_RUN_TESTS #run closest_point_to_tetrahedron(.{},.{},.{},.{},.{});
closest_point_to_tetrahedron :: (point: Vector3, a: Vector3, b: Vector3, c: Vector3, d: Vector3)-> (closest: Vector3)
{
    //tetrahedron: ABC, ACD,ADB, BDC are CCW
    // Start out assuming point inside all halfspaces, so closest to itself
    closest := point;
    bestSqDist := FLOAT32_MAX;
    // If point outside face abc then compute closest point on abc
    if (is_point_outside_plane(point, a, b, c))
    {
        q := closest_point_to_triangle(point, a, b, c);
        sqDist := dot(q - point, q - point);
        // Update best closest point if (squared) distance is less than current best
        if (sqDist < bestSqDist) then bestSqDist, closest = sqDist, q;
    }
    // Repeat test for face acd
    if (is_point_outside_plane(point, a, c, d))
    {
        q := closest_point_to_triangle(point, a, c, d);
        sqDist := dot(q - point, q - point);
        if (sqDist < bestSqDist) then bestSqDist, closest = sqDist, q;
    }
    // Repeat test for face adb
    if (is_point_outside_plane(point, a, d, b))
    {
        q := closest_point_to_triangle(point, a, d, b);
        sqDist := dot(q - point, q - point);
        if (sqDist < bestSqDist) then bestSqDist, closest = sqDist, q;
    }
    // Repeat test for face bdc
    if (is_point_outside_plane(point, b, d, c))
    {
        q := closest_point_to_triangle(point, b, d, c);
        sqDist := dot(q - point, q - point);
        if (sqDist < bestSqDist) then bestSqDist, closest = sqDist, q;
    }
    return closest;
}


Sphere :: struct {
    center: Vector3;
    radius: float;
};

#if SHOULD_RUN_TESTS #run TestSpherePlane(.{}, .{}, 0);
TestSpherePlane :: (sphere: Sphere, plane_normal: Vector3, plane_distance: float) -> bool
{
  dist := dot(sphere.center, plane_normal) - plane_distance;
  return abs(dist) <= sphere.radius;
}


#if SHOULD_RUN_TESTS #run InsideSpherePlane(.{}, .{}, 0);
InsideSpherePlane :: (sphere: Sphere, plane_normal: Vector3, plane_distance: float)-> bool
{
    dist := dot(sphere.center, plane_normal) - plane_distance;
    return dist < -sphere.radius;
}

#if SHOULD_RUN_TESTS #run TestSphereHalfspace(.{}, .{}, 0);
TestSphereHalfspace :: (sphere: Sphere, plane_normal: Vector3, plane_distance: float)-> bool
{
    dist := dot(sphere.center, plane_normal) - plane_distance;
    return dist <= sphere.radius;
}


#if SHOULD_RUN_TESTS #run TestAABBPlane(.{}, .{}, 0);
TestAABBPlane :: (aabb: AABB,  plane_normal: Vector3, plane_distance: float)-> bool
{
    // These two lines not necessary with a (center, extents) AABB representation
    aabb_center := (aabb.max + aabb.min) * 0.5; // Compute AABB aabb_center
    positive_extents := aabb.max - aabb_center; // Compute positive extents
    // Compute the projection interval radius of b onto L(t) = aabb.c + t * p.n
    r := positive_extents.component[0]*abs(plane_normal.component[0]) + positive_extents.component[1]*abs(plane_normal.component[1]) + positive_extents.component[2]*abs(plane_normal.component[2]);
    // Compute distance of box aabb_center from plane
    s := dot(plane_normal, aabb_center) - plane_distance;
    // Intersection occurs when distance s falls within [-r,+r] interval
    return abs(s) <= r;
}


#if SHOULD_RUN_TESTS #run TestSphereAABB(Sphere.{}, AABB.{});
TestSphereAABB :: (sphere: Sphere, aabb: AABB)-> bool
{
    sqDist := distance_point_to_aabb_squared(sphere.center, aabb);
    return sqDist <= sphere.radius * sphere.radius;
}

#if SHOULD_RUN_TESTS #run TestSphereAABB(Sphere.{}, precomputed_closest_point_to_aabb=.{});
TestSphereAABB :: (sphere: Sphere, precomputed_closest_point_to_aabb: Vector3)-> bool
{
    sqDist := length_squared(precomputed_closest_point_to_aabb);
    return sqDist <= sphere.radius * sphere.radius;
}



#if SHOULD_RUN_TESTS #run TestSphereTriangle(.{},.{},.{},.{});
TestSphereTriangle :: (sphere: Sphere, a: Vector3, b: Vector3, c: Vector3)-> (success: bool, closest: Vector3)
{
    closest := closest_point_to_triangle(sphere.center, a, b, c);
    v := closest - sphere.center;
    return dot(v, v) <= sphere.radius * sphere.radius, closest;
}


// Polygon :: struct {
//     MAX_NUM_VERTS :: 32;
//     _memory: [MAX_NUM_VERTS]Vector3;
//     vertices: []Vector3;
// }

// // Test whether sphere s intersects polygon p
// TestSpherePolygon :: (s: Sphere, p: Polygon) -> bool
//     // Compute normal for the plane of the polygon
//     plane_normal := normalize(cross_product(p.v[1] - p.v[0], p.v[2] - p.v[0]));
//     // Compute the plane equation for p
//     plane_distance := -dot(n, p.v[0]);
//     // No intersection if sphere not intersecting plane of polygon
//     if (!TestSpherePlane(s, m)) return 0;
//     // Test to see if any one of the polygon edges pierces the sphere
//     for p.vertices
//     {
//     for (int k = p.numVerts, i = 0, j = k - 1; i < k; j = i, i++) {
//     float t;
//     Point q;
//     // Test if edge (p.v[j], p.v[i]) intersects s
//     if (IntersectRaySphere(p.v[j], p.v[i] - p.v[j], s, t, q) && t <= 1.0
//         return 1;
//     }
//     // Test if the orthogonal projection q of the sphere center onto m is inside p
//     closest := ClosestPtPointPlane(s.c, m);
//     return PointInPolygon(q, p);
// }

// TestTriangleAABB :: (v0: Vector3, v1: Vector3, v2: Vector3, aabb: AABB) -> bool
// {
//     p0:, p1:, p2:, r: float;

//     // Compute box center and extents (if not already given in that format)
//     c := (aabb.min + aabb.max) * 0.5;
//     e0 := (aabb.max.x - aabb.min.x) * 0.5;
//     e1 := (aabb.max.y - aabb.min.y) * 0.5;
//     e2 := (aabb.max.z - aabb.min.z) * 0.5;
//     // Translate triangle as conceptually moving AABB to origin
//     v0 = v0 - c;
//     v1 = v1 - c;
//     v2 = v2 - c;
//     // Compute edge vectors for triangle
//     f0: = v1 - v0;
//     f1 := v2 - v1;
//     f2 := v0 - v2;
//     // Test axes a00..a22 (category 3)
//     // Test axis a00
//     p0 = v0.z*v1.y - v0.y*v1.z;
//     p2 = v2.z*(v1.y - v0.y) - v2.z*(v1.z - v0.z);
//     r = e1 * abs(f0.z) + e2 * abs(f0.y);
//     if (max(-max(p0, p2), min(p0, p2)) > r) return 0;
//     // Axis is a separating axis
//     // Repeat similar tests for remaining axes a01..a22
//     ...
//     // Test the three axes corresponding to the face normals of AABB b (category 1).
//     // Exit if...
//     // ... [-e0, e0] and [min(v0.x,v1.x,v2.x), max(v0.x,v1.x,v2.x)] do not overlap
//     if (max(v0.x, v1.x, v2.x) < -e0 || min(v0.x, v1.x, v2.x) > e0) return 0;
//     // ... [-e1, e1] and [min(v0.y,v1.y,v2.y), max(v0.y,v1.y,v2.y)] do not overlap
//     if (max(v0.y, v1.y, v2.y) < -e1 || min(v0.y, v1.y, v2.y) > e1) return 0;
//     // ... [-e2, e2] and [min(v0.z,v1.z,v2.z), max(v0.z,v1.z,v2.z)] do not overlap
//     if (max(v0.z, v1.z, v2.z) < -e2 || min(v0.z, v1.z, v2.z) > e2) return 0;
//     // Test separating axis corresponding to triangle face normal (category 2)
//     plane_normal := cross_product(f0, f1);
//     plane_distance := dot(plane_normal, v0);
//     return TestAABBPlane(aabb, plane_normal, plane_distance,);
// }
//
/*



*/

#if SHOULD_RUN_TESTS #run IntersectRaySphere(.{}, .{}, .{}, cast(*float) *context); //danger, never uncomment.
IntersectRaySphere :: (ray_start: Vector3, ray_dir: Vector3, sphere: Sphere, t: *float) -> (success: bool, intersect_point: Vector3)
{
    sphere_to_ray := ray_start - sphere.center;
    b := dot(sphere_to_ray, ray_dir);
    c := dot(sphere_to_ray, sphere_to_ray) - sphere.radius * sphere.radius;
    if c > 0.0 && b > 0.0 return false, .{};
    discriminant := b*b - c; //@Doubt b*b is metets^4 and c is meters^2.
    if discriminant < 0.0 return false, .{};
    t.* = -b - sqrt(discriminant); //maybe clamp?
    if t.* < 0.0 t.* = 0.0;
    return true, ray_start + (t.*) * ray_dir;
}

#if SHOULD_RUN_TESTS #run TestRaySphere(.{}, .{}, .{});
TestRaySphere :: (ray_start: Vector3, ray_dir: Vector3, sphere: Sphere)-> bool
{
    sphere_to_ray := ray_start - sphere.center;
    c := dot(sphere_to_ray, sphere_to_ray) - sphere.radius * sphere.radius;
    if c <= 0.0 return true;
    b := dot(sphere_to_ray, ray_dir);
    if b > 0.0 return false;
    disc := b*b - c;
    if disc < 0.0 return false;
    return true;
}


#if SHOULD_RUN_TESTS #run IntersectRayAABB(.{}, .{}, .{}, cast(*float) *context); //danger, never uncomment.
IntersectRayAABB :: (ray_start: Vector3, ray_dir: Vector3, aabb: AABB, tmin: *float)-> (success: bool, intersect_point: Vector3)
{
    EPSILON7 :: 0.00000001; //random small number, to twak I guess... @Todo create epsilons EPSILON4 EPSILON5 EPSILON6 EPSILON7 xD
    tmin.* = 0.0;
    tmax := FLOAT32_MAX; // set to -FLT_MAX to get first hit on line
    for 0..2 {
        if abs(ray_dir.component[it]) < EPSILON7 {
            // Ray is parallel to slab. No hit if origin not within slab
            if ray_start.component[it] < aabb.min.component[it] || ray_start.component[it] > aabb.max.component[it] return false, .{};
        } else {
            ood := 1.0 / ray_dir.component[it];
            t1 := (aabb.min.component[it] - ray_start.component[it]) * ood;
            t2 := (aabb.max.component[it] - ray_start.component[it]) * ood;
            if t1 > t2 then t1, t2 = t2, t1;
            if t1 > tmin.* tmin.* = t1;
            if t2 > tmax tmax = t2;
            if tmin.* > tmax return false, .{};
        }
    }
    return success=true, intersect_point=ray_start + (tmin.*)*ray_dir;
}


#if SHOULD_RUN_TESTS #run TestSegmentAABB(.{}, .{}, .{});
TestSegmentAABB :: (segment_start: Vector3, segment_end: Vector3, aabb: AABB)-> bool
{
    EPSILON7 :: 0.00000001; //random small number, to twak I guess... @Todo create epsilons EPSILON4 EPSILON5 EPSILON6 EPSILON7 xD
    box_center := (aabb.min + aabb.max) * 0.5; // Box center-point
    e := aabb.max - box_center; // Box halflength extents
    m := (segment_start + segment_end) * 0.5; // Segment midpoint
    d := segment_end - m; // Segment halflength vector
    m = m - box_center;

    //optimization:
    // e := aabb.max - aabb.min;
    // d := segment_end - segment_start;
    // m := segment_start + segment_end - aabb.min - aabb.max;
    //

    // Translate box and segment to origin
    // Try world coordinate axes as separating axes
    adx := abs(d.x);
    if (abs(m.x) > e.x + adx) return false;
    ady := abs(d.y);
    if (abs(m.y) > e.y + ady) return false;
    adz := abs(d.z);
    if (abs(m.z) > e.z + adz) return false;
    // Add in an epsilon term to counteract arithmetic errors when segment is
    // (near) parallel to a coordinate axis (see text for detail)
    adx += EPSILON7; ady += EPSILON7; adz += EPSILON7;

    // cross_products tests that can fail in precision!
    if (abs(m.y * d.z - m.z * d.y) > e.y * adz + e.z * ady) return false;
    if (abs(m.z * d.x - m.x * d.z) > e.x * adz + e.z * adx) return false;
    if (abs(m.x * d.y - m.y * d.x) > e.x * ady + e.y * adx) return false;
    // No separating axis found; segment must be overlapping AABB
    return true;
}



#if SHOULD_RUN_TESTS #run IntersectLineTriangle(.{}, .{}, .{},.{},.{});
IntersectLineTriangle :: (line_start: Vector3, line_end: Vector3, a: Vector3, b: Vector3, c: Vector3)-> (success: bool, barycentric_coordinates: Vector3)
{
    barycentric_coordinates: Vector3;
    pq := line_end - line_start;
    pa := a - line_start;
    pb := b - line_start;
    pc := c - line_start;
    // Test if pq is inside the edges bc, ca and ab. Done by testing
    // that the signed tetrahedral volumes, computed using scalar triple
    // products, are all positive
    using barycentric_coordinates;
    m1 := cross_product(pq,pc);
    x = dot(m1, pb);
    if x < 0.0 return false, .{};
    y = -dot(m1, pa);
    if y < 0.0 return false, .{};
    m2 := cross_product(pq,pb);
    z = dot(m2, pa);
    if z < 0.0 return false, .{};
    denom := 1.0 / (x + y + z);
    barycentric_coordinates *= denom;
    return true, barycentric_coordinates; // intersection point r, r = bary_x*a + bary_y*b + bary_z*c
}


#if SHOULD_RUN_TESTS #run IntersectLineQuad(.{}, .{}, .{},.{},.{},.{});
IntersectLineQuad :: ( line_start: Vector3,  line_end: Vector3,  a: Vector3,  b: Vector3,  c: Vector3,  d: Vector3)-> (success: bool, intersection_point: Vector3)
{
    intersection_point: Vector3;
    pq := line_end - line_start;
    pa := a - line_start;
    pb := b - line_start;
    pc := c - line_start;
    // Determine which triangle to test against by testing against diagonal first
    m := cross_product(pc, pq);
    v: = dot(pa, m);
    // ScalarTriple(pq, pa, pc);
    if v >= 0.0 {
        // Test intersection against triangle abc
        u: = -dot(pb, m);
        // ScalarTriple(pq, pc, pb);
        if u < 0.0 return false, .{};
        w: = dot(pq, cross_product(pb, pa));
        if w < 0.0 return false, .{};
        // Compute r, r = u*a + v*b + w*c, from barycentric coordinates (u, v, w)
        denom: = 1.0 / (u + v + w);
        u *= denom;
        v *= denom;
        w *= denom;
        // w = 1.0f - u - v;
        intersection_point = u*a + v*b + w*c;
    } else {
        // Test intersection against triangle dac
        pd := d - line_start;
        u: = dot(pd, m);
        // ScalarTriple(pq, pd, pc);
        if u < 0.0 return false, .{};
        w: = dot(pq, cross_product(pa, pd));
        if w < 0.0 return false, .{};
        v = -v;
        // Compute r, r = u*a + v*d + w*c, from barycentric coordinates (u, v, w)
        denom: = 1.0 / (u + v + w);
        u *= denom;
        v *= denom;
        w *= denom;
        // w = 1.0f - u - v;
        intersection_point = u*a + v*d + w*c;
    }
    return true, intersection_point;
}


// Given segment pq and triangle abc, returns whether segment intersects
// triangle and if so, also returns the barycentric coordinates (u,v,w)
// of the intersection point

#if SHOULD_RUN_TESTS #run IntersectSegmentTriangle(.{}, .{}, .{},.{},.{});
IntersectSegmentTriangle :: ( line_start: Vector3,  line_end: Vector3,  a: Vector3,  b: Vector3,  c: Vector3)-> (success: bool, barycentric_coordinates: Vector3, slider_t: float)
{
    barycentric_coordinates: Vector3;
    slider_t: float;

    ab := b - a;
    ac := c - a;
    qp := line_start - line_end;
    // Compute triangle normal. Can be precalculated or cached if
    // intersecting multiple segments against the same triangle
    n := cross_product(ab, ac);
    // Compute denominator d. If d <= 0, segment is parallel to or points
    // away from triangle, so exit early
    d := dot(qp, n);
    if d <= 0.0 return false, .{}, 0;
    // Compute intersection t value of pq with plane of triangle. A ray
    // intersects iff 0 <= t. Segment intersects iff 0 <= t <= 1. Delay
    // dividing by d until intersection has been found to pierce triangle

    ap := line_start - a;
    slider_t = dot(ap, n);
    if slider_t < 0.0 return false, .{}, 0;
    if slider_t > d return false, .{}, 0;
    // For segment; exclude this code line for a ray test
    // Compute barycentric coordinate components and test if ithin bounds
    e := cross_product(qp, ap);

    using barycentric_coordinates;
    y = dot(ac, e);
    if y < 0.0 || y > d return false, .{}, 0;
    z = -dot(ab, e);
    if z < 0.0 || y + z > d return false, .{}, 0;
    // Segment/ray intersects triangle. Perform delayed diyision and
    // compute the last barycentric coordinate component
    ood := 1.0 / d; //jacobian^-1
    slider_t *= ood;
    y *= ood;
    z *= ood;
    x = 1.0 - y - z;
    return true, barycentric_coordinates, slider_t;
}


Triangle :: struct {
     face_plane_normal: Vector3;
     face_plane_distance: float;

     edge1_plane_normal: Vector3; //bc  barycentric x
     edge1_plane_distance: float;

     edge2_plane_normal: Vector3; //ca barycentric y
     edge2_plane_distance: float;
};


#if SHOULD_RUN_TESTS #run create_triangle(.{}, .{}, .{});
create_triangle :: (a: Vector3, b: Vector3, c: Vector3) -> Triangle
{
    tri: Triangle = ---;
    //idk how to name d. but I dont like d, its not descriptive enough.
    big_d := cross_product(b-a,c-a);
    tri.face_plane_normal = normalize(big_d);
    tri.face_plane_distance = length(big_d);


    edge1_d := cross_product(big_d,c-b);
    tri.edge1_plane_normal = normalize(edge1_d);;
    tri.edge1_plane_distance = length(edge1_d);


    edge2_d := cross_product(big_d,a-c);
    tri.edge2_plane_normal = normalize(edge2_d);;
    tri.edge2_plane_distance = length(edge2_d);

    //@Investigate this crap
    edge1_precomputed_barycentric_jacobian := 1.0 / (dot(a, tri.edge1_plane_normal) - tri.edge1_plane_distance);
    tri.edge1_plane_distance *= edge1_precomputed_barycentric_jacobian;

    edge2_precomputed_barycentric_jacobian := 1.0 / (dot(b, tri.edge2_plane_normal) - tri.edge2_plane_distance);
    tri.edge2_plane_distance *= edge2_precomputed_barycentric_jacobian;

    return tri;
}

distance_point_to_triangle :: (point: Vector3, a: Vector3,b: Vector3,c: Vector3) -> distance: float {
    return distance=abs(distance_point_to_triangle_signed(point, a, b, c));
}

distance_point_to_triangle :: inline (point: Vector3, tri: Triangle) -> distance: float {
    assert(false, "TOODO");
    return 0.0;//distance=abs(distance_point_to_triangle_signed(point, tri.a, tri.b, tri.c));
}

distance_point_to_triangle_signed :: (point: Vector3, a: Vector3,b: Vector3,c: Vector3) -> distance: float {
    ab := b - a;
    ac := c - a;
    normal := normalize(cross_product(ab, ac));
    dir :=  point - a;
    return distance=dot(dir, normal);

}

distance_point_to_triangle_signed :: inline (point: Vector3, tri: Triangle) -> distance: float {
    return distance=dot(point, tri.face_plane_normal) - tri.face_plane_distance;;
}



#if SHOULD_RUN_TESTS #run IntersectSegmentTriangle(.{}, .{}, .{}, cast(*Vector3)*context); //danger dont uncomment
IntersectSegmentTriangle :: (segment_start: Vector3, segment_end: Vector3, tri: Triangle,  intersection_point: *Vector3) -> (success: bool, barycentric_coordinates: Vector3, slider_t: float)
{
    barycentric_coordinates: Vector3;
    slider_t: float;
    // Compute distance of p to triangle plane. Exit if p lies behind plane
    distp := dot(segment_start, tri.face_plane_normal) - tri.face_plane_distance;
    if distp < 0.0 return false, .{}, 0.0;
    // Compute distance of q to triangle plane. Exit if q lies in front of plane
    distq := dot(segment_end, tri.face_plane_normal) - tri.face_plane_distance;
    if distq >= 0.0 return false, .{}, 0.0;
    // Compute t value and point s of intersection with triangle plane
    denom := distp - distq;
    slider_t = distp / denom;
    intersection_point.* = segment_start + slider_t * (segment_end - segment_start);
    using barycentric_coordinates;
    // Compute the barycentric coordinate u; exit if outside 0..1 range
    x = dot(intersection_point.*, tri.edge1_plane_normal) - tri.edge1_plane_distance;

    if x < 0.0 || x > 1.0 return false, .{}, 0.0;
    // Compute the barycentric coordinate v; exit if negative
    y = dot(intersection_point.*, tri.edge2_plane_normal) - tri.edge2_plane_distance;
    if y < 0.0 return false, .{}, 0.0;
    // Compute the barycentric coordinate w; exit if negative
    z = 1.0 - x - y;
    if z < 0.0 return false, .{}, 0.0;
    return true, barycentric_coordinates, slider_t; // Segment intersects tri at distance t in position s (s = u*A + v*B + w*C)
}


#if SHOULD_RUN_TESTS #run IntersectSegmentPolyhedron(.{},.{},.[]);
IntersectSegmentPolyhedron :: (segment_start: Vector3, segment_end: Vector3, planes: []struct{plane_normal: Vector3; plane_distance: float;}) -> (success: bool,  tfirst: float, tlast:float )
{
    segment_dir := segment_end - segment_start;

    tfirst := 0.0; // For a line, additionally tfirst should be set to –FLT_MAX
    tlast := 1.0; //For a ray, tlast should be set to +FLT_MAX.

    // Intersect segment against each plane
    for planes
    {
        denom := dot(it.plane_normal, segment_dir);
        dist := it.plane_distance - dot(it.plane_normal, segment_start);
        // Test if segment runs parallel to the plane
        if denom == 0.0
        {
            if dist > 0.0 return false, 0, 0;
        }
        else
        {
            t := dist / denom;
            if denom < 0.0
            {
                if t > tfirst tfirst = t;
            }
            else
            {
                if t < tlast tlast = t;
            }
            if tfirst > tlast return false, 0, 0;
        }
    }

    return true, tfirst, tlast;
}

