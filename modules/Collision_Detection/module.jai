

SAT_dirk_gregorius :: (hullA: Convex_Hull, hullB: Convex_Hull, query: *SAT_Query) -> bool
{
	query.faceQuery_AB = QueryFaceDirections(hullA, hullB);
    if query.faceQuery_AB.distance > EPSILON4 return false;

	query.faceQuery_BA = QueryFaceDirections(hullB, hullA);
    if query.faceQuery_BA.distance > EPSILON4 return false;

    if collider_switch_face {
        query.edgeQuery = QueryEdgeDirections_deSenk(hullA, hullB);
    } else {
        query.edgeQuery = QueryEdgeDirections_gauss(hullA, hullB);
    }

    if query.edgeQuery.distance > EPSILON4  return false;
	return true;
}

//
// Imports and private module stuff
//
#load "procedures.jai";
#load "data_structures.jai";
#load "realtime.jai";
#import "Math";
#import "Basic"; //for asserts


//mock drawing procedures
#scope_module
#if !#exists(WorldToScreen)
WorldToScreen :: (position: Vector3) -> Vector3, is_inside_frustrum: bool {};


#if !#exists(doLabel)
doLabel :: (text: string, pos: Vector3)  {};

#if !#exists(push_line)
push_line :: (start : Vector3, end : Vector3, color : Vector4) {};

#if !#exists(push_arrow)
push_arrow ::  (start: Vector3, vector: Vector3, head_size: float, color: Vector4) {};


#if !#exists(push_circle)
push_circle :: (center: Vector3, radius: float = 0.4, segments: int = 16, normal: Vector3 = .{0,1,0}, color: Vector4 = .{.5,.7,.5,1}) {};


#if !#exists(push_vector)
push_vector :: (start : Vector3, end : Vector3, color : Vector4) {};

#if !#exists(push_grid)
push_grid :: (plane_origin: Vector3, plane_normal: Vector3, min_for: s64, max_for: s64 , density:float=1.0, color := Vector4.{0.4, 0.4, 0.4, 1}) {};
#scope_export
#scope_module
TODO :: (msg: string,loc:= #caller_location) {
 //this is for VSCODE |||filepath line:character|||:
  log("\n[TODO->] %, at-> % %:% [<-TODO]", msg, loc.fully_pathed_filename, loc.line_number, loc.character_number);
}
EPSILON7 :: 0.0000001;
EPSILON6 :: 0.000001;
EPSILON5 :: 0.00001;
EPSILON4 :: 0.0001;
EPSILON3 :: 0.001;
EPSILON2 :: 0.01;
EPSILON1 :: 0.1;
EPSILON0 :: 1.0;


operator * :: (q: Quaternion, v:Vector3) -> Vector3 #symmetric
{
    u := Vector3.{q.x, q.y, q.z};
    s := q.w;

    // Compute v' = 2 * dot(u, v) * u + (s^2 - dot(u, u)) * v + 2 * s * cross(u, v)
    dot_u_v := dot(u, v);
    dot_u_u := dot(u, u);
    cross_u_v := cross(u, v);

    result := 2.0 * dot_u_v * u + (s * s - dot_u_u) * v + 2.0 * s * cross_u_v;
    return result;
}


operator * :: (m: Matrix4, v: Vector3) -> Vector3 {

    result: Vector3 = ---;
    tmp: Vector4 = ---;
    tmp.xyz = v;
    tmp.w = 1.0;

    r4 := inline multiply(m, tmp);
    result = r4.xyz;
    return result;
}

#scope_export
