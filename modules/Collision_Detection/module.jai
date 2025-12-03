

SAT_dirk_gregorius :: (hullA: Convex_Hull, hullB: Convex_Hull, query: *SAT_Query) -> bool
{

	query.faceQuery_AB = QueryFaceDirections(hullA, hullB);
	if query.faceQuery_AB.distance > EPSILON4
		return false;

	query.faceQuery_BA = QueryFaceDirections(hullB, hullA);
	if query.faceQuery_BA.distance > EPSILON4
		return false;
#if 1
    query.edgeQuery = QueryEdgeDirections(hullA, hullB);
else
	query.edgeQuery = QueryEdgeDirections_bruteforce(hullA, hullB);


	if query.edgeQuery.distance > EPSILON4
		return false;

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
//We do an import here, grep this -> :tag_Extra_Import_Here_Maybe_Pull_Out


#scope_module

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
