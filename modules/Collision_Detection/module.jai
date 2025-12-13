

SAT_dirk_gregorius :: (hullA: Convex_Hull, hullB: Convex_Hull, query: *SAT_Query) -> bool
{
    center_A := hull_center(hullA);
	query.faceQuery_AB = QueryFaceDirections(hullA, hullB);
    if query.faceQuery_AB.distance > FLOAT32_EPSILON4 {
     draw_primitive.doLabel("faceQuery_AB", draw_primitive.WorldToScreen(center_A));
     return false;
     }

	query.faceQuery_BA = QueryFaceDirections(hullB, hullA);
    if query.faceQuery_BA.distance > FLOAT32_EPSILON4 {
     draw_primitive.doLabel("faceQuery_BA", draw_primitive.WorldToScreen(center_A));
     return false;
     }

    #if false {
        query.edgeQuery = QueryEdgeDirections_deSenk(hullA, hullB);
    } else {
        query.edgeQuery = QueryEdgeDirections_gauss(hullA, hullB);
    }

    if query.edgeQuery.distance > FLOAT32_EPSILON4  {
     draw_primitive.doLabel("edgeQuery", draw_primitive.WorldToScreen(center_A));
     return false;
     }
	return true;
}

#import "Drawing_Primitives";
draw_debug_hull :: (position: Vector3, orientation: Quaternion, hull_shape: Convex_Hull) {
    vert_count: u32;
    vertices_tmp: [4]Vector3;

    color_idx:=0;
    for :all_faces current_face: hull_shape.mesh {
        color_idx += 1;
        for :traverse_edge_ring current_edge: current_face {
	        localPoint := current_edge.tail.vertex;
    		vertices_tmp[vert_count] = localPoint;
    		vert_count += 1;
        }

        n := transform_normal(current_face.face_plane_normal, hull_shape.inv_worldSpace_for_bivectors);
        color := colors[color_idx];
        normalize(*color);

        if vert_count == {
            case 4;
    		    v0 := transform_point(hull_shape.worldSpace, vertices_tmp[0]);
                v1 := transform_point(hull_shape.worldSpace, vertices_tmp[1]);
                v2 := transform_point(hull_shape.worldSpace, vertices_tmp[2]);
                v3 := transform_point(hull_shape.worldSpace, vertices_tmp[3]);
                draw_primitive.polygon(.[ v0, v1, v2, v3 ], color);
            case 3;
    		    v0 := transform_point(hull_shape.worldSpace, vertices_tmp[0]);
                v1 := transform_point(hull_shape.worldSpace, vertices_tmp[1]);
                v2 := transform_point(hull_shape.worldSpace, vertices_tmp[2]);
                draw_primitive.triangle( v0, v1, v2, color);

            case;
                assert(false, "vert_count %", vert_count);
            }
	   vert_count = 0; //to use vertices_tmp array as a ring buffer
	}
}


//
// Imports and private module stuff
//
#load "procedures.jai";
#load "data_structures.jai";
#load "realtime.jai";
#import "Math";
#import "Basic"; //for asserts


#scope_module
TODO :: (msg: string,loc:= #caller_location) {
 //this is for VSCODE |||filepath line:character|||:
  log("\n[TODO->] %, at-> % %:% [<-TODO]", msg, loc.fully_pathed_filename, loc.line_number, loc.character_number);
}

FLOAT32_EPSILON7 :float32: 0h33d6bf95; //0.0000001
FLOAT32_EPSILON6 :float32: 0h358637bd; //0.000001
FLOAT32_EPSILON5 :float32: 0h3727c5ac; //0.00001
FLOAT32_EPSILON4 :float32: 0h38d1b717; //0.0001
FLOAT32_EPSILON3 :float32: 0h3a83126f; //0.001
FLOAT32_EPSILON2 :float32: 0h3c23d70a; //0.01
FLOAT32_EPSILON1 :float32: 0h3dcccccd; //0.1
FLOAT32_EPSILON0 :float32: 0h3f800000; //1.0


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
