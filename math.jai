
#scope_file
#import "Math";
#import "Basic";

#scope_export

make_quaternion_from_axis_and_angle :: (axiss: Vector3, angle_radians: float) -> Quaternion {
    q: Quaternion = ---;
    set_from_axis_and_angle(*q, axiss, angle_radians);
    return q;
}


is_valid :: inline (a: int) -> bool {
    return true;
}

is_valid :: inline (a: float32) -> bool {
    is_nan, is_inf := is_nan_is_inf(a);
    return !is_nan;
}

is_valid :: inline (a: float64) -> bool {
    is_nan, is_inf := is_nan_is_inf(a);
    return !is_nan;
}

is_valid :: inline (a: Quaternion) -> bool {
    for a.component {
        is_nan, is_inf := is_nan_is_inf(it);
        if is_nan || is_inf return false;
    }
    if !is_normalized(a) return false;
    return true;
}

is_valid :: inline (a: $T/.[Vector2, Vector3, Vector4]) -> bool {
    for a.component {
    is_nan, is_inf := is_nan_is_inf(it);
    if is_nan || is_inf return false;
    }
    return true;
}


is_normalized :: (a: Quaternion, $tolerance:float = 0.000001) -> bool {
    sum: float;
    sum += a.w*a.w;
    sum += a.x*a.x;
    sum += a.y*a.y;
    sum += a.z*a.z;

    return sum < 1+tolerance && sum > 1-tolerance; //Already tested, its badn we should use 6 or less->should I use epsilon7? 7 digits are the limits of flaot precisions, but idk if the first 0. counts
}

is_normalized :: (a: $T / .[Vector2, Vector3, Vector4], $tolerance:float = 0.000001) -> bool {
    sum: float;
    for a.component {
        sum += it*it;
    }
    return sum < 1+tolerance && sum > 1-tolerance; //Already tested, its badn we should use 6 or less->should I use epsilon7? 7 digits are the limits of flaot precisions, but idk if the first 0. counts
}

//maybe delete, idk.
// get_normal_from_normal :: (n: Vector3) -> Vector3 {
//     t1 : Vector3 = ---;
//     //trick to get a normal from a normal while avoiding degenerated cases.
//     if abs(n.x) >= 0.57735 //sqrt of 3
//         t1 = .{n.y, -n.x, 0.0};
//     else
//         t1 = .{0.0, n.z, -n.y};
//         return normalize(t1);
// }


shortest_arc :: (prenormalized_a: Vector3, prenormalized_b: Vector3) -> Quaternion {
    dq: Quaternion; //delta q
    a := cross_product(prenormalized_a, prenormalized_b);
    if length_squared(a) < 0.001 return .{};
    dq.xyz = a;
    premature_optimization := length_squared(prenormalized_a) * length_squared(prenormalized_b);
    dq.w = sqrt(premature_optimization) + dot(prenormalized_a, prenormalized_b);
    return normalize(dq);
}



cross_dot :: inline (a: Vector3, b: Vector3) -> float32 {
    return (a.y * b.z - a.z * b.y) +
           (a.z * b.x - a.x * b.z) +
           (a.x * b.y - a.y * b.x);
}

rotate_onto_plane :: (point: Vector3, plane_normal: Vector3, plane_distance: float) -> Vector3 {
    plane_origin := plane_normal * plane_distance;
    up := Vector3.{0, 1, 0};
    if abs(dot(plane_normal, up)) > 0.999 then up = Vector3.{1, 0, 0};
    basis_plane_x := normalize(cross(up, plane_normal));
    basis_plane_y := normalize(cross(plane_normal, basis_plane_x));
    offset_on_plane := point.x * basis_plane_x + point.y * basis_plane_y;
    result := plane_origin + offset_on_plane;
    return result;
}

outer :: inline (a: Vector2,  b: Vector2) -> Matrix2 {
    m: Matrix2;
    m.v = .[ b * a.x, b * a.y ];
    return m;
}

outer :: inline (a: Vector3,  b: Vector3) -> Matrix3 {
    m: Matrix3;
    m.v = .[ b * a.x, b * a.y, b * a.z ];
    return m;
}


clamp_length :: inline (v: *$T/.[Vector3], max: float) -> new_length: float
{
    len := length(v);
    if len > max {

        normalize(v);
        v.* *= max/len;
    }
    return len;
}

is_not_zero :: (a: $T / .[Vector3, Vector4]) -> bool {
    for a.component {
        if it != 0 then return true;
    }
    return false;
}



//PGS (projected gauss seidel) is iterating over each contact problem separately
//a is called schur matrix?
//PGS calculates normals, then tangents and projects solution into a cone (projects both or only tangnt?)
//https://www.diva-portal.org/smash/get/diva2:695410/FULLTEXT01.pdf
Solve :: inline (a : Matrix3, b: Vector3) -> Vector3 {
    assert_that_a_converges :: (a: Matrix3) {
        is_strictly_diagonally_dominant := true;
        c1 := abs(a._11) > abs(a._12) + abs(a._13);
        c2 := abs(a._22) > abs(a._21) + abs(a._23);
        c3 := abs(a._33) > abs(a._31) + abs(a._32);
        is_strictly_diagonally_dominant == c1 && c2 && c3;
        assert(is_strictly_diagonally_dominant);
        //we need another test too:  M: Matrix3, other test is finding eigenvalues of M and see if they are all less than 1
    }
    #if DEBUG assert_that_a_converges(a);
    // Compute LDL^T decomposition
    D1  := a.coef[0][0];
    L21 := a.coef[1][0] / a.coef[0][0];
    L31 := a.coef[2][0] / a.coef[0][0];
    D2  := a.coef[1][1] - L21 * L21 * D1;
    L32 := (a.coef[2][1] - L21 * L31 * D1) / D2;
    D3  := a.coef[2][2] - (L31 * L31 * D1 + L32 * L32 * D2);

    // Forward substitution: Solve Ly = b
    y1  := b.x;
    y2  := b.y - L21 * y1;
    y3  := b.z - L31 * y1 - L32 * y2;

    // Diagonal Solve: Solve Dz = y
    z1  := y1 / D1;
    z2  := y2 / D2;
    z3  := y3 / D3;

    // Backward substitution: Solve L^T x = z
    x: Vector3;
    x.component[2] = z3;
    x.component[1] = z2 - L32 * x.component[2];
    x.component[0] = z1 - L21 * x.component[1] - L31 * x.component[2];

    return x;
}


is_symmetric :: (a: Matrix3) -> bool{
    for i: 0..2 for j: 0..2 {
        c1 := a.coef[j][i] == a.coef[i][j];
         if !c1 return false;
    }
    return true;
}




//from koda (kuju), this is inteersting, might save a lot of sqrt operations:
// get_scale :: (mat: Matrix4) -> Vector3 {
//     column := mat.v[0];

//     if column.w == 0 {
//         return .{1, 1, 1};
//     } else {
//         scale := column.xyz;
//         scale /= column.w;
//         return scale;
//     }
// }

//mine, compatible with jai modules.
get_scale :: inline (m: Matrix4) -> Vector3 {
    scale := Vector3.{
        length(Vector3.{m._11, m._21, m._31}),
        length(Vector3.{m._12, m._22, m._32}),
        length(Vector3.{m._13, m._23, m._33}),
    };
    return scale;
}

get_translation :: inline (m: Matrix4) -> Vector3 {
    return Vector3.{m._14, m._24, m._34};
}

get_decomposed_transform :: (m: Matrix4) -> pos: Vector3, ori: Quaternion, scale: Vector3 {
    pos := get_translation(m);
    scale := get_scale(m);

    rotation_matrix := Matrix3.{
        m._11 / scale.x, m._12 / scale.y, m._13 / scale.z,
        m._21 / scale.x, m._22 / scale.y, m._23 / scale.z,
        m._31 / scale.x, m._32 / scale.y, m._33 / scale.z,
    };
    ori := get_rotation(rotation_matrix);

    return pos, ori, scale;
}

compose_transform :: (position: Vector3, orientation: Vector3, scale: Vector3) -> Matrix4 {
    return make_translation_matrix4(position)*rotation_matrix (Matrix4, orientation) * make_scale_matrix4(scale);;
}



