
#scope_file
#import "Math";
#import "SIMD";
#import "SIMD_Math";
#import "Basic";

#insert #run maybe_insert_DEBUG();
maybe_insert_DEBUG :: () -> string
{
    #import "Basic";
    #import "Compiler";
    current_w := get_current_workspace();
    options := get_build_options(current_w);
    should_debug_be_true: bool ;
    should_debug_be_true = options.backend == .X64;
    if !should_debug_be_true {
        //try guess if we are in debug from llvm options
        should_debug_be_true = options.llvm_options.bitcode_optimization_setting == .O0;
        should_debug_be_true |= options.llvm_options.machine_code_optimization_setting == .NONE;
        should_debug_be_true |= options.llvm_options.machine_code_optimization_setting == .UNSET;
    }
    //TODO(tr): check if debug already exsts with #if #exists(DEBUG) and... the TODO is to figure out what to do.
    #if #exists(DEBUG) then return ""; //:tagEarlyReturn

    b: String_Builder;
    print_to_builder(*b, "DEBUG :: %1; //inserted by maybe_insert_DEBUG", should_debug_be_true);
    return builder_to_string(*b);
}
#scope_export

Sign :: inline (x: float) -> float {
        return ifx x < 0 then -1.0 else (ifx x > 0 then 1.0 else 0.0);
    }
    // operator * :: (q: Quaternion, v:Vector3) -> Vector3 #symmetric
    // {
    //     u := Vector3.{q.x, q.y, q.z};
    //     s := q.w;

    //     // Compute v' = 2 * dot(u, v) * u + (s^2 - dot(u, u)) * v + 2 * s * cross(u, v)
    //     dot_u_v := dot(u, v);
    //     dot_u_u := dot(u, u);
    //     cross_u_v := cross(u, v);

    //     result := 2.0 * dot_u_v * u + (s * s - dot_u_u) * v + 2.0 * s * cross_u_v;
    //     return result;
    // }

    // operator * :: (m: Matrix4, v: Vector3) -> Vector3 {

    //     result: Vector3 = ---;
    //     tmp: Vector4 = ---;
    //     tmp.xyz = v;
    //     tmp.w = 1.0;

    //     r4 := inline multiply(m, tmp);
    //     result = r4.xyz;
    //     return result;
    // }


    make_quaternion_from_axis_and_angle :: (axiss: Vector3, theta: float) -> Quaternion {
        using q: Quaternion = ---;
        axis := unit_vector(axiss);

        st := sin(theta * 0.5);
        ct := cos(theta * 0.5);

        x = axis.x * st;
        y = axis.y * st;
        z = axis.z * st;
        w = ct;
        return q;
    }


is_valid :: inline (a: $T/SIMD_Array) -> bool {
    for a.data {
        is_nan, is_inf := is_nan_is_inf(it);
        if is_nan || is_inf return false;
    }
    return true;
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

//to be deleted:
quaternion_from_axes_to_axes :: (from_x: Vector3, from_y: Vector3, from_z: Vector3,
    to_x: Vector3, to_y: Vector3, to_z: Vector3
) -> Quaternion {
    // Build rotation matrix that transforms from_basis to to_basis
    // M_to = R * M_from, so R = M_to * M_from^T

    // M_from columns
    fx := from_x; fy := from_y; fz := from_z;
    // M_to columns
    tx := to_x; ty := to_y; tz := to_z;

    // Result matrix R (column-major, maps from_basis -> to_basis)
    m: Matrix3;
    m._11 = dot(tx, fx); m._12 = dot(ty, fx); m._13 = dot(tz, fx);
    m._21 = dot(tx, fy); m._22 = dot(ty, fy); m._23 = dot(tz, fy);
    m._31 = dot(tx, fz); m._32 = dot(ty, fz); m._33 = dot(tz, fz);

    return get_rotation(m);
}


//to be deleted:
quaternion_from_basis :: (forward: Vector3, left: Vector3, up: Vector3) -> Quaternion {
    // Build rotation matrix from basis vectors (column-wise)
    m : Matrix3;
    m._11 = left.x;    m._12 = up.x;    m._13 = forward.x;
    m._21 = left.y;    m._22 = up.y;    m._23 = forward.y;
    m._31 = left.z;    m._32 = up.z;    m._33 = forward.z;

    return get_rotation(m);
}

get_normal_from_normal :: (n: Vector3) -> Vector3 {
    t1 : Vector3 = ---;
    //trick to get a normal from a normal while avoiding degenerated cases.
    if abs(n.x) >= 0.57735 //sqrt of 3
        t1 = .{n.y, -n.x, 0.0};
    else
        t1 = .{0.0, n.z, -n.y};
        return normalize(t1);
}


shortest_arc :: (prenormalized_a: Vector3, prenormalized_b: Vector3) -> Quaternion {
    dq: Quaternion; //delta q
    a := cross_product(prenormalized_a, prenormalized_b);
    if length_squared(a) < 0.001 return .{};
    dq.xyz = a;
    premature_optimization := length_squared(prenormalized_a) * length_squared(prenormalized_b);
    dq.w = sqrt(premature_optimization) + dot(prenormalized_a, prenormalized_b);
    return normalize(dq);
}


abs_biggest :: (a:  $T/SIMD_Array) -> T.type, int {
    biggest: T.type;
    index: int;
    for a.data {
        if abs(it) > biggest {
            biggest = it;
            index = it_index;
        }
    }
    return biggest, index;
}

biggest :: (a:  $T/SIMD_Array) -> T.type, int {
    biggest: T.type;
    index: int;
    for a.data {
        if it > biggest {
            biggest = it;
            index = it_index;
        }
    }
    return biggest, index;
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

is_not_zero :: (a: $T / .[Vector3, Vector4, Vector6]) -> bool {
    for a.component {
        if it != 0 then return true;
    }
    return false;
}

is_normalized :: (a: $T / .[Vector6], $tolerance:float = 0.000001) -> bool {
    sum: float;
    for a.component {
        sum += it*it;
    }
    return sum < 1+tolerance && sum > 1-tolerance; //Already tested, its badn we should use 6 or less->should I use epsilon7? 7 digits are the limits of flaot precisions, but idk if the first 0. counts
}


// outer product for 6D vectors (for constraint jacobians)
outer :: (a: Vector6, b: Vector6) -> Matrix6 {
    result: Matrix6;
    for i: 0..5 for j: 0..5 {
        result.coef[i][j] = a.component[i] * b.component[j];
    }
    return result;
}

force_positive :: inline (a: Matrix3) -> Matrix3 {
    m: Matrix3 = ---;
    m.v[0] = abs(a.v[0]);
    m.v[1] = abs(a.v[1]);
    m.v[2] = abs(a.v[2]);
    return m;
}

force_positive :: inline (a: Matrix6) -> Matrix6 {
    m: Matrix6 = ---;
    m.v[0] = abs(a.v[0]);
    m.v[1] = abs(a.v[1]);
    m.v[2] = abs(a.v[2]);
    m.v[3] = abs(a.v[3]);
    m.v[4] = abs(a.v[4]);
    m.v[5] = abs(a.v[5]);
    return m;
}

force_positive :: inline (a: Vector6) -> Vector6 {
    m: Vector6 = ---;
    m.component[0] = abs(a.component[0]);
    m.component[1] = abs(a.component[1]);
    m.component[2] = abs(a.component[2]);
    m.component[3] = abs(a.component[3]);
    m.component[4] = abs(a.component[4]);
    m.component[5] = abs(a.component[5]);
    return m;
}


axis_abs_trace :: inline (a: Quaternion) -> float {

    return abs(a.x) + abs(a.y) + abs(a.z);
}

trace :: inline (a: Vector6) -> float32 {

    return a.component[0] +
    a.component[1] +
    a.component[2] +
    a.component[3] +
    a.component[4] +
    a.component[5];
}

trace :: inline (a: Matrix6) -> float32 {
    return a._11 + a._22 + a._33 + a._44 + a._55 + a._66;
}

Abs :: inline (a: Matrix2) -> Matrix2 {
    m: Matrix2 = ---;
    m.v[0] = abs(a.v[0]);
    m.v[1] = abs(a.v[1]);
    return m;
}

Column :: (m: Matrix2, i: int) -> Vector2 {
    return .{ m.coef[0][i], m.coef[1][i] };
}

Column :: (m: Matrix3, i: int) -> Vector3 {
    return .{ m.coef[0][i], m.coef[1][i], m.coef[2][i] };
}

Column :: (m: Matrix6, i: int) -> Vector6 {
    return .{ m.coef[0][i], m.coef[1][i], m.coef[2][i], m.coef[3][i] , m.coef[4][i] , m.coef[5][i]  };
}

make_diagonal_matrix :: inline (m00: float, m11: float, m22: float) -> Matrix3 {
    return .{
        m00, 0, 0,
        0, m11, 0,
        0, 0, m22
    };
}

make_diagonal_matrix_linear_angular :: inline (m00: float, m11: float, m22: float, m33: float, m44: float, m55: float) -> (linear: Matrix3, angular: Matrix3) {
    return .{
        m00, 0  , 0  ,
        0  , m11, 0  ,
        0  , 0  , m22
    }, .{
     m33, 0  ,  0 ,
     0  , m44,  0 ,
     0  , 0  , m55
     };
}

make_diagonal_matrix :: inline (m00: float, m11: float, m22: float, m33: float, m44: float, m55: float) -> Matrix6 {
    return .{
        m00, 0  , 0  , 0  , 0  ,  0 ,
        0  , m11, 0  , 0  , 0  ,  0 ,
        0  , 0  , m22, 0  , 0  ,  0 ,
        0  , 0  , 0  , m33, 0  ,  0 ,
        0  , 0  , 0  , 0  , m44,  0 ,
        0  , 0  , 0  , 0  , 0  , m55
    };
}

length :: (a: Vector6) -> float32 {
    return sqrt(a.x*a.x + a.y*a.y + a.z*a.z + a.w*a.w+ a.u*a.u+ a.v*a.v);
}


// is_valid :: inline (a: $T/SIMD_Array) -> bool {
//     for a.data {
//         is_nan, is_inf := is_nan_is_inf(it);
//         if is_nan || is_inf return false;
//     }
//     return true;
// }

// is_valid :: inline (a: Quaternion) -> bool {
//     for a.component {
//         is_nan, is_inf := is_nan_is_inf(it);
//         if is_nan || is_inf return false;
//     }
//     if !is_normalized(a) return false;
//     return true;
// }

// is_valid :: inline (a: $T/.[Vector2, Vector3,Vector4]) -> bool {
//     for a.component {
//     is_nan, is_inf := is_nan_is_inf(it);
//     if is_nan || is_inf return false;
//     }
//     return true;
// }

// is_valid :: inline (a: float32) -> bool {
//     is_nan, is_inf := is_nan_is_inf(a);
//     return !is_nan;
// }

// is_valid :: inline (a: float64) -> bool {
//     is_nan, is_inf := is_nan_is_inf(a);
//     return !is_nan;
// }

diagonals_are_zero :: (m: $T/Matrix_RC) -> bool {
    #assert((T.ROW + T.COL) % 2 == 0);
    for row: 0 ..T.ROW - 1
    {
        is_diagonal_zero := m.v[row].data[row]*m.v[row].data[row] == 0;
        // assert(is_diagonal_zero ,"m has diagonals with 0 \n m:%", m);
        if is_diagonal_zero return false;
    }
    return true;
}

converges :: (m: $T/Matrix_RC, $strict: bool=false) -> bool{
    #assert((T.ROW + T.COL) % 2 == 0);
    is_strictly_diagonally_dominant := true; //stupid name, it should be , are abs of same row elements lower and not equal than diagonal element? well the name makes sense now.

    for row: 0 ..T.ROW - 1
    {
        current_row_diag_elem := abs(m.v[row].data[row]);
        same_row_sum: T.type;
        for col: 0..T.COL-1 if row != col then same_row_sum += abs(m.v[row].data[col]);
        #if strict
        is_strictly_diagonally_dominant = current_row_diag_elem > same_row_sum;
        else
        is_strictly_diagonally_dominant = current_row_diag_elem >= same_row_sum;
        // assert(is_strictly_diagonally_dominant ,"m is not strictly diagonally dominant\n m:%", m);
         if !is_strictly_diagonally_dominant return false;
    }
    return true;
    //we need another test too:  M: Matrix3, other test is finding eigenvalues of M and see if they are all less than 1
}

// //@Check I never remember the convention, I use this mnemmotech-> from far I see a matrix, I cannot see it from top because is too far, but from the side it loos like it has R rows, so I wrtite Matrix_R when I get closer to the matrix I see the top and can count them, then I write Matrix_RC  a matrix  of R rows and C columns
// LCP_Gauss_Seidel :: (A: $T/Matrix_RC, b: T.Row_Type, $iterations: int = 1) -> T.Row_Type
// // #modify {
// //         return (T.ROW + T.COL) % 2 == 0; //maybe replace with bitwise, Since I dont use bitwise everyday I forget, is it (A.ROW | A.COL) & 1 == 0 ?
// //     }
//     {
//     #assert((T.ROW + T.COL) % 2 == 0);
//     // #assert(T.ROW == T.COL);
//     // if diagonals_are_zero(A) then return .{};
//     #if DEBUG converges(A);
//     //for convergence: //to Implement and delete these 3 lines of comments, repalce with asserts.
//     //  a must be symmetric
//     //  a must be positive definite || diagonally dominant
//     result: T.Row_Type; //init to 0 is key here.
//     for iter: 0..iterations/2 - 1
//     {
//         for row: 0 ..T.ROW - 1
//         {
//             dx := (b.data[row] - dot(A.v[row], result)) / A.v[row].data[row];
//             if is_valid(dx)
//             {
//                 result.data[row] += dx;
//                 // log("row %", (b.data[row] - dot(A.v[row], result)));
//                 // log("asd %", A);
//             }
//         }
//     }
//     return result;
// }

LCP_Gauss_Seidel :: (A: Matrix_RC, b: Vector3) -> Vector3 {
    //for convergence:
    //  a must be symmetric
    //  a must be positive definite || diagonally dominant
    result: Vector3;

    for iter: 0..2
    {
        for i: 0 ..2
        {
            dx := (b.component[i] - dot(A.v[i], result)) / A.v[i].component[i];
            if is_valid(dx)
            {
                result.component[i] += dx;
            }
        }
    }
    return result;
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

is_symmetric :: (a: Matrix6) -> bool{
    for i: 0..5 for j: 0..5 {
        c1 := a.coef[j][i] == a.coef[i][j];
         if !c1 return false;
    }
    return true;
}

//begin, from kodaphysics


compose_transform :: (local_position: Vector3, local_orientation: Vector3, local_scale: Vector3) -> Matrix4 {
    transform: Matrix4;
    // translation := Vector3.{matrix._14, matrix._24, matrix._34};

    // scale := Vector3.{
    //     length(Vector3.{matrix._11, matrix._21, matrix._31}),
    //     length(Vector3.{matrix._12, matrix._22, matrix._32}),
    //     length(Vector3.{matrix._13, matrix._23, matrix._33}),
    // };

    // rotation_matrix := Matrix3.{
    //     matrix._11 / scale.x, matrix._12 / scale.y, matrix._13 / scale.z,
    //     matrix._21 / scale.x, matrix._22 / scale.y, matrix._23 / scale.z,
    //     matrix._31 / scale.x, matrix._32 / scale.y, matrix._33 / scale.z,
    // };
    // rotation := get_rotation(rotation_matrix);
    return transform;
}

//from koda (kuju)
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


// end, from kodaphysics


//from rubikon, Counter Strike 2
// RN_FORCEINLINE rnPlane operator*( const rnTransform& Transform, const rnPlane& Plane )
// {
//     rnVector3 Normal = Transform.Rotation * Plane.Normal;  // Transform.Rotation is a 3x3 matrix
//     return rnPlane( Normal, Plane.Offset + rnDot( Normal, Transform.Translation ) );  // The Plane constructor takes a normal and offset
// }

// NOTE: I should say this before I forget, this way of trasnformig a plane doesnt support scaling (Im not 100% sure, but my assert triggered once)
// transform_plane2 :: (plane: Plane, transform: Matrix4) -> Plane {
//     Normal := transform_direction(plane.normal, transform);
//     Translation := get_translation(transform);
//     Distance := plane.distance + dot(Translation, Normal);
//     world_plane := Plane.{Normal, Distance};
//     previous_way := transform_plane(plane, transform);
//     assert(world_plane.normal == previous_way.normal, "planes normls are different l:% r:%", world_plane.normal , previous_way.normal);
//     // assert(abs(world_plane.distance - previous_way.distance) < EPSILON5, "planes distances are different % %", world_plane.distance , previous_way.distance);
//     return previous_way;

// }


//This, I hate this, please dont do this. dot product definition is sacred, dont overload it with extraneous behavior.
// Point :: #type,distinct Vector3;
// dot :: (plane: Plane3, p: Point) -> Vector3 {
//     return dot(p, plane.normal) + plane.distance;
// }





// 6x6 LDL^T solver (overloaded)
Solve ::  (a: Matrix6, b: Vector6) -> Vector6 {
    // LDL^T decomposition for 6x6 symmetric positive definite matrix
    L: Matrix6;
    D: [6] float32;

    assert_that_a_converges :: (a: Matrix6) {
        is_strictly_diagonally_dominant := true;
        // |a_11| > |a_12| + |a_13| + |a_14| + |a_15| + |a_16|
        // |a_22| > |a_21| + |a_23| + |a_24| + |a_25| + |a_26|
        // |a_33| > |a_31| + |a_33| + |a_34| + |a_35| + |a_36|
        // |a_44| > |a_42| + |a_43| + |a_44| + |a_45| + |a_46|
        // |a_55| > |a_51| + |a_53| + |a_54| + |a_55| + |a_56|
        // |a_66| > |a_61| + |a_63| + |a_64| + |a_65| + |a_66|
        c1 := abs(a._11) > abs(a._12) + abs(a._13) + abs(a._14) + abs(a._15) + abs(a._16);
        c2 := abs(a._22) > abs(a._21) + abs(a._23) + abs(a._24) + abs(a._25) + abs(a._26);
        c3 := abs(a._33) > abs(a._31) + abs(a._32) + abs(a._34) + abs(a._35) + abs(a._36);
        c4 := abs(a._44) > abs(a._41) + abs(a._42) + abs(a._43) + abs(a._45) + abs(a._46);
        c5 := abs(a._55) > abs(a._51) + abs(a._52) + abs(a._53) + abs(a._54) + abs(a._56);
        c6 := abs(a._66) > abs(a._61) + abs(a._62) + abs(a._63) + abs(a._64) + abs(a._65);

        is_strictly_diagonally_dominant == c1 && c2 && c3 && c4 && c5 && c6;
        assert(is_strictly_diagonally_dominant, "a is not strictly_diagonally_dominant! cant use LDL^T");
        //we need another test too:  M: Matrix3, other test is finding eigenvalues of M and see if they are all less than 1
    }
    #if DEBUG assert_that_a_converges(a);

    #if DEBUG assert(is_symmetric(a));

    // Initialize L as identity
    for i: 0..5 for j: 0..5 {
        L.coef[i][j] = ifx i == j then 1.0 else 0.0;
    }

    // LDL^T decomposition
    for i: 0..5 {
        // Compute diagonal element D[i]
        sum := 0.0;
        for k: 0..i-1 {
            sum += L.coef[i][k] * L.coef[i][k] * D[k];
        }
        D[i] = a.coef[i][i] - sum;


        // Compute column i of L
        for j: i+1..5 {
            sum = 0.0;
            for k: 0..i-1 {
                sum += L.coef[j][k] * L.coef[i][k] * D[k];
            }
            L.coef[j][i] = (a.coef[j][i] - sum) / D[i];
        }
    }

    // Check if D is long enough (this is not a joke)
    tr := trace(abs(Vector6.{component = D}));
    #if DEBUG assert(tr > 0.0001, tprint("D is too close to 0 or 0! %", D));
    // Forward substitution: Solve Ly = b
    y: Vector6;
    for i: 0..5 {
        sum := 0.0;
        for j: 0..i-1 {
            sum += L.coef[i][j] * y.component[j];
        }
        y.component[i] = b.component[i] - sum;
    }

    // Diagonal solve: Solve Dz = y
    z: Vector6;
    for i: 0..5 {
        z.component[i] = y.component[i] / D[i];
    }

    // Backward substitution: Solve L^T x = z
    x: Vector6;
    for < i: 0..5 {
        // assert(false);
        sum := 0.0;
        for j: i+1..5 {
            sum += L.coef[j][i] * x.component[j];
        }
        x.component[i] = z.component[i] - sum;
    }

    return x;
}

Symmetrize :: (m: Matrix6) -> Matrix6 {
    result: Matrix6;
    for i: 0..5 {
        for j: 0..5 {
            result.coef[i][j] = 0.5 * (m.coef[i][j] + m.coef[j][i]);
        }
    }
    return result;
}

// 6x6 matrix for full rigid body dynamics (3 translation + 3 rotation)
Matrix6 :: struct {
    _11, _12, _13, _14, _15, _16 : float;
    _21, _22, _23, _24, _25, _26 : float;
    _31, _32, _33, _34, _35, _36 : float;
    _41, _42, _43, _44, _45, _46 : float;
    _51, _52, _53, _54, _55, _56 : float;
    _61, _62, _63, _64, _65, _66 : float;
    #overlay(_11) v:      [6]    Vector6; // These are row vectors.
    #overlay(_11) coef:   [6][6] float;
    #overlay(_11) floats: [36]   float;

}
block_assign :: (a: *Matrix6, b: Matrix3, $cuadrant: int = 4)
{
    #assert(cuadrant>0 && cuadrant < 5);
    #if cuadrant == 4
    for i: 3..5
    for j: 3..5
    a.coef[i][j] = b.coef[i-3][j-3];

    #if cuadrant ==1
    for i: 0..2
    for j: 0..2
    a.coef[i][j] = b.coef[i][j];
}

compute_joint_angular_hessian_block :: (world_axis: Vector3, lever_arm: Vector3) -> (Matrix3)
{

    cross_mat :: (v: Vector3) -> Matrix3
    {
        return Matrix3.{
              0.0, -v.z,  v.y ,
              v.z,  0.0, -v.x ,
             -v.y,  v.x,  0.0 };
    }

    Xa := cross_mat(world_axis);
    Xr := cross_mat(lever_arm);
    H_ang := -1* (Xa * Xr);
    return H_ang;

}



Bivector :: struct {
    xy, yz, zx: float;
     #overlay( xy ) component: [3] float;
}

Rotor :: struct {
    s: float32;     // scalar part (cos(θ/2))
    b: Bivector;    // bivector part (sin(θ/2) * axis bivector)
}

Vector6 :: struct {

    x, y, z, w, u, v: float; //should be v, u but, in my mind, u comes first, then v, and w is for quaternions, so... whatever.

// #place x;
//     xy: Vector2 = ---;
// #place y;
//     yz: Vector2 = ---;
// #place z;
//     zw: Vector2 = ---;
#overlay( x) linear: Vector3;
#overlay(w) angular: Vector3;

#overlay(x) component: [6] float;
}

// Helper constructors
make_Vector6 :: (linear: Vector3, angular: Vector3) -> Vector6 {
    result: Vector6 = ---;
    result.linear = linear;
    result.angular = angular;
    return result;
}

to_Vector3_linear :: (v: Vector6) -> Vector3 {
    return Vector3.{v.component[0], v.component[1], v.component[2]};
}

to_Vector3_angular :: (v: Vector6) -> Vector3 {
    return Vector3.{v.component[3], v.component[4], v.component[5]};
}

// Block diagonal matrix constructor for rigid body mass matrix
make_Matrix6 :: (M_linear: Matrix3, M_angular: Matrix3) -> Matrix6 {
    result: Matrix6;

    // Zero initialize
    for i: 0..5 for j: 0..5 {
        result.coef[i][j] = 0.0;
    }

    // Top-left 3x3 block (linear mass matrix)
    for i: 0..2 for j: 0..2 {
        result.coef[i][j] = M_linear.coef[i][j];
    }

    // Bottom-right 3x3 block (angular mass matrix)
    for i: 0..2 for j: 0..2 {
        result.coef[i+3][j+3] = M_angular.coef[i][j];
    }

    return result;
}




// Alternative: Simpler approach using separate 3x3 solvers
// This can be more efficient if your constraints don't couple translation and rotation strongly
// SolveDecoupled :: (body: *RigidBody, linear_lhs: Matrix3, linear_rhs: Vector3,
//                    angular_lhs: Matrix3, angular_rhs: Vector3, dt: float32) {
//     // Solve linear motion
//     linear_delta := Solve(linear_lhs, linear_rhs);
//     body.position -= linear_delta;

//     // Solve angular motion
//     angular_delta := Solve(angular_lhs, angular_rhs);
//     if length(angular_delta) > 0 {
//         angle := length(angular_delta);
//         axis := angular_delta / angle;
//         dq := quaternion_from_axis_angle(axis, angle * 0.5);
//         body.orientation = quaternion_multiply(body.orientation, quaternion_conjugate(dq));
//         body.orientation = normalize(body.orientation);
//     }
// }



// Operator overloads for Matrix6

// operator * :: (a: Matrix6, b: float32) -> Matrix6 #symmetric  {
//     c := cast(float32)b;
//     for * a.component {
//         it.* = (it.*) * b;
//     }
// }


operator * :: (a: Matrix6, s: float32) -> Matrix6 {
    result: Matrix6;
    for i: 0..5 for j: 0..5 {
        result.coef[i][j] = a.coef[i][j] * s;
    }
    return result;
}

// operator + :: (a: Matrix6, b: Matrix6) -> Matrix6  {
//     for * a.component {
//         it.* = (it.*) + b.component[it_index];
//     }
// }


operator + :: (a: Matrix6, b: Matrix6) -> Matrix6 {
    result: Matrix6;
    for i: 0..5 for j: 0..5 {
        result.coef[i][j] = a.coef[i][j] + b.coef[i][j];
    }
    return result;
}


operator * :: (a: Matrix6, v: Vector6) -> Vector6 {
    result: Vector6;
    for i: 0..5 {
        sum := 0.0;
        for j: 0..5 {
            sum += a.coef[i][j] * v.component[j];
        }
        result.component[i] = sum;
    }
    return result;
}

operator * :: inline (a: Vector6, b: float32) -> Vector6 #symmetric  {
   result: Vector6 = ---;
   for a.component {
        result.component[it_index] =  a.component[it_index];
        result.component[it_index] *=  b;
    }
    return result;
}

operator * :: (a: Vector6, b: float64) -> Vector6 #symmetric  {
    #if DEBUG assert(abs(b) < FLOAT32_MAX);
    c := cast(float32)b;
    return a*c;
}

operator + :: (a: Vector6, b: Vector6) -> Vector6  {
    result: Vector6 = ---;
    for a.component {
        result.component[it_index] +=  a.component[it_index];
        result.component[it_index] +=  b.component[it_index];
    }
    return result;
}



