g_mesh_shader : GLuint;
g_soft_mesh_shader : GLuint;
g_line_shader : GLuint;
g_line_vao, g_line_vbo : GLuint;
g_big_data_ubo : GLuint;

GL_BIG_DATA_UNIFORM_BINDING :: 0;

// Used for line rendering
Vertex_Pos_Color :: struct
{
	position : Vector3;
	color : Vector4;
}

#scope_file

// In OpenGL land, we write programs in the GLSL language to
// (usually) be executed on the GPU.
// The 'vertex shader' is the first shader to be executed. It
// takes as input the data for a vertex, which we define, and
// it is responsible for transforming that vertex in any way it
// wants (such as applying the model transform, the camera projection).
// The 'fragment shader' is the last shader to be executed. It
// executes per-pixel, and takes as input anything that you want,
// and as output it gives the color of the pixel.
create_shader_program :: (vertex_source : string, fragment_source : string) -> GLuint, bool
{
	check_compile_errors :: (shader : GLuint) -> bool, info_log : string
	{
		success : s32;
		glGetShaderiv (shader, GL_COMPILE_STATUS, *success);
		if !success
		{
			info_len : s32;
			glGetShaderiv (shader, GL_INFO_LOG_LENGTH, *info_len);
			info_data := cast (*u8) alloc (info_len,, temp);
			glGetShaderInfoLog (shader, xx info_len, xx *info_len, info_data);
			result : string;
			result.data = info_data;
			result.count = info_len;

			return false, result;
		}

		return true, "";
	}

	check_link_errors :: (program : GLuint) -> bool, info_log : string
	{
		success : s32;
		glGetProgramiv (program, GL_LINK_STATUS, *success);
		if !success
		{
			info_len : s32;
			glGetProgramiv (program, GL_INFO_LOG_LENGTH, *info_len);
			info_data := cast (*u8) alloc (info_len,, temp);
			glGetProgramInfoLog (program, xx info_len, xx *info_len, info_data);
			result : string;
			result.data = info_data;
			result.count = info_len;

			return false, result;
		}

		return true, "";
	}

	// OpenGL requires us to create at least a vertex shader and a
	// fragment shader, to create a shader program that it can execute.
	vs := glCreateShader (GL_VERTEX_SHADER);
	defer glDeleteShader (vs);

    shaders := *u8.[   SHADER_PREFIX.data ,    vertex_source.data ];
    lengths := s32.[xx SHADER_PREFIX.count, xx vertex_source.count];
	glShaderSource (vs, 2, shaders.data, lengths.data);
	glCompileShader (vs);
	ok, info_log := check_compile_errors (vs);
	if !ok {
		log_error("Could not compile vertex shader %", info_log);
		return 0, false;
	}

	fs := glCreateShader (GL_FRAGMENT_SHADER);
	defer glDeleteShader (fs);
	shaders[1] =    fragment_source.data;
	lengths[1] = xx fragment_source.count;
	glShaderSource (fs, 2, shaders.data, lengths.data);
	glCompileShader (fs);
	ok, info_log = check_compile_errors (fs);
	if !ok {
		log_error ("Could not compile fragment shader: %", info_log);
		return 0, false;
	}

	result := glCreateProgram ();
	glAttachShader (result, vs);
	glAttachShader (result, fs);
	glLinkProgram (result);
	ok, info_log = check_link_errors (result);
	if !ok
	{
		log_error ("Could not link shader program: %", info_log);
		glDeleteProgram (result);
		return 0, false;
	}

	return result, true;
}

#scope_export

init_rendering_objects :: () -> bool
{
	// Check if we can store enough data for the skinning
	// information in a shader uniform block.
	uniform_block_size : s32;
	glGetIntegerv (GL_MAX_UNIFORM_BLOCK_SIZE, *uniform_block_size);
	assert (uniform_block_size >= MAX_JOINTS * size_of (Matrix4),
		"OpenGL implementation does not allow storing more than % bytes in uniform block, but we need at least % for skinning information. Recompile the program with a smaller MAX_JOINTS value.",
		uniform_block_size, MAX_JOINTS * size_of (Matrix4));

	// Create the shader programs we will use throughout
	// our application to render meshes and lines.
	// The source of these programs are at the end of the file.
	ok : bool;
	g_mesh_shader, ok = create_shader_program (MESH_VERTEX_SHADER, MESH_FRAGMENT_SHADER);
	if !ok return false;
	// We need to bind the block in the mesh shader to a binding point
	// that we define to be GL_BIG_DATA_UNIFORM_BINDING
	big_data_uniform_index := glGetUniformBlockIndex (g_mesh_shader, "Big_Data");
	glUniformBlockBinding (g_mesh_shader, big_data_uniform_index, GL_BIG_DATA_UNIFORM_BINDING);

	// Create the shader programs we will use throughout
	// our application to render meshes and lines.
	// The source of these programs are at the end of the file.
	{
	 ok : bool;
	 g_soft_mesh_shader, ok = create_shader_program (MESH_VERTEX_SHADER_SOFT, MESH_FRAGMENT_SHADER);
	 if !ok return false;
	 big_data_uniform_index := glGetUniformBlockIndex (g_mesh_shader, "Big_Data");
	 glUniformBlockBinding (g_mesh_shader, big_data_uniform_index, GL_BIG_DATA_UNIFORM_BINDING);
    }


	g_line_shader, ok = create_shader_program (LINE_VERTEX_SHADER, LINE_FRAGMENT_SHADER);
	if !ok
		return false;

	// Create line objects. We have a vertex buffer that we fill
	// every time we want to draw a line.
	glGenVertexArrays (1, *g_line_vao);
	glBindVertexArray (g_line_vao);
	glGenBuffers (1, *g_line_vbo);
	glBindBuffer (GL_ARRAY_BUFFER, g_line_vbo);
	glBufferData (GL_ARRAY_BUFFER, size_of (Vertex_Pos_Color) * 2, null, GL_DYNAMIC_DRAW);
	glEnableVertexAttribArray (0);
	glVertexAttribPointer (0, 3, GL_FLOAT, GL_FALSE, size_of (Vertex_Pos_Color), cast (*void) 0);
	glEnableVertexAttribArray (1);
	glVertexAttribPointer (1, 4, GL_FLOAT, GL_FALSE, size_of (Vertex_Pos_Color), cast (*void) size_of (Vector3));

	glBindVertexArray (0);
	glBindBuffer (GL_ARRAY_BUFFER, 0);


	// every time we want to draw a line.
	glGenVertexArrays (1, *g_debug_triangles_vao);
	glBindVertexArray (g_debug_triangles_vao);
	glGenBuffers (1, *g_debug_triangles_vbo);
	glBindBuffer (GL_ARRAY_BUFFER, g_debug_triangles_vbo);
	glBufferData (GL_ARRAY_BUFFER, size_of (DebugDrawVertex) * MAX_DEBUG_TRIANGLES, null, GL_DYNAMIC_DRAW);
	glEnableVertexAttribArray (0);
	glVertexAttribPointer (0, 3, GL_FLOAT, GL_FALSE, size_of (DebugDrawVertex), cast (*void) 0);
	glEnableVertexAttribArray (1);
	glVertexAttribPointer (1, 4, GL_FLOAT, GL_FALSE, size_of (DebugDrawVertex), cast (*void) size_of (Vector3));

	glBindVertexArray (0);
	glBindBuffer (GL_ARRAY_BUFFER, 0);

	// Create big data uniform buffer object
	glGenBuffers (1, *g_big_data_ubo);
	glBindBuffer (GL_UNIFORM_BUFFER, g_big_data_ubo);
	// Allocate the data for the uniform buffer. This data will
	// be filled with all the skinning matrices.
	glBufferData (GL_UNIFORM_BUFFER, size_of (Matrix4) * MAX_JOINTS, null, GL_DYNAMIC_DRAW);
	glBindBuffer (GL_UNIFORM_BUFFER, 0);
	glBindBufferBase (GL_UNIFORM_BUFFER, GL_BIG_DATA_UNIFORM_BINDING, g_big_data_ubo);

	return true;
}

init_mesh_rendering_objects :: (using mesh : *Skinned_Mesh, $dynamic := false)
{
	// OpenGL stuff... We need a vertex buffer, that holds
	// all our vertices, and an index buffer, that holds
	// all of our indices. We also need a vertex array, that
	// holds the information of the layout of our vertex buffer.
	glGenBuffers (1, *vbo);
	glBindBuffer (GL_ARRAY_BUFFER, vbo);
	defer glBindBuffer (GL_ARRAY_BUFFER, 0);
    #if dynamic
    then glBufferData (GL_ARRAY_BUFFER, size_of (Vertex) * vertices.count, vertices.data, GL_DYNAMIC_DRAW);
	else glBufferData (GL_ARRAY_BUFFER, size_of (Vertex) * vertices.count, vertices.data, GL_STATIC_DRAW);

	glGenBuffers (1, *ibo);
	glBindBuffer (GL_ELEMENT_ARRAY_BUFFER, ibo);
	defer glBindBuffer (GL_ELEMENT_ARRAY_BUFFER, 0);
	glBufferData (GL_ELEMENT_ARRAY_BUFFER, size_of (u32) * indices.count, indices.data, GL_STATIC_DRAW);

	glGenVertexArrays (1, *vao);
	glBindVertexArray (vao);
	defer glBindVertexArray (0);
	glEnableVertexAttribArray (0);
	glVertexAttribPointer (0, 3, GL_FLOAT, GL_FALSE, size_of (Vertex), cast (*void) 0);
	glEnableVertexAttribArray (1);
	glVertexAttribPointer (1, 3, GL_FLOAT, GL_FALSE, size_of (Vertex), cast (*void) size_of (Vector3));
	glEnableVertexAttribArray (2);
	glVertexAttribPointer (2, 3, GL_FLOAT, GL_FALSE, size_of (Vertex), cast (*void) (2 * size_of (Vector3)));
	glEnableVertexAttribArray (3);
	glVertexAttribIPointer (3, 4, GL_SHORT, size_of (Vertex), cast (*void) (3 * size_of (Vector3)));
}

upload_mesh_to_gpu :: (using mesh: *Skinned_Mesh, $dynamic := false) -> bool {
    if mesh.is_already_on_gpu_mem return true;
    if !mesh.vertices.count || !mesh.indices.count return false;

	glGenBuffers (1, *vbo);
	glBindBuffer (GL_ARRAY_BUFFER, vbo);
	defer glBindBuffer (GL_ARRAY_BUFFER, 0);
    #if dynamic
    then glBufferData (GL_ARRAY_BUFFER, size_of (Vertex) * vertices.count, vertices.data, GL_DYNAMIC_DRAW);
	else glBufferData (GL_ARRAY_BUFFER, size_of (Vertex) * vertices.count, vertices.data, GL_STATIC_DRAW);

	glGenBuffers (1, *ibo);
	glBindBuffer (GL_ELEMENT_ARRAY_BUFFER, ibo);
	defer glBindBuffer (GL_ELEMENT_ARRAY_BUFFER, 0);
	glBufferData (GL_ELEMENT_ARRAY_BUFFER, size_of (u32) * indices.count, indices.data, GL_STATIC_DRAW);

    // for *mesh.vertices
    // {
    //     it.texture_albedo  = 8;
    //     it.texture_normal = 9;
    //     it.texture_specular = 10;
    // }

// #run offset_of(Vertex, "position")
// #run offset_of(Vertex, "normal")
// #run offset_of(Vertex, "tangent")
// #run offset_of(Vertex, "texcoord_0")
// #run offset_of(Vertex, "colorScale")
// #run offset_of(Vertex, "material_index")
// #run offset_of(Vertex, "weights")
// #run offset_of(Vertex, "joints")


    glGenVertexArrays (1, *vao);
	glBindVertexArray (vao);
	defer glBindVertexArray (0);
	glEnableVertexAttribArray (0);
	glVertexAttribPointer (0, 3, GL_FLOAT, GL_FALSE, size_of (Vertex), cast (*void)#run offset_of(Vertex, "position"));
	glEnableVertexAttribArray (1);
	glVertexAttribPointer (1, 3, GL_FLOAT, GL_FALSE, size_of (Vertex), cast (*void)#run offset_of(Vertex, "normal"));
	glEnableVertexAttribArray (2);
	glVertexAttribPointer (2, 4, GL_FLOAT, GL_FALSE, size_of (Vertex), cast (*void)#run offset_of(Vertex, "weights"));
	glEnableVertexAttribArray (3);
	glVertexAttribIPointer (3, 4, GL_SHORT, size_of (Vertex), cast (*void)#run offset_of(Vertex, "joints"));

    mesh.vertex_count = xx mesh.vertices.count;
    mesh.index_count = xx mesh.indices.count;
    mesh.is_already_on_gpu_mem = true;

    return true;
}

/*

dx11_shader_create_skinnedshader_input_layout_for_compute :: (CS_bytecode:string) -> bool
{
    input_desc := D3D11_INPUT_ELEMENT_DESC .[
    .{ "POSITION",    0, .R32G32B32_FLOAT, 0,  #run offset_of(Vertex, "position"),  .VERTEX_DATA, 0 },
    .{ "NORMAL",      0, .R32G32B32_FLOAT, 0, #run offset_of(Vertex, "normal"), .VERTEX_DATA, 0 },
    .{ "TANGENT",     0, .R32G32B32A32_FLOAT, 0, #run offset_of(Vertex, "tangent"), .VERTEX_DATA, 0 },
    .{ "TEXCOORD",    0, .R32G32_FLOAT, 0, #run offset_of(Vertex, "texcoord_0"), .VERTEX_DATA, 0 },
    .{ "COLOR",       0, .R8G8B8A8_UINT, 0, #run offset_of(Vertex, "colorScale"), .VERTEX_DATA, 0 },
    .{ "MATERIAL_INDEX",       0, .R8G8B8A8_UINT, 0, #run offset_of(Vertex, "material_index"), .VERTEX_DATA, 0 },
    .{ "BLENDWEIGHT", 0, .R32G32B32A32_FLOAT, 0, #run offset_of(Vertex, "weights"), .VERTEX_DATA, 0 },
    .{ "BLENDINDICES", 0, .R32G32B32A32_SINT, 0, #run offset_of(Vertex, "joints"), .VERTEX_DATA, 0 },
    ];

    result := ID3D11Device_CreateInputLayout(
        d3d11_device,
        input_desc.data, input_desc.count,
        CS_bytecode.data,
        xx CS_bytecode.count,
        *d3d11_input_for_compute_layout
    );

    if result != S_OK {
        log_error("------Failed to create input layout-------");
        return false;
    }

    return true;
}
*/
//dx11
// upload_mesh_to_gpu :: (mesh: *Skinned_Mesh) -> bool {
//     if mesh.is_already_on_gpu_mem return true;
//     if !mesh.vertices.count || !mesh.indices.count return false;
//     // Create vertex buffer
//     vertex_buffer_desc: D3D11_BUFFER_DESC;
//     vertex_buffer_desc.ByteWidth = xx (mesh.vertices.count * size_of(Vertex));
//     vertex_buffer_desc.Usage = .DYNAMIC;
//     vertex_buffer_desc.CPUAccessFlags = D3D11_CPU_ACCESS_FLAG.WRITE;;
//     vertex_buffer_desc.BindFlags = xx D3D11_BIND_FLAG.VERTEX_BUFFER;

//     // for *mesh.vertices
//     // {
//     //     it.texture_albedo  = 8;
//     //     it.texture_normal = 9;
//     //     it.texture_specular = 10;
//     // }
//     vertex_buffer_data: D3D11_SUBRESOURCE_DATA;
//     vertex_buffer_data.pSysMem = mesh.vertices.data;

//     log("UPLOADING mesh 45 :\n% ",mesh.vertices[45]);
//     result := ID3D11Device_CreateBuffer(d3d11_device, *vertex_buffer_desc, *vertex_buffer_data, *mesh.vertex_buffer);
//     if result != S_OK {
//         log_error("Failed to create vertex buffer");
//         return false;
//     }

//     // Create index buffer
//     index_buffer_desc: D3D11_BUFFER_DESC;
//     index_buffer_desc.ByteWidth = xx (mesh.indices.count * size_of(u32));
//     index_buffer_desc.Usage = .IMMUTABLE;
//     index_buffer_desc.BindFlags = xx D3D11_BIND_FLAG.INDEX_BUFFER;

//     index_buffer_data: D3D11_SUBRESOURCE_DATA;
//     index_buffer_data.pSysMem = mesh.indices.data;

//     result = ID3D11Device_CreateBuffer(d3d11_device, *index_buffer_desc, *index_buffer_data, *mesh.index_buffer);
//     if result != S_OK {
//         log_error("Failed to create index buffer");
//         IUnknown_Release(mesh.vertex_buffer);
//         mesh.vertex_buffer = null;
//         return false;
//     }

//     mesh.vertex_count = xx mesh.vertices.count;
//     mesh.index_count = xx mesh.indices.count;
//     mesh.is_already_on_gpu_mem = true;

//     return true;
// }


LINE_RENDER_COMMAND :: struct {
    start: Vector3;
    end: Vector3;
    color: Vector4;
}


push_line :: (start : Vector3, end : Vector3, color : Vector4) {
    cmd: Draw_Command;
    cmd.kind = .LINE;
    cmd.should_render = true;
    cmd.line.start = start;
    cmd.line.end = end;
    cmd.line.color = color;

    array_add(*drawing_commands, cmd);
}

push_vector :: (start: Vector3, vector: Vector3, color: Vector4) {
    push_line(start, start + vector, color);
}

push_X :: (center: Vector3, size: float = 1.0, normal: Vector3 = game_coordinates.up, color: Vector4 = .{.5,.7,.5,1}) {
    horizontal := get_normal_from_normal(normal);
    vertical := cross_product(normal, horizontal);

    f := size / (1.51 * 2);
    start1 := center - f * (horizontal + vertical);
    end1 := center + f * (horizontal + vertical);
    start2 := center + f * (horizontal - vertical);
    end2 := center - f * (horizontal - vertical);

    push_line(start1, end1, color);
    push_line(start2, end2, color);
}

// Push a + marker for debugging
push_plus :: (center: Vector3, size: float = 1.0, normal: Vector3 = game_coordinates.up, color: Vector4 = .{.5,.7,.5,1}) {
    horizontal := get_normal_from_normal(normal);
    vertical := cross_product(normal, horizontal);

    half_size := size * 0.5;
    push_line(center - horizontal * half_size, center + horizontal * half_size, color);
    push_line(center - vertical * half_size, center + vertical * half_size, color);
}

// Push a circle marker
push_circle :: (center: Vector3, radius: float = 0.4, segments: int = 16, normal: Vector3 = game_coordinates.up, color: Vector4 = .{.5,.7,.5,1}) {
    horizontal := get_normal_from_normal(normal);
    vertical := cross_product(normal, horizontal);

    prev_point := center + horizontal * radius;

    for i: 1..segments {
        angle := (cast(float)i / cast(float)segments) * 2.0 * PI;
        x := cos(angle) * radius;
        y := sin(angle) * radius;

        point := center + horizontal * x + vertical * y;
        push_line(prev_point, point, color);
        prev_point = point;
    }
}

// Push a square marker
push_square :: (center: Vector3, size: float = 0.9, normal: Vector3 = game_coordinates.up, color: Vector4 = .{.5,.7,.5,1}) {
    horizontal := get_normal_from_normal(normal);
    vertical := cross_product(normal, horizontal);

    half_size := size * 0.5;

    p0 := center - horizontal * half_size - vertical * half_size;
    p1 := center + horizontal * half_size - vertical * half_size;
    p2 := center + horizontal * half_size + vertical * half_size;
    p3 := center - horizontal * half_size + vertical * half_size;

    push_line(p0, p1, color);
    push_line(p1, p2, color);
    push_line(p2, p3, color);
    push_line(p3, p0, color);
}


push_debug_polygon :: (vertices: [] Vector3, color := Vector4.{ 1, 1, 1, 1 }) {
    // dx11_set_shader_for_color(true);

    for j: 1..vertices.count - 2 {
        k := (j + 1) % vertices.count;

        v0 := vertices[0];
        v1 := vertices[j];
        v2 := vertices[k];
        push_debug_triangle(.{ v0.x, v0.y, v0.z }, .{ v1.x, v1.y, v1.z }, .{ v2.x, v2.y, v2.z }, color);
    }
}


push_debug_hull :: (position: Vector3, orientation: Quaternion, hull_shape: Convex_Hull) {
    vert_count: u32;
    vertices_tmp: [4]Vector3;
    for :each_face current_face: hull_shape.mesh {
        for :each_edge current_edge: current_face {
	        localPoint := hull_shape.scale * current_edge.tail.vertex;
    		vertices_tmp[vert_count] = localPoint;
    		vert_count += 1;
        }

        if vert_count == {
            case 4;
    		    v0 := l2w(position, orientation, vertices_tmp[0]);
                v1 := l2w(position, orientation, vertices_tmp[1]);
                v2 := l2w(position, orientation, vertices_tmp[2]);
                v3 := l2w(position, orientation, vertices_tmp[3]);
                n := current_face.face_plane_normal;
                color := Vector4.{ abs(n.x), abs(n.y), abs(n.z), 1 };
                push_debug_polygon(.[ v0, v1, v2, v3 ], color);
            case 3;
    		    v0 := l2w(position, orientation, vertices_tmp[0]);
                v1 := l2w(position, orientation, vertices_tmp[1]);
                v2 := l2w(position, orientation, vertices_tmp[2]);
                n := current_face.face_plane_normal;
                color := Vector4.{ abs(n.x), abs(n.y), abs(n.z), 1 };
                push_arrow(v0,n,color=color); // push_arrow(v1,n,color=color); // push_arrow(v2,n,color=color);
                push_debug_triangle( v0, v1, v2, color);

            case;
                assert(false, "vert_count %", vert_count);
            }
	   vert_count = 0; //to use vertices_tmp array as a ring buffer
	}
}


// Push a wireframe box
push_wireframe_box :: (center: Vector3, size: Vector3 = .{0.9,0.9,0.9}, color: Vector4 = .{.5,.7,.5,1}) {
    half := size * 0.5;

    // Bottom face
    p0 := center + .{-half.x, -half.y, -half.z};
    p1 := center + .{ half.x, -half.y, -half.z};
    p2 := center + .{ half.x, -half.y,  half.z};
    p3 := center + .{-half.x, -half.y,  half.z};

    // Top face
    p4 := center + .{-half.x, half.y, -half.z};
    p5 := center + .{ half.x, half.y, -half.z};
    p6 := center + .{ half.x, half.y,  half.z};
    p7 := center + .{-half.x, half.y,  half.z};

    // Bottom
    push_line(p0, p1, color);
    push_line(p1, p2, color);
    push_line(p2, p3, color);
    push_line(p3, p0, color);

    // Top
    push_line(p4, p5, color);
    push_line(p5, p6, color);
    push_line(p6, p7, color);
    push_line(p7, p4, color);

    // Verticals
    push_line(p0, p4, color);
    push_line(p1, p5, color);
    push_line(p2, p6, color);
    push_line(p3, p7, color);
}

// Push an arrow
push_arrow :: (start: Vector3, vector: Vector3= game_coordinates.up, head_size: float = 0.2, color: Vector4 = .{.5,.7,.5,1}) {
    end := vector+start;
    push_line(start, end, color);

    direction := normalize(end - start);
    perpendicular := get_normal_from_normal(direction);
    other_perp := cross_product(direction, perpendicular);

    head_base := end - direction * head_size;
    head_offset := head_size * 0.3;

    head1 := head_base + perpendicular * head_offset;
    head2 := head_base - perpendicular * head_offset;
    head3 := head_base + other_perp * head_offset;
    head4 := head_base - other_perp * head_offset;

    push_line(end, head1, color);
    push_line(end, head2, color);
    push_line(end, head3, color);
    push_line(end, head4, color);
}

push_grid :: (plane_origin: Vector3, plane_normal: Vector3, min_for: s64, max_for: s64 , density:float=1.0, color := Vector4.{0.4, 0.4, 0.4, 1}) {
    horizontal := get_normal_from_normal(plane_normal);
    vertical := cross_product(plane_normal, horizontal);
    for i : min_for*density.(s64)..max_for*density.(s64) {
        {
            start := plane_origin + horizontal * i.(float)/density + min_for.(float)*vertical;
            end :=  plane_origin + horizontal * i.(float)/density + max_for.(float)*vertical;
  	     push_line(start, end, color);
  	    }
        {
            start := plane_origin + vertical * i.(float)/density + min_for.(float)*horizontal;
            end :=  plane_origin + vertical * i.(float)/density + max_for.(float)*horizontal;
  	     push_line(start, end, color);
  	    }

    }
}

push_grid :: (plane_origin: Vector3, plane_normal: Vector3, area_length: s64 = 10, density:float=1.0, color := Vector4.{0.4, 0.4, 0.4, 1}) {
    push_grid(plane_origin, plane_normal,-area_length, area_length , density, color);
}

draw_line :: inline (start : Vector3, end : Vector3, color : Vector4)
{
	draw_line (start, end, color, color);
}

// This is maybe the slowest way possible to draw a line!
draw_line :: (start : Vector3, end : Vector3, start_color : Vector4, end_color : Vector4)
{
	data : [2]Vertex_Pos_Color = ---;
	data[0].position = start;
	data[0].color    = start_color;
	data[1].position = end;
	data[1].color    = end_color;

	glBindVertexArray (g_line_vao);
	glBindBuffer (GL_ARRAY_BUFFER, g_line_vbo);
	// Fill the vertex buffer with our two line vertices
	glBufferSubData (GL_ARRAY_BUFFER, 0, size_of (Vertex_Pos_Color) * 2, data.data);
	glUseProgram (g_line_shader);
	glUniformMatrix4fv (glGetUniformLocation (g_line_shader, "u_View_Projection_Matrix"), 1, GL_TRUE, *g_camera.view_projection_matrix.coef[0][0]);
	glDrawArrays (GL_LINES, 0, 2);
}


DebugDrawVertex :: struct {
    position: Vector3;
    color: Vector4;
}



g_debug_triangles_vao: GLuint;
g_debug_triangles_vbo: GLuint;
MAX_DEBUG_TRIANGLES :: 100_000;
debug_triangle_list: [..]DebugDrawVertex;
debug_triangle_list_wire: [..]DebugDrawVertex;
state_machine_wireframe:=false;
push_debug_triangle ::  (v1: Vector3, v2: Vector3, v3: Vector3, color_vec: Vector4)
{
    //check space and add triangle
    triangle_buf := *debug_triangle_list;
    if state_machine_wireframe then triangle_buf = *debug_triangle_list_wire;
    if triangle_buf.count + 3 <= MAX_DEBUG_TRIANGLES {
        array_add(triangle_buf, DebugDrawVertex.{
            position = Vector3.{v1.x, v1.y, v1.z},
            color = color_vec
        });

        array_add(triangle_buf, DebugDrawVertex.{
            position = Vector3.{v2.x, v2.y, v2.z},
            color = color_vec
        });

        array_add(triangle_buf, DebugDrawVertex.{
            position = Vector3.{v3.x, v3.y, v3.z},
            color = color_vec
        });
    }

}

draw_all_debug_triangles :: () {
    glBindVertexArray (g_debug_triangles_vao);
	glBindBuffer (GL_ARRAY_BUFFER, g_debug_triangles_vbo);
	// Fill the vertex buffer with our two line vertices
    // log("c : % % % ", debug_triangle_list.count);
	glBufferSubData (GL_ARRAY_BUFFER, 0, size_of (DebugDrawVertex) * debug_triangle_list.count.(u32), debug_triangle_list.data);
	glUseProgram (g_line_shader);
	glUniformMatrix4fv (glGetUniformLocation (g_line_shader, "u_View_Projection_Matrix"), 1, GL_TRUE, *g_camera.view_projection_matrix.coef[0][0]);
	glDrawArrays (GL_TRIANGLES, 0, debug_triangle_list.count.(u32));

}


// Following is the vertex and fragment shaders for rendering
// skinned meshes and lines.

// Separate prefixes for gles and gl shaders. These are prepended in create_shader_program
// in a similar way that simp does for it's shaders.
#if OS == .ANDROID || OS == .WASM {
	SHADER_PREFIX :: #string END
#version 300 es
precision mediump float;
END
} else {
	SHADER_PREFIX :: #string END
#version 330 core
END
}


MESH_VERTEX_SHADER :: #run sprint(#string GLSL
// These will be replaced with the values we defined in the Jai source code
const int MAX_JOINTS  = %1;
const int MAX_WEIGHTS = %2;

layout (location = 0) in vec3 a_Position;
layout (location = 1) in vec3 a_Normal;
layout (location = 2) in vec4 a_Weights;
layout (location = 3) in ivec4 a_Joint_Ids;

uniform mat4 u_Model_Matrix;
uniform mat4 u_View_Projection_Matrix;
// A simple uniform variable cannot hold a lot of data,
// so we use a uniform block to store the skinning information.
// A uniform block can hold more data, and we check if the
// OpenGL implementation allows us to store enough at the
// start of the program.
layout (std140) uniform Big_Data
{
	mat4 u_Skinning_Matrices[MAX_JOINTS];
};

out vec3 Normal;

void main ()
{
	vec3 model_position = vec3 (0);
	vec3 model_normal = vec3 (0);
	if (a_Joint_Ids[0] == -1)
	{
		model_position = a_Position;
		model_normal   = a_Normal;
	}
	for (int i = 0; i < MAX_WEIGHTS && a_Joint_Ids[i] != -1; i += 1)
	{
		int joint_id = a_Joint_Ids[i];
		float weight;
		if (i == MAX_WEIGHTS - 1)
			weight = 1.0 - (a_Weights.x + a_Weights.y + a_Weights.z);
		else
			weight = a_Weights[i];
		mat4 skinning_matrix = u_Skinning_Matrices[joint_id];
		vec3 pose_position = (skinning_matrix * vec4 (a_Position, 1)).xyz;
		model_position += pose_position * weight;
		vec3 pose_normal = (skinning_matrix * vec4 (a_Normal, 0)).xyz;
		model_normal += pose_normal * weight;
	}
	gl_Position = u_View_Projection_Matrix * u_Model_Matrix * vec4 (model_position, 1);
	Normal = model_normal.xyz;
}
GLSL, MAX_JOINTS, MAX_WEIGHTS);

MESH_FRAGMENT_SHADER :: #string GLSL

in vec3 Normal;

out vec4 Frag_Color;

void main ()
{
	Frag_Color = vec4 (Normal, 1);
}
GLSL

MESH_VERTEX_SHADER_SOFT :: #run sprint(#string END
// These will be replaced with the values we defined in the Jai source code
const int MAX_JOINTS  = %1;
const int MAX_WEIGHTS = %2;

layout (location = 0) in vec3 a_Position;
layout (location = 1) in vec3 a_Normal;
layout (location = 2) in vec3 a_Weights;
layout (location = 3) in ivec4 a_Joint_Ids;

uniform mat4 u_Model_Matrix;
uniform mat4 u_View_Projection_Matrix;
// A simple uniform variable cannot hold a lot of data,
// so we use a uniform block to store the skinning information.
// A uniform block can hold more data, and we check if the
// OpenGL implementation allows us to store enough at the
// start of the program.
layout (std140) uniform Big_Data
{
	mat4 u_Skinning_Matrices[MAX_JOINTS];
};

out vec3 Normal;

void main ()
{
	vec3 model_position = vec3 (0);
	vec3 model_normal = vec3 (0);
	model_position = a_Position;
	model_normal   = a_Normal;

	gl_Position = u_View_Projection_Matrix * u_Model_Matrix * vec4 (model_position, 1);
	Normal = model_normal.xyz;
}
END, MAX_JOINTS, MAX_WEIGHTS);

MESH_FRAGMENT_SHADER_SOFT :: #string GLSL

in vec3 Normal;

out vec4 Frag_Color;

void main ()
{
	Frag_Color = vec4 (Normal, 1);
}
GLSL

LINE_VERTEX_SHADER :: #string GLSL

layout (location = 0) in vec3 a_Position;
layout (location = 1) in vec4 a_Color;

uniform mat4 u_View_Projection_Matrix;

out vec4 Color;

void main ()
{
	gl_Position = u_View_Projection_Matrix * vec4 (a_Position, 1);
	Color = a_Color;
}
GLSL

LINE_FRAGMENT_SHADER :: #string GLSL

in vec4 Color;

out vec4 Frag_Color;

void main ()
{
	Frag_Color = Color;
}
GLSL
