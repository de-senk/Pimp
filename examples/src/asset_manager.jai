


rotate_vector_x_90 :: (v: Vector3, direction: int) -> Vector3 {
	result: Vector3;
	result.x = v.x;
	if direction == 1 {
		// +90°: Y → -Z, Z → Y
		result.y = -v.z;
		result.z = v.y;
	} else {
		// -90°: Y → Z, Z → -Y
		result.y = v.z;
		result.z = -v.y;
	}
	return result;
}

PARSE_SKINNED_MESH_V3 :: #string JAI
CURRENT_CUSTOM_MESH_VERSION :: 3;
parse_joint :: (handler : *Text_File_Handler, mesh : *Skinned_Mesh, joint : *Skeleton.Joint) -> bool
	{
		name, ok := consume_next_line (handler);
		if !ok
			return false;
		joint.id = cast (s16) (joint - mesh.joints.data);
		table_add (*mesh.name_to_joint_id, copy_string (name), joint.id);
		joint.name=copy_string (name);
		joint.local_bind_transform, ok = parse_matrix4 (handler);
		// Parse parent id
		parent_id : s16;
		parent_id, ok = parse_int (handler, s16);
		if !ok
			return false;
		// Set the inverse bind transform to the model space transform first.
		// We will invert the matrices after all the joints have been parsed.
		if parent_id >= 0
		{
			if parent_id >= mesh.joints.count
  			    return error_out (handler, "Invalid parent joint id.");
            parent := *mesh.joints[parent_id];
			joint.parent = parent;
			joint.inverse_bind_transform = parent.inverse_bind_transform * joint.local_bind_transform;
		} else {
			joint.parent = null;
			joint.inverse_bind_transform = joint.local_bind_transform;
		}

		return true;
	}

	parse_skinned_vertex :: (handler : *Text_File_Handler, mesh : *Skinned_Mesh, vertex : *Vertex) -> bool
	{
		SHOULD_MIRROR_X :: false;
		SHOULD_MIRROR_Y :: false;
		SHOULD_MIRROR_Z :: false;

		SHOULD_ROTATE_X_90_DEGREES :: false;
		ROTATION_DIRECTION :: 1; // +1 for +90°, -1 for -90°

		ok : bool;
		vertice_id:u32;
		vertice_id, ok = parse_int (handler, u32);
			assert(ok);//if !ok then return false;
		vertex.position, ok = parse_vector3 (handler);
			assert(ok);//if !ok then return false;
		vertex.normal, ok = parse_vector3 (handler);
			assert(ok);//if !ok then return false;
		vertex.tangent, ok = parse_vector3 (handler);
			assert(ok);//if !ok then return false;
		vertex.tangentw, ok = parse_float (handler);
			assert(ok);//if !ok then return false;
		vertex.texcoord_0, ok = parse_vector2 (handler);
			assert(ok);//if !ok then return false;
		vertex.colorScale, ok = parse_int (handler, u32);
			assert(ok);//if !ok then return false;
		vertex.material_index, ok = parse_int (handler, u32);
			assert(ok);//if !ok then return false;
		vertex.weights, ok = parse_vector4 (handler);
			assert(ok);//if !ok then return false;
		vertex.joints, ok = parse_int_tuple (handler, 4, s16);
			assert(ok);//if !ok then return false;

		#if SHOULD_MIRROR_X {
			vertex.position.x *= -1;
			vertex.normal.x *= -1;
			vertex.tangent.x *= -1;
			vertex.tangentw *= -1;
		}
		#if SHOULD_MIRROR_Y {
			vertex.position.y *= -1;
			vertex.normal.y *= -1;
			vertex.tangent.y *= -1;
			vertex.tangentw *= -1;
		}
		#if SHOULD_MIRROR_Z {
			vertex.position.z *= -1;
			vertex.normal.z *= -1;
			vertex.tangent.z *= -1;
			vertex.tangentw *= -1;
		}

		#if SHOULD_ROTATE_X_90_DEGREES {
			vertex.position = rotate_vector_x_90(vertex.position, ROTATION_DIRECTION);
			vertex.normal   = rotate_vector_x_90(vertex.normal, ROTATION_DIRECTION);
			vertex.tangent  = rotate_vector_x_90(vertex.tangent, ROTATION_DIRECTION);

			// vertex.position = rotate_vector_x_90(vertex.position, ROTATION_DIRECTION);
			// vertex.normal   = rotate_vector_x_90(vertex.normal, ROTATION_DIRECTION);
			// vertex.tangent  = rotate_vector_x_90(vertex.tangent, ROTATION_DIRECTION);
		}
		// log("%",vertex);
		return true;
	}

	parse_triangle :: (handler : *Text_File_Handler, mesh : *Skinned_Mesh, index : s64) -> bool
	{
		tri, ok := parse_int_tuple (handler, 3, u32);
		if !ok
			return false;
		mesh.indices[index + 0] = tri[0];
		mesh.indices[index + 1] = tri[1];
		mesh.indices[index + 2] = tri[2];
		return true;
	}

JAI;

load_custom_skinned_mesh_from_file :: (mesh : *Skinned_Mesh, filename : string) -> bool {
    #insert PARSE_SKINNED_MESH_V3;

	handler : Text_File_Handler;
	defer deinit (*handler);

	found_slash, dir, basename := split_from_right (filename, #char "/");
	if !found_slash then basename = filename;

    start_file (*handler, basename, filename, optional = false);

	if handler.failed
		return false;

	if handler.version != CURRENT_CUSTOM_MESH_VERSION
		return error_out (handler, "Invalid version number.");

	ok : bool;
	mesh.joints.count, ok = parse_ident_and_int_line (*handler, "joint_count");
	if !ok
		return false;
	// assert(mesh.joints.count>0);
	if mesh.joints.count > MAX_JOINTS
		return error_out (handler, "Too many joints (expected at most %, got %).", MAX_JOINTS, mesh.joints.count);

	mesh.vertices.count, ok = parse_ident_and_int_line (*handler, "vertex_count");
	if !ok
		return false;

	triangle_count : s64;
	triangle_count, ok = parse_ident_and_int_line (*handler, "triangle_count");
	if !ok
		return false;
	mesh.indices.count = triangle_count * 3;

	// Allocate all the data in one batch, so everything is contiguous
	all_data := alloc (size_of (Skeleton.Joint) * mesh.joints.count
		+ size_of (Vertex) * mesh.vertices.count
		+ size_of (u32) * mesh.indices.count
	);

	init (*mesh.name_to_joint_id, mesh.joints.count);
	mesh.joints.data   = all_data;
	mesh.vertices.data = cast (*Vertex) (mesh.joints.data + mesh.joints.count);
	mesh.indices.data  = cast (*u32) (mesh.vertices.data + mesh.vertices.count);

	// joints
	line, found := consume_next_line (*handler);
	if line != "joints:"
		return error_out (handler, "Expected 'joints:', got '%' instead.", line);
	for * mesh.joints
		if !parse_joint (*handler, mesh, it)
			return error_out (handler, "Could not parse joint %.", it_index);
	// Inverse all the bind transforms we calculated while parsing the joints
	for * mesh.joints
		_, it.inverse_bind_transform = inverse(it.inverse_bind_transform);
	//

	// vertices
	line, found = consume_next_line (*handler);
	if line != "vertices:"
		return error_out (handler, "Expected 'vertices:', got '%' instead.", line);
	for * mesh.vertices
		if !parse_skinned_vertex (*handler, mesh, it)
			return error_out (handler, "Could not parse vertex %.", it_index);
    //

	// triangles
	line, found = consume_next_line (*handler);
	if line != "triangles:"
		return error_out (handler, "Expected 'triangles:', got '%' instead.", line);
	for i : 0..triangle_count - 1
		if !parse_triangle (*handler, mesh, i * 3)
			return error_out (handler, "Could not parse triangle %.", i);
    //

	line, found = consume_next_line (*handler);
	if found
		return error_out (handler, "Expected EOF.");

	return true;
}

