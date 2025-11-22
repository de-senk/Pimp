MESH_RENDER_COMMAND :: struct {
    kind: enum {SKINNED;};
    skinned_mesh: *Skinned_Mesh;
    world_model: Matrix4;
    skinning_matrices : []Matrix4;
}

Vertex :: struct
{
	position  : Vector3; _:float32; //16
	normal    : Vector3; _:float32; //16
	tangent   : Vector3 = Vector3.{1,0,0}; tangentw: float32 = 1.0; //16
	texcoord_0: Vector2;
    colorScale: u32;
    material_index: u32; // 16
	weights  : Vector4; // _:float32; //16

	joints   : [4]s16; //16 //why using 16? breaks compatibility with windows 8.

}


VertexOutput :: struct
{
	position  : Vector3; _:float32;
	normal    : Vector3; _:float32;
	tangent   : Vector3 = Vector3.{1,0,0}; tangentw: float32 = 1.0;
	texcoord_0: Vector2;
    colorScale: u32;
    material_index: u32;

}



#if !#exists(ID3D11Buffer)
ID3D11Buffer :: u64;

Skinned_Mesh :: struct
{
	#as using skeleton : Skeleton; @NoSerialize
	vertices : []Vertex; @NoSerialize
	vertices_dirty : bool; @NoSerialize
	indices  : []u32; @NoSerialize
	model_name: string;
    // world_model := Matrix4_Identity;

    _padding_render_res: bool=  ---;
    #place _padding_render_res; // OpenGL resources
	vao, vbo, ibo : GLuint; @NoSerialize
    #place _padding_render_res; // D3D11 resources
    vertex_buffer: *ID3D11Buffer; @NoSerialize
    index_buffer: *ID3D11Buffer; @NoSerialize
    vertex_count: u32; @NoSerialize
    index_count: u32; @NoSerialize
    is_already_on_gpu_mem: bool; @NoSerialize
}




#assert MAX_JOINTS <= 0x7fff "MAX_JOINTS can't exceed the max of an s16, since we use that for joint ids.";

// The skeleton is separate from the mesh so that you
// can operate on a skeleton without refering to a mesh.
Skeleton :: struct
{
	Joint :: struct
	{
		id : s16;
		// This is the matrix of the joint in the rest position,
		// relative to the parent's coordinate system
		local_bind_transform : Matrix4;
		// This is the inverse of the matrix of the joint in model space.
		// Why we have this matrix is explained in anim.jai:generate_skinning_matrices
		inverse_bind_transform : Matrix4;
		parent : *Joint;
		name:string; //maybe delete
	}

	name_to_joint_id : Table (string, s16);
	joints : [] Joint;	// Root joint is joints[0]
}
