// For now I will try to put on this file draws that are built on top of draw primitives
// I think that the render commands strategy leads to less complicated code than not using it.
// I'm yet to find a real practical weakness of the render commands queue strategy.

//:tagHighestLevelOfAbstractionExplanationAndCode

skinning_commands: [..]ToSkin_Command;
ToSkin_Command :: struct {
        pose: *Skeleton_Pose;
}

push_skinning_change :: (pose: *Skeleton_Pose)
{
    cmd: ToSkin_Command;
    cmd.pose = pose;
    array_add(*skinning_commands, cmd);
}

drawing_commands: [..]Draw_Command;
Draw_Command :: struct {
    should_render: bool; //maybe nonsense because it is IMMEDIATE MODE, BUT, maybe some frustrum culling sets this.
    kind: enum{TRIANGLE_MESH;DEBUG_SPHERE;SKELETON;UI;LINE;};
    union { //I dont like that I have these structs sparsed around files.
        ui: UI_RENDER_COMMAND;
        mesh: MESH_RENDER_COMMAND;
        skeleton: SKELETON_RENDER_COMMAND;
        line: LINE_RENDER_COMMAND;
        // debug_sphere: DEBUG_SPHERE_RENDER_COMMAND;
    }
}

//for usage in quicksort
compare_render_commands :: (a: type_of(Draw_Command.kind), b: type_of(Draw_Command.kind)) -> s64 {
    //NOTE: UI IS LAST!, matbe create a separate UI3D for text on 3d world that can be discarded by x-buffer, oh I always forget that this is automatic in modern gpus!
    //TODO: Sort triangle meshes by materials.
    if a > b return -1;
    return 0;
}

actual_draw :: (using cmd: Draw_Command) {
    if !should_render return;
    if kind == {
        case .SKELETON; draw_pose(skeleton.pose, skeleton.world_model, skeleton.color);
        case .TRIANGLE_MESH; draw_mesh(mesh.skinned_mesh, mesh.world_model, mesh.skinning_matrices);
        // case .DEBUG_SPHERE; draw_sphere(mesh.pose, mesh.world_model);
        case .UI; draw_ui_element(ui);
        case .LINE; draw_line(line.start, line.end, line.color);
    }
}


draw_mesh :: (using mesh : Skinned_Mesh, model_matrix : Matrix4, skinning_matrices : []Matrix4)
{

	glDepthMask (GL_TRUE); //:tagMaybeCache
	glEnable (GL_DEPTH_TEST);
	assert (skinning_matrices.count <= MAX_JOINTS, "Skinning matrices array cannot have more than % elements for now.");

	// Fill the uniform buffer with the skinning data
	glBindBuffer (GL_UNIFORM_BUFFER, g_big_data_ubo);
	glBufferSubData (GL_UNIFORM_BUFFER, 0, size_of (Matrix4) * skinning_matrices.count, skinning_matrices.data);
	glBindBuffer (GL_UNIFORM_BUFFER, 0);

	glBindVertexArray (vao);
	glBindBuffer (GL_ARRAY_BUFFER, vbo);
	glBindBuffer (GL_ELEMENT_ARRAY_BUFFER, ibo);

	shader_to_use := g_mesh_shader;
	if mesh.joints.count == 0 then {
	   shader_to_use = g_soft_mesh_shader;
	   glBufferSubData (GL_ARRAY_BUFFER, 0, size_of (Vertex) * vertices.count, vertices.data);
	}
	glUseProgram (shader_to_use);
	glUniformMatrix4fv (glGetUniformLocation (shader_to_use, "u_Model_Matrix"), 1, GL_TRUE, *model_matrix.coef[0][0]);
	glUniformMatrix4fv (glGetUniformLocation (shader_to_use, "u_View_Projection_Matrix"), 1, GL_TRUE, *g_camera.view_projection_matrix.coef[0][0]);

	glDrawElements (GL_TRIANGLES, cast (u32) indices.count, GL_UNSIGNED_INT, null);
}


draw_ui_element :: (using uircmd : UI_RENDER_COMMAND) {
    glDisable (GL_DEPTH_TEST); //:tagMaybeCache
	glDepthMask (GL_FALSE);
	simp.set_shader_for_color ();

    getrect.label (r, text, *theme);
}

draw_pose :: (using pose : Skeleton_Pose, transform : Matrix4, color : Vector4)
{

    // Disable depth mask and testing so we can see
    // the skeleton through other things
    glDepthMask (GL_FALSE); //:tagMaybeCache ? but opengl already does caching internally.
	glDisable (GL_DEPTH_TEST);

	for * joints
	{
		parent_transform : Matrix4 = ---;
        parent := it.parent;
		if parent
		    parent_transform = transform * parent.model_space_transform;
		else
			parent_transform = transform;

		world_space_transform := transform * it.model_space_transform;
		start := xyz (
			parent_transform._14,
			parent_transform._24,
			parent_transform._34
		);
		end := xyz (
			world_space_transform._14,
			world_space_transform._24,
			world_space_transform._34
		);
		draw_line (start, end, color);
	}
}



#if 0 {
NUM_VERTEX_OF_SPHERE :: 3072;
#no_reset g_sphere_vertices_lod0: [NUM_VERTEX_OF_SPHERE] DebugDrawVertex;
#no_reset g_sphere_vertices_lod1: [9*9*2-9] Vector3;
#no_reset g_sphere_vertices_lod2: [5*5*2-5] Vector3;


fill_sphere_vertices :: ()  {
    modify_color :: () #expand
    {
            vv := `v00.position*10.01;
            `modifiable_color.x = vv.x;
            `modifiable_color.y = vv.y;
            `modifiable_color.z = vv.z;
    };


    generate_sphere_vertices :: (vertices: []DebugDrawVertex, subdivisions: int = 16)
    {
        lat_segments := subdivisions;
        lon_segments := subdivisions * 2;
        ccountter := 0;
        for lat: 0..lat_segments
        {
            phi := PI * cast(float)lat / cast(float)lat_segments;
            for lon: 0..lon_segments
            {
                theta := TAU * cast(float)lon / cast(float)lon_segments;

                x := sin(phi) * cos(theta);
                y := cos(phi);
                z := sin(phi) * sin(theta);

                // vertex := position + Vector3.{x, y, z} * radius;
                // vertex := l2w(position, orientation, Vector3.{x, y, z} * radius);
                vertex :=  Vector3.{x, y, z};
                vertices[ccountter].position = vertex;
                vertices[ccountter].color = .{vertex.x,vertex.y,vertex.z,1};
                ccountter+=1;
            }
        }
        log("%",ccountter);
    }

    generate_sphere_triangles :: (vertices: []DebugDrawVertex, lat_segments: int, lon_segments: int) {
        count := 0;

        modifiable_color := Vector4.{1.0,1.0,1.0,1};
        for lat: 0..lat_segments-1 {
            phi0 := PI * lat / lat_segments;
            phi1 := PI * (lat+1) / lat_segments;

            for lon: 0..lon_segments-1 {
                theta0 := TAU * lon / lon_segments;
                theta1 := TAU * (lon+1) / lon_segments;

                // four corner positions on the unit sphere
                v00 := Vector3.{ sin(phi0)*cos(theta0), cos(phi0), sin(phi0)*sin(theta0) };
                v01 := Vector3.{ sin(phi0)*cos(theta1), cos(phi0), sin(phi0)*sin(theta1) };
                v10 := Vector3.{ sin(phi1)*cos(theta0), cos(phi1), sin(phi1)*sin(theta0) };
                v11 := Vector3.{ sin(phi1)*cos(theta1), cos(phi1), sin(phi1)*sin(theta1) };

                // two triangles: (v00,v10,v01) and (v01,v10,v11)
                modifiable_color.x = (v00.x + 1)/2;
                modifiable_color.y = (v00.y + 1)/2;
                modifiable_color.z = (v00.z + 1)/2;
                // log("%",v00.y + 1);
                vertices[count].position = v00; vertices[count].color = modifiable_color; count+=1;
                vertices[count].position = v10; vertices[count].color = modifiable_color; count+=1;
                vertices[count].position = v01; vertices[count].color = modifiable_color; count+=1;

                vertices[count].position = v01; vertices[count].color = modifiable_color; count+=1;
                vertices[count].position = v10; vertices[count].color = modifiable_color; count+=1;
                vertices[count].position = v11; vertices[count].color = modifiable_color; count+=1;
            }
        }
        log("count %", count); // actual vertex count to draw
    }


     // generate_sphere_vertices(g_sphere_vertices_lod0, 16 );
     generate_sphere_triangles(g_sphere_vertices_lod0, 16 ,32);
     // generate_sphere_vertices(g_sphere_vertices_lod1, 16/2 );
     // generate_sphere_vertices(g_sphere_vertices_lod2, 16/4 );


}
#run fill_sphere_vertices();

DrawSphere :: (position: Vector3, orientation: Quaternion, radius: float, subdivisions := 16, color := Vector4.{ 1, 1, 1, 1 }, mass : float32 = 1.0)  {
    transform := Matrix4_Identity;
    transform = translate(transform, position);
    transform = rotate(transform, orientation);
    transform = scale(transform, .{radius,radius,radius});

    jdraw_push_sphere(.{transform, color });
}


DrawCircle :: (pos: Vector3, r: float, color := Vector4.{ 1, 1, 1, 1 }) {
    POINTS_PER_CIRCLE :: 10;
    THETA :: TAU / POINTS_PER_CIRCLE;
    {
        vertices : [POINTS_PER_CIRCLE] Vector3;
        for i: 0..POINTS_PER_CIRCLE - 1 {
            vertices[i] = pos + .{ cos(THETA * i), sin(THETA * i) , 0} * r;
        }
        push_debug_polygon(vertices, color);
    }
    {
        vertices : [POINTS_PER_CIRCLE] Vector3;
        for i: 0..POINTS_PER_CIRCLE - 1 {
            vertices[i] = pos + .{0,  cos(THETA * i), sin(THETA * i) } * r;
        }
        push_debug_polygon(vertices, color);
    }
    {
        vertices : [POINTS_PER_CIRCLE] Vector3;
        for i: 0..POINTS_PER_CIRCLE - 1 {
            vertices[i] = pos + .{ cos(THETA * i), 0, sin(THETA * i) } * r;
        }
        push_debug_polygon(vertices, color);
    }
}


biggest_mass_so_far: float= 0.1;
Draw :: (using body: *Body) {

   if biggest_mass_so_far < body.mass then biggest_mass_so_far = body.mass;
    draw_as_capsule :: (using body: *Body) { //draw_capsule for grep
        if g_jolt_renderer.enable_wireframe
        then g_jolt_renderer.current_state = .WIREFRAME;
        defer g_jolt_renderer.current_state = .NORMAL;

        capsule_height := size.capsule_height;
        capsule_center :=/*this is not center its base tip!*/ l2w(body.position, body.orientation, .{0,-size.capsule_height/2,0});
        capsule_radius := size.capsule_radius;

        // if sparse_if(2) //transparency
        DrawCapsule (capsule_center,orientation, capsule_radius , capsule_height, subdivisions = 16, color = Vector4.{ 1, 1, 1, 1 });

    }
   draw_as_sphere :: (using body: *Body) {//draw_sphere for grep
        if g_jolt_renderer.enable_wireframe
        then g_jolt_renderer.current_state = .WIREFRAME;
        defer g_jolt_renderer.current_state = .NORMAL;
        // if sparse_if(2) //transparency
        // DrawCircle(position, length(size));
        DrawSphere(position, orientation, size.sphere_radius, mass = mass);
        // DrawWireframeSphere(position, length(size));

    }
   draw_as_plane :: (using body: *Body) { //draw_plane for grep
        R3D := rotation_matrix(Matrix3, orientation);
        density_alpha_blend := clamp(body.mass / biggest_mass_so_far, 0.99, 1.0);

        center := Vector3.{ position.x, position.y, position.z };
        {
            plane_normal := R3D * size *0.5;

            plane_normal.x = clamp(plane_normal.x, 0.10, 0.20);
            plane_normal.y = clamp(plane_normal.y, 0.10, 0.20);
            plane_normal.z = clamp(plane_normal.z, 0.10, 0.20);


            v0 := R3D * Vector3.{-100, -100 ,  0} + center;
            v1 := R3D * Vector3.{-100,  100 ,  0} + center;
            v3 := R3D * Vector3.{ 100, -100 ,  0} + center;
            v2 := R3D * Vector3.{ 100,  100 ,  0} + center;
            push_debug_polygon(.[ v0, v1, v2, v3 ], Vector4.{ plane_normal.x, plane_normal.y, plane_normal.z, density_alpha_blend });
            // debug_break();
        }
    }
    draw_as_hull_shape  :: (using body: *Body) {
        if g_jolt_renderer.enable_wireframe
        then g_jolt_renderer.current_state = .WIREFRAME;
        defer g_jolt_renderer.current_state = .NORMAL;

        maxidx := shape.local_points.count -1;
        for tri: shape.trianglemesh
        {
            // reached_max_idx := tri.a > maxidx ||tri.b > maxidx || tri.c > maxidx;
            // if reached_max_idx {
            //     log("maxidx %, abc % % %",maxidx,tri.a,tri.b,tri.c, );
            //     log("maxidx %, abc % % %",maxidx,tri.a,tri.b,tri.c, );
            //     log("maxidx %, abc % % %",maxidx,tri.a,tri.b,tri.c, );
            // }
            //     log("shape.local_point %, abc % % %",maxidx,tri.a,tri.b,tri.c, );
                local_v0 := shape.local_points[tri.a];
                local_v1 := shape.local_points[tri.b];
                local_v2 := shape.local_points[tri.c];
                // log("   aa 4 % % %",local_v0, local_v1, local_v2);
        		v0 := l2w(body.position, body.orientation, local_v0);
                v1 := l2w(body.position, body.orientation, local_v1);
                v2 := l2w(body.position, body.orientation, local_v2);

                /* AI CODE HERE, I ASKED AI TO ADD COLOR TO MY CODE */
                    // compute face normal
                    n := 0.4*normalize(cross(local_v1 - local_v0,
                                         local_v2 - local_v0));
                    // simple color mapping from normal to RGB
                    color := Vector4.{ abs(n.x), abs(n.y), abs(n.z), 1 };
                /* AI CODE ENDS HERE  AI CODE ENDS HERE AI CODE ENDS HERE*/

                push_debug_polygon(.[ v0, v1, v2 ], color);
        }
    }

    draw_as_hull  :: (using body: *Body) {
        if g_jolt_renderer.enable_wireframe
        then g_jolt_renderer.current_state = .WIREFRAME;
        defer g_jolt_renderer.current_state = .NORMAL;
        return;
	    currentIndex: u32;
	    local_vertex: [4]Vector3;

            for :each_face current_face: hull_shape.mesh {
    	    for :each_edge current_edge: current_face {
        		localPoint := hull_shape.scale * current_edge.tail.vertex;
        		local_vertex[currentIndex] = localPoint;
        		currentIndex += 1;
        		should_draw_face := currentIndex == 4;
        		if should_draw_face
        		{

            		v0 := l2w(body.position, body.orientation, local_vertex[0]);
                    v1 := l2w(body.position, body.orientation, local_vertex[1]);
                    v2 := l2w(body.position, body.orientation, local_vertex[2]);
                    v3 := l2w(body.position, body.orientation, local_vertex[3]);

                    /* AI CODE HERE, I ASKED AI TO ADD COLOR TO MY CODE */
                        // compute face normal
                        n := 0.4*normalize(cross(local_vertex[1] - local_vertex[0],
                                             local_vertex[2] - local_vertex[0]));
                        // simple color mapping from normal to RGB
                        color := Vector4.{ abs(n.x), abs(n.y), abs(n.z), 1 };
                    /* AI CODE ENDS HERE  AI CODE ENDS HERE AI CODE ENDS HERE*/

                    push_debug_polygon(.[ v0, v1, v2, v3 ], color);


                    currentIndex = 0; //to use  local_vertex array as a ring buffer
                }
            }
    	}
    }
    // draw_as_hull  :: (using body: *Body) {
    //     if g_jolt_renderer.enable_wireframe
    //     then g_jolt_renderer.current_state = .WIREFRAME;
    //     defer g_jolt_renderer.current_state = .NORMAL;

    //     currentIndex: u32;
    //     local_vertex: [4]Vector3;
    //     start_face := hull_shape.mesh.faces;
    //     current_face := start_face;
    //     while true { defer if current_face == start_face break;
    //     	start_edge := current_face.edge;
    //         current_edge := start_edge;
    // 	    while true { defer if current_edge == start_edge then break;
    //     		localPoint := hull_shape.scale * current_edge.tail.vertex;
    //     		local_vertex[currentIndex] = localPoint;
    //     		currentIndex += 1;
    //     		should_draw_face := currentIndex == 4;
    //     		if should_draw_face
    //     		{

    //         		v0 := l2w(body.position, body.orientation, local_vertex[0]);
    //                 v1 := l2w(body.position, body.orientation, local_vertex[1]);
    //                 v2 := l2w(body.position, body.orientation, local_vertex[2]);
    //                 v3 := l2w(body.position, body.orientation, local_vertex[3]);

    //                 /* AI CODE HERE, I ASKED AI TO ADD COLOR TO MY CODE */
    //                     // compute face normal
    //                     n := 0.4*normalize(cross(local_vertex[1] - local_vertex[0],
    //                                          local_vertex[2] - local_vertex[0]));
    //                     // simple color mapping from normal to RGB
    //                     color := Vector4.{ abs(n.x), abs(n.y), abs(n.z), 1 };
    //                 /* AI CODE ENDS HERE  AI CODE ENDS HERE AI CODE ENDS HERE*/

    //                 push_debug_polygon(.[ v0, v1, v2, v3 ], color);


    //                 currentIndex = 0; //to use  local_vertex array as a ring buffer
    //             }
    //             //while loop edge related:
    //             current_edge = current_edge.next;
    //         }
    //         //while loop face related:
    //         current_face = current_face.next;
    // 	}
    // }

    draw_as_cube :: (using body: *Body) {
        R3D := rotation_matrix(Matrix3, orientation);
        density_alpha_blend := clamp(body.mass / biggest_mass_so_far, 0.1, 0.5);
        rotate_around_fake :: (v0: *Vector3, v1: *Vector3, v2: *Vector3, v3: *Vector3) {
            SHOULD_ROTATE_VISUALLY :: false;

            qv0 := Vector4.{v0.x, v0.y, v0.z, 1 };
            qv1 := Vector4.{v1.x, v1.y, v1.z, 1 };
            qv2 := Vector4.{v2.x, v2.y, v2.z, 1 };
            qv3 := Vector4.{v3.x, v3.y, v3.z, 1 };

            qp0 := Vector4.{v0.x, v0.y, v0.z, 1 };
            qp1 := Vector4.{v1.x, v1.y, v1.z, 1 };
            qp2 := Vector4.{v2.x, v2.y, v2.z, 1 };
            qp3 := Vector4.{v3.x, v3.y, v3.z, 1 };
            rquat :Quaternion;
            varang :float32= cast(float32)g_timer.current/4;//sin(cast(float32)g_timer.current);
            // set_from_axis_and_angle(*rquat, .{0,-.5,1},-3.141592/2);
            // qp0.xyz = rotate( qp0.xyz, rquat);
            // qp1.xyz = rotate( qp1.xyz, rquat);
            // qp2.xyz = rotate( qp2.xyz, rquat);
            // qp3.xyz = rotate( qp3.xyz, rquat);

            #if SHOULD_ROTATE_VISUALLY set_from_axis_and_angle(*rquat, .{0,1,0},varang);
            qp0.xyz = rotate( qp0.xyz, rquat);
            qp1.xyz = rotate( qp1.xyz, rquat);
            qp2.xyz = rotate( qp2.xyz, rquat);
            qp3.xyz = rotate( qp3.xyz, rquat);




            v0.* = qp0.xyz;
            v1.* = qp1.xyz;
            v2.* = qp2.xyz;
            v3.* = qp3.xyz;

        }
    // 3D
    {
        R :=  R3D; //its a matrix4!
        center := Vector3.{ position.x, position.y, position.z };
          //front face
        {
            v0 := R * Vector3.{ -size.x , -size.y  , -size.z } + center;
            v1 := R * Vector3.{  size.x , -size.y  , -size.z } + center;
            v2 := R * Vector3.{  size.x ,  size.y  , -size.z } + center;
            v3 := R * Vector3.{ -size.x ,  size.y  , -size.z } + center;
            rotate_around_fake(*v0, *v1, *v2, *v3);
            push_debug_polygon(.[ v0, v1, v2, v3 ], .{ .4, 0, 0, density_alpha_blend });
            // debug_break();
        }

        //back face
        {
            v0 := R * Vector3.{ -size.x , -size.y  , size.z } + center;
            v1 := R * Vector3.{  size.x , -size.y  , size.z } + center;
            v2 := R * Vector3.{  size.x ,  size.y  , size.z } + center;
            v3 := R * Vector3.{ -size.x ,  size.y  , size.z } + center;
            rotate_around_fake(*v0, *v1, *v2, *v3);
            push_debug_polygon(.[v0, v1, v2, v3 ], .{ .4, 1, 0, density_alpha_blend });
        }

         //top 1 face
        {
            v0 := R * Vector3.{ -size.x , -size.y  , -size.z } + center;
            v1 := R * Vector3.{  size.x , -size.y  , -size.z } + center;
            v2 := R * Vector3.{  size.x ,  -size.y  , size.z } + center;
            v3 := R * Vector3.{ -size.x ,  -size.y  , size.z } + center;
            rotate_around_fake(*v0, *v1, *v2, *v3);
            push_debug_polygon(.[v0, v1, v2, v3 ], .{ .6, .2, 0, density_alpha_blend });
        }

         //bottom face
       {
            v0 := R * Vector3.{ -size.x , size.y  , -size.z } + center;
            v1 := R * Vector3.{  size.x , size.y  , -size.z } + center;
            v2 := R * Vector3.{  size.x , size.y  , size.z } + center;
            v3 := R * Vector3.{ -size.x , size.y  , size.z } + center;
            rotate_around_fake(*v0, *v1, *v2, *v3);
            push_debug_polygon(.[v0, v1, v2, v3 ], .{ .4, .5, 0, density_alpha_blend });
        }

         //side 1 face
        {
            v0 := R * Vector3.{ -size.x , size.y  , -size.z } + center;
            v1 := R * Vector3.{ -size.x , -size.y  , -size.z } + center;
            v2 := R * Vector3.{ -size.x , -size.y  , size.z } + center;
            v3 := R * Vector3.{ -size.x , size.y  , size.z } + center;
            rotate_around_fake(*v0, *v1, *v2, *v3);
            push_debug_polygon(.[v0, v1, v2, v3 ], .{ .1, .1, .1, density_alpha_blend });
        }

         //side 2 face
       {
            v0 := R * Vector3.{ size.x , size.y  , -size.z } + center;
            v1 := R * Vector3.{ size.x , -size.y  , -size.z } + center;
            v2 := R * Vector3.{ size.x , -size.y  , size.z } + center;
            v3 := R * Vector3.{ size.x , size.y  , size.z } + center;
            rotate_around_fake(*v0, *v1, *v2, *v3);
            push_debug_polygon(.[v0, v1, v2, v3 ], .{ .1, .1, 0.5, density_alpha_blend });
        }
    }

    }

    if g_should_draw && shape_kind == .CUBE draw_as_cube(body);
    if g_should_draw && shape_kind == .HULL draw_as_hull(body);
    if !g_should_draw && shape_kind == .HULL draw_as_hull_shape(body);
    if shape_kind == .PLANE draw_as_plane(body);
    if shape_kind == .SPHERE draw_as_sphere(body);
    if g_should_draw && shape_kind == .CAPSULE draw_as_capsule(body);

}



dx11_prepare_and_draw_text ::  (font: *Dynamic_Font, x: s64, y: s64, text: string, color := Vector4.{1,1,1,1}, effects: MyFont_Effects = 0) -> width: s64 {
    width := prepare_text(font, text, effects);
    dx11_draw_prepared_text(font,  x,  y,  color);
    return width;
}

dx11_draw_prepared_text :: (font: *Dynamic_Font, x: s64, y: s64, color := Vector4.{1, 1, 1, 1}, effects: MyFont_Effects = 0) {
    generate_quads_for_prepared_text(font, x, y, effects);
    dx11_draw_generated_quads(font, color);
}


dx11_draw_generated_quads :: (font: *Dynamic_Font, color := Vector4.{1, 1, 1, 1}) {
    quads := font.current_quads;

	dx11_set_shader (.UI_TEXT);

    draw_letter_quad :: (font: *Dynamic_Font, quad: Font_Quad, color: Vector4) {
        page := quad.glyph.page;
        if page.bitmap_data.width  <= 1 return;
        if page.bitmap_data.height <= 1 return;
        using quad;

        uv0 := Vector2.{u0, v0};
        uv1 := Vector2.{u1, v0};
        uv2 := Vector2.{u1, v1};
        uv3 := Vector2.{u0, v1};

        dx11_immediate_quad_for_text(p0, p1, p2, p3, color, uv0, uv1, uv2, uv3);
    }


    last_texture: *ID3D11ShaderResourceView = null;
    for quad, i: quads {
        page := quad.glyph.page;
        map  := *page.texture;

        if page.dirty {
            page.dirty = false;
            assert(fill_mTexture_from_Bitmap(map, page.bitmap_data));
            queue_glyph_add_texture(map); //so we upload only the latest one, this might destroy different font styles, yet to be verified.
        }

        if map.srv != last_texture {
            // dx11_immediate_flush();
            handle := map.srv;
            last_texture = handle;
            ID3D11DeviceContext_PSSetShaderResources(d3d11_device_context, 2, 1, *handle);
        }


        context.imm_ctx.current_texture_cell_index = map.cell_index;
        draw_letter_quad(font, quad, color);
    }
}



//dont look at me.
try_compile_debug_3d_shader :: () -> (success:bool)
{
//:tagCode_Starts_Here_hashdasjdah

DEBUG_3D_SHADER :: #string HLSL
      // cbuffer ConstantBuffer : register(b0)
// {
//     matrix ViewProjection;
// };

cbuffer CameraData : register(b0) {
    matrix u_View_Matrix;
    matrix u_Projection_Matrix;
    float3 u_Camera_Position;
};

struct VSInputLayout
{
    float3 Position : POSITION;
    float4 Color : COLOR;
};

struct PSInputLayout
{
    float4 Position : SV_POSITION;
    float4 Color : COLOR0;
};

PSInputLayout VSMain(VSInputLayout input)
{
    PSInputLayout output;
    output.Position = mul(float4(input.Position, 1.0), mul(u_View_Matrix, u_Projection_Matrix));
    // output.Position = mul(float4(input.Position, 1.0), ViewProjection);
    output.Color = input.Color;
    return output;
}

float4 PSMain(PSInputLayout input) : SV_TARGET
{
    return input.Color;
}
HLSL;

DEBUG_3D_SHADER_FOR_INSTANCING :: #string HLSL
cbuffer CameraData : register(b0) {
    matrix u_View_Matrix;
    matrix u_Projection_Matrix;
    float3 u_Camera_Position;
};


struct Sphere_Instancing_Data
{
    float4x4 Model;
    float4 ColorScale;
};

StructuredBuffer<Sphere_Instancing_Data> u_Instance_Model : register(t0);

struct VSInputLayout
{
    float3 Position : POSITION;
    float4 Color : COLOR;
};

struct PSInputLayout
{
    float4 Position : SV_POSITION;
    float4 Color : COLOR0;
};

PSInputLayout VSMain(VSInputLayout input, uint instanceID : SV_InstanceID)
{
    PSInputLayout output;
    float4x4 instanceModel = u_Instance_Model[instanceID].Model;
    float4 colorScale = u_Instance_Model[instanceID].ColorScale;

    float3 world_pos = mul(float4(input.Position, 1.0f), instanceModel);
    output.Position = mul(float4(world_pos, 1.0), mul(u_View_Matrix, u_Projection_Matrix));

    output.Color.x = colorScale.x*input.Color.x;
    output.Color.y = colorScale.y*input.Color.y;
    output.Color.z = colorScale.z*input.Color.z;
    output.Color.w = colorScale.w*input.Color.w;

    return output;
}

float4 PSMain(PSInputLayout input) : SV_TARGET
{
    return input.Color;
}
HLSL;

//:tagCode_Starts_Here_hashdasjdah
    {
      success_compiling_shader: bool;
      shader_text := DEBUG_3D_SHADER;
      vs_bytecode: string;
      ps_bytecode: string;

      ppVS: **ID3D11VertexShader = *g_jolt_renderer.vertex_shader;
      ppPS: **ID3D11PixelShader = *g_jolt_renderer.pixel_shader;




      vs_bytecode, success_compiling_shader = dx11_compile_vertex_shader(shader_text, ppVS);
      if !success_compiling_shader then {log("[ERROR] Failed to compile vs_bytecode");return false;}
      defer free(vs_bytecode);

      ps_bytecode, success_compiling_shader = dx11_compile_pixel_shader(shader_text, ppPS);
      if !success_compiling_shader then {log("[ERROR] Failed to compile ps_bytecode");return false;}
      defer free(ps_bytecode);

      //create input_layout, I think this was only needed once, whatever.
      success_compiling_shader = dx11_shader_create_jolt_debug_input_layout(vs_bytecode);

      if !success_compiling_shader then { log("[ERROR] failed at creating input layout"); return false; }
      log("Success compiling shader \n");
  }

  #if SPHERE_INSTANCING_ENABLED
  {
        success_compiling_shader: bool;
      shader_text := DEBUG_3D_SHADER_FOR_INSTANCING;
      vs_bytecode: string;
      ps_bytecode: string;

      ppVS: **ID3D11VertexShader = *g_jolt_renderer.vertex_shader_for_instancing;
      ppPS: **ID3D11PixelShader = *g_jolt_renderer.pixel_shader_for_instancing;




      vs_bytecode, success_compiling_shader = dx11_compile_vertex_shader(shader_text, ppVS);
      if !success_compiling_shader then {log("[ERROR] Failed to compile vs_bytecode");return false;}
      defer free(vs_bytecode);

      ps_bytecode, success_compiling_shader = dx11_compile_pixel_shader(shader_text, ppPS);
      if !success_compiling_shader then {log("[ERROR] Failed to compile ps_bytecode");return false;}
      defer free(ps_bytecode);

      //create input_layout, I think this was only needed once, whatever.
      success_compiling_shader = dx11_shader_create_jolt_debug_input_layout(vs_bytecode);

      if !success_compiling_shader then { log("[ERROR] failed at creating input layout"); return false; }
      log("Success compiling shader \n");

  }
  return true;
}
} //#if 0


//
// I use uppercase to determine the Highest level of abstraction.
//
//:tagHighestLevelOfAbstractionExplanationAndCode
push_draw :: (using pose : *Skeleton_Pose, world_model : Matrix4 = Matrix4_Identity, color : Vector4 = Vector4.{0,1,0,1}) {
    cmd: Draw_Command;
    cmd.kind = .SKELETON;
    cmd.should_render = g_params.show_skeleton;
    cmd.skeleton.world_model = world_model;
    cmd.skeleton.pose = pose;
    cmd.skeleton.color = color;
    array_add(*drawing_commands, cmd);
}

push_draw :: (using mesh : *Skinned_Mesh, world_model : Matrix4 = Matrix4_Identity, skinning_matrices : []Matrix4) {
    cmd: Draw_Command;
    cmd.kind = .TRIANGLE_MESH;
    cmd.should_render = g_params.show_mesh;
    cmd.mesh.kind = .SKINNED;
    cmd.mesh.world_model = world_model;
    cmd.mesh.skinned_mesh = mesh;
    cmd.mesh.skinning_matrices = skinning_matrices;
    array_add(*drawing_commands, cmd);
}


Draw :: (using entity: *Entity, delta_time : float) {

    if rendering_flags & .Has_Skin_Mesh != 0 {

        if skin_mesh == *stub_skin_mesh {
            log("% %",asset_guid, skin_mesh);
            skin_mesh = table_find_pointer(*RES.skin_meshes, asset_guid);
            log("% %",asset_guid, skin_mesh);
		    free_pose (*skin_pose);
		    if skin_mesh
		        skin_pose = make_pose_for_skeleton (skin_mesh);

        }

        if skin_mesh.joints.count != 0
        {
        // wpos := get_world_position(*g_pose, "DEF-spine.004");
        // spos := WorldToScreen(world_model*wpos);
        // if g_params.show_mesh doLabel("GLADIATOR", spos);
        }



	anim : *Sampled_Animation;
	if g_params.animation_index > 0
		anim = table_find_pointer(*g_loaded_animations, current_anim_guid);
	else
		anim = null;



	// Increment animation time
	if anim
	{
		g_params.animation_time += delta_time * g_params.animation_speed_in_fps*anim_speed;
		g_params.animation_time = fmod_cycling(g_params.animation_time, xx anim.sample_count);
	}
	else
		g_params.animation_time = 0;


       // We reset to the bind pose because the animation
	   // might not have all of the joints our pose has.
	   reset_to_bind_pose (*skin_pose);

   // if skin_pose.skeleton
   {
	   if anim && g_params.show_animation
	   {
	       sample_animation( anim, g_params.animation_time, *skin_pose, g_params.interpolation_mode );
	   }
       push_draw(*skin_pose, world_model, xyzw(g_params.skeleton_color, 1));
    }
       push_skinning_change(*skin_pose);
       push_draw(skin_mesh, world_model, skin_pose.skinning_matrices);

   }

}


Draw_Dun_Cmds :: (mcmds: []MESH_RENDER_COMMAND) {
    for mcmds {
        cmd: Draw_Command;
        cmd.kind = .TRIANGLE_MESH;
        cmd.should_render = true;
        cmd.mesh = it;

        array_add(*drawing_commands, cmd);
    }
}

