game_coordinates :: struct { //TODO: To figure it out.
    left 	:: Vector3.{1,0,0};
    up 	    :: Vector3.{0,1,0};
    forward :: Vector3.{0,0,1};

    right  :: Vector3.{-1,0,0};
    down   :: Vector3.{0,-1,0};
    back   :: Vector3.{0,0,-1};
}


make_rotation_matrix4 :: inline (axis: Vector3, angle: float) -> Matrix4 {
    q: Quaternion = ---;
    set_from_axis_and_angle(*q, axis, angle);
    return rotation_matrix(Matrix4, q);
}

offset_of :: ($T: Type, $member: string) -> int {
    for type_info(T).members {
        if it.name == member return it.offset_in_bytes;
    }

    assert(false, "Type '%' does not have member '%'", T, member);
    return -1;
}

do_single_step: bool;
step_physics: bool = true;;
main :: () {
	// This is here because on Windows, segfaults and stuff are silent.
	// If the program ends without printing this message, then something bad happened.
	defer print ("Program ended gracefully.\n");

    do_platform_weird_initializations();

	#if OS == .WASM
	{
	   do_touch_input = device_supports_touch_input(); // TODO: maybe just stub out on all platforms with return false?
	}

	// Set the working directory so we can access the data folder
	executable_path := path_strip_filename(get_path_of_running_executable());
	set_working_directory(executable_path);

    g_width = 1280;
    g_height = 720;
	g_window = create_window(g_width, g_height, "Skeletal Animation");
	simp.set_render_target(g_window);
	#if OS == .ANDROID {
	    // @Cleanup Make us fullscreen. This could be done in Android manfiest.
        toggle_fullscreen(g_window, true, null);
    }

	getrect.ui_init ();
	init_fonts ();
	assert(init_rendering_objects());
	find_and_load_meshes_and_animations ();



	// Position the camera in a nice way
	g_camera.position = xyz (1, 1.3, -2.3);
	g_camera.euler_angles.x = 200 * PI / 180.0;
	g_camera.euler_angles.y = 10 * PI / 180.0;
	g_running = true;
	delta_time := 0.01667;
	glEnable (GL_BLEND);
	glBlendFunc (GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

        draw_primitive.line = push_line;
        draw_primitive.triangle = push_debug_triangle;
        draw_primitive.start_recording = debug_renderer_start_recording_commands;
        draw_primitive.end_recording = debug_renderer_STOP_recording_commands;
        draw_primitive.doLabel = doLabel;
        draw_primitive.WorldToScreen = WorldToScreen;
    iii:bool;
	while g_running
	{
        iii = !iii;
		frame_start := current_time_monotonic ();
		defer reset_temporary_storage ();
		update_window_events ();
		for get_window_resizes ()
		{
			if it.window == g_window
			&& ((g_width != it.width) || (g_height != it.height))
			{
				g_width = it.width;
				g_height = it.height;
				reinit_fonts ();	// We want the fonts to scale with the window
			}
		}

		for events_this_frame handle_window_event (it);


        {
		  update_frame (delta_time);
		  mouse_x, mouse_y := get_mouse_pointer_position(right_handed=false);
          handle_inputs_shared_between_modes();
	    }

        //
        // Physics STEP simulate, TODO: force it it to 60hz
        //
        should_Step_this_frame: bool;
        if do_single_step {
            do_single_step = false;
            should_Step_this_frame = true;
        }
        if step_physics should_Step_this_frame = true;
        if should_Step_this_frame then  simulate();

    	//:tagMovedHereBecausePhysics
	   // We transpose the result because OpenGL stores matrices in column-major
	   for skinning_commands
        	generate_skinning_matrices (it.pose, transpose_result = true);
        //
        // Drawing pass
        //
		glDepthMask (GL_TRUE);
		glEnable (GL_DEPTH_TEST);
		simp.update_window (g_window);
		simp.clear_render_target (0.1, 0.1, 0.1, 1);
		glClear (GL_DEPTH_BUFFER_BIT);


        // update_instance_animation(*dungeon, door_instance_idx, door_animation);

		//draw_coordinate_axes, if game and physcis coordinte overlap in position and orientation, then we have the same coords!
		{
		  //draw opengl coordinates to the rightmost corner to the rightmost of that coordinate
          using game_coordinates;
          offset := -4.0*left +0.10*up;
          doLabel("GAME COORDINATES", WorldToScreen(offset+1.20*up));
		  draw_primitive.line(offset, offset+forward, .{x=1, w=1}); doLabel("forward", WorldToScreen(offset+forward));
		  draw_primitive.line(offset, offset+up, .{y=1, w=1});      doLabel("up", WorldToScreen(offset+up));
		  draw_primitive.line(offset, offset+left, .{z=1, w=1});    doLabel("left", WorldToScreen(offset+left));
        }
        {

		  //draw opengl coordinates to the rightmost corner
          using game_coordinates;
		  draw_primitive.line(.{}, .{x=1}, .{x=1, w=1});doLabel("x", WorldToScreen(.{x=1}));
		  draw_primitive.line(.{}, .{y=1}, .{y=1, w=1});doLabel("y", WorldToScreen(.{y=1}));
		  draw_primitive.line(.{}, .{z=1}, .{z=1, w=1});doLabel("z", WorldToScreen(.{z=1}));
        }
        {
		  //draw Physics coordinates to the rightmost corner to the rightmost of that coordinate
          using physics_coordinates;
          offset := -4.0*game_coordinates.left +0.10*game_coordinates.up-3.0*game_coordinates.forward;
          doLabel("PHYSICS COORDINATES", WorldToScreen(offset+1.20*game_coordinates.up));
		  draw_primitive.line(offset, offset+forward, .{x=1, w=1}); doLabel("forward", WorldToScreen(offset+forward));
		  draw_primitive.line(offset, offset+up, .{y=1, w=1});      doLabel("up", WorldToScreen(offset+up));
		  draw_primitive.line(offset, offset+left, .{z=1, w=1});    doLabel("left", WorldToScreen(offset+left));
		  // Draw grid
		  draw_primitive.grid(.{0,0,0}, .{0,1,0},);
          // drawDungeon();
        }

        //
        //  Here is where we are actually drawing
        //
        debug_renderer_feed_recorded_commands();
        quick_sort(drawing_commands, (a, b) => compare_render_commands(a.kind, b.kind));
		for drawing_commands {
		  actual_draw(it);
		}
        if iii
		draw_all_debug_triangles();

		for *context.solver.world.soft_bodies init_softbody_mesh(it, it_index);
	    array_reset(*drawing_commands);
	    array_reset(*skinning_commands);
	    array_reset_keeping_memory(*debug_triangle_list);
	    array_reset_keeping_memory(*debug_triangle_list_wire);

		update_ui (delta_time); //Hopefully I will be able to get rid of this procedure in the future.

		simp.swap_buffers (g_window);

		// On android, drawing is throttled to the screen update rate, so there is no need
		// to do timing here.
		#if (OS != .ANDROID)  && (OS != .WASM) {
			sleep_milliseconds (10);
		}

		//do not use defers for these, why? because having explicit control flow is always better.
		frame_end := current_time_monotonic ();
		delta_time = cast (float) to_float64_seconds (frame_end - frame_start);
	}
}

//https://www.youtube.com/watch?v=gKNJKce1p8M
//https://www.youtube.com/watch?v=ZZY9YE7rZJw
dunmap : [12][12]int;
drawDungeon :: () {


    for rowarr, row: dunmap {
        for cell, col: rowarr {
            center := Vector3.{row.(float), 1.0, col.(float)};
            if cell == {
            case 0;
            draw_primitive.X(center);
            case 1;
            draw_primitive.square(center);
            case 2;
            draw_primitive.plus(center);
            case 3;
            draw_primitive.circle(center);
            case 4;
            draw_primitive.arrow(center);
            case 5;
            draw_primitive.wireframe_box(center);
            case;
            }
        }
    }

}

leastConflicts :: () {

}

handle_inputs_shared_between_modes:: () {
    for Input.events_this_frame {

        user_pressed_the_close_window_button := it.type == .QUIT;
        if user_pressed_the_close_window_button then g_running = false;

        if it.type == .KEYBOARD {

            on_alt_f4: bool = it.key_code == .F4 && it.alt_pressed;
            on_alt_q: bool = it.key_code == #char "Q" && it.alt_pressed;
            if on_alt_f4 || on_alt_q {
                g_running = false;
            }

        }
    }
}


// test1pos: Vector3 = Vector3.{.5,0.7, 2.2};
// test1ori := #run make_quaternion_from_axis_and_angle(.{1,-2.2,0}, 3.141593/4);
// test1ori := Quaternion.{-0.291444, -0.490207, -0.050201, 0.819901};

update_frame :: (delta_time : float)
{

	if !input_application_has_focus && g_control_camera set_camera_control (false);
	// Lock the cursor in the middle of the screen when controlling the camera
	if g_control_camera
	{
		x, y, w, h := get_dimensions (g_window, false);
		set_mouse_pointer_position (w / 2, h / 2);
	}
    if g_game_state.should_start_scene {
        g_game_state.should_start_scene = false;
        using game_coordinates;
        create_player0(.{1,1,1}, scale={4,1,2});
        // create_static_box_collider(test1pos, test1ori, scale=.{1,1,1});
        create_static_box_collider(test1pos+2.0*right,  scale=.{4,1,1});
        create_static_box_collider(.{0,-1,0},scale=.{10,1,10});
        create_static_box_collider(.{0,10,0},scale=.{10,.1,10});
        create_static_box_collider(.{5,0,0},scale=.{.1,10,10});
        create_static_box_collider(.{-5,0,0},scale=.{.1,10,10});
        create_static_box_collider(.{0,0,5},scale=.{10,10,0.1});
        create_static_box_collider(.{0,0,-5},scale=.{10,10,0.1});
    }
	update_flying_camera (*g_camera, delta_time);

    for *all_entities {
        // using it; //I don't need to use it anymore.
        if it.is_player0 then do_player0_gamecode(it, delta_time);
        if it.is_static_collider then do_static_box_collider_gamecode(it, delta_time);




	} //for all entities
}


g_game_state : struct {
    should_start_scene: bool = true;
}

all_entities: [..]Entity;
stub_skin_mesh:: Skinned_Mesh.{};

Entity :: struct {
    world_model: Matrix4 = Matrix4_Identity;
    kind: enum {Player0;Wall;Door;};
    rendering_flags: Rendering_Flags;
    is_player0: bool;

    is_static_collider: bool;

    is_a_box_collider: bool;;

    is_a_wall: bool;

    is_a_door: bool;
    is_a_floor_or_ceiling: bool;

    asset_guid: string;
    current_anim_guid: string;
    anim_speed: float;
    max_velocity: float : 2.0;


    is_walking: bool;
    is_running: bool;
    is_idle: bool;
    is_rolling: bool;

    skin_mesh : *Skinned_Mesh;
    skin_pose: Skeleton_Pose;
    skin_anim: Sampled_Animation;
}

Rendering_Flags :: enum_flags {
    Unused;
    Has_Skin_Mesh;
    Has_Animation;
}



create_entity :: () -> *Entity {
       entity := array_add(*all_entities);
       entity.skin_mesh = *stub_skin_mesh;

       return entity;
}

create_static_box_collider :: (pos: Vector3, ori := Quaternion.{0,0,0,1}, scale := Vector3.{1,1,1}) {
    box := create_entity();
    box.is_a_box_collider = true;
    box.is_static_collider = true;
    box.world_model = compose_transform(pos, ori, scale);
}


create_player0 :: (pos: Vector3, ori := Quaternion.{0,0,0,1}, scale := Vector3.{1,1,1}) {
    player0 := create_entity();
    player0.is_player0 = true;
    player0.rendering_flags = .Has_Skin_Mesh;
    player0.asset_guid = copy_string("Player0");
    player0.current_anim_guid = copy_string("Run_Forward");
    player0.world_model = compose_transform(pos, ori, scale);
}




// This is incomplete, and won't work for matrices that have shear.
// In that case we would use a polar decomposition, so this assumes
// your matrix does not have shear.
decompose_transform :: inline (mat : Matrix4) -> translation : Vector3, rotation : Quaternion, scale : Vector3
{
	translation := xyz (
		mat._14,
		mat._24,
		mat._34
	);
	rotation := get_rotation (mat);
	scale := xyz (
		length (xyz (mat._11, mat._21, mat._31)),
		length (xyz (mat._12, mat._22, mat._32)),
		length (xyz (mat._13, mat._23, mat._33))
	);

	return translation, rotation, scale;
}

compose_transform :: (position: Vector3, orientation: Quaternion, scale: Vector3 = .{1,1,1}) -> Matrix4 {
    return make_translation_matrix4(position)*rotation_matrix (Matrix4, orientation) * make_scale_matrix4(scale);;
}


MAX_WEIGHTS :: 4;
MAX_JOINTS :: 1000;

g_window : Window_Type;
g_width, g_height : s32;
g_camera : Camera;
g_running : bool;
g_control_camera : bool;

g_font : *simp.Dynamic_Font;
g_checkbox_font : *simp.Dynamic_Font;
g_slider_font : *simp.Dynamic_Font;
g_dropdown_font : *simp.Dynamic_Font;

// We are using hash tables to store our meshes and animations,
// but in a real game we would have a better data structure(s), like
// a Bucket_Array (a hash table would still be useful to look-up asset
// by their name).
g_loaded_meshes : Table (string, Skinned_Mesh);
g_loaded_animations : Table (string, Sampled_Animation);
// g_mesh : *Skinned_Mesh;
// g_pose : Skeleton_Pose;



handle_window_event :: (event : Event)
{
	getrect.getrect_handle_event (event);
	if event.type ==
	{
	case .QUIT;
		g_running = false;
	case .KEYBOARD;
		if event.key_pressed
		{
			if event.key_code == .ESCAPE
				set_camera_control (!g_control_camera);
			if event.key_code == .F4 && event.alt_pressed
				g_running = false;
		}
	}
}

set_camera_control :: inline (control : bool)
{
	g_control_camera = control;
	if g_control_camera
		hide_cursor ();
	else
		show_cursor ();
}

reinit_fonts :: init_fonts;
init_fonts :: ()
{
	// I want a pixel size of 20 with a window height of 720 => 720 / 36 = 20
	pixel_height := g_height / 36;
	g_font = simp.get_font_at_size ("data/fonts", "OpenSans-Regular.ttf", pixel_height);
	assert (g_font != null);

    // pixel_height := g_height / 36;
    // g_font2 = simp.get_font_at_size ("data/fonts", "OpenSans-Regular.ttf", pixel_height);
    // assert (g_font2 != null);

	pixel_height = g_height / 40;
	g_checkbox_font = simp.get_font_at_size ("data/fonts", "OpenSans-Regular.ttf", pixel_height);
	assert (g_checkbox_font != null);
	pixel_height = g_height / 45;
	g_slider_font = simp.get_font_at_size ("data/fonts", "OpenSans-Italic.ttf", pixel_height);
	assert (g_slider_font != null);
	g_dropdown_font = g_slider_font;
}

init_softbody_mesh :: (this: *Soft_Body, idx: int) {
    if !this.is_initialized then return;
    if this.is_graphics_initialized then return;

    mesh : Skinned_Mesh;
    populate_mesh(this, *mesh);
    init_mesh_rendering_objects (*mesh, dynamic=true);
    name_on_heap := sprint("softbody_%",idx);
    table_add (*g_loaded_meshes, name_on_heap, mesh);
    array_add(*g_params.mesh_names, (name_on_heap));
    this.is_graphics_initialized = true;

}



populate_mesh :: (this: *Soft_Body, mesh: *Skinned_Mesh) {

// Vertex :: struct
// {
// 	position : Vector3;
// 	normal   : Vector3;
// 	// We can have at most 4 weights, but they have to sum up
// 	// to 1, so the last one can be calculated and we don't store it.
// 	weights  : Vector3;
// 	joints   : [4]s16;
// }

// Skinned_Mesh :: struct
// {
// 	#as using skeleton : Skeleton;
// 	vertices : []Vertex;
// 	indices  : []u32;
// 	vao, vbo, ibo : GLuint;
// }
    // mesh.joints.count
    mesh.vertices.count = this.numParticles;
    mesh.indices.count = this.triangleList.count;

	all_data := alloc (size_of (Skeleton.Joint) * mesh.joints.count
		+ size_of (Vertex) * mesh.vertices.count
		+ size_of (u32) * mesh.indices.count
	);
	assert(all_data!=null, "allocation failed");

    //oh init table...
	init (*mesh.name_to_joint_id, mesh.joints.count);
	mesh.joints.data   = all_data;
	mesh.vertices.data = cast (*Vertex) (mesh.joints.data + mesh.joints.count);
	mesh.indices.data  = cast (*u32) (mesh.vertices.data + mesh.vertices.count);
    // for * mesh.joints
    // for * mesh.joints {
    // mesh.vertices.data = this.pos.data.(*Vector3);
    sync_positions_with_renderer(this,mesh);
    mesh.indices.data = this.triangleList.data;
    // for * mesh.indices {
    //     it.* = this.triangleList[it_index];
    // }
}

sync_positions_with_renderer :: (this: *Soft_Body, mesh: *Skinned_Mesh) {

    for * mesh.vertices {
        it.position.x = this.pos[it_index*3 + 0];
        it.position.y = this.pos[it_index*3 + 1];
        it.position.z = this.pos[it_index*3 + 2];
        // it.position.x = bunnyMesh.verts[it_index*3 + 0];
        // it.position.y = bunnyMesh.verts[it_index*3 + 1];
        // it.position.z = bunnyMesh.verts[it_index*3 + 2];
        it.normal = .{1,0,0};
    }
}

find_and_load_meshes_and_animations :: inline () {
	visitor :: (info : *File_Visit_Info, data : *void) {
		path, name_without_ext, ext := path_decomp (info.full_name);
		if ext == {
		  //comented out, proabability of deletion 100%
          // case "mesh";
          // 	mesh : Skinned_Mesh;
          // 	loaded := load_mesh_from_file (*mesh, info.full_name);
          // 	if loaded {
          // 		init_mesh_rendering_objects (*mesh);
          // 		table_add (*g_loaded_meshes, copy_string (name_without_ext), mesh);
          // 	} else {
          // 		log_error ("Could not load % mesh.", name_without_ext);
          //   }
		  case "skinned_mesh";
			mesh : Skinned_Mesh;
			loaded := load_custom_skinned_mesh_from_file(*mesh, info.full_name);
			if loaded {

			     upload_mesh_to_gpu (*mesh);
                // init_mesh_rendering_objects (*mesh);
				table_add (*g_loaded_meshes, copy_string (name_without_ext), mesh);
				table_add(*RES.skin_meshes, copy_string(name_without_ext), mesh);
			} else {
				log_error ("Could not load % mesh.", name_without_ext);
            }
		case"skinned_anim"; #through;
		case"anim";
		    anim : Sampled_Animation;
			loaded := load_animation_from_file (*anim, info.full_name);
			if loaded {
				table_add (*g_loaded_animations, copy_string (name_without_ext), anim);
				table_add(*RES.animations, copy_string(name_without_ext), anim);

            } else {
				log_error ("Could not load % animation.", name_without_ext);
            }
		}
	}

    #if OS == .ANDROID {
        visit_files(context.android_app.activity.assetManager, "data", false, null, visitor);
    } else {
        visit_files("data", false, null, visitor);
    }
	// Initialize the asset names arrays.
	// This is used in the UI to choose the current mesh and animation.
	array_reserve(*g_params.mesh_names, g_loaded_meshes.count + 2);
	array_add(*g_params.mesh_names, copy_string("None"));
	i := 1;
	for val, name : g_loaded_meshes
	{
        // g_params.mesh_names[i] = name;
        array_add(*g_params.mesh_names, copy_string(name));
		i += 1;
	}
	g_params.animation_names = NewArray (g_loaded_animations.count + 1, string);
	g_params.animation_names[0] = "None";
	i = 1;
	for val, name : g_loaded_animations
	{
		g_params.animation_names[i] = name;
		i += 1;
	}

	// Start the program with a mesh and some animation
	g_params.mesh_index = xx (g_params.mesh_names.count - 1);
	g_params.animation_index = xx (g_params.animation_names.count - 1);
}




#import "Basic";
#import "String";
#import "Window_Creation";

using Input :: #import "Input";

#if !#exists(DEBUG)
DEBUG :: false;

#if OS == {
    case .WINDOWS; #load "platform/windows.jai";
}
#load "platform/stub.jai";

was_key_released:: inline (key : Key_Code) -> bool
{
    return (input_button_states[cast (s64) key] & .END) != 0;
}

was_key_pressed :: inline (key : Key_Code) -> bool
{
    return (input_button_states[cast (s64) key] & .START) != 0;
}

my_is_key_down :: inline (key : Key_Code) -> bool
{
    return (input_button_states[cast (s64) key] & .DOWN) != 0;
}

// Things that we define ourselves but they probably should be
// added to the appropriate modules:

is_key_down :: inline (key : Key_Code) -> bool
{
	return (input_button_states[cast (s64) key] & .DOWN) != 0;
}


// Math Matrices are row-major. In other words,
// you have [row 0][row 1][row 2][...] in memory.
#import "Math";
#import "File";
#import "File_Utilities";
#import "System";
#import "Hash_Table";

#import "Sort";
#import "Bucket_Array";

#if OS == .ANDROID {
    #import "Android"()(main);
    #import "Android/File";
    #import "Android/GLES";
    #import "Android/Jni";
    #import "Android/Log";
    #import "Android/Native_App";
} else #if OS == .WASM {
    #import "Toolchains/Web/WebGL";
} else {
    #import "GL";
}

simp :: #import "Simp";
getrect :: #import "GetRect";


#import "Drawing_Primitives";
#import,dir "../../Pimp";

#load "src/parse_utils.jai";
#load "src/render.jai";
#load "src/camera.jai";
#load "src/mesh.jai";
#load "src/anim.jai";
#load "src/ui.jai";
#load "src/draw_commands.jai";
#load "src/gameplay_code.jai";

#load "src/asset_manager.jai";