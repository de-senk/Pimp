game_coordinates :: struct {
    forward :: Vector3.{1,0,0};
    left 	:: Vector3.{0,1,0};
    up 	    :: Vector3.{0,0,1};
}

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

    g_current_window_width = 1280;
    g_current_window_height = 720;
	g_window = create_window(g_current_window_width, g_current_window_height, "Skeletal Animation");
	simp.set_render_target(g_window);
	#if OS == .ANDROID {
	    // @Cleanup Make us fullscreen. This could be done in Android manfiest.
        toggle_fullscreen(g_window, true, null);
    }

	getrect.ui_init ();
	init_fonts ();
	if !init_rendering_objects ()
		return;
	find_and_load_meshes_and_animations ();



	// Position the camera in a nice way
	g_camera.position = xyz (1, 1.3, 2.3);
	g_camera.euler_angles.x = -20 * PI / 180.0;
	g_camera.euler_angles.y = 10 * PI / 180.0;
	g_running = true;
	delta_time := 0.01667;
	glEnable (GL_BLEND);
	glBlendFunc (GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

	while g_running
	{
	    array_reset(*drawing_commands);
		frame_start := current_time_monotonic ();
		defer reset_temporary_storage ();
		update_window_events ();
		for get_window_resizes ()
		{
			if it.window == g_window
			&& ((g_current_window_width != it.width) || (g_current_window_height != it.height))
			{
				g_current_window_width = it.width;
				g_current_window_height = it.height;
				reinit_fonts ();	// We want the fonts to scale with the window
			}
		}
		for events_this_frame
			handle_window_event (it);
        {
		  update_frame (delta_time);
		  mouse_x, mouse_y := get_mouse_pointer_position(right_handed=false);
          handle_inputs_shared_between_modes();

	    }
		glDepthMask (GL_TRUE);
		glEnable (GL_DEPTH_TEST);
		simp.update_window (g_window);
		simp.clear_render_target (0.1, 0.1, 0.1, 1);
		glClear (GL_DEPTH_BUFFER_BIT);





		// Draw coordinate axes
		draw_line (.{}, .{x=1}, .{x=1, w=1});
		draw_line (.{}, .{y=1}, .{y=1, w=1});
		draw_line (.{}, .{z=1}, .{z=1, w=1});
        using game_coordinates;
        offset := -10.0*forward -10.0*up;
        doLabel("physics coordinates", WorldToScreen(offset+2.0*left));
		draw_line (offset, offset+1.0*forward, .{x=1, w=1}); doLabel("forward", WorldToScreen(offset+forward));
		draw_line (offset, offset+1.0*up, .{y=1, w=1});      doLabel("up", WorldToScreen(offset+up));
		draw_line (offset, offset+1.0*left, .{z=1, w=1});    doLabel("left", WorldToScreen(offset+left));
		// Draw grid
		for i : -10..10
			draw_line (xyz (xx i, -.1, -10), xyz (xx i, -.1, 10), .{0.4, 0.4, 0.4, 1});
		for i : -10..10
			draw_line (xyz (-10, -.1, xx i), xyz (10, -.1, xx i), .{0.4, 0.4, 0.4, 1});


        quick_sort(drawing_commands, (a, b) => compare_render_commands(a.kind, b.kind));
		for drawing_commands {
		  actual_draw(it);
		}

		update_ui (delta_time);

		simp.swap_buffers (g_window);
		// On android, drawing is throttled to the screen update rate, so there is no need
		// to do timing here.
		#if (OS != .ANDROID)  && (OS != .WASM) {
			sleep_milliseconds (10);
		}
		frame_end := current_time_monotonic ();
		delta_time = cast (float) to_float64_seconds (frame_end - frame_start);
	}
}

all_entities: [64]Entity;
Entity :: struct {
    position: Vector3;
    orientation: Vector3;
    world_model: Matrix4;

    skin_mesh : *Skinned_Mesh;
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


update_frame :: (delta_time : float)
{

	if !input_application_has_focus && g_control_camera set_camera_control (false);
	// Lock the cursor in the middle of the screen when controlling the camera
	if g_control_camera
	{
		x, y, w, h := get_dimensions (g_window, false);
		set_mouse_pointer_position (w / 2, h / 2);
	}

	update_flying_camera (*g_camera, delta_time);

    for *all_entities {
        using it;

        if it_index != 0 then break; //@Prototype first entity
    	// Update the mesh if the user selected another one.
    	// Ideally we would only call table_find_pointer when we know
    	// the user selected another mesh, but I can't find a way to
    	// do that in GetRect.

    	if g_params.mesh_index == 0
    		skin_mesh = null;
    	else
    		skin_mesh = table_find_pointer (*g_loaded_meshes, g_params.mesh_names[g_params.mesh_index]);

    	if g_mesh != skin_mesh
    	{
    		g_mesh = skin_mesh;
    		// We changed the mesh, so reallocate a new pose
    		free_pose (*g_pose);
    		if g_mesh
    			g_pose = make_pose_for_skeleton (g_mesh);
    	}

    	anim : *Sampled_Animation;
    	if g_params.animation_index > 0
    		anim = table_find_pointer (*g_loaded_animations, g_params.animation_names[g_params.animation_index]);
    	else
    		anim = null;

    	if g_mesh
    	{
    		// We reset to the bind pose because the animation
    		// might not have all of the joints our pose has.
    		reset_to_bind_pose (*g_pose);
    		if anim && g_params.show_animation
    		{
    			sample_animation (
    				anim,
    				g_params.animation_time,
    				*g_pose,
    				g_params.interpolation_mode
    			);
    		}


           wpos := get_world_position(*g_pose, "DEF-spine.004");
           spos := WorldToScreen(wpos);

    	   if g_params.show_mesh doLabel("GLADIATOR", spos.x, spos.y, spos.z);
           if g_params.show_mesh Draw(g_mesh, Matrix4_Identity, g_pose.skinning_matrices);
           if g_params.show_skeleton Draw(*g_pose, Matrix4_Identity, xyzw (g_params.skeleton_color, 1));

    	}

    	// Increment animation time
    	if anim
    	{
    		g_params.animation_time += delta_time * g_params.animation_speed_in_fps;
    		g_params.animation_time = fmod_cycling (g_params.animation_time, xx anim.sample_count);
    	}
    	else
    		g_params.animation_time = 0;

    	// We transpose the result because OpenGL stores matrices in column-major
    	if g_mesh
    		generate_skinning_matrices (*g_pose, transpose_result = true);

	} //for all entities
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


#import "Basic";
#import "String";
#import "Window_Creation";
#import "SIMD_Math";
using Input :: #import "Input";

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

physics ::#import,file"../module.jai";

#load "src/parse_utils.jai";
#load "src/render.jai";
#load "src/camera.jai";
#load "src/mesh.jai";
#load "src/anim.jai";
#load "src/ui.jai";
#load "src/draw_commands.jai";



MAX_WEIGHTS :: 4;
MAX_JOINTS :: 1000;

g_window : Window_Type;
g_current_window_width, g_current_window_height : s32;
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
g_mesh : *Skinned_Mesh;
g_pose : Skeleton_Pose;



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
	pixel_height := g_current_window_height / 36;
	g_font = simp.get_font_at_size ("data/fonts", "OpenSans-Regular.ttf", pixel_height);
	assert (g_font != null);

    // pixel_height := g_current_window_height / 36;
    // g_font2 = simp.get_font_at_size ("data/fonts", "OpenSans-Regular.ttf", pixel_height);
    // assert (g_font2 != null);

	pixel_height = g_current_window_height / 40;
	g_checkbox_font = simp.get_font_at_size ("data/fonts", "OpenSans-Regular.ttf", pixel_height);
	assert (g_checkbox_font != null);
	pixel_height = g_current_window_height / 45;
	g_slider_font = simp.get_font_at_size ("data/fonts", "OpenSans-Italic.ttf", pixel_height);
	assert (g_slider_font != null);
	g_dropdown_font = g_slider_font;
}

find_and_load_meshes_and_animations :: inline () {
	visitor :: (info : *File_Visit_Info, data : *void) {
		path, basename, ext := path_decomp (info.full_name);
		if ext == "mesh" {
			mesh : Skinned_Mesh;
			loaded := load_mesh_from_file (*mesh, info.full_name);
			if loaded {
				init_mesh_rendering_objects (*mesh);
				table_add (*g_loaded_meshes, copy_string (basename), mesh);
			} else {
				log_error ("Could not load % mesh.", basename);
            }
		} else if ext == "anim" {
			anim : Sampled_Animation;
			loaded := load_animation_from_file (*anim, info.full_name);
			if loaded {
				table_add (*g_loaded_animations, copy_string (basename), anim);
            } else {
				log_error ("Could not load % animation.", basename);
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
	g_params.mesh_names = NewArray (g_loaded_meshes.count + 1, string);
	g_params.mesh_names[0] = "None";
	i := 1;
	for val, name : g_loaded_meshes
	{
		g_params.mesh_names[i] = name;
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