Camera :: struct
{
	FLY_SPEED :: 1.0;
	ROTATION_SPEED :: 0.3;
	ROTATION_SMOOTHING :: 0.3;

	position : Vector3;
	euler_angles : Vector3;
	rotation : Quaternion;
	rotation_input : Vector2;
	fov := 60.0;
	transform := Matrix4_Identity;
	view_matrix := Matrix4_Identity;
	projection_matrix := Matrix4_Identity;
	view_projection_matrix := Matrix4_Identity;
}

WorldToScreen :: (position: Vector3) -> Vector3, is_inside_frustrum: bool {
    vx, vy, vw, vh := get_dimensions(g_window, false);
    world_pos := Vector4.{position.x, position.y, position.z, 1.0};
    clip_space := g_camera.view_projection_matrix * world_pos;

    is_behind_camera := clip_space.w <= 0;
    if is_behind_camera return .{}, false;

    ndc := Vector3.{
        clip_space.x / clip_space.w,
        clip_space.y / clip_space.w,
        clip_space.z / clip_space.w
    };

    is_inside_frustrum := (
        ndc.x >= -1.0 && ndc.x <= 1.0 &&
        ndc.y >= -1.0 && ndc.y <= 1.0 &&
        ndc.z >= -1.0 && ndc.z <= 1.0
    );

    screen_pos := Vector3.{
        (ndc.x + 1.0) * 0.5 * vw,
        (ndc.y + 1.0) * 0.5 * vh,
        ndc.z
    };
    return screen_pos, is_inside_frustrum;
}


WorldToScreenw :: (position: Vector3) -> Vector3, is_inside_frustrum: bool {
    is_inside_frustrum: bool = true; //hardcoded for now
    vx, vy, vw, vh := get_dimensions (g_window, false);
	camera_position := g_camera.position;
    half := Vector3.{ vw * 0.5, vh * 0.5,0 };
    // return (position - camera_position) * camera_zoom() + half;
    _, inv_view := inverse(g_camera.view_projection_matrix);
    _, inv_proj := inverse(g_camera.projection_matrix);
    return inv_proj*inv_view*(position - camera_position), is_inside_frustrum;
}

do_touch_input := OS == .ANDROID; // kinda hacky

update_flying_camera :: (using camera : *Camera, delta_time : float)
{
	mouse_delta : Vector2;
	move_input  : Vector3;
	move_speed  : float;

	// Some very @Hack code to make touch one finger drag to orbt, two fingers drag to free look.
	// There is an ugly snap when you stop free looking as we instantly teleport to the position we would be
	// in orbit with the free look angle.
	touch_grabbing := false;
	orbitting := true;
    // if do_touch_input {
        // orbitting = touch_num_pointers < 2;

        // // Invert the delta and scale it on touch so dragging feels how you expect.
        // scale_factor := ifx orbitting  0.5 else 0.25;
        // mouse_delta_x = xx (cast(float) mouse_delta_x * scale_factor);
        // mouse_delta_y = xx (cast(float) mouse_delta_y * scale_factor);

        // if orbitting mouse_delta_y *= -1;
        // else         mouse_delta_x *= -1;

        // touch_grabbing = !getrect.is_grabbing();
    // }

	if g_control_camera || touch_grabbing {
		move_speed = FLY_SPEED;
		mouse_delta = xy (xx mouse_delta_x, xx mouse_delta_y);
		if is_key_down (.SHIFT) then move_speed *= 10;
		move_speed *= delta_time;
		move_input.x = cast (float) is_key_down (xx #char "D")
			- cast (float) is_key_down (xx #char "A");
		move_input.y = cast (float) is_key_down (xx #char "E")
			- cast (float) is_key_down (xx #char "Q");
		move_input.z = cast (float) is_key_down (xx #char "S")
			- cast (float) is_key_down (xx #char "W");
		move_input = rotate (unit_vector (move_input), rotation);
	}
	position += move_input * move_speed;

	rotation_input = lerp (rotation_input, mouse_delta, ROTATION_SMOOTHING);
	delta := rotation_input * ROTATION_SPEED;
	euler_angles.x += delta.x * PI / 180.0;
	euler_angles.y += delta.y * PI / 180.0;
	euler_angles.y = clamp (euler_angles.y, -80.0  * PI / 180.0, 80.0 * PI / 180.0);
	yaw : Quaternion;
	// I am not sure why I need to invert the axis of rotation...
	set_from_axis_and_angle (*yaw, 0, -1, 0, euler_angles.x);
	pitch : Quaternion;
	set_from_axis_and_angle (*pitch, -1, 0, 0, euler_angles.y);
	rotation = yaw * pitch;

	// @Hack orbit camera movment.
	if orbitting {
		ORBIT_DISTANCE :: 2.5;
		FOCUS :: Vector3.{0, 1, 0};
		offset := rotate (.{0, 0, ORBIT_DISTANCE}, rotation); // -z is forward!
		position = FOCUS + offset;
	}

	transform = make_translation_matrix4 (position) * rotation_matrix (Matrix4, rotation);

    success:, view_matrix = inverse (transform);
    if !success {
        log_error("Unable to invert the transform in update_flying_camera()!\n");
    }

	vx, vy, vw, vh := get_dimensions (g_window, false);
	aspect_ratio := vw / cast (float) vh;
	projection_matrix = make_projection_matrix (fov * PI / 180.0, aspect_ratio, 0.1, 1000.0);
	view_projection_matrix = projection_matrix * view_matrix;
}


 get_ray_from_screen :: (mouse_x: float, mouse_y: float) -> (origin: Vector3, dir: Vector3) {
    using g_camera;

    // screen dimensions
    vx, vy, vw, vh := get_screen_dimensions();
    aspect_ratio := vw / cast(float) vh;

    ndc_x := (2.0 * (mouse_x - vx)) / vw - 1.0;
    ndc_y := 1.0 - (2.0 * (mouse_y - vy)) / vh; // flip Y openGL?

    ray_clip_near := Vector4.{ ndc_x, ndc_y, -1.0, 1.0 };
    ray_clip_far  := Vector4.{ ndc_x, ndc_y,  1.0, 1.0 };

    success, inv_vp := inverse(view_projection_matrix);

    // unproject near and far
    world_ray_near := inv_vp * ray_clip_near;
    world_ray_near /= world_ray_near.w;

    world_ray_far := inv_vp * ray_clip_far;
    world_ray_far /= world_ray_far.w;

    // ray definition
    origin := world_ray_near.xyz;
    dir := (world_ray_far.xyz - world_ray_near.xyz);
    log("TODO: DRAWING? add_to_array_of_vector_lines(origin,dir);");
    dir_n := normalize(dir);
    return origin, dir_n;
}

ScreenToWorld :: (position: Vector3) -> Vector3 {
    vx, vy, vw, vh := get_dimensions (g_window, false);
	camera_position := g_camera.position;
    half := Vector3.{ vw * 0.5, vh * 0.5 ,0};
    return camera_position + (position - half) / camera_zoom();
}

camera_zoom :: ()-> float32 {
    Deg2Rad :: 3.161593/180;
    vx, vy, vw, vh := get_dimensions (g_window, false);
	down_expression := 2*tan(g_camera.fov / 2);
	zoom := vw / down_expression;
	return zoom;
}