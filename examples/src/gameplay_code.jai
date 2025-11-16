do_player0_gamecode :: (using player0: *Entity, delta_time : float) {
    using game_coordinates;
    if context.solver == null then return;
    this_soft := ifx context.solver.soft_bodies.count!=0 then *context.solver.soft_bodies[0] else null ;


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


       if  g_mesh.joints.count != 0 {
       // wpos := get_world_position(*g_pose, "DEF-spine.004");
       // spos := WorldToScreen(world_model*wpos);
       // if g_params.show_mesh doLabel("GLADIATOR", spos);
        }
       if g_params.show_mesh Draw(g_mesh, world_model, g_pose.skinning_matrices);
       if g_params.show_skeleton Draw(*g_pose, world_model, xyzw (g_params.skeleton_color, 1));

	}

	// Increment animation time
	if anim
	{
		g_params.animation_time += delta_time * g_params.animation_speed_in_fps;
		g_params.animation_time = fmod_cycling (g_params.animation_time, xx anim.sample_count);
	}
	else
		g_params.animation_time = 0;

	//:tagMovedHereBecausePhysics
	   // We transpose the result because OpenGL stores matrices in column-major
        // if g_mesh
        // 	generate_skinning_matrices (*g_pose, transpose_result = true);



    ball: *Body;

    if !is_key_down(#char "G") {
        // ball = doBall(*world_model);;
        ball = doBall(world_model, *g_pose);;
        }

    move_direction: Vector3;
    is_ctrl_down := is_key_down(Key_Code.CTRL);
    if is_key_down(#char "W") move_direction += game_coordinates.forward;
    if is_key_down(#char "S") move_direction += game_coordinates.back;
    if is_key_down(#char "A") move_direction += game_coordinates.left;
    if is_key_down(#char "D") move_direction += game_coordinates.right;

    if is_key_down(Key_Code.SPACEBAR) move_direction += game_coordinates.up;;

    normalize(*move_direction);
    if ball {

    if !is_ctrl_down
        ball.angular_velocity +=  0.2*move_direction;
    else
        ball.velocity +=  0.5*move_direction;


        // log("BALL POS %", ball.position);
        // if is_key_down(#char "H") doHookDontUpdate(ball,world_attach_point=ball.position+up*4.1+forward*2.0, 2.0);;
        if is_key_down(#char "J") doHook(ball,world_attach_point=up*4.1+forward*2.0, 1.9);;
        // if is_key_down(#char "E") doWeld(ball,world_attach_point=up*1.1+forward,weld_relative_axis=game_coordinates.right, rest_angle_radians=3.151593/2.0, target_length=.0);;
        // if is_key_down(#char "Q") doWeld(ball,world_attach_point=up*1.1+forward,weld_relative_axis=game_coordinates.up, rest_angle_radians=3.151593/2.0, target_length=.0);;
    if is_key_down(#char "Q") doDoor(ball,world_hinge_attach_position=up*1.1+forward, world_hinge_axis=game_coordinates.up, game_coordinates.left);;
    if is_key_down(#char "E") doDoor(ball, world_hinge_attach_position = up*1.1+forward, world_hinge_axis = game_coordinates.up+ game_coordinates.right, game_coordinates.right);
    if is_key_down(#char "R") doDoor(ball, world_hinge_attach_position = up*1.1+forward, world_hinge_axis = game_coordinates.down, game_coordinates.left);
    if is_key_down(#char "T") doDoor(ball, world_hinge_attach_position = up*1.1+forward, world_hinge_axis = game_coordinates.right, game_coordinates.forward);
    if is_key_down(#char "C") doDoor(ball, world_hinge_attach_position = up*1.1+forward, world_hinge_axis = game_coordinates.up, game_coordinates.right);
    if is_key_down(#char "V") doDoor(ball,    world_hinge_attach_position = up*1.1+forward,world_hinge_axis = game_coordinates.down,game_coordinates.back);
    } else {
        world_model = translate(world_model, delta_time*move_direction);;
    }
}