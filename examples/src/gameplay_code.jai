step_draw: int;
do_player0_gamecode :: (using player0: *Entity, delta_time : float) {
    using game_coordinates;
    if context.solver == null then return;
    this_soft := ifx context.solver.soft_bodies.count!=0 then *context.solver.soft_bodies[0] else null ;


	// Update the mesh if the user selected another one.
	// Ideally we would only call table_find_pointer when we know
	// the user selected another mesh, but I can't find a way to
	// do that in GetRect.
    // if g_params.mesh_index == 0
    //    skin_mesh = null;
    // else
    // if is_key_down(#char "J") {
    //    log("% %",asset_guid, skin_mesh);
    //    skin_mesh = table_find_pointer (*g_loaded_meshes, asset_guid);
    // }


    ball: *Body;

    // Draw(player0, delta_time);
    if was_key_pressed(#char "R") step_draw = 0;
    if was_key_pressed(#char "I") step_draw -= ifx step_draw>0 then 1 else 0;
    if was_key_pressed(#char "O") step_draw += 1;
    if is_key_down(#char "Z") collider_switch_face = true;
    if is_key_down(#char "X") collider_switch_face = false;
    if is_key_down(#char "Y") log("orio %", test1ori);
    if !is_key_down(#char "G") {
        ball = doBall(*world_model);;
        // ball = doBall(world_model, *skin_pose);;
    }


    move_direction: Vector3;
    rotate_direction: Vector3;
    is_ctrl_down := is_key_down(Key_Code.CTRL);
    if is_key_down(#char "W") move_direction += game_coordinates.forward;
    if is_key_down(#char "S") move_direction += game_coordinates.back;
    if is_key_down(#char "A") move_direction += game_coordinates.left;
    if is_key_down(#char "D") move_direction += game_coordinates.right;
    if is_key_down(Key_Code.SPACEBAR) move_direction += 4.0*game_coordinates.up;;
    if is_key_down(#char "C") move_direction -= 4.0*game_coordinates.up;;

    if is_key_down(#char "Q") rotate_direction += 1.0*game_coordinates.up;;
    if is_key_down(#char "E") rotate_direction -= 1.0*game_coordinates.up;;
    if is_key_down(Key_Code.ARROW_UP) rotate_direction += game_coordinates.left;
    if is_key_down(Key_Code.ARROW_DOWN) rotate_direction += game_coordinates.right;
    if is_key_down(Key_Code.ARROW_LEFT) rotate_direction += game_coordinates.back;
    if is_key_down(Key_Code.ARROW_RIGHT) rotate_direction += game_coordinates.forward;


    normalize(*move_direction);
    normalize(*rotate_direction);

    // new_len := clamp_length(*ball.velocity, max_velocity);
    anim_speed =  0.3*length(move_direction) ;//20.0*new_len/max_velocity;

    if ball {

    // if !is_ctrl_down
        rotate_direction.y *= -1.0; //because camera -z or something

        ball.angular_velocity +=  2.2*rotate(-rotate_direction, g_camera.rotation);
    // else
        move_direction.y *= -1.0; //because camera -z or something
        ball.position +=  0.05*rotate(-move_direction, g_camera.rotation);
        g_camera.FOCUS = ball.position;
    //
    //Complxity detected: Maybe the physics cod shoudl go into animation code for the cases that manipulates aniamtions?
    //
        // log("BALL POS %", ball.position);
        // if is_key_down(#char "H") doHookDontUpdate(ball,world_attach_point=ball.position+up*4.1+forward*2.0, 2.0);;
        if is_key_down(#char "J") doHook(ball,world_attach_point=up*4.1+forward*2.0, 1.9);;
        // if is_key_down(#char "E") doWeld(ball,world_attach_point=up*1.1+forward,weld_relative_axis=game_coordinates.right, rest_angle_radians=3.151593/2.0, target_length=.0);;
        // if is_key_down(#char "Q") doWeld(ball,world_attach_point=up*1.1+forward,weld_relative_axis=game_coordinates.up, rest_angle_radians=3.151593/2.0, target_length=.0);;
    // if is_key_down(#char "Q") doDoor(ball,world_hinge_attach_position=up*1.1+forward, world_hinge_axis=game_coordinates.up, game_coordinates.left);;
    // if is_key_down(#char "E") doDoor(ball, world_hinge_attach_position = up*1.1+forward, world_hinge_axis = game_coordinates.up+ game_coordinates.right, game_coordinates.right);
    if is_key_down(#char "R") doDoor(ball, world_hinge_attach_position = up*1.1+forward, world_hinge_axis = game_coordinates.down, game_coordinates.left);
    if is_key_down(#char "T") doDoor(ball, world_hinge_attach_position = up*1.1+forward, world_hinge_axis = game_coordinates.right, game_coordinates.forward);
    if is_key_down(#char "C") doDoor(ball, world_hinge_attach_position = up*1.1+forward, world_hinge_axis = game_coordinates.up, game_coordinates.right);
    if is_key_down(#char "V") doDoor(ball,    world_hinge_attach_position = up*1.1+forward,world_hinge_axis = game_coordinates.down,game_coordinates.back);
    } else {
        // g_camera.FOCUS = ball.position;
        world_model = translate(world_model, delta_time*move_direction);;
    }
}