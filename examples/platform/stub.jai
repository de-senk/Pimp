//
//  Implement:
//
#if !#exists(show_cursor)
show_cursor :: inline ()
{
	log_UNIMPLEMENTED();
}


#if !#exists(hide_cursor)
hide_cursor :: inline ()
{
    log_UNIMPLEMENTED();
}


#if !#exists(set_mouse_pointer_position)
do_platform_weird_initializations :: () {
    // log_UNIMPLEMENTED(); commented out because not all platforms require some silly stuff like windows does.

}

#if !#exists(set_mouse_pointer_position)
set_mouse_pointer_position :: inline (x : int, y : int)
{
    log_UNIMPLEMENTED();
}

#if !#exists(get_mouse_pointer_position)
get_mouse_pointer_position :: inline ( right_handed: bool) -> (x: int, y: int, success: bool)
{
    log_UNIMPLEMENTED();
	return 0, 0, false;
}



//
// Per platform Implementation not required:
//
// #if !#exists(get_screen_dimensions) forced enable! Input module already handles this for us
get_screen_dimensions :: inline () ->  (atX: float,atY:float,w:float,h:float) {
    return 0,0,xx g_current_window_width,xx g_current_window_height;
}

log_UNIMPLEMENTED :: (loc:= #caller_location) {
  //this is for VSCODE |||filepath line:character||| but I dont use vscode...
  log("[UNIMPLEMENTED] % %:%", loc.fully_pathed_filename, loc.line_number, loc.character_number, flags = .WARNING);
}