
win32 :: #import "Windows";

show_cursor :: inline ()
{
    //I think in windows we need a while loop to do this
	win32.ShowCursor (1);
}


hide_cursor :: inline ()
{
    //I think in windows we need a while loop to do this
	win32 :: #import "Windows";
	win32.ShowCursor (0);
}

do_platform_weird_initializations :: () {
	win32.SetProcessDPIAware ();
}

set_mouse_pointer_position :: inline (x : int, y : int)
{
	// Windows module does not have SetCursorPos
	// win32 :: #import "Windows";
	// win32.SetCursorPos (xx x, xx y);
	// Hugh...
	user32 :: #library,system "User32";
	SetCursorPos :: (x : s32, y : s32) -> s32 #foreign user32;
	SetCursorPos (xx x, xx y);

}


//disabled because Input module already handles the window size! so we just set a global var and use it.
DISABLED_get_screen_dimensions :: () -> (atX: float,atY:float,w:float,h:float) {
    return 0,0,xx g_width,xx g_height;

}