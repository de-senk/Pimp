#load "generated_generated.jai";

//idk if to put to another file here:
forward :: Vector3.{1,0,0};
left 	:: Vector3.{0,1,0};
up 		:: Vector3.{0,0,1};
back    :: #run -forward; //forward backward? rightward? downward?
right 	:: #run -left;
down 	:: #run -up;

//this should not be here, but I'm prototyping
rotate90aroundx := #run make_quaternion_from_axis_and_angle(.{1,0,0}, 3.141593/2);
rotate90aroundy := #run make_quaternion_from_axis_and_angle(.{0,1,0}, 3.141593/2);
rotate90aroundz := #run make_quaternion_from_axis_and_angle(.{0,0,1}, 3.141593/2);
rotate45aroundx := #run make_quaternion_from_axis_and_angle(.{1,0,0}, 3.141593/4);


#scope_file

#if OS ==  {
    case .WINDOWS; pimp :: #library "windows/pimp";
    case .LINUX;   pimp :: #library "linux/pimp";
    case .MACOS;   pimp :: #library "macos/pimp";
    case .ANDROID;
        #if CPU ==
            case .X64;   pimp :: #library "android/x64/pimp";
            case .ARM64; pimp :: #library "android/arm64/pimp";
        }
    case .PS5; pimp :: #library "ps5/pimp";
} else {
    #assert false;
}