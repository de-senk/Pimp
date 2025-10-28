

#import "Compiler";
#import "Basic";
#import "Process";
#import "File";

#import "Metaprogram_Plugins";

OUTPUT_EXECUTABLE_NAME :: "example1";
OUTPUT_EXECUTABLE_NAME_WITH_EXYENSION :: "example1.exe";
OUTPUT_BUILD_DIR :: "./";
FILE_ENTRY_POINT :: "./main.jai";
// FOR_CODEX :: true;

#run {

    set_build_options_dc(.{do_output = false}); //to avoid outputting this "first.jai"
    current_w := get_current_workspace();

    //
    // New Workspace (Target)
    //
    w := compiler_create_workspace("Target Programm");
    options := get_build_options(w);

    copy_commonly_propagated_fields(get_build_options(), *options); //we make the new workspace inherit the commands from current_w

    options.output_executable_name = OUTPUT_EXECUTABLE_NAME;
    options.output_path = OUTPUT_BUILD_DIR;

    modules_paths: [..]string;
    array_add(*modules_paths, "C:/_paths/my_modules/");
    if modules_paths {
        array_add(*modules_paths, ..options.import_path);  // Put these behind whatever the user specified.
        options.import_path = modules_paths;
    }

    set_build_options(options, w);
    compiler_begin_intercept(w);
    add_build_file(FILE_ENTRY_POINT,w); //compilin starts here on another thread process
    compiler_end_intercept(w);
}