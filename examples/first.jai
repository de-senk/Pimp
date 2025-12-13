

#import "Compiler";
#import "Basic";
#import "Process";
#import "String";
#import "File";

#import "Metaprogram_Plugins";

OUTPUT_EXECUTABLE_NAME :: "example1";
OUTPUT_EXECUTABLE_NAME_WITH_EXYENSION :: "example1.exe";
OUTPUT_BUILD_DIR :: "./";
FILE_ENTRY_POINT :: "./main.jai";

// FOR_CODEX :: true;
plugins: [..] *Metaprogram_Plugin;

CAPTURE_PLUGINS::true;

VERSION      :: #run -> string {
    result, revision := run_command("git", "rev-parse", "--short", "HEAD", capture_and_return_output = true);
    if revision return sprint("0.3.8-%", trim(revision));
    return "0.3.8-dev";
}
RELEASE_DATE :: #run -> string {
    current_time := to_calendar(current_time_consensus());
    return calendar_to_string(current_time);
}
#run {




    set_build_options_dc(.{do_output = false}); //to avoid outputting this "first.jai"
    // current_w := get_current_workspace();

    //
    // New Workspace (Target)
    //

    w := compiler_create_workspace("Target Programm");
    options := get_build_options(w);
    args := options.compile_time_command_line;
    // copy_commonly_propagated_fields(get_build_options(), *options); //we make the new workspace inherit the commands from current_w

    options.output_executable_name = OUTPUT_EXECUTABLE_NAME;
    options.output_path = OUTPUT_BUILD_DIR;

    modules_paths: [..]string;

    array_add(*modules_paths, "C:/_paths/my_modules/");
    array_add(*modules_paths, "../modules/");
    if modules_paths {
        array_add(*modules_paths, ..options.import_path);  // Put these behind whatever the user specified.
        options.import_path = modules_paths;
    }


    success:, plugins_to_create:, args = parse_plugin_arguments(args);
    if !success {
        compiler_report("Failed to parse plugin arguments.\n");
        exit(1);
    }


    optimized:=false;
    for arg: args {
        if arg == {

           case "-x64";
            options.backend = .X64;
            // options.use_natvis_compatible_types = true;
            // options.use_visual_studio_message_format = true;
            // options.debug_for_expansions = true;
           case "-llvm";
            options.backend = .LLVM;
          case "-release";
            optimized = true;
            set_optimization(*options, .VERY_OPTIMIZED);
            options.llvm_options.enable_split_modules = false;
            options.array_bounds_check = .ON;
            options.null_pointer_check = .ON;
            options.arithmetic_overflow_check = .OFF;
            options.stack_trace = false;
          case "debug";
          case "no_output";
            options.output_type = .NO_OUTPUT;
          case "import_dir";
            compiler_report(tprint("Command-line argument #%, '%', is invalid. Valid options are: 'debug', 'release'.\n", it_index+1, arg));
        }
    }

    set_build_options(options, w);

    #if CAPTURE_PLUGINS {

         success = init_plugins(plugins_to_create, *plugins, w);
        if !success {
            compiler_report("A plugin init() failed. Exiting.\n");
            exit(1);
        }

        intercept_flags: Intercept_Flags;
        for plugins if it.before_intercept it.before_intercept(it, *intercept_flags);
    }
    compiler_begin_intercept(w, intercept_flags);
    #if CAPTURE_PLUGINS for plugins if it.add_source it.add_source(it);
    add_build_file(FILE_ENTRY_POINT,w); //compilin starts here on another thread process

    build_constants := tprint(#string STRING
            VERSION         :: "%";
            RELEASE_DATE    :: "%";
            DEBUG           :: %;
            NO_OUTPUT_BUILD :: %;
        STRING,
            VERSION,
            RELEASE_DATE,
            ifx optimized then "false" else "true",
            options.output_type == .NO_OUTPUT,
    );
    add_build_string(build_constants, w);

    failed := false;
    while true {
        message := compiler_wait_for_message();
        if message.workspace != w continue;

        for plugins if it.message it.message(it, message);

        if message.kind == {
            case .TYPECHECKED;
                if options.output_type != .NO_OUTPUT {
                    typechecked := cast(*Message_Typechecked) message;
                    for typechecked.structs {
                        // s := it.expression.defined_type;
                        // if s.name == "Settings"  register_settings_members(s, it.expression);
                    }
                    for typechecked.declarations {
                        if it.expression.name == "ADDED_SETTINGS" {
                            // register_added_settings(it.expression);
                        } else if it.expression.name == "RENAMED_SETTINGS" {
                            // register_renamed_settings(it.expression);
                        }
                    }
                }

            case .PHASE;
                phase_message := cast(*Message_Phase) message;
                if phase_message.phase == {
                    case .ALL_TARGET_CODE_BUILT;
                        if options.output_type == .NO_OUTPUT  exit(0);
                        // check_settings();

                    case .READY_FOR_CUSTOM_LINK_COMMAND;
                        // failed = !run_custom_link(message.workspace, phase_message);
                        // compiler_custom_link_command_is_complete(message.workspace);
                        // if failed break;
                }

            case .COMPLETE;
                m := cast(*Message_Complete) message;
                if m.error_code == .COMPILATION_FAILED then failed = true;
                break;
        }
    }

    compiler_end_intercept(w);

    #if CAPTURE_PLUGINS for plugins if it.finish   it.finish  (it);
    #if CAPTURE_PLUGINS for plugins if it.shutdown it.shutdown(it);
}

