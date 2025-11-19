/*
NOTE: I dont use AI for code because I like to understand what the code does.

This file was generated with claude with this prompt at it sucks because it only forward decalres but hey, at least its a good start, Ive not tested the output, I just wanted to see how good AI were these days:

PROMPT:
I will give you my build_dll.jai, what that file does is to grab a jai project, check all procedures that have the note @dll_export, creates a file called generated_generated.jai and FIlls in all structs and structs declaration depenendies into that file, ald ograbs proedure signatures and pasthes the all inside that file.
I want you to use that as a reference and instead, create me a buld_transpile_to_c_with_overloads.jai and that build fiel should grab all the project including modules dependencies, and regenerate the code in simple C with procedure overloadings code, dont use class use structs, everything as close to the project as possible.
and I want you to crate a header only generated_generated.hpp where you paste all the C code.
ATTACHED FILE:
build_dll.jai


*/
#import "Compiler";
#import "Basic";

#import "Process";
#import "File";
#import "Hash_Table";
#import "Program_Print";


OUTPUT_FILE_NAME :: "../generated_transpiled.hpp";
FILE_ENTRY_POINT :: "./module.jai";

// Track all declarations we need to transpile
all_structs: Table(string, *Code_Declaration);
all_enums: Table(string, *Code_Declaration);
all_procedures: [..]*Code_Procedure_Header;
all_type_definitions: Table(string, *Code_Declaration);
struct_dependencies: Table(string, Table(string, void));
processed_types: Table(string, void);

#run {
    set_build_options_dc(.{do_output = false});

    w := compiler_create_workspace("C++ Transpiler");
    options := get_build_options(w);
    copy_commonly_propagated_fields(get_build_options(), *options);

    options.output_type = .NO_OUTPUT;

    // Add custom module paths if needed
    modules_paths: [..]string;
    array_add(*modules_paths, "C:/_paths/my_modules/");
    if modules_paths {
        array_add(*modules_paths, ..options.import_path);
        options.import_path = modules_paths;
    }

    set_build_options(options, w);

    compiler_begin_intercept(w);
    add_build_file(FILE_ENTRY_POINT, w);
    message_loop(w);
    compiler_end_intercept(w);
}

message_loop :: (w: Workspace) {
    while true {
        message := compiler_wait_for_message();
        if message.workspace != w then continue;

        if message.kind == {
            case .TYPECHECKED;
                tc := cast(*Message_Typechecked) message;

                // Collect all declarations
                for tc.declarations {
                    if it.expression.kind == .DECLARATION {
                        decl := cast(*Code_Declaration) it.expression;
                        process_declaration(decl);
                    }
                }

                // Collect all procedure headers
                for tc.procedure_headers {
                    header := it.expression;
                    if header.procedure_flags & .POLYMORPHIC continue;
                    if header.procedure_flags & .MACRO continue;

                    array_add(*all_procedures, header);
                }

            case .COMPLETE;
                // Generate the C++ code
                generate_cpp_output();
                break;
        }
    }
}

process_declaration :: (decl: *Code_Declaration) {
    if !decl.name return;

    load := decl.enclosing_load;
    if !load return;

    import := load.enclosing_import;
    if !import return;

    // Process based on expression type
    if decl.expression {
        if decl.expression.kind == {
            case .STRUCT;
                table_set(*all_structs, decl.name, decl);
                collect_struct_dependencies(decl);

            case .ENUM;
                table_set(*all_enums, decl.name, decl);

            case .TYPE_DEFINITION;
                table_set(*all_type_definitions, decl.name, decl);
        }
    }
}

collect_struct_dependencies :: (decl: *Code_Declaration) {
    if !decl.type_inst return;

    dep_table := find_or_add(*struct_dependencies, decl.name);
    collect_type_dependencies(decl.type_inst, dep_table);

    // Also process the struct body
    if decl.expression && decl.expression.kind == .STRUCT {
        struct_node := cast(*Code_Struct) decl.expression;
        if struct_node.block {
            for struct_node.block.statements {
                if it.kind == .DECLARATION {
                    member_decl := cast(*Code_Declaration) it;
                    if member_decl.type_inst {
                        collect_type_dependencies(member_decl.type_inst, dep_table);
                    }
                }
            }
        }
    }
}

collect_type_dependencies :: (inst: *Code_Type_Instantiation, dep_table: *Table(string, void)) {
    if !inst return;

    if inst.pointer_to {
        collect_type_dependencies(inst.pointer_to, dep_table);
        return;
    }

    if inst.array_element_type {
        collect_type_dependencies(inst.array_element_type, dep_table);
        return;
    }

    if inst.type_valued_expression {
        if inst.type_valued_expression.kind == .IDENT {
            ident := cast(*Code_Ident) inst.type_valued_expression;
            if ident.name {
                find_or_add(dep_table, ident.name);
            }
        }
    }
}

generate_cpp_output :: () {
    builder: String_Builder;

    // Header
    print_to_builder(*builder, "// Auto-generated by build_transpile_to_c_with_overloads.jai\n");
    print_to_builder(*builder, "// Date: %\n\n", current_time_monotonic());
    print_to_builder(*builder, "#ifndef GENERATED_TRANSPILED_HPP\n");
    print_to_builder(*builder, "#define GENERATED_TRANSPILED_HPP\n\n");

    // Standard includes
    print_to_builder(*builder, "#include <stdint.h>\n");
    print_to_builder(*builder, "#include <stdbool.h>\n");
    print_to_builder(*builder, "#include <stddef.h>\n\n");

    // Type aliases for Jai types
    print_to_builder(*builder, "// Jai type mappings\n");
    print_to_builder(*builder, "typedef int8_t   s8;\n");
    print_to_builder(*builder, "typedef int16_t  s16;\n");
    print_to_builder(*builder, "typedef int32_t  s32;\n");
    print_to_builder(*builder, "typedef int64_t  s64;\n");
    print_to_builder(*builder, "typedef uint8_t  u8;\n");
    print_to_builder(*builder, "typedef uint16_t u16;\n");
    print_to_builder(*builder, "typedef uint32_t u32;\n");
    print_to_builder(*builder, "typedef uint64_t u64;\n");
    print_to_builder(*builder, "typedef float    f32;\n");
    print_to_builder(*builder, "typedef double   f64;\n\n");

    // Forward declarations for structs
    print_to_builder(*builder, "// Forward declarations\n");
    for decl, name: all_structs {
        print_to_builder(*builder, "struct %;\n", name);
    }
    print_to_builder(*builder, "\n");

    // Generate enums
    print_to_builder(*builder, "// Enumerations\n");
    for decl, name: all_enums {
        transpile_enum(*builder, decl);
    }
    print_to_builder(*builder, "\n");

    // Generate type definitions
    print_to_builder(*builder, "// Type definitions\n");
    for decl, name: all_type_definitions {
        transpile_type_definition(*builder, decl);
    }
    print_to_builder(*builder, "\n");

    // Generate structs in dependency order
    print_to_builder(*builder, "// Structures\n");
    ordered_structs := topological_sort_structs();
    for struct_name: ordered_structs {
        decl := table_find_pointer(*all_structs, struct_name);
        if decl {
            transpile_struct(*builder, decl.*);
        }
    }
    print_to_builder(*builder, "\n");

    // Generate procedures (with overloading support)
    print_to_builder(*builder, "// Procedures\n");
    for procedure: all_procedures {
        transpile_procedure(*builder, procedure);
    }

    print_to_builder(*builder, "\n#endif // GENERATED_TRANSPILED_HPP\n");

    // Write to file
    success := write_entire_file(OUTPUT_FILE_NAME, *builder);
    if success {
        log("Successfully generated %\n", OUTPUT_FILE_NAME);
    } else {
        log_error("Failed to write %\n", OUTPUT_FILE_NAME);
    }
}

topological_sort_structs :: () -> [..] string {
    result: [..] string;
    visited: Table(string, void);
    temp_mark: Table(string, void);

    visit :: (name: string, visited: *Table(string, void), temp_mark: *Table(string, void), result: *[..] string) -> bool {
        if table_find_pointer(visited, name) return true;
        if table_find_pointer(temp_mark, name) {
            // Circular dependency detected
            return false;
        }

        find_or_add(temp_mark, name);

        // Visit dependencies first
        dep_table := table_find_pointer(*struct_dependencies, name);
        if dep_table {
            for _, dep_name: dep_table {
                if table_find_pointer(*all_structs, dep_name) {
                    if !visit(dep_name, visited, temp_mark, result) return false;
                }
            }
        }

        table_remove(temp_mark, name);
        find_or_add(visited, name);
        array_add(result, name);
        return true;
    }

    for _, name: all_structs {
        if !table_find_pointer(*visited, name) {
            visit(name, *visited, *temp_mark, *result);
        }
    }

    return result;
}

transpile_enum :: (builder: *String_Builder, decl: *Code_Declaration) {
    if decl.expression.kind != .ENUM return;

    enum_node := cast(*Code_Enum) decl.expression;

    print_to_builder(builder, "enum % {\n", decl.name);

    if enum_node.block {
        for enum_node.block.statements {
            if it.kind == .DECLARATION {
                member := cast(*Code_Declaration) it;
                print_to_builder(builder, "    %", member.name);

                // Handle explicit values if present
                if member.expression {
                    print_to_builder(builder, " = ");
                    transpile_expression(builder, member.expression);
                }

                print_to_builder(builder, ",\n");
            }
        }
    }

    print_to_builder(builder, "};\n\n");
}

transpile_type_definition :: (builder: *String_Builder, decl: *Code_Declaration) {
    print_to_builder(builder, "typedef ");

    if decl.type_inst {
        transpile_type(builder, decl.type_inst);
    }

    print_to_builder(builder, " %;\n", decl.name);
}

transpile_struct :: (builder: *String_Builder, decl: *Code_Declaration) {
    if decl.expression.kind != .STRUCT return;

    struct_node := cast(*Code_Struct) decl.expression;

    print_to_builder(builder, "struct % {\n", decl.name);

    if struct_node.block {
        for struct_node.block.statements {
            if it.kind == .DECLARATION {
                member := cast(*Code_Declaration) it;
                print_to_builder(builder, "    ");
                transpile_type(builder, member.type_inst);
                print_to_builder(builder, " %", member.name);

                // Handle array dimensions
                if member.type_inst && member.type_inst.array_dimension {
                    print_to_builder(builder, "[");
                    transpile_expression(builder, member.type_inst.array_dimension);
                    print_to_builder(builder, "]");
                }

                print_to_builder(builder, ";\n");
            }
        }
    }

    print_to_builder(builder, "};\n\n");
}

transpile_procedure :: (builder: *String_Builder, header: *Code_Procedure_Header) {
    if !header.name return;

    // Return type
    if header.returns.count > 0 {
        first_return := header.returns[0];
        if first_return.kind == .DECLARATION {
            ret_decl := cast(*Code_Declaration) first_return;
            if ret_decl.type_inst {
                transpile_type(builder, ret_decl.type_inst);
            } else {
                print_to_builder(builder, "void");
            }
        }
    } else {
        print_to_builder(builder, "void");
    }

    print_to_builder(builder, " %(", header.name);

    // Parameters
    first := true;
    for header.arguments {
        if it.kind == .DECLARATION {
            arg_decl := cast(*Code_Declaration) it;

            if !first print_to_builder(builder, ", ");
            first = false;

            if arg_decl.type_inst {
                transpile_type(builder, arg_decl.type_inst);
            }

            if arg_decl.name {
                print_to_builder(builder, " %", arg_decl.name);
            }
        }
    }

    print_to_builder(builder, ");\n");
}

transpile_type :: (builder: *String_Builder, inst: *Code_Type_Instantiation) {
    if !inst {
        print_to_builder(builder, "void");
        return;
    }

    // Handle pointers
    if inst.pointer_to {
        transpile_type(builder, inst.pointer_to);
        print_to_builder(builder, "*");
        return;
    }

    // Handle arrays
    if inst.array_element_type {
        if inst.inst_flags & .RESIZABLE {
            // Dynamic array - use pointer for simplicity
            transpile_type(builder, inst.array_element_type);
            print_to_builder(builder, "*");
        } else if inst.inst_flags & .VARARGS {
            // Varargs
            transpile_type(builder, inst.array_element_type);
            print_to_builder(builder, "*");
        } else {
            // Fixed or dynamic array
            transpile_type(builder, inst.array_element_type);
            print_to_builder(builder, "*");
        }
        return;
    }

    // Handle base types
    if inst.type_valued_expression {
        if inst.type_valued_expression.kind == .IDENT {
            ident := cast(*Code_Ident) inst.type_valued_expression;
            type_name := ident.name;

            // Map Jai types to C types
            if type_name == {
                case "int";      print_to_builder(builder, "s64");
                case "float";    print_to_builder(builder, "f32");
                case "float64";  print_to_builder(builder, "f64");
                case "float32";  print_to_builder(builder, "f32");
                case "bool";     print_to_builder(builder, "bool");
                case "string";   print_to_builder(builder, "char*");
                case "void";     print_to_builder(builder, "void");
                case;            print_to_builder(builder, "%", type_name);
            }
        } else {
            print_to_builder(builder, "void");
        }
    } else {
        print_to_builder(builder, "void");
    }
}

transpile_expression :: (builder: *String_Builder, expr: *Code_Node) {
    if !expr return;

    if expr.kind == {
        case .LITERAL;
            lit := cast(*Code_Literal) expr;
            // Handle different literal types
            if lit._s64 {
                print_to_builder(builder, "%", lit._s64);
            } else if lit._float64 != 0 {
                print_to_builder(builder, "%", lit._float64);
            }

        case .IDENT;
            ident := cast(*Code_Ident) expr;
            print_to_builder(builder, "%", ident.name);

        case;
            // Default: just add placeholder
            print_to_builder(builder, "0");
    }
}