//this is located at modules/Drawing_Primitives/module.jai
#import "Math";
draw_primitive: Draw_Primitive_Procs;
Draw_Primitive_Procs :: struct {
    start_recording := () {};
    end_recording := () {};
    //default colors
    RED:Vector4: .{1,1,0,1}; GREEN:Vector4: .{0,1,0,1}; BLUE:Vector4: .{0,0,1,1}; YELLOW:Vector4: .{1,1,0,1}; WHITE:Vector4: .{1,1,1,1};
    //procedure pointers to be replaced
    line          := (start : Vector3, end: Vector3, color: Vector4 = .{0,1,0,1})  {};
    triangle_full := (p0: Vector3, p1: Vector3, p2: Vector3, c0 := Vector4.{1,1,1,1}, c1 := Vector4.{1,1,1,1}, c2 := Vector4.{1,1,1,1}, uv0 := Vector2.{}, uv1 := Vector2.{}, uv2 := Vector2.{}, normal := Vector3.{z=1}) {};
    triangle      := (v1: Vector3, v2: Vector3, v3: Vector3, color_vec: Vector4) {};
    quad          := (p0: Vector2, p1: Vector2, p2: Vector2, p3: Vector2, color := Vector4.{1,1,1,1}, uv0 := Vector2.{0,0}, uv1 := Vector2.{1,0}, uv2 := Vector2.{1,1}, uv3 := Vector2.{0, 1}) {};

    WorldToScreen := (position: Vector3) -> Vector3, is_inside_frustrum: bool {return .{}, false;};
    doLabel := (text: string, pos: Vector3)  {};
    doText := (text: string, pos: Vector3)  {};
    //helpers
    get_normal_from_normal :: (n: Vector3) -> Vector3 {
        t1 : Vector3 = ---;
        //trick to get a normal from a normal while avoiding degenerated cases.
        if abs(n.x) >= 0.57735 //sqrt of 3
            t1 = .{n.y, -n.x, 0.0};
        else
            t1 = .{0.0, n.z, -n.y};
            return normalize(t1);
    }

    //procedures that use the above primitives, inside this struct just to be able to use dot accessor
    vector :: (start: Vector3, vector: Vector3, color: Vector4) {
        draw_primitive.line(start, start + vector, color);
    }

    X :: (center: Vector3, size: float = 1.0, normal: Vector3 = .{0,1,0}, color: Vector4 = .{.5,.7,.5,1}) {
        horizontal := get_normal_from_normal(normal);
        vertical := cross_product(normal, horizontal);

        f := size / (1.51 * 2);
        start1 := center - f * (horizontal + vertical);
        end1 := center + f * (horizontal + vertical);
        start2 := center + f * (horizontal - vertical);
        end2 := center - f * (horizontal - vertical);

        draw_primitive.line(start1, end1, color);
        draw_primitive.line(start2, end2, color);
    }

    plus :: (center: Vector3, size: float = 1.0, normal: Vector3 = .{0,1,0}, color: Vector4 = .{.5,.7,.5,1}) {
        horizontal := get_normal_from_normal(normal);
        vertical := cross_product(normal, horizontal);

        half_size := size * 0.5;
        draw_primitive.line(center - horizontal * half_size, center + horizontal * half_size, color);
        draw_primitive.line(center - vertical * half_size, center + vertical * half_size, color);
    }

    circle :: (center: Vector3, radius: float = 0.4, segments: int = 4, normal: Vector3 = .{0,1,0}, color: Vector4 = .{.5,.7,.5,1}) {
        horizontal := get_normal_from_normal(normal);
        vertical := cross_product(normal, horizontal);

        prev_point := center + horizontal * radius;

        for i: 1..segments {
            angle := (cast(float)i / cast(float)segments) * 2.0 * PI;
            x := cos(angle) * radius;
            y := sin(angle) * radius;

            point := center + horizontal * x + vertical * y;
            draw_primitive.line(prev_point, point, color);
            prev_point = point;
        }
    }

    square :: (center: Vector3, size: float = 0.9, normal: Vector3 = .{0,1,0}, color: Vector4 = .{.5,.7,.5,1}) {
        horizontal := get_normal_from_normal(normal);
        vertical := cross_product(normal, horizontal);

        half_size := size * 0.5;

        p0 := center - horizontal * half_size - vertical * half_size;
        p1 := center + horizontal * half_size - vertical * half_size;
        p2 := center + horizontal * half_size + vertical * half_size;
        p3 := center - horizontal * half_size + vertical * half_size;

        draw_primitive.line(p0, p1, color);
        draw_primitive.line(p1, p2, color);
        draw_primitive.line(p2, p3, color);
        draw_primitive.line(p3, p0, color);
    }


    polygon :: (vertices: [] Vector3, color := Vector4.{ 1, 1, 1, 1 }) {
        for j: 1..vertices.count - 2 {
            k := (j + 1) % vertices.count;

            v0 := vertices[0];
            v1 := vertices[j];
            v2 := vertices[k];
            draw_primitive.triangle(.{ v0.x, v0.y, v0.z }, .{ v1.x, v1.y, v1.z }, .{ v2.x, v2.y, v2.z }, color);
        }
    }


    wireframe_box :: (center: Vector3, size: Vector3 = .{0.9,0.9,0.9}, color: Vector4 = .{.5,.7,.5,1}) {
        half := size * 0.5;

        // Bottom face
        p0 := center + .{-half.x, -half.y, -half.z};
        p1 := center + .{ half.x, -half.y, -half.z};
        p2 := center + .{ half.x, -half.y,  half.z};
        p3 := center + .{-half.x, -half.y,  half.z};

        // Top face
        p4 := center + .{-half.x, half.y, -half.z};
        p5 := center + .{ half.x, half.y, -half.z};
        p6 := center + .{ half.x, half.y,  half.z};
        p7 := center + .{-half.x, half.y,  half.z};

        // Bottom
        draw_primitive.line(p0, p1, color);
        draw_primitive.line(p1, p2, color);
        draw_primitive.line(p2, p3, color);
        draw_primitive.line(p3, p0, color);

        // Top
        draw_primitive.line(p4, p5, color);
        draw_primitive.line(p5, p6, color);
        draw_primitive.line(p6, p7, color);
        draw_primitive.line(p7, p4, color);

        // Verticals
        draw_primitive.line(p0, p4, color);
        draw_primitive.line(p1, p5, color);
        draw_primitive.line(p2, p6, color);
        draw_primitive.line(p3, p7, color);
    }

    arrow :: (start: Vector3, vector: Vector3= .{0,1,0}, head_size: float = 0.2, color: Vector4 = .{.5,.7,.5,1}) {
        end := vector+start;
        draw_primitive.line(start, end, color);

        direction := normalize(end - start);
        perpendicular := get_normal_from_normal(direction);
        other_perp := cross_product(direction, perpendicular);

        head_base := end - direction * head_size;
        head_offset := head_size * 0.3;

        head1 := head_base + perpendicular * head_offset;
        head2 := head_base - perpendicular * head_offset;
        head3 := head_base + other_perp * head_offset;
        head4 := head_base - other_perp * head_offset;

        draw_primitive.line(end, head1, color);
        draw_primitive.line(end, head2, color);
        draw_primitive.line(end, head3, color);
        draw_primitive.line(end, head4, color);
    }

    grid :: (plane_origin: Vector3, plane_normal: Vector3, min_for: s64, max_for: s64 , density:float=1.0, color := Vector4.{0.4, 0.4, 0.4, 1}) {
        horizontal := get_normal_from_normal(plane_normal);
        vertical := cross_product(plane_normal, horizontal);
        for i : min_for*density.(s64)..max_for*density.(s64) {
            {
                start := plane_origin + horizontal * i.(float)/density + min_for.(float)*vertical;
                end :=  plane_origin + horizontal * i.(float)/density + max_for.(float)*vertical;
      	        draw_primitive.line(start, end, color);
      	        if i == min_for*density.(s64) || i==max_for*density.(s64)-1 draw_primitive.arrow(start,plane_normal*0.1,head_size=0.01, color=color);
      	        if i == min_for*density.(s64) || i==max_for*density.(s64)-1 draw_primitive.arrow(end,plane_normal*0.1,head_size=0.01, color=color);
      	    }
            {
                start := plane_origin + vertical * i.(float)/density + min_for.(float)*horizontal;
                end :=  plane_origin + vertical * i.(float)/density + max_for.(float)*horizontal;
      	        draw_primitive.line(start, end, color);
      	        if i == min_for*density.(s64) || i==max_for*density.(s64)-1 draw_primitive.arrow(start,plane_normal*0.1,head_size=0.01, color=color);
      	        if i == min_for*density.(s64) || i==max_for*density.(s64)-1 draw_primitive.arrow(end,plane_normal*0.1,head_size=0.01, color=color);
      	    }
        }
    }

    grid :: (plane_origin: Vector3, plane_normal: Vector3, area_length: s64 = 10, density:float=1.0, color := Vector4.{0.4, 0.4, 0.4, 1}) {
        draw_primitive.grid(plane_origin, plane_normal,-area_length, area_length , density, color);
    }



}




colors :: Vector4.[
    .{1,0,0,1},
    .{0,1,0,1},
    .{0,1,1,1},
    .{.75,.5,0,1},
    .{0,.5,.5,1},
    .{.5,0,.5,1},
    .{.5,.75,.5,1},
    .{1,1,1,1},
    .{0.1,0.73,0.8,1},
];

