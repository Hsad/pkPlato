package game

import rl "vendor:raylib"
import "core:fmt"
import "core:math"
import "core:math/linalg"
Fling :: struct {
    center: point_idx,
    character_type: Character_Type,
    rotation: rl.Vector3,
    frst_pos: rl.Vector3,
    thrd_pos: rl.Vector3,
    ideal_camera_pos: rl.Vector3,
    look_target: rl.Vector3,
    points: [dynamic]point_idx,
    crouch_amount: f32,
    fuel_locked: bool,
}
Character_Type :: enum {
    Tetrahedron,
    Cube,
    Octahedron,
    Dodecahedron,
    Icosahedron,
}


get_fling_camera :: proc() -> rl.Camera3D {
	if g.camera.using_first_person {
		look_dir := linalg.normalize(g.fling.look_target - g.fling.frst_pos)
		return {
			position = g.fling.frst_pos - look_dir * 1,
			target = g.fling.look_target,
			up = {0, 1, 0},
			fovy = 75,
			projection = .PERSPECTIVE,
		}
	} else {
		return {
			position = g.fling.thrd_pos,
			target = g.pbd_world.points[g.fling.center].position,
			up = {0, 1, 0},
			fovy = 45,
			projection = .PERSPECTIVE,
		}
	}
}

create_fling :: proc(pbd_world: ^PBD_World) -> Fling {
    fmt.println("fling")

    fling := Fling{}
	center := pbd_create_point(pbd_world, rl.Vector3{GRID_LEN*TILE_SIZE/2, 100, GRID_LEN*TILE_SIZE/2})
	fling.points = make([dynamic]point_idx)

    if g.fling.character_type == .Tetrahedron {
        create_tetrahedron(pbd_world, center, &fling.points)
        center.mass = 0.5
    } else if g.fling.character_type == .Cube {
        create_cube(pbd_world, center, &fling.points)
    } else if g.fling.character_type == .Octahedron {
        create_octahedron(pbd_world, center, &fling.points)
    } else if g.fling.character_type == .Dodecahedron {
        create_dodecahedron(pbd_world, center, &fling.points)
    } else if g.fling.character_type == .Icosahedron {
        create_icosahedron(pbd_world, center, &fling.points)
    }

	fling.center = center.id

	return fling
}

simulate_fling :: proc(fling: ^Fling) {

    // Update rotation based on look input
    look_sensitivity :: 2.0
    look_x :f32 = 0
    look_y :f32 = 0
    if !rl.IsGamepadAvailable(0) { 
        //mouse look
        look_x = g.input_intent.look_x * 0.2
        look_y = g.input_intent.look_y * -0.1
    } else {
        //gamepad look
        look_x = g.input_intent.look_x 
        look_y = g.input_intent.look_y
        if math.abs(look_x) < 0.3 {look_x = 0}
        if math.abs(look_y) < 0.3 {look_y = 0}
    }
    fling.rotation.y += look_x * -look_sensitivity * rl.GetFrameTime() // Horizontal rotation
    fling.rotation.x += look_y * look_sensitivity * rl.GetFrameTime() // Vertical rotation

    // Clamp vertical rotation to avoid over-rotation
    max_vertical_angle :: math.PI * 0.4 // About 72 degrees up/down
    fling.rotation.x = clamp(fling.rotation.x, -max_vertical_angle, max_vertical_angle)

    // Movement
    move_speed :: 1
    move_x := g.input_intent.move_x
    move_z := g.input_intent.move_z
    if math.abs(move_x) < 0.3 {move_x = 0}
    if math.abs(move_z) < 0.3 {move_z = 0}

    // Get forward vector from rotation (ignoring vertical component)
    forward := rl.Vector3{
        math.sin(fling.rotation.y),
        0,
        math.cos(fling.rotation.y),
    }
    // Get right vector by rotating forward 90 degrees
    right := rl.Vector3{
        math.sin(fling.rotation.y + math.PI/2),
        0,
        math.cos(fling.rotation.y + math.PI/2),
    }

    
    // Calculate constraint value (how far from desired length)
    g.fling.crouch_amount = ((g.input_intent.trigger_left + 1) + (g.input_intent.trigger_right + 1)) / 4


    
    center_point := &g.pbd_world.points[g.fling.center]
    // distance to ground

    center := center_point.position
    _, _, height := get_box_at_position(center)
    height_from_ground := center.y - height

    ground_mod :f32= 4 if height_from_ground < 4 else 1.5
    // combine forward/back and right/left movement
    move_direction := forward * -move_z + right * -move_x
    move_velocity := move_direction * move_speed * ground_mod

    if g.fuel > 1 && !g.fling.fuel_locked { // if we have fuel, use it
        center_point.velocity += move_velocity

        move_fuel_cost := linalg.length(move_velocity) * rl.GetFrameTime() * height_from_ground / ground_mod

        if g.input_intent.button_a {
            center_point.velocity.y += 2
            g.fuel -= 1
        }

        //fmt.println("move_fuel_cost", move_fuel_cost)
        if center_point.position.y > 0 {  // being below ground was giving negative fuel
            g.fuel -= min(move_fuel_cost, 1)
        }

        if g.fuel < 0 {
            g.fuel = 0
        }

    } else if height_from_ground < 4 { // if we don't have fuel, move on the ground
        center_point.velocity += move_velocity / 2 // and move slower
    }

    if center_point.position.y < 0 {
        g.fuel -= 0.27
        if g.fuel < 0 {g.fuel = 0}
    }

    if g.fuel < 50 && linalg.length(move_velocity) < 0.2 {
        g.fuel += 1 * rl.GetFrameTime() 
    }

    //if g.fuel == 0 {
    //    g.player_dead = true
    //    g.pause = true
    //}

    if g.restart {
        restart_fling(g.pbd_world.points[g.fling.center].position, g.pbd_world.points[g.fling.center].prev_pos)
        g.fuel = 490
        g.restart = false
        g.player_dead = false
        g.pause = false
    }

    if g.input_intent.bumper_right && !g.input_previous.bumper_right {
        g.camera.using_first_person = !g.camera.using_first_person
    }

    if g.input_intent.bumper_left && !g.input_previous.bumper_left {
        g.fling.fuel_locked = !g.fling.fuel_locked
    }


    if g.input_intent.button_y && !g.input_previous.button_y && g.fuel > 20 * 100 + 100{
        // switch to new solid
        center := pbd_create_point(g.pbd_world, g.pbd_world.points[g.fling.center].position)
        if g.fling.character_type == .Tetrahedron {
            fmt.println("cube")
            create_cube(g.pbd_world, center, &g.fling.points)
            g.fling.character_type = .Cube
        } else if g.fling.character_type == .Cube {
            fmt.println("octahedron")
            create_octahedron(g.pbd_world, center, &g.fling.points)
            g.fling.character_type = .Octahedron
        } else if g.fling.character_type == .Octahedron {
            fmt.println("dodecahedron")
            create_dodecahedron(g.pbd_world, center, &g.fling.points)
            g.fling.character_type = .Dodecahedron
        } else if g.fling.character_type == .Dodecahedron {
            fmt.println("icosahedron")
            create_icosahedron(g.pbd_world, center, &g.fling.points)
            g.fling.character_type = .Icosahedron
        } else if g.fling.character_type == .Icosahedron {
            fmt.println("tetrahedron")
            create_tetrahedron(g.pbd_world, center, &g.fling.points)
            center.mass = 0.5
            g.fling.character_type = .Tetrahedron
        }
        g.fling.center = center.id

    }

    if g.input_intent.button_b && !g.input_previous.button_b && g.fuel > 20 * 100 + 100{
        g.pause = !g.pause
    }

    if g.input_intent.button_x && g.fuel > 20 * 100 + 10{
        g.fuel += 10
    }

    // switch to new solid based on fuel
    if g.fuel < 4 * 100 + 100 {
        if g.fling.character_type != .Tetrahedron {
            g.fling.character_type = .Tetrahedron
            restart_fling(g.pbd_world.points[g.fling.center].position, g.pbd_world.points[g.fling.center].prev_pos)
        }
    } else if g.fuel < 6 * 100 + 100 {
        if g.fling.character_type != .Cube {
            g.fling.character_type = .Cube
            restart_fling(g.pbd_world.points[g.fling.center].position, g.pbd_world.points[g.fling.center].prev_pos)
        }
    } else if g.fuel < 8 * 100 + 100 {
        if g.fling.character_type != .Octahedron {
            g.fling.character_type = .Octahedron
            restart_fling(g.pbd_world.points[g.fling.center].position, g.pbd_world.points[g.fling.center].prev_pos)
        }
    } else if g.fuel < 12 * 100 + 100 {
        if g.fling.character_type != .Dodecahedron {
            g.fling.character_type = .Dodecahedron
            restart_fling(g.pbd_world.points[g.fling.center].position, g.pbd_world.points[g.fling.center].prev_pos)
        }
    } else if g.fuel < 20 * 100 + 100 {
        if g.fling.character_type != .Icosahedron {
            g.fling.character_type = .Icosahedron
            restart_fling(g.pbd_world.points[g.fling.center].position, g.pbd_world.points[g.fling.center].prev_pos)
        }
    } 

    if rl.IsGamepadAvailable(0) {
        //start button
        if rl.IsGamepadButtonPressed(0, .MIDDLE_RIGHT) {
            g.show_instructions = !g.show_instructions
        }
    }


    
}

restart_fling :: proc(start: rl.Vector3, prev_pos: rl.Vector3 = {0,0,0}) {
    center := pbd_create_point(g.pbd_world, start)
    if prev_pos != {0,0,0} {
        center.prev_pos = prev_pos
    }
    if g.fling.character_type == .Tetrahedron {
        fmt.println("tetrahedron")
        create_tetrahedron(g.pbd_world, center, &g.fling.points)
        center.mass = 0.5
        g.fling.character_type = .Tetrahedron
    } else if g.fling.character_type == .Cube {
        fmt.println("cube")
        create_cube(g.pbd_world, center, &g.fling.points)
        g.fling.character_type = .Cube
    } else if g.fling.character_type == .Octahedron {
        fmt.println("octahedron")
        create_octahedron(g.pbd_world, center, &g.fling.points)
        g.fling.character_type = .Octahedron
    } else if g.fling.character_type == .Dodecahedron {
        fmt.println("dodecahedron")
        create_dodecahedron(g.pbd_world, center, &g.fling.points)
        g.fling.character_type = .Dodecahedron
    } else if g.fling.character_type == .Icosahedron {
        fmt.println("icosahedron")
        create_icosahedron(g.pbd_world, center, &g.fling.points)
        g.fling.character_type = .Icosahedron
    }
    g.fling.center = center.id
}

draw_fling :: proc() {
    // rotation
    rl.DrawLine3D(g.pbd_world.points[g.fling.center].position, 
        {g.pbd_world.points[g.fling.center].position.x + math.cos(g.fling.rotation.y), 
         g.pbd_world.points[g.fling.center].position.y, 
         g.pbd_world.points[g.fling.center].position.z + math.sin(g.fling.rotation.y)}, rl.RED)
}


create_tetrahedron :: proc(pbd_world: ^PBD_World, center: ^Point, points_array: ^[dynamic]point_idx) {
    radius :f32= 3
    cent :f32= math.sqrt_f32(6)/4
    
    // Create vertices of a regular tetrahedron
    // The center point is at the barycenter (centroid)
    p0 := pbd_create_point(pbd_world, center.position + rl.Vector3{radius * math.sqrt_f32(8/9), 0, -radius/3})
    p1 := pbd_create_point(pbd_world, center.position + rl.Vector3{-radius * math.sqrt_f32(2/9), 0, radius * math.sqrt_f32(2/3)})
    p2 := pbd_create_point(pbd_world, center.position + rl.Vector3{-radius * math.sqrt_f32(2/9), 0, -radius * math.sqrt_f32(2/3)})
    p3 := pbd_create_point(pbd_world, center.position + rl.Vector3{0, radius, 0})

    // Create springs between all vertices to maintain shape
    pbd_create_spring(pbd_world, p0.id, p1.id, radius)
    pbd_create_spring(pbd_world, p0.id, p2.id, radius)
    pbd_create_spring(pbd_world, p0.id, p3.id, radius)
    pbd_create_spring(pbd_world, p1.id, p2.id, radius)
    pbd_create_spring(pbd_world, p1.id, p3.id, radius)
    pbd_create_spring(pbd_world, p2.id, p3.id, radius)

    pbd_create_spring(pbd_world, p0.id, center.id, radius*cent)
    pbd_create_spring(pbd_world, p1.id, center.id, radius*cent)
    pbd_create_spring(pbd_world, p2.id, center.id, radius*cent)
    pbd_create_spring(pbd_world, p3.id, center.id, radius*cent)

    // clear previous points
    clear(points_array)
    append(points_array, p0.id)
    append(points_array, p1.id)
    append(points_array, p2.id)
    append(points_array, p3.id)
}

create_cube :: proc(pbd_world: ^PBD_World, center: ^Point, points_array: ^[dynamic]point_idx) {
    /// create a cube
    radius :f32= 3.0
    cent :f32= math.sqrt_f32(3)/2
    // create 8 points
    p0 := pbd_create_point(pbd_world, center.position + rl.Vector3{radius, radius, radius})
    p1 := pbd_create_point(pbd_world, center.position + rl.Vector3{radius, radius, -radius})
    p2 := pbd_create_point(pbd_world, center.position + rl.Vector3{radius, -radius, radius})
    p3 := pbd_create_point(pbd_world, center.position + rl.Vector3{radius, -radius, -radius})
    p4 := pbd_create_point(pbd_world, center.position + rl.Vector3{-radius, radius, radius})
    p5 := pbd_create_point(pbd_world, center.position + rl.Vector3{-radius, radius, -radius})
    p6 := pbd_create_point(pbd_world, center.position + rl.Vector3{-radius, -radius, radius})
    p7 := pbd_create_point(pbd_world, center.position + rl.Vector3{-radius, -radius, -radius})

    // clear previous points
    clear(points_array)
    append(points_array, p0.id)
    append(points_array, p1.id)
    append(points_array, p2.id)
    append(points_array, p3.id)
    append(points_array, p4.id)
    append(points_array, p5.id)
    append(points_array, p6.id)
    append(points_array, p7.id)

    // create 12 springs
    pbd_create_spring(pbd_world, p0.id, p1.id, radius)
    pbd_create_spring(pbd_world, p0.id, p2.id, radius)
    pbd_create_spring(pbd_world, p1.id, p3.id, radius)
    pbd_create_spring(pbd_world, p2.id, p3.id, radius)


    pbd_create_spring(pbd_world, p4.id, p5.id, radius)
    pbd_create_spring(pbd_world, p4.id, p6.id, radius)
    pbd_create_spring(pbd_world, p5.id, p7.id, radius)
    pbd_create_spring(pbd_world, p6.id, p7.id, radius)

    pbd_create_spring(pbd_world, p0.id, p7.id, radius)
    pbd_create_spring(pbd_world, p1.id, p6.id, radius)
    pbd_create_spring(pbd_world, p2.id, p5.id, radius)
    pbd_create_spring(pbd_world, p3.id, p4.id, radius)

    // 4 cross springs
    pbd_create_spring(pbd_world, p0.id, p4.id, radius*cent*2)
    pbd_create_spring(pbd_world, p1.id, p5.id, radius*cent*2)
    pbd_create_spring(pbd_world, p2.id, p6.id, radius*cent*2)
    pbd_create_spring(pbd_world, p3.id, p7.id, radius*cent*2)
    // create 8 springs to center
    pbd_create_spring(pbd_world, p0.id, center.id, radius*cent)
    pbd_create_spring(pbd_world, p1.id, center.id, radius*cent)
    pbd_create_spring(pbd_world, p2.id, center.id, radius*cent)
    pbd_create_spring(pbd_world, p3.id, center.id, radius*cent)
    pbd_create_spring(pbd_world, p4.id, center.id, radius*cent)
    pbd_create_spring(pbd_world, p5.id, center.id, radius*cent)
    pbd_create_spring(pbd_world, p6.id, center.id, radius*cent)
    pbd_create_spring(pbd_world, p7.id, center.id, radius*cent)
}

create_octahedron :: proc(pbd_world: ^PBD_World, center: ^Point, points_array: ^[dynamic]point_idx) {
    /// create an octahedron
    radius :f32= 3.0
    cent :f32= math.sqrt_f32(2)/2
    
    // Create vertices of a regular octahedron
    p0 := pbd_create_point(pbd_world, center.position + rl.Vector3{radius, 0, 0})
    p1 := pbd_create_point(pbd_world, center.position + rl.Vector3{0, radius, 0})
    p2 := pbd_create_point(pbd_world, center.position + rl.Vector3{0, 0, radius})
    p3 := pbd_create_point(pbd_world, center.position + rl.Vector3{-radius, 0, 0})
    p4 := pbd_create_point(pbd_world, center.position + rl.Vector3{0, -radius, 0})
    p5 := pbd_create_point(pbd_world, center.position + rl.Vector3{0, 0, -radius})

    // clear previous points
    clear(points_array)
    append(points_array, p0.id)
    append(points_array, p1.id)
    append(points_array, p2.id)
    append(points_array, p3.id)
    append(points_array, p4.id)
    append(points_array, p5.id)

    // create 12 springs
    pbd_create_spring(pbd_world, p0.id, p1.id, radius)
    pbd_create_spring(pbd_world, p0.id, p2.id, radius)
    pbd_create_spring(pbd_world, p0.id, p3.id, radius)
    pbd_create_spring(pbd_world, p0.id, p4.id, radius)

    pbd_create_spring(pbd_world, p1.id, p2.id, radius)
    pbd_create_spring(pbd_world, p2.id, p3.id, radius)
    pbd_create_spring(pbd_world, p3.id, p4.id, radius)
    pbd_create_spring(pbd_world, p4.id, p1.id, radius)

    pbd_create_spring(pbd_world, p5.id, p1.id, radius)
    pbd_create_spring(pbd_world, p5.id, p2.id, radius)
    pbd_create_spring(pbd_world, p5.id, p3.id, radius)
    pbd_create_spring(pbd_world, p5.id, p4.id, radius)
    
    pbd_create_spring(pbd_world, p0.id, center.id, radius*cent)
    pbd_create_spring(pbd_world, p1.id, center.id, radius*cent)
    pbd_create_spring(pbd_world, p2.id, center.id, radius*cent)
    pbd_create_spring(pbd_world, p3.id, center.id, radius*cent)
    pbd_create_spring(pbd_world, p4.id, center.id, radius*cent)
    pbd_create_spring(pbd_world, p5.id, center.id, radius*cent)

    pbd_create_spring(pbd_world, p0.id, p5.id, radius*cent*2)
    pbd_create_spring(pbd_world, p1.id, p3.id, radius*cent*2)
    pbd_create_spring(pbd_world, p2.id, p4.id, radius*cent*2)
}


create_dodecahedron :: proc(pbd_world: ^PBD_World, center: ^Point, points_array: ^[dynamic]point_idx) {
    /// create a dodecahedron
    radius :f32= 3.0
    //cent :f32= math.sqrt_f32(3*(5+math.sqrt_f32(5)))/4

    off1 :f32= 0.382 * radius
    off2 :f32= 0.6182 * radius
    
    // Create vertices of a regular dodecahedron
    // The center point is at the barycenter (centroid)
    // 8 points, 
    p0 := pbd_create_point(pbd_world, center.position + rl.Vector3{off1, radius, 0})
    p1 := pbd_create_point(pbd_world, center.position + rl.Vector3{-off1, radius, 0})
    p2 := pbd_create_point(pbd_world, center.position + rl.Vector3{off1, -radius, 0})
    p3 := pbd_create_point(pbd_world, center.position + rl.Vector3{-off1, -radius, 0})

    p4 := pbd_create_point(pbd_world, center.position + rl.Vector3{radius, 0, off1})
    p5 := pbd_create_point(pbd_world, center.position + rl.Vector3{radius, 0, -off1})
    p6 := pbd_create_point(pbd_world, center.position + rl.Vector3{-radius, 0, off1})
    p7 := pbd_create_point(pbd_world, center.position + rl.Vector3{-radius, 0, -off1})

    p8 := pbd_create_point(pbd_world, center.position + rl.Vector3{0, off1, radius})
    p9 := pbd_create_point(pbd_world, center.position + rl.Vector3{0, -off1, radius})
    p10 := pbd_create_point(pbd_world, center.position + rl.Vector3{0, off1, -radius})
    p11 := pbd_create_point(pbd_world, center.position + rl.Vector3{0, -off1, -radius})
    //create cube points with offset of off2
    p12 := pbd_create_point(pbd_world, center.position + rl.Vector3{off2, off2, off2})
    p13 := pbd_create_point(pbd_world, center.position + rl.Vector3{off2, off2, -off2})
    p14 := pbd_create_point(pbd_world, center.position + rl.Vector3{off2, -off2, off2})
    p15 := pbd_create_point(pbd_world, center.position + rl.Vector3{off2, -off2, -off2})

    p16 := pbd_create_point(pbd_world, center.position + rl.Vector3{-off2, off2, off2})
    p17 := pbd_create_point(pbd_world, center.position + rl.Vector3{-off2, off2, -off2})
    p18 := pbd_create_point(pbd_world, center.position + rl.Vector3{-off2, -off2, off2})
    p19 := pbd_create_point(pbd_world, center.position + rl.Vector3{-off2, -off2, -off2})

    // clear previous points
    clear(points_array)
    append(points_array, p0.id)
    append(points_array, p1.id)
    append(points_array, p2.id)
    append(points_array, p3.id)
    append(points_array, p4.id)
    append(points_array, p5.id)
    append(points_array, p6.id)
    append(points_array, p7.id)
    append(points_array, p8.id)
    append(points_array, p9.id)
    append(points_array, p10.id)
    append(points_array, p11.id)
    append(points_array, p12.id)
    append(points_array, p13.id)
    append(points_array, p14.id)
    append(points_array, p15.id)
    append(points_array, p16.id)
    append(points_array, p17.id)
    append(points_array, p18.id)
    append(points_array, p19.id)
    // create 30 springs
    // Othogonal flat edges
    pbd_create_spring(pbd_world, p0.id, p1.id, radius)
    pbd_create_spring(pbd_world, p2.id, p3.id, radius)
    pbd_create_spring(pbd_world, p4.id, p5.id, radius)
    pbd_create_spring(pbd_world, p6.id, p7.id, radius)
    pbd_create_spring(pbd_world, p8.id, p9.id, radius)
    pbd_create_spring(pbd_world, p10.id, p11.id, radius)
    //
    // Connect cube points to closest orthogonal points
    // p12 (+off2, +off2, +off2) connects to p0 (+off1,+r,0), p4 (+r,0,+off1), p8 (0,+off1,+r)
    pbd_create_spring(pbd_world, p12.id, p0.id, radius)
    pbd_create_spring(pbd_world, p12.id, p4.id, radius) 
    pbd_create_spring(pbd_world, p12.id, p8.id, radius)

    // p13 (+off2, +off2, -off2) connects to p0 (+off1,+r,0), p5 (+r,0,-off1), p10 (0,+off1,-r)
    pbd_create_spring(pbd_world, p13.id, p0.id, radius)
    pbd_create_spring(pbd_world, p13.id, p5.id, radius)
    pbd_create_spring(pbd_world, p13.id, p10.id, radius)

    // p14 (+off2, -off2, +off2) connects to p2 (+off1,-r,0), p4 (+r,0,+off1), p9 (0,-off1,+r)
    pbd_create_spring(pbd_world, p14.id, p2.id, radius)
    pbd_create_spring(pbd_world, p14.id, p4.id, radius)
    pbd_create_spring(pbd_world, p14.id, p9.id, radius)

    // p15 (+off2, -off2, -off2) connects to p2 (+off1,-r,0), p5 (+r,0,-off1), p11 (0,-off1,-r)
    pbd_create_spring(pbd_world, p15.id, p2.id, radius)
    pbd_create_spring(pbd_world, p15.id, p5.id, radius)
    pbd_create_spring(pbd_world, p15.id, p11.id, radius)

    // p16 (-off2, +off2, +off2) connects to p1 (-off1,+r,0), p6 (-r,0,+off1), p8 (0,+off1,+r)
    pbd_create_spring(pbd_world, p16.id, p1.id, radius)
    pbd_create_spring(pbd_world, p16.id, p6.id, radius)
    pbd_create_spring(pbd_world, p16.id, p8.id, radius)

    // p17 (-off2, +off2, -off2) connects to p1 (-off1,+r,0), p7 (-r,0,-off1), p10 (0,+off1,-r)
    pbd_create_spring(pbd_world, p17.id, p1.id, radius)
    pbd_create_spring(pbd_world, p17.id, p7.id, radius)
    pbd_create_spring(pbd_world, p17.id, p10.id, radius)

    // p18 (-off2, -off2, +off2) connects to p3 (-off1,-r,0), p6 (-r,0,+off1), p9 (0,-off1,+r)
    pbd_create_spring(pbd_world, p18.id, p3.id, radius)
    pbd_create_spring(pbd_world, p18.id, p6.id, radius)
    pbd_create_spring(pbd_world, p18.id, p9.id, radius)

    // p19 (-off2, -off2, -off2) connects to p3 (-off1,-r,0), p7 (-r,0,-off1), p11 (0,-off1,-r)
    pbd_create_spring(pbd_world, p19.id, p3.id, radius)
    pbd_create_spring(pbd_world, p19.id, p7.id, radius)
    pbd_create_spring(pbd_world, p19.id, p11.id, radius)


    half_dist := linalg.length(p0.position - center.position)
    // connect every point to center
    pbd_create_spring(pbd_world, p0.id, center.id, half_dist)
    pbd_create_spring(pbd_world, p1.id, center.id, half_dist)
    pbd_create_spring(pbd_world, p2.id, center.id, half_dist)
    pbd_create_spring(pbd_world, p3.id, center.id, half_dist)
    pbd_create_spring(pbd_world, p4.id, center.id, half_dist)
    pbd_create_spring(pbd_world, p5.id, center.id, half_dist)
    pbd_create_spring(pbd_world, p6.id, center.id, half_dist)
    pbd_create_spring(pbd_world, p7.id, center.id, half_dist)
    pbd_create_spring(pbd_world, p8.id, center.id, half_dist)
    pbd_create_spring(pbd_world, p9.id, center.id, half_dist)
    pbd_create_spring(pbd_world, p10.id, center.id, half_dist)
    pbd_create_spring(pbd_world, p11.id, center.id, half_dist)
    pbd_create_spring(pbd_world, p12.id, center.id, half_dist)
    pbd_create_spring(pbd_world, p13.id, center.id, half_dist)
    pbd_create_spring(pbd_world, p14.id, center.id, half_dist)
    pbd_create_spring(pbd_world, p15.id, center.id, half_dist)
    pbd_create_spring(pbd_world, p16.id, center.id, half_dist)
    pbd_create_spring(pbd_world, p17.id, center.id, half_dist)
    pbd_create_spring(pbd_world, p18.id, center.id, half_dist)
    pbd_create_spring(pbd_world, p19.id, center.id, half_dist)


    full_dist := linalg.length(p0.position - p3.position)
    // cross bars
    pbd_create_spring(pbd_world, p0.id, p3.id, full_dist)
    pbd_create_spring(pbd_world, p1.id, p2.id, full_dist)
    pbd_create_spring(pbd_world, p4.id, p7.id, full_dist)
    pbd_create_spring(pbd_world, p5.id, p6.id, full_dist)
    pbd_create_spring(pbd_world, p8.id, p11.id, full_dist)
    pbd_create_spring(pbd_world, p9.id, p10.id, full_dist)
    
}


create_icosahedron :: proc(pbd_world: ^PBD_World, center: ^Point, points_array: ^[dynamic]point_idx) {
    /// create an icosahedron
    radius :f32= 3.0
    cent :f32= 0.951
    
    off := 0.6180 * radius

    // Create the two rings of 5 points each
    golden_ratio :f32= (1.0 + math.sqrt(f32(5.0))) / 2.0
    fmt.println(golden_ratio)

    p0 := pbd_create_point(pbd_world, center.position + rl.Vector3{off, radius, 0})  // Top
    p1 := pbd_create_point(pbd_world, center.position + rl.Vector3{-off, radius, 0})  // Top
    p2 := pbd_create_point(pbd_world, center.position + rl.Vector3{off, -radius, 0})  // Top
    p3 := pbd_create_point(pbd_world, center.position + rl.Vector3{-off, -radius, 0})  // Top

    p4 := pbd_create_point(pbd_world, center.position + rl.Vector3{radius, 0, off})  // Top
    p5 := pbd_create_point(pbd_world, center.position + rl.Vector3{radius, 0, -off})  // Top
    p6 := pbd_create_point(pbd_world, center.position + rl.Vector3{-radius, 0, off})  // Top
    p7 := pbd_create_point(pbd_world, center.position + rl.Vector3{-radius, 0, -off})  // Top

    p8 := pbd_create_point(pbd_world, center.position + rl.Vector3{0, off, radius})  // Top
    p9 := pbd_create_point(pbd_world, center.position + rl.Vector3{0, -off, radius})  // Top
    p10 := pbd_create_point(pbd_world, center.position + rl.Vector3{0, off, -radius})  // Top
    p11 := pbd_create_point(pbd_world, center.position + rl.Vector3{0, -off, -radius})  // Top

    // clear previous points
    clear(points_array)
    append(points_array, p0.id)  // Top
    append(points_array, p1.id)  // Bottom
    append(points_array, p2.id)  // Top ring
    append(points_array, p3.id)
    append(points_array, p4.id)
    append(points_array, p5.id)
    append(points_array, p6.id)
    append(points_array, p7.id)  // Bottom ring
    append(points_array, p8.id)
    append(points_array, p9.id)
    append(points_array, p10.id)
    append(points_array, p11.id)

    // Create springs
    // Connect top point to top ring
    pbd_create_spring(pbd_world, p0.id, p2.id, radius)
    pbd_create_spring(pbd_world, p0.id, p3.id, radius)
    pbd_create_spring(pbd_world, p0.id, p4.id, radius)
    pbd_create_spring(pbd_world, p0.id, p5.id, radius)
    pbd_create_spring(pbd_world, p0.id, p6.id, radius)
    
    // Connect bottom point to bottom ring
    pbd_create_spring(pbd_world, p1.id, p7.id, radius)
    pbd_create_spring(pbd_world, p1.id, p8.id, radius)
    pbd_create_spring(pbd_world, p1.id, p9.id, radius)
    pbd_create_spring(pbd_world, p1.id, p10.id, radius)
    pbd_create_spring(pbd_world, p1.id, p11.id, radius)
    
    // Connect top ring points to each other
    pbd_create_spring(pbd_world, p2.id, p3.id, radius)
    pbd_create_spring(pbd_world, p3.id, p4.id, radius)
    pbd_create_spring(pbd_world, p4.id, p5.id, radius)
    pbd_create_spring(pbd_world, p5.id, p6.id, radius)
    pbd_create_spring(pbd_world, p6.id, p2.id, radius)
    
    // Connect bottom ring points to each other
    pbd_create_spring(pbd_world, p7.id, p8.id, radius)
    pbd_create_spring(pbd_world, p8.id, p9.id, radius)
    pbd_create_spring(pbd_world, p9.id, p10.id, radius)
    pbd_create_spring(pbd_world, p10.id, p11.id, radius)
    pbd_create_spring(pbd_world, p11.id, p7.id, radius)
    
    // Connect between rings
    pbd_create_spring(pbd_world, p2.id, p7.id, radius)
    pbd_create_spring(pbd_world, p2.id, p11.id, radius)
    pbd_create_spring(pbd_world, p3.id, p7.id, radius)
    pbd_create_spring(pbd_world, p3.id, p8.id, radius)
    pbd_create_spring(pbd_world, p4.id, p8.id, radius)
    pbd_create_spring(pbd_world, p4.id, p9.id, radius)
    pbd_create_spring(pbd_world, p5.id, p9.id, radius)
    pbd_create_spring(pbd_world, p5.id, p10.id, radius)
    pbd_create_spring(pbd_world, p6.id, p10.id, radius)
    pbd_create_spring(pbd_world, p6.id, p11.id, radius)
    
    // Connect all points to center
    pbd_create_spring(pbd_world, p0.id, center.id, radius*cent)
    pbd_create_spring(pbd_world, p1.id, center.id, radius*cent)
    pbd_create_spring(pbd_world, p2.id, center.id, radius*cent)
    pbd_create_spring(pbd_world, p3.id, center.id, radius*cent)
    pbd_create_spring(pbd_world, p4.id, center.id, radius*cent)
    pbd_create_spring(pbd_world, p5.id, center.id, radius*cent)
    pbd_create_spring(pbd_world, p6.id, center.id, radius*cent)
    pbd_create_spring(pbd_world, p7.id, center.id, radius*cent)
    pbd_create_spring(pbd_world, p8.id, center.id, radius*cent)
    pbd_create_spring(pbd_world, p9.id, center.id, radius*cent)
    pbd_create_spring(pbd_world, p10.id, center.id, radius*cent)
    pbd_create_spring(pbd_world, p11.id, center.id, radius*cent)
    
    // Cross struts
    pbd_create_spring(pbd_world, p0.id, p1.id, radius*cent*2)
    pbd_create_spring(pbd_world, p2.id, p9.id, radius*cent*2)
    pbd_create_spring(pbd_world, p3.id, p10.id, radius*cent*2)
    pbd_create_spring(pbd_world, p4.id, p11.id, radius*cent*2)
    pbd_create_spring(pbd_world, p5.id, p7.id, radius*cent*2)
    pbd_create_spring(pbd_world, p6.id, p8.id, radius*cent*2)
}

calc_fling_1st_person :: proc() {
	// Set camera position to player position with a slight height offset for eye level
	g.fling.frst_pos = g.pbd_world.points[g.fling.center].position + rl.Vector3{0, 1.7, 0}  // Typical eye height
	
	// Convert player rotation to a point on unit sphere
	// Assuming g.player.rotation contains pitch (x-axis) and yaw (y-axis) in radians
	pitch := g.fling.rotation.x
	yaw := g.fling.rotation.y
	
	// Calculate direction vector on unit sphere
	x := linalg.sin(yaw) * linalg.cos(pitch) * 1
	y := linalg.sin(pitch)
	z := linalg.cos(yaw) * linalg.cos(pitch) * 1
	direction := rl.Vector3{x, y, z}
	
	// Set look target as position + direction
	g.fling.look_target = g.fling.frst_pos + direction
}

calc_fling_3rd_person :: proc() {
    // Camera constants
    CAMERA_DISTANCE :f32 = 20.0
    CAMERA_HEIGHT :f32 = 10.0
    HORIZONTAL_SMOOTHING :f32: 5.0 // Faster horizontal movement
    VERTICAL_SMOOTHING :f32: 5.0  // Slower vertical movement
    
    // Calculate direction vector from rotation
    pitch := g.fling.rotation.x
    yaw := g.fling.rotation.y
    
    x := linalg.sin(yaw) * linalg.cos(pitch)
    y := linalg.sin(pitch) 
    z := linalg.cos(yaw) * linalg.cos(pitch)
    look_dir := rl.Vector3{x, y, z}
    
    // Calculate ideal camera position behind the fling
    base_camera_pos := g.pbd_world.points[g.fling.center].position - look_dir * CAMERA_DISTANCE
    
    // Add height offset based on pitch
    vertical_offset := CAMERA_HEIGHT * (1.0 - y)
    ideal_camera_pos := base_camera_pos + rl.Vector3{0, vertical_offset, 0}
    
    // Split current and target positions into horizontal and vertical components
    current_horizontal := rl.Vector3{g.fling.thrd_pos.x, 0, g.fling.thrd_pos.z}
    current_vertical := rl.Vector3{0, g.fling.thrd_pos.y, 0}
    
    target_horizontal := rl.Vector3{ideal_camera_pos.x, 0, ideal_camera_pos.z}
    target_vertical := rl.Vector3{0, ideal_camera_pos.y, 0}
    
    // Lerp horizontal and vertical components separately with different smoothing
    new_horizontal := linalg.lerp(
        current_horizontal,
        target_horizontal,
        min(1.0, rl.GetFrameTime() * HORIZONTAL_SMOOTHING),
    )
    
    new_vertical := linalg.lerp(
        current_vertical,
        target_vertical,
        min(1.0, rl.GetFrameTime() * VERTICAL_SMOOTHING),
    )
    
    // Combine components for final camera position
    g.fling.thrd_pos = new_horizontal + new_vertical
    
    g.fling.ideal_camera_pos = ideal_camera_pos
}

fling_color :: proc(n: int) -> rl.Color {
    switch n {
    case 0:
        return rl.RED
    case 1:
        return rl.ORANGE
    case 2:
        return rl.YELLOW
    case 3:
        return rl.BLUE
    case 4:
        return rl.DARKGREEN
    }
    return rl.WHITE
}

fling_draw_points :: proc() {
    color := fling_color(int(g.fling.character_type))
    if g.fuel > 20* 100 + 100 {
        color = rl.GOLD
    }
    //dark := rl.Color{color.r - color.r/3, color.g - color.g/3, color.b - color.b/3, 100}
    rl.DrawSphere(g.pbd_world.points[g.fling.center].position, 0.3 * min(1.0, g.fuel / 100), color)
    //power := int(g.fuel / 100)
    //for point in 0..<min(len(g.fling.points), power) {
    //    rl.DrawSphere(g.pbd_world.points[g.fling.points[point]].position, 0.3, color)
    //}
    for point in g.fling.points {
        //rl.DrawSphereWires(g.pbd_world.points[point].position, 0.25, 3, 6, dark)
        rl.DrawSphere(g.pbd_world.points[point].position, 0.3, color)
    }
    // danger
    if g.fuel < 85{
        danger := rl.WHITE
        danger.g = u8(g.fuel - 85) * 3
        danger.b = u8(g.fuel - 85) * 3
        for point in g.fling.points {
            rl.DrawLine3D(g.pbd_world.points[g.fling.center].position, g.pbd_world.points[point].position, danger)
        }
    }
}

