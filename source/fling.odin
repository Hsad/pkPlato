package game

import rl "vendor:raylib"
import "core:fmt"
import "core:math"
import "core:math/linalg"
Fling :: struct {
    Center: ^Point,
    character_type: Character_Type,
    rotation: rl.Vector3,
    frst_pos: rl.Vector3,
    thrd_pos: rl.Vector3,
    ideal_camera_pos: rl.Vector3,
    look_target: rl.Vector3,
    crouch_amount: f32,
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
			target = g.fling.Center.position,
			up = {0, 1, 0},
			fovy = 45,
			projection = .PERSPECTIVE,
		}
	}
}

create_fling :: proc(pbd_world: ^PBD_World) -> Fling {
    fmt.println("fling")

    fling := Fling{}
	center := pbd_create_point(pbd_world, rl.Vector3{10, 100, 10})
    if g.fling.character_type == .Tetrahedron {
        create_tetrahedron(pbd_world, center)
    } else if g.fling.character_type == .Cube {
        create_cube(pbd_world, center)
    } else if g.fling.character_type == .Octahedron {
        create_octahedron(pbd_world, center)
    } else if g.fling.character_type == .Dodecahedron {
        create_dodecahedron(pbd_world, center)
    } else if g.fling.character_type == .Icosahedron {
        create_icosahedron(pbd_world, center)
    }

	fling.Center = center


	return fling
}

simulate_fling :: proc(fling: ^Fling) {
    if g.input_intent.button_a {
        fling.Center.velocity.y += 5
    }

    // Update rotation based on look input
    look_sensitivity :: 2.0
    look_x := g.input_intent.look_x 
    look_y := g.input_intent.look_y
    if math.abs(look_x) < 0.3 {look_x = 0}
    if math.abs(look_y) < 0.3 {look_y = 0}
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

	// distance to ground
	center := g.fling.Center.position
	_, _, height := get_box_at_position(center)
	height_from_ground := center.y - height

    ground_mod :f32= 5 if height_from_ground < 4 else 2

    // Combine forward/back and right/left movement
    move_direction := forward * -move_z + right * -move_x
    move_velocity := move_direction * move_speed * ground_mod
    fling.Center.velocity += move_velocity

    move_fuel_cost := linalg.length(move_velocity) * rl.GetFrameTime() * height_from_ground / ground_mod
    //fmt.println("move_fuel_cost", move_fuel_cost)
    g.fuel -= move_fuel_cost

    if g.fuel < 0 {
        g.fuel = 0
    }
}

draw_fling :: proc() {
    // rotation
    rl.DrawLine3D(g.fling.Center.position, 
        {g.fling.Center.position.x + math.cos(g.fling.rotation.y), 
         g.fling.Center.position.y, 
         g.fling.Center.position.z + math.sin(g.fling.rotation.y)}, rl.RED)
}


create_tetrahedron :: proc(pbd_world: ^PBD_World, center: ^Point) {
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


}

create_cube :: proc(pbd_world: ^PBD_World, center: ^Point) {
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

create_octahedron :: proc(pbd_world: ^PBD_World, center: ^Point) {
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


create_dodecahedron :: proc(pbd_world: ^PBD_World, center: ^Point) {
    /// create a dodecahedron
    radius :f32= 3.0
    cent :f32= math.sqrt_f32(3*(5+math.sqrt_f32(5)))/4

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


    // connect every point to center
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
    pbd_create_spring(pbd_world, p12.id, center.id, radius*cent)
    pbd_create_spring(pbd_world, p13.id, center.id, radius*cent)
    pbd_create_spring(pbd_world, p14.id, center.id, radius*cent)
    pbd_create_spring(pbd_world, p15.id, center.id, radius*cent)
    pbd_create_spring(pbd_world, p16.id, center.id, radius*cent)
    pbd_create_spring(pbd_world, p17.id, center.id, radius*cent)
    pbd_create_spring(pbd_world, p18.id, center.id, radius*cent)
    pbd_create_spring(pbd_world, p19.id, center.id, radius*cent)


    // cross bars
    pbd_create_spring(pbd_world, p0.id, p3.id, radius*cent*2)
    pbd_create_spring(pbd_world, p1.id, p2.id, radius*cent*2)
    pbd_create_spring(pbd_world, p4.id, p7.id, radius*cent*2)
    pbd_create_spring(pbd_world, p5.id, p6.id, radius*cent*2)
    pbd_create_spring(pbd_world, p8.id, p11.id, radius*cent*2)
    pbd_create_spring(pbd_world, p9.id, p10.id, radius*cent*2)
    
}


create_icosahedron :: proc(pbd_world: ^PBD_World, center: ^Point) {
    /// create an icosahedron
    radius :f32= 3.0
    
    // First create all points with proper initial positions
    // Top point
    top_point_idx := len(pbd_world.points)
    top_point := pbd_create_point(pbd_world, center.position + rl.Vector3{0, radius, 0})
    assert(int(top_point.id) == top_point_idx)
    
    // Bottom point
    bottom_point_idx := len(pbd_world.points)
    bottom_point := pbd_create_point(pbd_world, center.position + rl.Vector3{0, -radius, 0})
    assert(int(bottom_point.id) == bottom_point_idx)
    // Create arrays to store point indices rather than the points themselves
    top_points_idx := make([]point_idx, 5)
    bottom_points_idx := make([]point_idx, 5)
    defer delete(top_points_idx)
    defer delete(bottom_points_idx)
    
    golden_ratio :f32= (1.0 + math.sqrt(f32(5.0))) / 2.0
    //golden_ratio = 1.3
    
    // Create top and bottom ring points
    for i in 0..<5 {
        angle := f32(i) * 2.0 * math.PI / 5.0
        next_angle := f32((i+1) % 5) * 2.0 * math.PI / 5.0
        
        // Top ring points
        x := math.cos(angle) * radius / golden_ratio
        z := math.sin(angle) * radius / golden_ratio
        y := radius / 2.0
        top_idx := len(pbd_world.points)
        pbd_create_point(pbd_world, center.position + rl.Vector3{x, y, z})
        top_points_idx[i] = point_idx(top_idx)
        
        // Bottom ring points
        x = math.cos(next_angle) * radius / golden_ratio
        z = math.sin(next_angle) * radius / golden_ratio
        y = -radius / 2.0
        bottom_idx := len(pbd_world.points)
        pbd_create_point(pbd_world, center.position + rl.Vector3{x, y, z})
        bottom_points_idx[i] = point_idx(bottom_idx)
    }

    // Now create all springs using the stored indices
    cent :f32= 0.951 
    for i in 0..<5 {
        // Connect to center
        pbd_create_spring(pbd_world, top_points_idx[i], center.id, radius*cent)
        pbd_create_spring(pbd_world, bottom_points_idx[i], center.id, radius*cent)
        
        // Connect to top/bottom points
        pbd_create_spring(pbd_world, point_idx(top_point_idx), top_points_idx[i], radius)
        pbd_create_spring(pbd_world, point_idx(bottom_point_idx), bottom_points_idx[i], radius)
        
        // Connect rings
        pbd_create_spring(pbd_world, top_points_idx[i], top_points_idx[(i+1)%5], radius)
        pbd_create_spring(pbd_world, bottom_points_idx[i], bottom_points_idx[(i+1)%5], radius)
        
        // Connect between rings
        pbd_create_spring(pbd_world, top_points_idx[i], bottom_points_idx[i], radius)
        pbd_create_spring(pbd_world, top_points_idx[i], bottom_points_idx[(i+1)%5], radius)
    }
    
    //// Connect top and bottom points to center
    pbd_create_spring(pbd_world, point_idx(top_point_idx), center.id, radius*cent)
    pbd_create_spring(pbd_world, point_idx(bottom_point_idx), center.id, radius*cent)


    // cross struts
    pbd_create_spring(pbd_world, point_idx(top_point_idx), point_idx(bottom_point_idx), radius*cent*2)
    pbd_create_spring(pbd_world, point_idx(top_points_idx[0]), point_idx(bottom_points_idx[3]), radius*cent*2)
    pbd_create_spring(pbd_world, point_idx(top_points_idx[1]), point_idx(bottom_points_idx[4]), radius*cent*2)
    pbd_create_spring(pbd_world, point_idx(top_points_idx[2]), point_idx(bottom_points_idx[0]), radius*cent*2)
    pbd_create_spring(pbd_world, point_idx(top_points_idx[3]), point_idx(bottom_points_idx[1]), radius*cent*2)
    pbd_create_spring(pbd_world, point_idx(top_points_idx[4]), point_idx(bottom_points_idx[2]), radius*cent*2)


}

calc_fling_1st_person :: proc() {
	// Set camera position to player position with a slight height offset for eye level
	g.fling.frst_pos = g.fling.Center.position + rl.Vector3{0, 1.7, 0}  // Typical eye height
	
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
    base_camera_pos := g.fling.Center.position - look_dir * CAMERA_DISTANCE
    
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