package game

import "core:math/linalg"
import "core:math"
import rl "vendor:raylib"
import "core:fmt"
Point :: struct {
    position: rl.Vector3,
    prev_pos: rl.Vector3,
    velocity: rl.Vector3,
    prev_vel: rl.Vector3,
    mass: f32,
    id: point_idx,
}

point_idx :: distinct int

Spring :: struct {
    point_a: point_idx,
    point_b: point_idx,
    rest_length: f32,
}

PBD_World :: struct {
    points: [dynamic]Point,
    springs: [dynamic]Spring,
    substeps: int,
}



pbd_simulate :: proc(delta_time: f32) {
    dts := delta_time / f32(g.pbd_world.substeps)
    gravity := rl.Vector3{0, -9.81, 0}
    for _ in 0..<g.pbd_world.substeps {
        // prev_pos, gravity, add velocity
        for i in 0..<len(g.pbd_world.points) {
            g.pbd_world.points[i].prev_vel = g.pbd_world.points[i].velocity
            g.pbd_world.points[i].velocity = g.pbd_world.points[i].velocity + dts*gravity
            g.pbd_world.points[i].prev_pos = g.pbd_world.points[i].position
            g.pbd_world.points[i].position = g.pbd_world.points[i].position + dts*g.pbd_world.points[i].velocity
        }
        // constraints
        for i in 0..<len(g.pbd_world.points) {
            if i > 420 {
                //fmt.println("points[%d]", i, g.pbd_world.points[i])
            }
            solve_ground(&g.pbd_world.points[i], dts)
            solve_box_collision(&g.pbd_world.points[i])
        }
        for i in 0..<len(g.pbd_world.springs) { //solve springs
            solve_spring(&g.pbd_world.springs[i], dts)
        }
        // update velocity
        for i in 0..<len(g.pbd_world.points) {
            g.pbd_world.points[i].velocity = (g.pbd_world.points[i].position - g.pbd_world.points[i].prev_pos) / dts
        }

        solve_player(dts)
    }
}

solve_player :: proc(dts: f32) {

    /*
    feet contact ground
    wall kick contacts wall, sticks, kicks upwards/sideways
    foot body spring, change to spring
    */

    // standing     ; ground point, feet
    // moveing      ; ground point, feet
    // jumping      ; ground point, feet
    // tack         ; wall point, side foot
    // falling      ; physics
    // crouching    ; ground point, feet
    // landing      ; ground point, feet
    // climbing     ; ledge, wall point, arms, feet
    // cat          ; ledge, wall point, arms, feet
}

solve_spring_old :: proc(spring: ^Spring, dts: f32) {
    //fmt.println("solve_spring")
    point_a := &g.pbd_world.points[spring.point_a]
    point_b := &g.pbd_world.points[spring.point_b]
    delta_pos := point_a.position - point_b.position
    current_length := linalg.length(delta_pos)
    //fmt.println("current_length", current_length)
    // Calculate inverse masses (w0 and w1)
    w0 := point_a.mass > 0.0 ? 1.0 / point_a.mass : 0.0
    w1 := point_b.mass > 0.0 ? 1.0 / point_b.mass : 0.0
    
    // Calculate correction factor
    correction := (spring.rest_length - current_length) / current_length / (w0 + w1)
    
    // Apply position corrections
    point_a.position += w0 * correction * delta_pos
    point_b.position -= w1 * correction * delta_pos
}

solve_spring :: proc(spring: ^Spring, dts: f32) {
    point_a := &g.pbd_world.points[spring.point_a]
    point_b := &g.pbd_world.points[spring.point_b]
    
    delta_pos := point_a.position - point_b.position
    current_length := linalg.length(delta_pos)
    
    // Calculate constraint value (how far from desired length)
    c := current_length - (spring.rest_length - (g.fling.crouch_amount * spring.rest_length/2))

        //(g.input_intent.trigger_left * spring.rest_length / 4) - 
        //(g.input_intent.trigger_right * spring.rest_length / 4)
    
    // Calculate inverse masses
    w0 := point_a.mass > 0.0 ? 1.0 / point_a.mass : 0.0
    w1 := point_b.mass > 0.0 ? 1.0 / point_b.mass : 0.0
    
    compliance :f32 = 0.0005
    compliance += g.fling.crouch_amount * 0.002
    lambda :f32 = 0.0
    // Calculate compliance scaled by timestep (α̃ = α/h²)
    alpha_tilde := compliance / (dts * dts)
    
    // Calculate Lagrange multiplier update (XPBD)
    delta_lambda := (-c - alpha_tilde * lambda) / (w0 + w1 + alpha_tilde)
    lambda += delta_lambda
    
    // Calculate the correction direction (normalized delta_pos)
    direction := delta_pos
    if current_length > 0.001 {  // Avoid division by zero
        direction /= current_length
    }
    
    // Apply the correction based on the Lagrange multiplier
    correction := delta_lambda * direction
    
    // Apply position corrections
    point_a.position += w0 * correction
    point_b.position -= w1 * correction
}

solve_ground :: proc(point: ^Point, dts: f32) {
    
    if point.position.y < 0 {
        // Set position to ground
        //point.position.y = 0
    }

    if point.position.y < -300 && g.fling.center == point.id {
        //reset_player_position(point)
        position := point.position
        position.y += 600
        restart_fling(position)
    }

    //collision_point, normal := new_grid_collision(point)
    //point.position = collision_point
    //point.velocity = point.velocity * 0.95
    //point.velocity = point.velocity + normal * 0.1
}

solve_box_collision :: proc(point: ^Point) {
    if math.is_nan(point.position.x) || math.is_nan(point.position.y) || math.is_nan(point.position.z) {
        fmt.println("NaN cant solve_box_collision", point.position, point.id)
        return
    } else if point.position.x < 0 ||  
              point.position.z < 0 || 
              point.position.x > GRID_LEN*TILE_SIZE || 
              point.position.z > GRID_WIDTH*TILE_SIZE {
        point.position.x = clamp(point.position.x, 0, GRID_LEN*TILE_SIZE)
        point.position.z = clamp(point.position.z, 0, GRID_WIDTH*TILE_SIZE)
        return
    }
    center, size, height := get_box_at_position(point.position)

    // Early exits
    if point.position.y > height || point.position.y < 0 {
        return // Above or below box
    }
    if point.position.x > center.x + size.x/2 || point.position.x < center.x - size.x/2 {
        fmt.println("outside box", point.position, center, size, height)
        fmt.println("point.position.x", point.position.x, center.x + size.x/2, center.x - size.x/2)
        assert(false)
        return // Outside box
    }
    if point.position.z > center.z + size.z/2 || point.position.z < center.z - size.z/2 {
        assert(false)
        return // Outside box
    }
    // if point is just above the box, move it up
    if point.position.y > height - 0.2 {
        //fmt.println("top hit")
        point.position.y = height
        //// slide friction if really crouched
        //friction :f32 = g.fling.crouch_amount
        //assert(friction <= 1 && friction >= 0)
        //point.position += (point.prev_pos - point.position) * (friction * 0.5)
        return
    }
    if point.position.x > center.x + size.x/2 - 0.2 {
        point.position.x = center.x + size.x/2 + 0.03
        return
    }
    if point.position.x < center.x - size.x/2 + 0.2 {
        point.position.x = center.x - size.x/2 - 0.03
        return
    }
    if point.position.z > center.z + size.z/2 - 0.2 {
        point.position.z = center.z + size.z/2 + 0.03
        return
    }
    if point.position.z < center.z - size.z/2 + 0.2 {
        point.position.z = center.z - size.z/2 - 0.03
        return
    }
}

pbd_init :: proc() -> ^PBD_World {
    pbd_world := new(PBD_World)
    pbd_world.points = make([dynamic]Point)
    pbd_world.springs = make([dynamic]Spring)
    pbd_world.substeps = 10

    return pbd_world
}

/*
pbd_init_player :: proc(pbd_world: ^PBD_World) {
    g.player.pos
    g.player.vel

    g.player.foot_pos
    g.player.foot_vel
    g.player.foot_dist  // distance to core
    g.player.foot_sprint_idx  // index of sprint point

    g.player.arm_pos
    g.player.arm_vel
    g.player.arm_dist  // distance to core
    g.player.arm_sprint_idx  // index of sprint point
}
*/

pbd_create_point :: proc(pbd_world: ^PBD_World, position: rl.Vector3) -> ^Point {
    len := len(pbd_world.points)
    //fmt.println("pbd_create_point", position, len)
    p := Point{position, position, {0, 0, 0}, {0, 0, 0}, 1.0, point_idx(len)}
    append(&pbd_world.points, p)
    //fmt.println("pbd_create_point", p)
    return &pbd_world.points[len]
}

pbd_create_spring :: proc(pbd_world: ^PBD_World, point_a: point_idx, point_b: point_idx, rest_length: f32) -> Spring {
    s := Spring{point_a, point_b, rest_length}
    append(&pbd_world.springs, s)
    return s
}

pbd_set_position :: proc(point: ^Point, position: rl.Vector3) {
    point.position = position
}

pbd_create_boxes :: proc(pbd_world: ^PBD_World) {
    for _ in 0..<50 {
        random_size := f32(rl.GetRandomValue(5, 15))
        random_position := rl.Vector3{
            f32(rl.GetRandomValue(0, GRID_LEN*TILE_SIZE)), 
            f32(rl.GetRandomValue(50, 130)), 
            f32(rl.GetRandomValue(0, GRID_WIDTH*TILE_SIZE))}
        pbd_create_box(pbd_world, random_position, {random_size, random_size, random_size}, 1.0)
    }
}

pbd_deinit :: proc(pbd_world: ^PBD_World) {
    delete(pbd_world.points)
    delete(pbd_world.springs)
    free(pbd_world)
}

pbd_create_box :: proc(pbd_world: ^PBD_World, position: rl.Vector3, size: rl.Vector3, mass: f32) {
    box := [8]Point{
        {position + rl.Vector3{-size.x/2, -size.y/2, -size.z/2}, position, {0, 0, 0}, {0, 0, 0}, mass, 0},
        {position + rl.Vector3{size.x/2, -size.y/2, -size.z/2}, position, {0, 0, 0}, {0, 0, 0}, mass, 0},
        {position + rl.Vector3{-size.x/2, size.y/2, -size.z/2}, position, {0, 0, 0}, {0, 0, 0}, mass, 0},
        {position + rl.Vector3{size.x/2, size.y/2, -size.z/2}, position, {0, 0, 0}, {0, 0, 0}, mass, 0},
        {position + rl.Vector3{-size.x/2, -size.y/2, size.z/2}, position, {0, 0, 0}, {0, 0, 0}, mass, 0},
        {position + rl.Vector3{size.x/2, -size.y/2, size.z/2}, position, {0, 0, 0}, {0, 0, 0}, mass, 0},
        {position + rl.Vector3{-size.x/2, size.y/2, size.z/2}, position, {0, 0, 0}, {0, 0, 0}, mass, 0},
        {position + rl.Vector3{size.x/2, size.y/2, size.z/2}, position, {0, 0, 0}, {0, 0, 0}, mass, 0},
    }
    for i in 0..<len(box) {
        //fmt.println("box[i]", box[i])
        box[i].mass = mass
        //fmt.println("box[i].mass", box[i].mass)
        box[i].id = point_idx(len(pbd_world.points))
        //fmt.println("box[i].id", box[i].id)
        append(&pbd_world.points, box[i])
        //fmt.println("pbd_world.points", pbd_world.points)
    }

    sl := pbd_world.points[len(pbd_world.points)-8:]

    ofs :point_idx = point_idx(len(pbd_world.points)-8)
    // create springs for every edge of the box and every corner to every other corner

    springs := [16]Spring{}
    // Front face
    springs[0] = Spring{ofs+0, ofs+1, linalg.length(sl[0].position - sl[1].position)}  // Bottom edge
    springs[1] = Spring{ofs+1, ofs+3, linalg.length(sl[1].position - sl[3].position)}  // Right edge
    springs[2] = Spring{ofs+3, ofs+2, linalg.length(sl[3].position - sl[2].position)}  // Top edge
    springs[3] = Spring{ofs+2, ofs+0, linalg.length(sl[2].position - sl[0].position)}  // Left edge

    // Back face
    springs[4] = Spring{ofs+4, ofs+5, linalg.length(sl[4].position - sl[5].position)}  // Bottom edge
    springs[5] = Spring{ofs+5, ofs+7, linalg.length(sl[5].position - sl[7].position)}  // Right edge
    springs[6] = Spring{ofs+7, ofs+6, linalg.length(sl[7].position - sl[6].position)}  // Top edge
    springs[7] = Spring{ofs+6, ofs+4, linalg.length(sl[6].position - sl[4].position)}  // Left edge

    // Connecting edges
    springs[8] = Spring{ofs+0, ofs+4, linalg.length(sl[0].position - sl[4].position)}
    springs[9] = Spring{ofs+1, ofs+5, linalg.length(sl[1].position - sl[5].position)}
    springs[10] = Spring{ofs+2, ofs+6, linalg.length(sl[2].position - sl[6].position)}
    springs[11] = Spring{ofs+3, ofs+7, linalg.length(sl[3].position - sl[7].position)}

    // Internal diagonal springs for structural stability
    springs[12] = Spring{ofs+0, ofs+7, linalg.length(sl[0].position - sl[7].position)}  // Front-bottom-left to back-top-right
    springs[13] = Spring{ofs+1, ofs+6, linalg.length(sl[1].position - sl[6].position)}  // Front-bottom-right to back-top-left
    springs[14] = Spring{ofs+2, ofs+5, linalg.length(sl[2].position - sl[5].position)}  // Front-top-left to back-bottom-right
    springs[15] = Spring{ofs+3, ofs+4, linalg.length(sl[3].position - sl[4].position)}  // Front-top-right to back-bottom-left

    //for i in 0..<len(springs) {
    append(&pbd_world.springs, ..springs[:])
    //}
}

pbd_draw_points :: proc(points: [dynamic]Point) {
    for i in 0..<len(points) {
        rl.DrawSphere(points[i].position, 0.3, rl.ORANGE)
    }
}

pbd_draw_springs :: proc(springs: [dynamic]Spring) {
    for i in 0..<len(springs) {
        idx_a := springs[i].point_a
        idx_b := springs[i].point_b
        rl.DrawLine3D(g.pbd_world.points[idx_a].position, g.pbd_world.points[idx_b].position, rl.RAYWHITE)
    }
}
