package game

import "core:fmt"
//import "core:math"
import "core:math/linalg"
import rl "vendor:raylib"

Point :: struct {
    position: rl.Vector3,
    prev_pos: rl.Vector3,
    velocity: rl.Vector3,
    mass: f32,
    id: int,
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
    //fmt.println("pbd_simulate")
    dts := delta_time / f32(g_mem.pbd_world.substeps)
    gravity := rl.Vector3{0, -9.81, 0}
    for _ in 0..<g_mem.pbd_world.substeps {
        for i in 0..<len(g_mem.pbd_world.points) {
            g_mem.pbd_world.points[i].velocity = g_mem.pbd_world.points[i].velocity + dts*gravity
            g_mem.pbd_world.points[i].prev_pos = g_mem.pbd_world.points[i].position
            g_mem.pbd_world.points[i].position = g_mem.pbd_world.points[i].position + dts*g_mem.pbd_world.points[i].velocity
        }
        for i in 0..<len(g_mem.pbd_world.points) {
            solve_ground(&g_mem.pbd_world.points[i], dts)
        }
        for i in 0..<len(g_mem.pbd_world.springs) { //solve springs
            //fmt.println("i", i)
            solve_spring(&g_mem.pbd_world.springs[i], dts)
        }
        for i in 0..<len(g_mem.pbd_world.points) {
            g_mem.pbd_world.points[i].velocity = (g_mem.pbd_world.points[i].position - g_mem.pbd_world.points[i].prev_pos) / dts
        }
    }
}

solve_spring :: proc(spring: ^Spring, dts: f32) {
    //fmt.println("solve_spring")
    point_a := &g_mem.pbd_world.points[spring.point_a]
    point_b := &g_mem.pbd_world.points[spring.point_b]
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

solve_ground :: proc(point: ^Point, dts: f32) {
    if point.position.y < 0 {
        point.position.y = 0
    }
}
pbd_init :: proc() -> ^PBD_World {
    fmt.println("pbd_init")
    pbd_world := new(PBD_World)
    pbd_world.points = make([dynamic]Point)
    pbd_world.springs = make([dynamic]Spring)
    pbd_world.substeps = 10

    return pbd_world
}

pbd_create_boxes :: proc(pbd_world: ^PBD_World) {
    fmt.println("pbd_create_boxes")
    pbd_create_box(pbd_world, {2, 20, 0}, {10, 10, 10}, 1.0)
    pbd_create_box(pbd_world, {-2, 20, 0}, {10, 10, 10}, 1.0)
    pbd_create_box(pbd_world, {2, 10, 0}, {5, 5, 5}, 1.0)

    for _ in 0..<10 {
        random_size := f32(rl.GetRandomValue(1, 10))
        random_position := rl.Vector3{
            f32(rl.GetRandomValue(-30, 30)), 
            f32(rl.GetRandomValue(-30, 30)), 
            f32(rl.GetRandomValue(-30, 30))}
        pbd_create_box(pbd_world, random_position, {random_size, random_size, random_size}, 1.0)
    }
}

pbd_deinit :: proc(pbd_world: ^PBD_World) {
    delete(pbd_world.points)
    delete(pbd_world.springs)
    free(pbd_world)
}

pbd_create_box :: proc(pbd_world: ^PBD_World, position: rl.Vector3, size: rl.Vector3, mass: f32) {
    fmt.printf("len(pbd_world.points): %v\n", len(pbd_world.points))
    fmt.println("pbd_create_box")
    box := [8]Point{
        {position + rl.Vector3{-size.x/2, -size.y/2, -size.z/2}, position, {0, 0, 0}, mass, 0},
        {position + rl.Vector3{size.x/2, -size.y/2, -size.z/2}, position, {0, 0, 0}, mass, 0},
        {position + rl.Vector3{-size.x/2, size.y/2, -size.z/2}, position, {0, 0, 0}, mass, 0},
        {position + rl.Vector3{size.x/2, size.y/2, -size.z/2}, position, {10, 0, 0}, mass, 0},
        {position + rl.Vector3{-size.x/2, -size.y/2, size.z/2}, position, {0, 0, 0}, mass, 0},
        {position + rl.Vector3{size.x/2, -size.y/2, size.z/2}, position, {0, 10, 0}, mass, 0},
        {position + rl.Vector3{-size.x/2, size.y/2, size.z/2}, position, {0, 0, 0}, mass, 0},
        {position + rl.Vector3{size.x/2, size.y/2, size.z/2}, position, {0, 0, 0}, mass, 0},
    }
    for i in 0..<len(box) {
        //fmt.println("box[i]", box[i])
        box[i].mass = mass
        //fmt.println("box[i].mass", box[i].mass)
        box[i].id = len(pbd_world.points)
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
        rl.DrawSphere(points[i].position, 1.0, rl.ORANGE)
    }
}

pbd_draw_springs :: proc(springs: [dynamic]Spring) {
    for i in 0..<len(springs) {
        idx_a := springs[i].point_a
        idx_b := springs[i].point_b
        rl.DrawLine3D(g_mem.pbd_world.points[idx_a].position, g_mem.pbd_world.points[idx_b].position, rl.BLACK)
    }
}
