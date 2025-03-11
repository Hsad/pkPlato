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

Spring :: struct {
    point_a: ^Point,
    point_b: ^Point,
    rest_length: f32,
}

PBD_World :: struct {
    points: [dynamic]Point,
    springs: [dynamic]Spring,
    substeps: int,
}


pbd_simulate :: proc(delta_time: f32) {
    fmt.println("pbd_simulate")
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
            solve_spring(&g_mem.pbd_world.springs[i], dts)
        }
        for i in 0..<len(g_mem.pbd_world.points) {
            g_mem.pbd_world.points[i].velocity = (g_mem.pbd_world.points[i].position - g_mem.pbd_world.points[i].prev_pos) / dts
        }
    }
}

solve_spring :: proc(spring: ^Spring, dts: f32) {
    fmt.println("solve_spring")
    fmt.println(spring.point_a.id)
    fmt.println(spring.point_b.id)
    delta_pos := spring.point_a.position - spring.point_b.position
    current_length := linalg.length(delta_pos)
    
    fmt.println("current_length", current_length)
    // Calculate inverse masses (w0 and w1)
    w0 := spring.point_a.mass > 0.0 ? 1.0 / spring.point_a.mass : 0.0
    w1 := spring.point_b.mass > 0.0 ? 1.0 / spring.point_b.mass : 0.0
    
    // Calculate correction factor
    correction := (spring.rest_length - current_length) / current_length / (w0 + w1)
    
    // Apply position corrections
    spring.point_a.position -= w0 * correction * delta_pos
    spring.point_b.position += w1 * correction * delta_pos
}

solve_ground :: proc(point: ^Point, dts: f32) {
    fmt.println("solve_ground")
    fmt.println(point.position)
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

    pbd_create_box(pbd_world, {0, 3, 0}, {10, 10, 10}, 1.0)
    pbd_create_box(pbd_world, {2, 8, 0}, {1, 1, 1}, 1.0)

    return pbd_world
}

pbd_deinit :: proc(pbd_world: ^PBD_World) {
    delete(pbd_world.points)
    delete(pbd_world.springs)
    free(pbd_world)
}

pbd_create_box :: proc(pbd_world: ^PBD_World, position: rl.Vector3, size: rl.Vector3, mass: f32) {
    box := [8]Point{
        {position + rl.Vector3{-size.x/2, -size.y/2, -size.z/2}, position, {0, 0, 0}, mass, 0},
        {position + rl.Vector3{size.x/2, -size.y/2, -size.z/2}, position, {0, 0, 0}, mass, 0},
        {position + rl.Vector3{-size.x/2, size.y/2, -size.z/2}, position, {0, 0, 0}, mass, 0},
        {position + rl.Vector3{size.x/2, size.y/2, -size.z/2}, position, {0, 0, 0}, mass, 0},
        {position + rl.Vector3{-size.x/2, -size.y/2, size.z/2}, position, {0, 0, 0}, mass, 0},
        {position + rl.Vector3{size.x/2, -size.y/2, size.z/2}, position, {0, 0, 0}, mass, 0},
        {position + rl.Vector3{-size.x/2, size.y/2, size.z/2}, position, {0, 0, 0}, mass, 0},
        {position + rl.Vector3{size.x/2, size.y/2, size.z/2}, position, {0, 0, 0}, mass, 0},
    }
    for i in 0..<len(box) {
        box[i].mass = mass
        box[i].id = len(pbd_world.points)
        append(&pbd_world.points, box[i])
    }

    sl := pbd_world.points[len(pbd_world.points)-8:]

    // create springs for every edge of the box and every corner to every other corner
    springs := [16]Spring{}
    // Front face
    springs[0] = Spring{&sl[0], &sl[1], linalg.length(sl[0].position - sl[1].position)}  // Bottom edge
    springs[1] = Spring{&sl[1], &sl[3], linalg.length(sl[1].position - sl[3].position)}  // Right edge
    springs[2] = Spring{&sl[3], &sl[2], linalg.length(sl[3].position - sl[2].position)}  // Top edge
    springs[3] = Spring{&sl[2], &sl[0], linalg.length(sl[2].position - sl[0].position)}  // Left edge

    // Back face
    springs[4] = Spring{&sl[4], &sl[5], linalg.length(sl[4].position - sl[5].position)}  // Bottom edge
    springs[5] = Spring{&sl[5], &sl[7], linalg.length(sl[5].position - sl[7].position)}  // Right edge
    springs[6] = Spring{&sl[7], &sl[6], linalg.length(sl[7].position - sl[6].position)}  // Top edge
    springs[7] = Spring{&sl[6], &sl[4], linalg.length(sl[6].position - sl[4].position)}  // Left edge

    // Connecting edges
    springs[8] = Spring{&sl[0], &sl[4], linalg.length(sl[0].position - sl[4].position)}
    springs[9] = Spring{&sl[1], &sl[5], linalg.length(sl[1].position - sl[5].position)}
    springs[10] = Spring{&sl[2], &sl[6], linalg.length(sl[2].position - sl[6].position)}
    springs[11] = Spring{&sl[3], &sl[7], linalg.length(sl[3].position - sl[7].position)}

    // Internal diagonal springs for structural stability
    springs[12] = Spring{&sl[0], &sl[7], linalg.length(sl[0].position - sl[7].position)}  // Front-bottom-left to back-top-right
    springs[13] = Spring{&sl[1], &sl[6], linalg.length(sl[1].position - sl[6].position)}  // Front-bottom-right to back-top-left
    springs[14] = Spring{&sl[2], &sl[5], linalg.length(sl[2].position - sl[5].position)}  // Front-top-left to back-bottom-right
    springs[15] = Spring{&sl[3], &sl[4], linalg.length(sl[3].position - sl[4].position)}  // Front-top-right to back-bottom-left

    for i in 0..<len(springs) {
        append(&pbd_world.springs, springs[i])
    }
}

pbd_draw_points :: proc(points: [dynamic]Point) {
    for i in 0..<len(points) {
        rl.DrawSphere(points[i].position, 1.0, rl.ORANGE)
    }
}

pbd_draw_springs :: proc(springs: [dynamic]Spring) {
    for i in 0..<len(springs) {
        rl.DrawLine3D(springs[i].point_a.position, springs[i].point_b.position, rl.BLACK)
    }
}
