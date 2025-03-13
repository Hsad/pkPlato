package game

import rl "vendor:raylib"
import "core:fmt"
import "core:math"

Fling :: struct {
    Center: Point,

    rotation: rl.Vector3,
}


get_fling_camera :: proc() -> rl.Camera {
	return rl.Camera{
		position = g.fling.Center.position + rl.Vector3{-3, 1, 0},
		target = g.fling.Center.position,
		up = rl.Vector3{0, 1, 0},
	}
}

create_fling :: proc(pbd_world: ^PBD_World) -> Fling {
    fmt.println("fling")
	center := pbd_create_point(pbd_world, rl.Vector3{0, 0, 0})

    create_ball(pbd_world, center)

	fling := Fling{
		Center = center,
	}

	return fling
}

simulate_fling :: proc(fling: ^Fling) {

}


create_ball :: proc(pbd_world: ^PBD_World, center: Point) {
    /// create a isohedron
    // points
    top_points: [5]Point
    bottom_points: [5]Point
    for i in 0..<5{
        top_points[i] = pbd_create_point(pbd_world, rl.Vector3{0, 0, 0})
        bottom_points[i] = pbd_create_point(pbd_world, rl.Vector3{0, 0, 0})
    }
    top_point := pbd_create_point(pbd_world, rl.Vector3{0, 0, 0})
    bottom_point := pbd_create_point(pbd_world, rl.Vector3{0, 0, 0})

    // springs
    for i in 0..<5{
        // top and bottom adjacent
        pbd_create_spring(pbd_world, top_points[i].id, top_point.id, 1.0)
        pbd_create_spring(pbd_world, bottom_points[i].id, bottom_point.id, 1.0)
        // top and bottom neighbors
        pbd_create_spring(pbd_world, top_points[i].id, top_points[(i+1)%5].id, 1.0)
        pbd_create_spring(pbd_world, bottom_points[i].id, bottom_points[(i+1)%5].id, 1.0)
        // top and bottom interconnect
        pbd_create_spring(pbd_world, top_points[i].id, bottom_points[i].id, 1.0)
        pbd_create_spring(pbd_world, top_points[i].id, bottom_points[(i+1)%5].id, 1.0)
    }
    
    // link to center
    golden_ratio :f32= (1.0 + math.sqrt(f32(5.0))) / 2.0
    for i in 0..<5{
        pbd_create_spring(pbd_world, top_points[i].id, center.id, golden_ratio)
        pbd_create_spring(pbd_world, bottom_points[i].id, center.id, golden_ratio)
    }

    pbd_create_spring(pbd_world, top_point.id, center.id, golden_ratio)
    pbd_create_spring(pbd_world, bottom_point.id, center.id, golden_ratio)
}