package game

import rl "vendor:raylib"
import fmt "core:fmt"

init_power_ups :: proc() {
    for i in 0..<GRID_LEN {
        for j in 0..<GRID_WIDTH {
            pixel := &g.ground_grid.pixels[i*g.ground_grid.width + j]
            pixel.a = 0
        }
    }
    min_fuel := 20
    min_hight :u8= 250
    for min_fuel >= 0 {
        grid_x := int(rl.GetRandomValue(0, i32(GRID_LEN-1)))
        grid_z := int(rl.GetRandomValue(0, i32(GRID_WIDTH-1)))
        pixel := &g.ground_grid.pixels[grid_x*g.ground_grid.width + grid_z]
        if pixel.r > min_hight {
            pixel.a = 255
            min_fuel -= 1
        } else {
            pixel.a = 0
        }
        min_hight -= 1
    }
}


check_fuel_pickup :: proc() {
    pos := g.pbd_world.points[g.fling.center].position
    grid_x, grid_z := get_grid_coordinates(pos)

    pixel := &g.ground_grid.pixels[grid_x*g.ground_grid.width + grid_z]
    current := f32(pixel[g.ground_grid.current_color]) / HEIGHT_SCALE

    center := rl.Vector3{f32(grid_x*TILE_SIZE) + TILE_SIZE/2, current/2, f32(grid_z*TILE_SIZE) + TILE_SIZE/2}
    //size := rl.Vector3{f32(TILE_SIZE), current, f32(TILE_SIZE)}
    height := f32(pixel[g.ground_grid.current_color]) / HEIGHT_SCALE

    h := pixel.a
    if h <= 0 {return}

    fuel_pos := rl.Vector3{center.x, height + 2, center.z}
    if rl.CheckCollisionSpheres(fuel_pos, f32(h)/255*3, pos, 1) {
        fmt.println("fuel pickup")
        g.fuel += 100
        pixel.a = 0
         // randomly pick new cell to be have fuel
        grid_x = int(rl.GetRandomValue(0, i32(GRID_LEN-1)))
        grid_z = int(rl.GetRandomValue(0, i32(GRID_WIDTH-1)))
        pixel = &g.ground_grid.pixels[grid_x*g.ground_grid.width + grid_z]
        pixel.a = 255
    }
}


draw_power_ups :: proc() {
    color := fling_color(int(g.fling.character_type) + 1)

    for i in 0..<GRID_LEN {
        for j in 0..<GRID_WIDTH {
            pixel := g.ground_grid.pixels[i*GRID_WIDTH + j]
            height := f32(pixel[g.ground_grid.current_color]) / HEIGHT_SCALE
            h := pixel.a
            if h > 0 {
                rl.DrawSphereWires(rl.Vector3{f32(i*TILE_SIZE)+TILE_SIZE/2, height + 2, f32(j*TILE_SIZE)+TILE_SIZE/2}, f32(h)/255*3, 5, 7, color)
                // line up
                rl.DrawLine3D(rl.Vector3{f32(i*TILE_SIZE)+TILE_SIZE/2, height, f32(j*TILE_SIZE)+TILE_SIZE/2}, rl.Vector3{f32(i*TILE_SIZE)+TILE_SIZE/2, height + 50, f32(j*TILE_SIZE)+TILE_SIZE/2}, color)
            }
        }
    }
}