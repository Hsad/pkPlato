package game

import rl "vendor:raylib"


//ground grid
Ground_Grid :: struct {
    image: rl.Image,
}

TILE_SIZE :: 10

init_ground_grid :: proc() -> Ground_Grid {
    return Ground_Grid{rl.GenImageCellular(100, 100, TILE_SIZE)}
}

draw_ground_grid :: proc() {
    pixels := rl.LoadImageColors(g_mem.ground_grid.image)
    defer rl.UnloadImageColors(pixels)
    for i in 0..<g_mem.ground_grid.image.width {
        for j in 0..<g_mem.ground_grid.image.height {
            pixel := pixels[i*g_mem.ground_grid.image.width + j]
            h := pixel.x
            if h > 0 {
                rl.DrawCubeV(
                    rl.Vector3{f32(i*TILE_SIZE), f32(h)/20, f32(j*TILE_SIZE)}, 
                    rl.Vector3{f32(TILE_SIZE), f32(h)/10, f32(TILE_SIZE)}, 
                    pixel)
            }
        }
    }
}