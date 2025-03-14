package game

import rl "vendor:raylib"
import math "core:math"
import fmt "core:fmt"
//ground grid
Ground_Grid :: struct {
    image: rl.Image,
    pixels: [^]rl.Color,
    width: int,
    current_color: box_color,
}

box_color :: distinct int

TILE_SIZE :: 10
HEIGHT_SCALE :: 10

init_ground_grid :: proc() -> Ground_Grid {
    fmt.println("init_ground_grid")
    r_image := rl.GenImageCellular(100, 100, 5)
    g_image := rl.GenImageCellular(100, 100, 5)
    b_image := rl.GenImageCellular(100, 100, 5)
    a_image := rl.GenImageCellular(100, 100, 5)

    image := rl.ImageCopy(a_image)

    r_pixels := rl.LoadImageColors(r_image)
    g_pixels := rl.LoadImageColors(g_image)
    b_pixels := rl.LoadImageColors(b_image)
    a_pixels := rl.LoadImageColors(a_image)

    for x in 0..<(r_image.width) {
        for y in 0..<(r_image.height) {
            rl.ImageDrawPixel(&image, x, y, rl.Color{r_pixels[x*y].r, g_pixels[x*y].g, b_pixels[x*y].b, a_pixels[x*y].a})
        }
    }

    pixels : [^]rl.Color = rl.LoadImageColors(image)

    rl.UnloadImage(r_image)
    rl.UnloadImage(g_image)
    rl.UnloadImage(b_image)
    rl.UnloadImage(a_image)

    return Ground_Grid{image, pixels, int(image.width), 0}
}

deinit_ground_grid :: proc(ground_grid: ^Ground_Grid) {
    rl.UnloadImageColors(ground_grid.pixels)
    rl.UnloadImage(ground_grid.image)
}

draw_ground_grid :: proc() {
    //red := rl.Color{255, 0, 0, 255}
    //green := rl.Color{0, 255, 0, 255}
    //blue := rl.Color{0, 0, 255, 255}
    for i in 0..<g.ground_grid.image.width {
        for j in 0..<g.ground_grid.image.height {
            pixel := g.ground_grid.pixels[i*g.ground_grid.image.width + j]
            h := pixel[g.ground_grid.current_color]
            //r := pixel.r
            //g := pixel.g
            //b := pixel.b
            if h > 0 {
                rl.DrawCubeV(
                    rl.Vector3{f32(i*TILE_SIZE)+TILE_SIZE/2, f32(h)/(2*HEIGHT_SCALE), f32(j*TILE_SIZE)+TILE_SIZE/2}, 
                    rl.Vector3{f32(TILE_SIZE), f32(h)/HEIGHT_SCALE, f32(TILE_SIZE)}, 
                    rl.Color{h, h, h, 255})
            }
            /*
            rl.DrawCubeWiresV(
                rl.Vector3{f32(i*TILE_SIZE)+TILE_SIZE/2, f32(r)/(2*HEIGHT_SCALE), f32(j*TILE_SIZE)+TILE_SIZE/2}, 
                rl.Vector3{f32(TILE_SIZE), f32(r)/HEIGHT_SCALE, f32(TILE_SIZE)}, 
                red)
            rl.DrawCubeWiresV(
                rl.Vector3{f32(i*TILE_SIZE)+TILE_SIZE/2, f32(g)/(2*HEIGHT_SCALE), f32(j*TILE_SIZE)+TILE_SIZE/2}, 
                rl.Vector3{f32(TILE_SIZE), f32(g)/HEIGHT_SCALE, f32(TILE_SIZE)}, 
                green)
            rl.DrawCubeWiresV(
                rl.Vector3{f32(i*TILE_SIZE)+TILE_SIZE/2, f32(b)/(2*HEIGHT_SCALE), f32(j*TILE_SIZE)+TILE_SIZE/2}, 
                rl.Vector3{f32(TILE_SIZE), f32(b)/HEIGHT_SCALE, f32(TILE_SIZE)}, 
                blue)
            */
        }
    }
}

check_map_toggle :: proc(color: box_color) -> bool {
    grid_x, grid_z := get_grid_coordinates(g.player.pos)
    pixel := g.ground_grid.pixels[grid_x*g.ground_grid.width + grid_z]
    next := f32(pixel[color]) / HEIGHT_SCALE
    if next > g.player.pos.y {
        return false
    }
    return true
}

get_height_at_position :: proc(position: rl.Vector3) -> (f32, f32) {
    grid_x, grid_z := get_grid_coordinates(position)
    pixel := g.ground_grid.pixels[grid_x*g.ground_grid.width + grid_z]
    current := f32(pixel[g.ground_grid.current_color]) / HEIGHT_SCALE
    r := f32(pixel[0]) / HEIGHT_SCALE
    g := f32(pixel[1]) / HEIGHT_SCALE
    b := f32(pixel[2]) / HEIGHT_SCALE
    max := max(r, g, b)
    return current, max
}


get_grid_coordinates :: proc(position: rl.Vector3) -> (int, int) {
    //fmt.println("get_grid_coordinates", position)
    grid_x := int(position.x / TILE_SIZE)
    grid_z := int(position.z / TILE_SIZE)
    //fmt.println("get_grid_coordinates", grid_x, grid_z)
    return grid_x, grid_z
}

get_box_at_position :: proc(position: rl.Vector3) -> (rl.Vector3, rl.Vector3, f32) {
    grid_x, grid_z := get_grid_coordinates(position)
    pixel := g.ground_grid.pixels[grid_x*g.ground_grid.width + grid_z]
    current := f32(pixel[g.ground_grid.current_color]) / HEIGHT_SCALE

    center := rl.Vector3{f32(grid_x*TILE_SIZE) + TILE_SIZE/2, current/2, f32(grid_z*TILE_SIZE) + TILE_SIZE/2}
    size := rl.Vector3{f32(TILE_SIZE), current, f32(TILE_SIZE)}
    height := f32(pixel[g.ground_grid.current_color]) / HEIGHT_SCALE

    return center, size, height
}

new_grid_collision :: proc(point: ^Point) -> (rl.Vector3, rl.Vector3) {
    normal := rl.Vector3{0, 0, 0}
    // Check for NaN values in position
    if math.is_nan(point.position.x) || math.is_nan(point.position.y) || math.is_nan(point.position.z) {
        return point.position, normal
    }
    // Get grid coordinates for current and previous positions
    curr_grid_x, curr_grid_z := get_grid_coordinates(point.position)
    prev_grid_x, prev_grid_z := get_grid_coordinates(point.prev_pos)

    if (curr_grid_x >= 0 && curr_grid_x < 100 &&
        curr_grid_z >= 0 && curr_grid_z < 100 &&
        prev_grid_x >= 0 && prev_grid_x < 100 &&
        prev_grid_z >= 0 && prev_grid_z < 100) {

        // Get height at current position
        curr_height, _ := get_height_at_position(point.position)
        //prev_height, _ := get_height_at_position(point.prev_pos)
        
        // Check if we're below the surface
        if curr_height > point.position.y + 0.002 {
            // Calculate movement direction
            x_diff := curr_grid_x - prev_grid_x
            z_diff := curr_grid_z - prev_grid_z
            
            // Calculate position within grid cell (0.0 to 1.0)
            cell_x := point.position.x / TILE_SIZE - math.floor(point.position.x / TILE_SIZE)
            cell_z := point.position.z / TILE_SIZE - math.floor(point.position.z / TILE_SIZE)
            
            // Calculate vertical distance to surface
            vert_dist := curr_height - point.position.y
            
            // Determine if this is more likely a side collision or top collision
            is_side_collision := false
            
            // If we crossed grid boundaries and are not falling significantly
            if (x_diff != 0 || z_diff != 0) && point.velocity.y > -0.5 {
                // Check if we're close to a cell edge
                edge_threshold :f32= 0.2
                near_x_edge := cell_x < edge_threshold || cell_x > (1.0 - edge_threshold)
                near_z_edge := cell_z < edge_threshold || cell_z > (1.0 - edge_threshold)
                
                // If we're near an edge and not too far below the surface
                if (near_x_edge || near_z_edge) && vert_dist < 0.5 {
                    is_side_collision = true
                }
            }
            
            if is_side_collision {
                // Handle side collision
                if x_diff != 0 {
                    point.position.x = point.prev_pos.x
                    if x_diff > 0 {
                        normal = rl.Vector3{1, 0, 0}
                    } else {
                        normal = rl.Vector3{-1, 0, 0}
                    }
                }
                
                if z_diff != 0 {
                    point.position.z = point.prev_pos.z
                    if z_diff > 0 {
                        normal = rl.Vector3{0, 0, 1}
                    } else {
                        normal = rl.Vector3{0, 0, -1}
                    }
                }
            } else {
                // Handle top collision
                point.prev_pos.y = curr_height
                point.position.y = curr_height
                
                // Apply friction
                point.velocity = point.velocity * 0.95
                
                // Slight bounce-back effect
                dir := point.prev_pos - point.position
                point.position = point.position + dir * 0.1
                
                normal = rl.Vector3{0, 1, 0}
            }
        }
    }
    
    return point.position, normal
}

grid_collision :: proc(point: ^Point) -> (rl.Vector3, rl.Vector3) {
    normal := rl.Vector3{0, 0, 0}
    // Check for NaN values in position
    if math.is_nan(point.position.x) || math.is_nan(point.position.y) || math.is_nan(point.position.z) {
        return point.position, normal
    }

    // Get grid coordinates for current and previous positions
    curr_grid_x, curr_grid_z := get_grid_coordinates(point.position)
    prev_grid_x, prev_grid_z := get_grid_coordinates(point.prev_pos)

    if (curr_grid_x >= 0 && curr_grid_x < 100 &&
        curr_grid_z >= 0 && curr_grid_z < 100 &&
        prev_grid_x >= 0 && prev_grid_x < 100 &&
        prev_grid_z >= 0 && prev_grid_z < 100) {


        // Get height at current position
        curr_height, _ := get_height_at_position(point.position)
        prev_height, _ := get_height_at_position(point.prev_pos)


        z_diff := curr_grid_z - prev_grid_z
        x_diff := curr_grid_x - prev_grid_x
        if curr_height > point.position.y + 0.002 { // below
             // side hit
            if x_diff != 0 || z_diff != 0 {
                if x_diff != 0 {
                    point.position.x = point.prev_pos.x
                    if x_diff > 0 {
                        normal = rl.Vector3{1, 0, 0}
                    } else {
                        normal = rl.Vector3{-1, 0, 0}
                    }
                }
                if z_diff != 0 {
                    point.position.z = point.prev_pos.z
                    if z_diff > 0 {
                        normal = rl.Vector3{0, 0, 1}
                    } else {
                        normal = rl.Vector3{0, 0, -1}
                    }
                }
            } else if prev_height > point.prev_pos.y && curr_height > point.position.y { //glitched
                    point.prev_pos.y = curr_height
                    point.position.y = curr_height
            } else { // ground
                point.prev_pos.y = curr_height
                point.position.y = curr_height
                //point.velocity = point.velocity * 0.95
                dir := point.prev_pos - point.position
                point.position = point.position + dir * 0.1
                normal = rl.Vector3{0, 1, 0}
            } 
        }
    }
    return point.position, normal
}




check_grid_collision :: proc(vect: rl.Vector3) -> (bool, rl.Vector3, rl.Vector3, f32) {
    contact := vect
    normal := rl.Vector3{0, 0, 0}

    // Get grid coordinates for current and previous positions
    curr_grid_x, curr_grid_z := get_grid_coordinates(vect)

    if (curr_grid_x >= 0 && curr_grid_x < 100 &&
        curr_grid_z >= 0 && curr_grid_z < 100) {

        // Get height at current position
        curr_height, _ := get_height_at_position(vect)

        grid_x := vect.x / TILE_SIZE
        grid_z := vect.z / TILE_SIZE

        x_diff := grid_x - math.floor(grid_x)
        z_diff := grid_z - math.floor(grid_z)

        assert(x_diff >= 0 && x_diff <= 1)
        assert(z_diff >= 0 && z_diff <= 1)

        if curr_height > vect.y + 0.002 { // below
             // side hit
            if abs(x_diff - 0.5) > abs(z_diff - 0.5) { // find the smaller value closer to the edge
                if x_diff > 0.5 {
                    normal = rl.Vector3{1, 0, 0}
                    contact.x = math.floor(vect.x) + 1
                } else {
                    normal = rl.Vector3{-1, 0, 0}
                    contact.x = math.floor(vect.x)
                }
            } else {
                if z_diff > 0.5 {
                    normal = rl.Vector3{0, 0, 1}
                    contact.z = math.floor(vect.z) + 1
                } else {
                    normal = rl.Vector3{0, 0, -1}
                    contact.z = math.floor(vect.z)
                }
            }
            if vect.y > curr_height - 0.1 { // vec is close to top height, overide normal
                contact.y = curr_height
                normal = rl.Vector3{0, 1, 0}
            } 

            return true, contact, normal, curr_height
        }
    }
    // really need normal, and if we are at the top edge.
    // want to return height of the contacted grid
    return false, rl.Vector3{0, 0, 0}, rl.Vector3{0, 0, 0}, 0.0
}