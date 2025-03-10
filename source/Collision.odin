package game

import rl "vendor:raylib"

AABB :: struct {
    min: rl.Vector3, // Minimum point of the box
    max: rl.Vector3, // Maximum point of the box
}


Point_AABB_Result :: struct {
    is_inside: bool,
    nearest_surface_point: rl.Vector3,
}

point_in_aabb :: proc(point: rl.Vector3, box: AABB) -> Point_AABB_Result {
    // First check if point is inside
    is_inside := point.x >= box.min.x && point.x <= box.max.x &&
                 point.y >= box.min.y && point.y <= box.max.y && 
                 point.z >= box.min.z && point.z <= box.max.z

    // Find nearest surface point by clamping to box bounds
    nearest := rl.Vector3{
        clamp(point.x, box.min.x, box.max.x),
        clamp(point.y, box.min.y, box.max.y),
        clamp(point.z, box.min.z, box.max.z),
    }

    // If point is inside, we need to find the nearest face
    if is_inside {
        // Find distances to each face
        dx_min := abs(point.x - box.min.x)
        dx_max := abs(point.x - box.max.x)
        dy_min := abs(point.y - box.min.y)
        dy_max := abs(point.y - box.max.y)
        dz_min := abs(point.z - box.min.z)
        dz_max := abs(point.z - box.max.z)

        // Find minimum distance and set that coordinate
        min_dist := min(dx_min, min(dx_max, min(dy_min, min(dy_max, min(dz_min, dz_max)))))

        if min_dist == dx_min {
            nearest.x = box.min.x
        } else if min_dist == dx_max {
            nearest.x = box.max.x
        } else if min_dist == dy_min {
            nearest.y = box.min.y
        } else if min_dist == dy_max {
            nearest.y = box.max.y
        } else if min_dist == dz_min {
            nearest.z = box.min.z
        } else {
            nearest.z = box.max.z
        }
    }

    return Point_AABB_Result{
        is_inside = is_inside,
        nearest_surface_point = nearest,
    }
}

clamp :: proc(value, min, max: f32) -> f32 {
    if value < min do return min
    if value > max do return max
    return value
}
