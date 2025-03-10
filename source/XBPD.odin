package game

import "core:math"
import "core:math/linalg"

Vector3 :: [3]f32
Quaternion :: [4]f32

Shape_Type :: enum {
    Box,
    Sphere,
}

RigidBody :: struct {
    // Position and orientation
    position: Vector3,
    position_prev: Vector3,
    orientation: Quaternion,
    orientation_prev: Quaternion,
    
    // Velocities
    linear_velocity: Vector3,
    angular_velocity: Vector3,
    
    // Physical properties
    mass: f32,
    inverse_mass: f32,
    inertia_tensor: matrix[3, 3]f32,
    inverse_inertia_tensor: matrix[3, 3]f32,
    
    // Contact and friction
    restitution: f32,
    static_friction: f32,
    dynamic_friction: f32,
    
    // Shape information
    shape_type: Shape_Type,
    shape_size: Vector3,  // For box: half-extents, For sphere: x component is radius
}

Contact :: struct {
    // Bodies involved
    body_a: ^RigidBody,
    body_b: ^RigidBody,
    
    // Contact point in local coordinates
    local_point_a: Vector3,
    local_point_b: Vector3,
    
    // Contact normal and penetration depth
    normal: Vector3,
    penetration: f32,
    
    // Friction coefficients
    static_friction: f32,
    dynamic_friction: f32,
    
    // Lagrange multipliers for normal and tangential forces
    lambda_normal: f32,
    lambda_tangent: f32,
}

// Helper function to transform a local point to world space
transform_point :: proc(body: ^RigidBody, local_point: Vector3) -> Vector3 {
    // In a full implementation, we'd use quaternion to rotate the point
    // Simplified for clarity
    world_point := local_point + body.position
    return world_point
}

// Helper function to calculate relative velocity at contact point
calculate_relative_velocity :: proc(contact: ^Contact) -> Vector3 {
    r1 := transform_point(contact.body_a, contact.local_point_a) - contact.body_a.position
    r2 := transform_point(contact.body_b, contact.local_point_b) - contact.body_b.position
    
    v1 := contact.body_a.linear_velocity + cross(contact.body_a.angular_velocity, r1)
    v2 := contact.body_b.linear_velocity + cross(contact.body_b.angular_velocity, r2)
    
    return v1 - v2
}

// Calculate cross product of two Vector3s
cross :: proc(a, b: Vector3) -> Vector3 {
    return {
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    }
}

// Transform a vector using the rotation represented by a quaternion
rotate_vector :: proc(q: Quaternion, v: Vector3) -> Vector3 {
    // Implementation of quaternion rotation
    // For a complete implementation, use a proper quaternion library
    return v // Simplified for clarity
}

// Main contact constraint handling function
solve_contact :: proc(contact: ^Contact, dt: f32) {
    body_a := contact.body_a
    body_b := contact.body_b
    
    // Skip if both bodies have infinite mass (static objects)
    if body_a.inverse_mass == 0 && body_b.inverse_mass == 0 {
        return
    }
    
    // Get current world space contact points
    p1 := transform_point(body_a, contact.local_point_a)
    p2 := transform_point(body_b, contact.local_point_b)
    
    // Get previous world space contact points
    r1_prev := rotate_vector(body_a.orientation_prev, contact.local_point_a)
    r2_prev := rotate_vector(body_b.orientation_prev, contact.local_point_b)
    p1_prev := body_a.position_prev + r1_prev
    p2_prev := body_b.position_prev + r2_prev
    
    // Calculate current penetration
    d := linalg.dot(p1 - p2, contact.normal)
    
    // Skip if not in contact
    if d > 0 {
        return
    }
    
    // Vectors from center of mass to contact points
    r1 := p1 - body_a.position
    r2 := p2 - body_b.position
    
    // Calculate generalized inverse masses
    r1_cross_n := cross(r1, contact.normal)
    r2_cross_n := cross(r2, contact.normal)
    
    w1 := body_a.inverse_mass + linalg.dot(r1_cross_n, linalg.matrix_mul_vector(body_a.inverse_inertia_tensor, r1_cross_n))
    w2 := body_b.inverse_mass + linalg.dot(r2_cross_n, linalg.matrix_mul_vector(body_b.inverse_inertia_tensor, r2_cross_n))
    
    // Combined inverse mass
    w := w1 + w2
    
    // Skip if infinite mass
    if w <= 0 {
        return
    }
    
    // ------ Normal constraint (non-penetration) ------
    
    // Calculate positional correction
    alpha := f32(0.0)  // Explicitly use f32
    alpha_prime := alpha / (dt * dt)
    delta_lambda := (-d - alpha_prime * contact.lambda_normal) / (w + alpha_prime)
    contact.lambda_normal += delta_lambda
    
    // Apply positional correction for normal constraint
    impulse := contact.normal * delta_lambda
    
    // Apply to bodies
    if body_a.inverse_mass > 0 {
        body_a.position += impulse * body_a.inverse_mass
        body_a.orientation = apply_angular_impulse(body_a, cross(r1, impulse))
    }
    
    if body_b.inverse_mass > 0 {
        body_b.position -= impulse * body_b.inverse_mass
        body_b.orientation = apply_angular_impulse(body_b, cross(r2, -impulse))
    }
    
    // ------ Static friction constraint (position level) ------
    
    // Calculate movement at contact points
    delta_p := (p1 - p1_prev) - (p2 - p2_prev)
    
    // Extract tangential component
    delta_p_tangent := delta_p - contact.normal * linalg.dot(delta_p, contact.normal)
    delta_p_tangent_len := linalg.length(delta_p_tangent)
    
    // Apply static friction correction
    if delta_p_tangent_len > 0 {
        // Normalize tangent direction
        tangent := delta_p_tangent / delta_p_tangent_len
        
        // Friction coefficient
        combined_static_friction := (contact.body_a.static_friction + contact.body_b.static_friction) * 0.5
        
        // Apply only if within static friction cone
        if contact.lambda_tangent < combined_static_friction * contact.lambda_normal {
            // Calculate tangent correction
            tangent_correction := delta_p_tangent
            
            // Calculate generalized inverse masses for tangent direction
            r1_cross_t := cross(r1, tangent)
            r2_cross_t := cross(r2, tangent)
            
            w1_t := body_a.inverse_mass + linalg.dot(r1_cross_t, linalg.matrix_mul_vector(body_a.inverse_inertia_tensor, r1_cross_t))
            w2_t := body_b.inverse_mass + linalg.dot(r2_cross_t, linalg.matrix_mul_vector(body_b.inverse_inertia_tensor, r2_cross_t))
            
            w_t := w1_t + w2_t
            
            // Skip if infinite mass in tangent direction
            if w_t <= 0 {
                return
            }
            
            // Calculate impulse magnitude
            delta_lambda_t := -linalg.length(tangent_correction) / w_t
            
            // Apply impulse
            impulse_t := tangent * delta_lambda_t
            
            if body_a.inverse_mass > 0 {
                body_a.position += impulse_t * body_a.inverse_mass
                body_a.orientation = apply_angular_impulse(body_a, cross(r1, impulse_t))
            }
            
            if body_b.inverse_mass > 0 {
                body_b.position -= impulse_t * body_b.inverse_mass
                body_b.orientation = apply_angular_impulse(body_b, cross(r2, -impulse_t))
            }
            
            // Track accumulated tangent impulse magnitude
            contact.lambda_tangent += delta_lambda_t
        }
    }
}

// Apply dynamic friction at velocity level
apply_dynamic_friction :: proc(contact: ^Contact, dt: f32) {
    // Calculate relative velocity at contact point
    rel_vel := calculate_relative_velocity(contact)
    
    // Extract normal component
    normal_vel := linalg.dot(rel_vel, contact.normal)
    
    // Extract tangential component
    tangent_vel := rel_vel - contact.normal * normal_vel
    tangent_vel_len := linalg.length(tangent_vel)
    
    // Skip if no tangential velocity
    if tangent_vel_len < 1e-6 {
        return
    }
    
    // Normalized tangent direction
    tangent_dir := tangent_vel / tangent_vel_len
    
    // Bodies involved
    body_a := contact.body_a
    body_b := contact.body_b
    
    // Vectors from center of mass to contact points
    r1 := transform_point(body_a, contact.local_point_a) - body_a.position
    r2 := transform_point(body_b, contact.local_point_b) - body_b.position
    
    // Compute dynamic friction coefficient
    combined_friction := (contact.body_a.dynamic_friction + contact.body_b.dynamic_friction) * 0.5
    
    // Normal force magnitude approximation
    normal_force := contact.lambda_normal / (dt * dt)
    
    // Compute maximum friction impulse magnitude
    max_friction_impulse := combined_friction * normal_force * dt
    
    // Clamp friction impulse to avoid reversing velocity direction
    friction_impulse_mag := math.min(max_friction_impulse, tangent_vel_len)
    
    // Compute friction impulse
    friction_impulse := -tangent_dir * friction_impulse_mag
    
    // Calculate generalized inverse masses for velocity corrections
    r1_cross_t := cross(r1, tangent_dir)
    r2_cross_t := cross(r2, tangent_dir)
    
    w1 := body_a.inverse_mass + linalg.dot(r1_cross_t, linalg.matrix_mul_vector(body_a.inverse_inertia_tensor, r1_cross_t))
    w2 := body_b.inverse_mass + linalg.dot(r2_cross_t, linalg.matrix_mul_vector(body_b.inverse_inertia_tensor, r2_cross_t))
    
    w := w1 + w2
    
    // Skip if infinite mass
    if w <= 0 {
        return
    }
    
    // Calculate impulse
    impulse := friction_impulse
    
    // Apply impulse to linear velocities
    if body_a.inverse_mass > 0 {
        body_a.linear_velocity += impulse * body_a.inverse_mass
        body_a.angular_velocity += linalg.matrix_mul_vector(body_a.inverse_inertia_tensor, cross(r1, impulse))
    }
    
    if body_b.inverse_mass > 0 {
        body_b.linear_velocity -= impulse * body_b.inverse_mass
        body_b.angular_velocity -= linalg.matrix_mul_vector(body_b.inverse_inertia_tensor, cross(r2, impulse))
    }
}

// Apply angular impulse to a rigid body
apply_angular_impulse :: proc(body: ^RigidBody, angular_impulse: Vector3) -> Quaternion {
    // This is a simplified implementation
    // In a complete version, we would properly integrate the angular velocity
    // and update the quaternion
    
    delta_rotation := Quaternion{0, 0, 0, 1} // Identity quaternion
    
    // In a real implementation, update the quaternion based on angular impulse
    // delta_rotation = compute_delta_rotation(body, angular_impulse)
    
    // Combine with current orientation
    new_orientation := quaternion_multiply(delta_rotation, body.orientation)
    
    // Normalize
    length := math.sqrt(
        new_orientation[0] * new_orientation[0] +
        new_orientation[1] * new_orientation[1] +
        new_orientation[2] * new_orientation[2] +
        new_orientation[3] * new_orientation[3])
    return {
        new_orientation[0] / length,
        new_orientation[1] / length,
        new_orientation[2] / length,
        new_orientation[3] / length,
    }
}

// Multiply two quaternions
quaternion_multiply :: proc(a, b: Quaternion) -> Quaternion {
    return {
        a[3] * b[0] + a[0] * b[3] + a[1] * b[2] - a[2] * b[1],
        a[3] * b[1] - a[0] * b[2] + a[1] * b[3] + a[2] * b[0],
        a[3] * b[2] + a[0] * b[1] - a[1] * b[0] + a[2] * b[3],
        a[3] * b[3] - a[0] * b[0] - a[1] * b[1] - a[2] * b[2],
    }
}

// Main XPBD solver function
solve_xpbd :: proc(bodies: []RigidBody, contacts: []Contact, dt: f32) {
    // Store previous state for all bodies
    for i := 0; i < len(bodies); i += 1 {
        bodies[i].position_prev = bodies[i].position
        bodies[i].orientation_prev = bodies[i].orientation
    }
    
    // Apply external forces (like gravity) and integrate velocities
    for i := 0; i < len(bodies); i += 1 {
        if bodies[i].inverse_mass > 0 {
            // Apply gravity
            gravity := Vector3{0, -9.81, 0}
            bodies[i].linear_velocity += gravity * dt
            
            // Integrate positions
            bodies[i].position += bodies[i].linear_velocity * dt
            
            // Integrate orientations (simplified)
            // In a complete implementation, use proper quaternion integration
            bodies[i].orientation = apply_angular_impulse(&bodies[i], bodies[i].angular_velocity * dt)
        }
    }
    
    // Position-based solver iterations
    iterations := 20  // Adjust based on desired stability/performance
    for iter := 0; iter < iterations; iter += 1 {
        // Solve all contact constraints
        for i := 0; i < len(contacts); i += 1 {
            solve_contact(&contacts[i], dt)
        }
    }
    
    // Update velocities from positions
    for i := 0; i < len(bodies); i += 1 {
        if bodies[i].inverse_mass > 0 {
            // Linear velocity update
            bodies[i].linear_velocity = (bodies[i].position - bodies[i].position_prev) / dt
            
            // Angular velocity update (simplified)
            // In a full implementation, derive angular velocity from quaternions
        }
    }
    
    // Velocity-based solver for dynamic friction
    for i := 0; i < len(contacts); i += 1 {
        apply_dynamic_friction(&contacts[i], dt)
    }
}

// Physics world to manage all bodies and contacts
PhysicsWorld :: struct {
    bodies: [dynamic]RigidBody,
    contacts: [dynamic]Contact,
}

// Create a new physics world
create_physics_world :: proc() -> ^PhysicsWorld {
    world := new(PhysicsWorld)
    world.bodies = make([dynamic]RigidBody)
    world.contacts = make([dynamic]Contact)
    return world
}

// Destroy a physics world and free its resources
destroy_physics_world :: proc(world: ^PhysicsWorld) {
    delete(world.bodies)
    delete(world.contacts)
    free(world)
}

// Create a box rigid body
create_box :: proc(world: ^PhysicsWorld, position: Vector3, size: Vector3, mass: f32) -> int {
    body := RigidBody{
        position = position,
        position_prev = position,
        orientation = {0, 0, 0, 1}, // Identity quaternion
        orientation_prev = {0, 0, 0, 1},
        linear_velocity = {0, 0, 0},
        angular_velocity = {0, 0, 0},
        mass = mass,
        restitution = 0.5,
        static_friction = 0.5,
        dynamic_friction = 0.3,
        shape_type = .Box,
        shape_size = size * 0.5,  // Convert to half-extents
    }
    
    // Calculate inverse mass (0 for static objects)
    if mass <= 0 {
        body.inverse_mass = 0
    } else {
        body.inverse_mass = 1.0 / mass
    }
    
    // Calculate inertia tensor for a box
    // This is a simplified calculation - a proper implementation would use the box dimensions
    ix := (1.0/12.0) * mass * (size[1]*size[1] + size[2]*size[2])
    iy := (1.0/12.0) * mass * (size[0]*size[0] + size[2]*size[2])
    iz := (1.0/12.0) * mass * (size[0]*size[0] + size[1]*size[1])
    
    body.inertia_tensor = matrix[3, 3]f32{
        ix, 0, 0,
        0, iy, 0,
        0, 0, iz,
    }
    
    // Calculate inverse inertia tensor
    if mass <= 0 {
        body.inverse_inertia_tensor = {}
    } else {
        body.inverse_inertia_tensor = matrix[3, 3]f32{
            1/ix, 0, 0,
            0, 1/iy, 0,
            0, 0, 1/iz,
        }
    }
    
    append(&world.bodies, body)
    return len(world.bodies) - 1
}

// Create a sphere rigid body
create_sphere :: proc(world: ^PhysicsWorld, position: Vector3, radius: f32, mass: f32) -> int {
    body := RigidBody{
        position = position,
        position_prev = position,
        orientation = {0, 0, 0, 1}, // Identity quaternion
        orientation_prev = {0, 0, 0, 1},
        linear_velocity = {0, 0, 0},
        angular_velocity = {0, 0, 0},
        mass = mass,
        restitution = 0.7,
        static_friction = 0.3,
        dynamic_friction = 0.2,
        shape_type = .Sphere,
        shape_size = {radius, 0, 0},  // Store radius in x component
    }
    
    // Calculate inverse mass (0 for static objects)
    if mass <= 0 {
        body.inverse_mass = 0
    } else {
        body.inverse_mass = 1.0 / mass
    }
    
    // Calculate inertia tensor for a sphere
    inertia := (2.0/5.0) * mass * radius * radius
    
    body.inertia_tensor = matrix[3, 3]f32{
        inertia, 0, 0,
        0, inertia, 0,
        0, 0, inertia,
    }
    
    // Calculate inverse inertia tensor
    if mass <= 0 {
        body.inverse_inertia_tensor = {}
    } else {
        body.inverse_inertia_tensor = matrix[3, 3]f32{
            1/inertia, 0, 0,
            0, 1/inertia, 0,
            0, 0, 1/inertia,
        }
    }
    
    append(&world.bodies, body)
    return len(world.bodies) - 1
}

// Set velocity for a rigid body
set_velocity :: proc(world: ^PhysicsWorld, body_index: int, linear_velocity: Vector3, angular_velocity: Vector3) {
    if body_index >= 0 && body_index < len(world.bodies) {
        world.bodies[body_index].linear_velocity = linear_velocity
        world.bodies[body_index].angular_velocity = angular_velocity
    }
}

// Get position of a rigid body
get_position :: proc(world: ^PhysicsWorld, body_index: int) -> Vector3 {
    if body_index >= 0 && body_index < len(world.bodies) {
        return world.bodies[body_index].position
    }
    return {0, 0, 0}
}

// Get orientation of a rigid body
get_orientation :: proc(world: ^PhysicsWorld, body_index: int) -> Quaternion {
    if body_index >= 0 && body_index < len(world.bodies) {
        return world.bodies[body_index].orientation
    }
    return {0, 0, 0, 1}
}

// Get velocity of a rigid body
get_velocity :: proc(world: ^PhysicsWorld, body_index: int) -> Vector3 {
    if body_index >= 0 && body_index < len(world.bodies) {
        return world.bodies[body_index].linear_velocity
    }
    return {0, 0, 0}
}

// Add collision detection helper functions
detect_sphere_sphere :: proc(a, b: ^RigidBody) -> (bool, Contact) {
    delta := a.position - b.position
    distance_squared := linalg.dot(delta, delta)
    
    radius_sum := a.shape_size.x + b.shape_size.x
    
    if distance_squared < radius_sum * radius_sum {
        distance := math.sqrt(distance_squared)
        normal := delta / distance
        
        contact := Contact{
            body_a = a,
            body_b = b,
            normal = normal,
            penetration = radius_sum - distance,
            static_friction = (a.static_friction + b.static_friction) * 0.5,
            dynamic_friction = (a.dynamic_friction + b.dynamic_friction) * 0.5,
            local_point_a = normal * -a.shape_size.x,
            local_point_b = normal * b.shape_size.x,
        }
        
        return true, contact
    }
    
    return false, Contact{}
}

detect_box_sphere :: proc(box, sphere: ^RigidBody) -> (bool, Contact) {
    // Find closest point on box to sphere center
    closest := Vector3{
        clamp(sphere.position.x, box.position.x - box.shape_size.x, box.position.x + box.shape_size.x),
        clamp(sphere.position.y, box.position.y - box.shape_size.y, box.position.y + box.shape_size.y),
        clamp(sphere.position.z, box.position.z - box.shape_size.z, box.position.z + box.shape_size.z),
    }
    
    // Vector from closest point to sphere center
    delta := sphere.position - closest
    dist_squared := linalg.dot(delta, delta)
    
    if dist_squared < sphere.shape_size.x * sphere.shape_size.x {
        dist := math.sqrt(dist_squared)
        normal := delta
        if dist > 0 {
            normal = delta / dist
        } else {
            normal = {0, 1, 0}
        }
        
        contact := Contact{
            body_a = sphere,
            body_b = box,
            normal = normal,
            penetration = sphere.shape_size.x - dist,
            static_friction = (sphere.static_friction + box.static_friction) * 0.5,
            dynamic_friction = (sphere.dynamic_friction + box.dynamic_friction) * 0.5,
            local_point_a = normal * -sphere.shape_size.x,
            local_point_b = closest - box.position,
        }
        
        return true, contact
    }
    
    return false, Contact{}
}

detect_box_box :: proc(a, b: ^RigidBody) -> (bool, Contact) {
    // For now, we'll implement AABB vs AABB collision
    // Later this could be extended to handle oriented boxes using SAT
    
    // Check for overlap on each axis
    overlap_x := a.shape_size.x + b.shape_size.x - 
                 abs(a.position.x - b.position.x)
    overlap_y := a.shape_size.y + b.shape_size.y - 
                 abs(a.position.y - b.position.y)
    overlap_z := a.shape_size.z + b.shape_size.z - 
                 abs(a.position.z - b.position.z)
    
    // If there's no overlap on any axis, no collision
    if overlap_x <= 0 || overlap_y <= 0 || overlap_z <= 0 {
        return false, Contact{}
    }
    
    // Find smallest overlap to determine collision normal
    normal := Vector3{0, 1, 0}
    penetration := overlap_y
    
    if overlap_x < overlap_y && overlap_x < overlap_z {
        penetration = overlap_x
        normal = {1, 0, 0}
        if a.position.x < b.position.x {
            normal = {-1, 0, 0}
        }
    } else if overlap_z < overlap_y {
        penetration = overlap_z
        normal = {0, 0, 1}
        if a.position.z < b.position.z {
            normal = {0, 0, -1}
        }
    } else {
        if a.position.y < b.position.y {
            normal = {0, -1, 0}
        }
    }
    
    // Find contact point (center of overlap region)
    min_b := b.position - b.shape_size
    max_b := b.position + b.shape_size
    
    // Project point a onto box b's bounds
    contact_point := Vector3{
        clamp(a.position.x, min_b.x, max_b.x),
        clamp(a.position.y, min_b.y, max_b.y), 
        clamp(a.position.z, min_b.z, max_b.z),
    }
    contact := Contact{
        body_a = a,
        body_b = b,
        normal = normal,
        penetration = penetration,
        static_friction = (a.static_friction + b.static_friction) * 0.5,
        dynamic_friction = (a.dynamic_friction + b.dynamic_friction) * 0.5,
        local_point_a = contact_point - a.position,
        local_point_b = contact_point - b.position,
    }
    
    return true, contact
}

// Update detect_contacts to use shape types
detect_contacts :: proc(world: ^PhysicsWorld) {
    clear(&world.contacts)
    
    for i := 0; i < len(world.bodies); i += 1 {
        for j := i + 1; j < len(world.bodies); j += 1 {
            body_a := &world.bodies[i]
            body_b := &world.bodies[j]
            
            // Skip if both bodies are static
            if body_a.inverse_mass == 0 && body_b.inverse_mass == 0 {
                continue
            }
            
            has_contact := false
            contact: Contact
            
            // Choose collision detection based on shape types
            if body_a.shape_type == .Sphere && body_b.shape_type == .Sphere {
                has_contact, contact = detect_sphere_sphere(body_a, body_b)
            } else if body_a.shape_type == .Box && body_b.shape_type == .Sphere {
                has_contact, contact = detect_box_sphere(body_a, body_b)
            } else if body_a.shape_type == .Sphere && body_b.shape_type == .Box {
                has_contact, contact = detect_box_sphere(body_b, body_a)
                if has_contact {
                    // Swap normal direction since we swapped bodies
                    contact.normal = -contact.normal
                    contact.body_a = body_a
                    contact.body_b = body_b
                }
            } else if body_a.shape_type == .Box && body_b.shape_type == .Box {
                has_contact, contact = detect_box_box(body_a, body_b)
            }
            
            if has_contact {
                append(&world.contacts, contact)
            }
        }
    }
}

// Update the physics world by one time step
update_physics :: proc(world: ^PhysicsWorld, dt: f32) {
    // Detect collisions and generate contacts
    detect_contacts(world)
    
    // Solve the system using XPBD
    solve_xpbd(world.bodies[:], world.contacts[:], dt)
}