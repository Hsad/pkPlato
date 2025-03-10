package game

import "core:math"
import "core:math/linalg"

Vector3 :: [3]f32
Quaternion :: [4]f32

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