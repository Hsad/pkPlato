package game

import "core:math"
import "core:math/linalg"
import "core:fmt"
import rl "vendor:raylib"
//Vector3 :: [3]f32
Quaternion :: [4]f32

Shape_Type :: enum {
    Box,
    Sphere,
}

RigidBody :: struct {
    // Position and orientation
    position: rl.Vector3,
    position_prev: rl.Vector3,
    orientation: Quaternion,
    orientation_prev: Quaternion,
    
    // Velocities
    linear_velocity: rl.Vector3,
    angular_velocity: rl.Vector3,
    
    // Physical properties
    mass: f32,
    inverse_mass: f32,
    inertia_tensor: matrix[3, 3]f32,
    inverse_inertia_tensor: matrix[3, 3]f32,
    
    // Contact and friction
    restitution: f32,
    static_friction: f32,
    dynamic_friction: f32,
    
    // Compliance parameters for XPBD
    compliance: f32,  // Material compliance (inverse of stiffness)
    
    // Damping factor (0 = no damping, 1 = full damping)
    damping: f32,
    
    // Shape information
    shape_type: Shape_Type,
    shape_size: rl.Vector3,  // For box: half-extents, For sphere: x component is radius
}

// Distance constraint between two rigid bodies
DistanceConstraint :: struct {
    // Bodies involved
    body_a: ^RigidBody,
    body_b: ^RigidBody,
    
    // Attachment points in local coordinates
    local_point_a: rl.Vector3,
    local_point_b: rl.Vector3,
    
    // Target distance
    distance: f32,
    
    // Constraint properties
    compliance: f32,
    unilateral: bool,  // If true, only enforces distance when stretched beyond target
    
    // Accumulated impulse for warm starting
    lambda: f32,
}

Contact :: struct {
    // Bodies involved
    body_a: ^RigidBody,
    body_b: ^RigidBody,
    
    // Contact point in local coordinates
    local_point_a: rl.Vector3,
    local_point_b: rl.Vector3,
    
    // Contact normal and penetration depth
    normal: rl.Vector3,
    penetration: f32,
    
    // Friction coefficients
    static_friction: f32,
    dynamic_friction: f32,
    
    // Lagrange multipliers for normal and tangential forces
    lambda_normal: f32,
    lambda_tangent: f32,
}

// Helper function to transform a local point to world space
transform_point :: proc(body: ^RigidBody, local_point: rl.Vector3) -> rl.Vector3 {
    // Rotate the local point using the body's orientation quaternion
    rotated_point := rotate_vector(body.orientation, local_point)
    
    // Translate to world space
    world_point := rotated_point + body.position
    return world_point
}

// Helper function to transform a world point to local space
world_to_local :: proc(body: ^RigidBody, world_point: rl.Vector3) -> rl.Vector3 {
    // Translate to origin-relative coordinates
    origin_relative := world_point - body.position
    
    // Create inverse quaternion
    inv_orientation := quaternion_conjugate(body.orientation)
    
    // Rotate using inverse orientation
    local_point := rotate_vector(inv_orientation, origin_relative)
    
    return local_point
}

// Helper function to calculate relative velocity at contact point
calculate_relative_velocity :: proc(contact: ^Contact) -> rl.Vector3 {
    r1 := transform_point(contact.body_a, contact.local_point_a) - contact.body_a.position
    r2 := transform_point(contact.body_b, contact.local_point_b) - contact.body_b.position
    
    v1 := contact.body_a.linear_velocity + cross(contact.body_a.angular_velocity, r1)
    v2 := contact.body_b.linear_velocity + cross(contact.body_b.angular_velocity, r2)
    
    return v1 - v2
}

// Calculate cross product of two Vector3s
cross :: proc(a, b: rl.Vector3) -> rl.Vector3 {
    return {
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    }
}

// Transform a vector using the rotation represented by a quaternion
rotate_vector :: proc(q: Quaternion, v: rl.Vector3) -> rl.Vector3 {
    // Proper quaternion rotation: q * v * q^-1
    // Using the formula: v' = v + 2 * cross(q.xyz, cross(q.xyz, v) + q.w * v)
    qv := rl.Vector3{q.x, q.y, q.z}
    t := cross(qv, cross(qv, v) + v * q.w) * 2.0
    return v + t
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
    
    // Calculate positional correction using XPBD
    // Combine compliance from both bodies
    combined_compliance := body_a.compliance + body_b.compliance
    
    // Add a small bias to prevent objects from sinking
    bias_factor : f32= 0.2  // Baumgarte stabilization factor
    bias := bias_factor * max(0, -d) / dt
    
    // Convert compliance to alpha_tilde (α̃ = α/h²)
    alpha_tilde := combined_compliance / (dt * dt)
    
    // XPBD constraint update: Δλ = (-c - α̃λ) / (w + α̃)
    denominator := w + alpha_tilde
    
    // Add safety check to prevent division by zero
    if denominator < 1e-10 {
        return
    }
    
    delta_lambda := (-d - bias - alpha_tilde * contact.lambda_normal) / denominator
    contact.lambda_normal += delta_lambda
    
    // Apply positional correction for normal constraint
    impulse := contact.normal * delta_lambda
    
    // Apply to bodies
    if body_a.inverse_mass > 0 {
        body_a.position += impulse * body_a.inverse_mass
        
        // Apply angular correction
        angular_impulse := cross(r1, impulse)
        angular_correction := linalg.matrix_mul_vector(body_a.inverse_inertia_tensor, angular_impulse)
        
        // Create quaternion from angular correction
        delta_q := Quaternion{
            angular_correction[0] * 0.5,
            angular_correction[1] * 0.5,
            angular_correction[2] * 0.5,
            0}
        
        // Apply to orientation
        body_a.orientation = quaternion_multiply(delta_q, body_a.orientation)
        body_a.orientation = normalize_quaternion(body_a.orientation)
    }
    
    if body_b.inverse_mass > 0 {
        body_b.position -= impulse * body_b.inverse_mass
        
        // Apply angular correction
        angular_impulse := cross(r2, -impulse)
        angular_correction := linalg.matrix_mul_vector(body_b.inverse_inertia_tensor, angular_impulse)
        
        // Create quaternion from angular correction
        delta_q := Quaternion{
            angular_correction[0] * 0.5,
            angular_correction[1] * 0.5,
            angular_correction[2] * 0.5,
            0,
        }
        
        // Apply to orientation
        body_b.orientation = quaternion_multiply(delta_q, body_b.orientation)
        body_b.orientation = normalize_quaternion(body_b.orientation)
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
            
            // Calculate impulse magnitude using XPBD
            // For friction, we typically use zero compliance (infinite stiffness)
            delta_lambda_t := -linalg.length(tangent_correction) / w_t
            
            // Clamp to friction cone
            max_lambda_t := combined_static_friction * contact.lambda_normal
            delta_lambda_t = math.max(-max_lambda_t - contact.lambda_tangent, 
                                     math.min(delta_lambda_t, max_lambda_t - contact.lambda_tangent))
            
            // Apply impulse
            impulse_t := tangent * delta_lambda_t
            
            if body_a.inverse_mass > 0 {
                body_a.position += impulse_t * body_a.inverse_mass
                
                // Apply angular correction
                angular_impulse := cross(r1, impulse_t)
                angular_correction := linalg.matrix_mul_vector(body_a.inverse_inertia_tensor, angular_impulse)
                
                // Create quaternion from angular correction
                delta_q := Quaternion{
                    angular_correction[0] * 0.5,
                    angular_correction[1] * 0.5,
                    angular_correction[2] * 0.5,
                    0,
                }
                
                // Apply to orientation
                body_a.orientation = quaternion_multiply(delta_q, body_a.orientation)
                body_a.orientation = normalize_quaternion(body_a.orientation)
            }
            
            if body_b.inverse_mass > 0 {
                body_b.position -= impulse_t * body_b.inverse_mass
                
                // Apply angular correction
                angular_impulse := cross(r2, -impulse_t)
                angular_correction := linalg.matrix_mul_vector(body_b.inverse_inertia_tensor, angular_impulse)
                
                // Create quaternion from angular correction
                delta_q := Quaternion{
                    angular_correction[0] * 0.5,
                    angular_correction[1] * 0.5,
                    angular_correction[2] * 0.5,
                    0,
                }
                
                // Apply to orientation
                body_b.orientation = quaternion_multiply(delta_q, body_b.orientation)
                body_b.orientation = normalize_quaternion(body_b.orientation)
            }
            
            // Track accumulated tangent impulse magnitude
            contact.lambda_tangent += delta_lambda_t
        }
    }
}

// Normalize a quaternion
normalize_quaternion :: proc(q: Quaternion) -> Quaternion {
    length := math.sqrt(
        q.x * q.x +
        q.y * q.y +
        q.z * q.z +
        q.w * q.w)
    
    // Add safety check to prevent division by zero
    if length < 1e-10 {
        return Quaternion{0, 0, 0, 1} // Return identity quaternion if length is too small
    }
    
    return {
        q.x / length,
        q.y / length,
        q.z / length,
        q.w / length,
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
apply_angular_impulse :: proc(body: ^RigidBody, angular_impulse: rl.Vector3) -> Quaternion {
    // Convert angular impulse to a quaternion change
    // For small rotations: dq = [0.5 * dt * ω, 0] * q
    delta_rotation := Quaternion{
        angular_impulse[0] * 0.5,
        angular_impulse[1] * 0.5,
        angular_impulse[2] * 0.5,
        0,
    }
    
    // Combine with current orientation
    new_orientation := quaternion_multiply(delta_rotation, body.orientation)
    
    // Use the normalize_quaternion function which now has safety checks
    return normalize_quaternion(new_orientation)
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

// Integrate quaternion with angular velocity
integrate_quaternion :: proc(q: Quaternion, angular_velocity: rl.Vector3, dt: f32) -> Quaternion {
    // Create a quaternion from angular velocity
    // For small time steps: dq = [0.5 * dt * ω, 0] * q
    omega_quat := Quaternion{
        angular_velocity[0] * dt * 0.5,
        angular_velocity[1] * dt * 0.5,
        angular_velocity[2] * dt * 0.5,
        0,
    }
    
    // Apply to current quaternion
    result := quaternion_multiply(omega_quat, q)
    
    // Normalize to ensure unit quaternion
    length := math.sqrt(
        result[0] * result[0] +
        result[1] * result[1] +
        result[2] * result[2] +
        result[3] * result[3])
    
    return {
        result[0] / length,
        result[1] / length,
        result[2] / length,
        result[3] / length,
    }
}

// Apply restitution at velocity level
apply_restitution :: proc(contact: ^Contact, dt: f32) {
    // Calculate relative velocity at contact point
    rel_vel := calculate_relative_velocity(contact)
    
    // Extract normal component of relative velocity
    normal_vel := linalg.dot(rel_vel, contact.normal)
    
    // Skip if separating (positive normal velocity) or very small velocity
    if normal_vel >= 0 || abs(normal_vel) < 0.01 {
        return
    }
    
    // Bodies involved
    body_a := contact.body_a
    body_b := contact.body_b
    
    // Skip if both bodies have infinite mass
    if body_a.inverse_mass == 0 && body_b.inverse_mass == 0 {
        return
    }
    
    // Vectors from center of mass to contact points
    r1 := transform_point(body_a, contact.local_point_a) - body_a.position
    r2 := transform_point(body_b, contact.local_point_b) - body_b.position
    
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
    
    // Compute combined restitution coefficient
    // Use the higher restitution value for more bounce
    combined_restitution := math.max(body_a.restitution, body_b.restitution)
    
    // Apply velocity threshold for restitution to prevent tiny bounces
    RESTITUTION_THRESHOLD :: f32(1.0)  // Only apply restitution for significant impacts
    
    // Scale restitution based on impact velocity
    effective_restitution := combined_restitution
    if abs(normal_vel) < RESTITUTION_THRESHOLD {
        // Linearly reduce restitution for low-speed impacts
        effective_restitution *= abs(normal_vel) / RESTITUTION_THRESHOLD
    }
    
    // Calculate restitution impulse
    // The target velocity is -restitution * normal_vel
    delta_vel := -normal_vel * (1.0 + effective_restitution)
    
    // Calculate impulse magnitude
    impulse_magnitude := delta_vel / w
    
    // Apply impulse
    impulse := contact.normal * impulse_magnitude
    
    // Apply to linear velocities with a minimum impulse to prevent sticking
    MIN_IMPULSE :: f32(0.05)  // Minimum impulse to apply
    
    if body_a.inverse_mass > 0 {
        // Add a small upward component to prevent sticking
        adjusted_impulse := impulse
        if adjusted_impulse.y < MIN_IMPULSE && contact.normal.y > 0.7 {
            adjusted_impulse.y += MIN_IMPULSE
        }
        
        body_a.linear_velocity += adjusted_impulse * body_a.inverse_mass
        body_a.angular_velocity += linalg.matrix_mul_vector(body_a.inverse_inertia_tensor, cross(r1, impulse))
    }
    
    if body_b.inverse_mass > 0 {
        // Add a small upward component to prevent sticking
        adjusted_impulse := impulse
        if adjusted_impulse.y < MIN_IMPULSE && contact.normal.y < -0.7 {
            adjusted_impulse.y -= MIN_IMPULSE
        }
        
        body_b.linear_velocity -= adjusted_impulse * body_b.inverse_mass
        body_b.angular_velocity -= linalg.matrix_mul_vector(body_b.inverse_inertia_tensor, cross(r2, impulse))
    }
}

// Physics world to manage all bodies and contacts
PhysicsWorld :: struct {
    bodies: [dynamic]RigidBody,
    contacts: [dynamic]Contact,
    distance_constraints: [dynamic]DistanceConstraint,
    
    // Solver configuration
    num_substeps: int,           // Number of substeps per update
    gravity: rl.Vector3,            // Gravity vector
}

// Create a new physics world
create_physics_world :: proc() -> ^PhysicsWorld {
    world := new(PhysicsWorld)
    world.bodies = make([dynamic]RigidBody)
    world.contacts = make([dynamic]Contact)
    world.distance_constraints = make([dynamic]DistanceConstraint)
    
    // Default configuration
    world.num_substeps = 10
    world.gravity = {0, -9.81, 0}
    
    return world
}

// Configure the physics world
configure_physics_world :: proc(world: ^PhysicsWorld, num_substeps: int, gravity: rl.Vector3) {
    world.num_substeps = num_substeps
    world.gravity = gravity
}

// Destroy a physics world and free its resources
destroy_physics_world :: proc(world: ^PhysicsWorld) {
    fmt.println("Destroying physics world")
    delete(world.bodies)
    delete(world.contacts)
    delete(world.distance_constraints)
    free(world)
}

// Create a box rigid body with compliance parameter
create_box :: proc(world: ^PhysicsWorld, position: rl.Vector3, size: rl.Vector3, mass: f32, compliance: f32 = 0.01) -> int {
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
        compliance = compliance,  // Use provided compliance value
        damping = 0.5,
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

// Create a sphere rigid body with compliance parameter
create_sphere :: proc(world: ^PhysicsWorld, position: rl.Vector3, radius: f32, mass: f32, compliance: f32 = 0.005) -> int {
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
        compliance = compliance,  // Use provided compliance value
        damping = 0.5,
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
set_velocity :: proc(world: ^PhysicsWorld, body_index: int, linear_velocity: rl.Vector3, angular_velocity: rl.Vector3) {
    if body_index >= 0 && body_index < len(world.bodies) {
        world.bodies[body_index].linear_velocity = linear_velocity
        world.bodies[body_index].angular_velocity = angular_velocity
    } else {
        fmt.println("set_velocity: body_index:", body_index, "linear_velocity:", linear_velocity, "angular_velocity:", angular_velocity)
        fmt.println("set_velocity: body_index out of bounds")
    }
}

// Get position of a rigid body
get_position :: proc(world: ^PhysicsWorld, body_index: int) -> rl.Vector3 {
    if body_index >= 0 && body_index < len(world.bodies) {
        return world.bodies[body_index].position
    }
    fmt.println("get_position: body_index out of bounds")
    return {0, 0, 0}
}

// Get orientation of a rigid body
get_orientation :: proc(world: ^PhysicsWorld, body_index: int) -> Quaternion {
    if body_index >= 0 && body_index < len(world.bodies) {
        return world.bodies[body_index].orientation
    }
    fmt.println("get_orientation: body_index out of bounds")
    return {0, 0, 0, 1}
}

// Get velocity of a rigid body
get_velocity :: proc(world: ^PhysicsWorld, body_index: int) -> rl.Vector3 {
    if body_index >= 0 && body_index < len(world.bodies) {
        return world.bodies[body_index].linear_velocity
    }
    fmt.println("get_velocity: body_index out of bounds")
    return {0, 0, 0}
}

// Add collision detection helper functions
detect_sphere_sphere :: proc(a, b: ^RigidBody) -> (bool, Contact) {
    delta := a.position - b.position
    distance_squared := linalg.dot(delta, delta)
    
    radius_sum := a.shape_size.x + b.shape_size.x
    
    if distance_squared < radius_sum * radius_sum {
        distance := math.sqrt(distance_squared)
        
        // Avoid division by zero with a more robust check
        normal := rl.Vector3{0, 1, 0}
        if distance > 0.0001 {
            normal = delta / distance
        }
        
        // Validate normal vector
        if math.is_nan(normal.x) || math.is_nan(normal.y) || math.is_nan(normal.z) {
            normal = {0, 1, 0}
        }
        
        contact := Contact{
            body_a = a,
            body_b = b,
            normal = normal,
            penetration = radius_sum - distance,
            static_friction = (a.static_friction + b.static_friction) * 0.5,
            dynamic_friction = (a.dynamic_friction + b.dynamic_friction) * 0.5,
            local_point_a = normal * -a.shape_size.x,
            local_point_b = normal * b.shape_size.x,
            lambda_normal = 0,
            lambda_tangent = 0,
        }
        
        return true, contact
    }
    
    return false, Contact{}
}

detect_box_sphere :: proc(box, sphere: ^RigidBody) -> (bool, Contact) {
    // Find closest point on box to sphere center
    closest := rl.Vector3{
        clamp(sphere.position.x, box.position.x - box.shape_size.x, box.position.x + box.shape_size.x),
        clamp(sphere.position.y, box.position.y - box.shape_size.y, box.position.y + box.shape_size.y),
        clamp(sphere.position.z, box.position.z - box.shape_size.z, box.position.z + box.shape_size.z),
    }
    
    // Vector from closest point to sphere center
    delta := sphere.position - closest
    dist_squared := linalg.dot(delta, delta)
    
    if dist_squared < sphere.shape_size.x * sphere.shape_size.x {
        dist := math.sqrt(dist_squared)
        
        // Avoid division by zero
        normal := rl.Vector3{0, 1, 0}
        if dist > 0.0001 {
            normal = delta / dist
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
            lambda_normal = 0,
            lambda_tangent = 0,
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
    normal := rl.Vector3{0, 1, 0}
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
    min_a := a.position - a.shape_size
    max_a := a.position + a.shape_size
    min_b := b.position - b.shape_size
    max_b := b.position + b.shape_size
    
    // Calculate overlap box
    overlap_min := rl.Vector3{
        math.max(min_a.x, min_b.x),
        math.max(min_a.y, min_b.y),
        math.max(min_a.z, min_b.z),
    }
    
    overlap_max := rl.Vector3{
        math.min(max_a.x, max_b.x),
        math.min(max_a.y, max_b.y),
        math.min(max_a.z, max_b.z),
    }
    
    // Contact point at center of overlap region
    contact_point := (overlap_min + overlap_max) * 0.5
    
    contact := Contact{
        body_a = a,
        body_b = b,
        normal = normal,
        penetration = penetration,
        static_friction = (a.static_friction + b.static_friction) * 0.5,
        dynamic_friction = (a.dynamic_friction + b.dynamic_friction) * 0.5,
        local_point_a = contact_point - a.position,
        local_point_b = contact_point - b.position,
        lambda_normal = 0,
        lambda_tangent = 0,
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
    //fmt.println("update_physics: dt:", dt)
    // Use the configured number of substeps and iterations
    substep_dt := dt / f32(world.num_substeps)
    
    // Run the simulation for each substep
    for substep := 0; substep < world.num_substeps; substep += 1 {
        //fmt.println("update_physics: substep:", substep)
        // Store previous state for all bodies
        for i := 0; i < len(world.bodies); i += 1 {
            world.bodies[i].position_prev = world.bodies[i].position
            world.bodies[i].orientation_prev = world.bodies[i].orientation
        }
        
        // Apply external forces (like gravity) and integrate velocities
        for i := 0; i < len(world.bodies); i += 1 {
            if world.bodies[i].inverse_mass > 0 {
                // Apply gravity
                world.bodies[i].linear_velocity += world.gravity * substep_dt
                
                // Integrate positions
                world.bodies[i].position += world.bodies[i].linear_velocity * substep_dt
                
                // Integrate orientations using proper quaternion integration
                world.bodies[i].orientation = integrate_quaternion(
                    world.bodies[i].orientation, 
                    world.bodies[i].angular_velocity, 
                    substep_dt)
            }
        }
        
        // Detect collisions and generate contacts
        detect_contacts(world)
        
        // Solve distance constraints
        for i := 0; i < len(world.distance_constraints); i += 1 {
            solve_distance_constraint(&world.distance_constraints[i], substep_dt)
        }
        
        // Solve contact constraints
        for i := 0; i < len(world.contacts); i += 1 {
            solve_contact(&world.contacts[i], substep_dt)
        }
        
        // Update velocities from positions
        VELOCITY_DAMPING :: 0.98  // Slight damping to prevent excessive oscillation
        for i := 0; i < len(world.bodies); i += 1 {
            if world.bodies[i].inverse_mass > 0 {
                // Linear velocity update with damping
                new_velocity := (world.bodies[i].position - world.bodies[i].position_prev) / substep_dt
                
                // Blend between previous velocity and new velocity to prevent sudden stops
                world.bodies[i].linear_velocity = linalg.lerp(
                    world.bodies[i].linear_velocity,
                    new_velocity,
                    0.8,  // Blend factor - higher values follow position changes more closely
                ) * math.max(1.0 - world.bodies[i].damping * substep_dt, 0.0)
                
                // Angular velocity update from quaternion difference
                q_diff := quaternion_multiply(
                    world.bodies[i].orientation, 
                    quaternion_conjugate(world.bodies[i].orientation_prev))
                
                // For small rotations, angular velocity can be approximated as:
                // ω = 2 * q_diff.xyz / dt (when q_diff.w is close to 1)
                new_angular_velocity := rl.Vector3{
                    q_diff[0] * 2.0 / substep_dt,
                    q_diff[1] * 2.0 / substep_dt,
                    q_diff[2] * 2.0 / substep_dt,
                }
                
                // Blend angular velocities with damping
                world.bodies[i].angular_velocity = linalg.lerp(
                    world.bodies[i].angular_velocity,
                    new_angular_velocity,
                    0.8) * math.max(1.0 - world.bodies[i].damping * substep_dt, 0.0)
                
                // Apply velocity-level constraints
                
                // Apply restitution
                for j := 0; j < len(world.contacts); j += 1 {
                    apply_restitution(&world.contacts[j], substep_dt)
                }
                
                // Apply dynamic friction
                for j := 0; j < len(world.contacts); j += 1 {
                    apply_dynamic_friction(&world.contacts[j], substep_dt)
                }
                
                // Add safety check for NaN values
                if math.is_nan(world.bodies[i].position.x) || 
                   math.is_nan(world.bodies[i].position.y) || 
                   math.is_nan(world.bodies[i].position.z) {
                    world.bodies[i].position = world.bodies[i].position_prev
                    world.bodies[i].linear_velocity = {0, 0, 0}
                }
                
                if math.is_nan(world.bodies[i].angular_velocity.x) || 
                   math.is_nan(world.bodies[i].angular_velocity.y) || 
                   math.is_nan(world.bodies[i].angular_velocity.z) {
                    world.bodies[i].angular_velocity = {0, 0, 0}
                    world.bodies[i].orientation = world.bodies[i].orientation_prev
                }
                
                // Ensure objects don't sink below the ground
                if world.bodies[i].position.y < 0.01 && world.bodies[i].linear_velocity.y < 0 {
                    world.bodies[i].position.y = 0.01
                    
                    // Apply a small upward velocity to prevent sticking
                    if abs(world.bodies[i].linear_velocity.y) < 0.1 {
                        world.bodies[i].linear_velocity.y = 0.1
                    }
                }
            }
        }
    }
}

// Quaternion conjugate (inverse for unit quaternions)
quaternion_conjugate :: proc(q: Quaternion) -> Quaternion {
    return {-q[0], -q[1], -q[2], q[3]}
}

// Set material properties for a rigid body
set_material_properties :: proc(world: ^PhysicsWorld, body_index: int, restitution: f32, static_friction: f32, dynamic_friction: f32, compliance: f32) {
    if body_index >= 0 && body_index < len(world.bodies) {
        world.bodies[body_index].restitution = restitution
        world.bodies[body_index].static_friction = static_friction
        world.bodies[body_index].dynamic_friction = dynamic_friction
        world.bodies[body_index].compliance = compliance
    }
}

// Set compliance for a rigid body
set_compliance :: proc(world: ^PhysicsWorld, body_index: int, compliance: f32) {
    if body_index >= 0 && body_index < len(world.bodies) {
        world.bodies[body_index].compliance = compliance
    }
}

// Create a new distance constraint
create_distance_constraint :: proc(world: ^PhysicsWorld, body_a_index, body_b_index: int, 
                                  point_a, point_b: rl.Vector3, distance: f32, 
                                  compliance: f32 = 0.0, unilateral: bool = false) -> int {
    if body_a_index < 0 || body_a_index >= len(world.bodies) || 
       body_b_index < 0 || body_b_index >= len(world.bodies) {
        return -1
    }
    
    body_a := &world.bodies[body_a_index]
    body_b := &world.bodies[body_b_index]
    
    // Convert world points to local coordinates using proper quaternion rotation
    local_a := world_to_local(body_a, point_a)
    local_b := world_to_local(body_b, point_b)
    
    constraint := DistanceConstraint{
        body_a = body_a,
        body_b = body_b,
        local_point_a = local_a,
        local_point_b = local_b,
        distance = distance,
        compliance = compliance,
        unilateral = unilateral,
        lambda = 0,
    }
    
    append(&world.distance_constraints, constraint)
    return len(world.distance_constraints) - 1
}

// Solve a distance constraint
solve_distance_constraint :: proc(constraint: ^DistanceConstraint, dt: f32) {
    // Get current world space attachment points
    world_point_a := transform_point(constraint.body_a, constraint.local_point_a)
    world_point_b := transform_point(constraint.body_b, constraint.local_point_b)
    
    // Calculate current distance vector
    delta := world_point_b - world_point_a
    current_distance := linalg.length(delta)
    
    // Skip if zero distance to avoid division by zero
    if current_distance < 1e-6 {
        return
    }
    
    // Normalize direction
    direction := delta / current_distance
    
    // For unilateral constraints, only enforce if stretched beyond target
    if constraint.unilateral && current_distance <= constraint.distance {
        return
    }
    
    // Calculate constraint violation
    c := current_distance - constraint.distance
    
    // Calculate inverse mass contributions
    w_a := calculate_inverse_mass(constraint.body_a, direction, world_point_a)
    w_b := calculate_inverse_mass(constraint.body_b, direction, world_point_b)
    
    // Combined inverse mass
    w := w_a + w_b
    
    // Skip if infinite mass
    if w <= 0 {
        return
    }
    
    // XPBD constraint update
    alpha := constraint.compliance / (dt * dt)
    delta_lambda := (-c - alpha * constraint.lambda) / (w + alpha)
    constraint.lambda += delta_lambda
    
    // Calculate impulse
    impulse := direction * delta_lambda
    
    // Apply impulse to bodies
    apply_impulse(constraint.body_a, impulse, world_point_a)
    apply_impulse(constraint.body_b, -impulse, world_point_b)
}

// Calculate inverse mass contribution for a point and direction
calculate_inverse_mass :: proc(body: ^RigidBody, direction: rl.Vector3, world_point: rl.Vector3) -> f32 {
    if body.inverse_mass == 0 {
        return 0
    }
    
    // Calculate moment arm
    r := world_point - body.position
    
    // Calculate angular contribution
    r_cross_n := cross(r, direction)
    angular_term := linalg.dot(r_cross_n, linalg.matrix_mul_vector(body.inverse_inertia_tensor, r_cross_n))
    
    // Total inverse mass including both linear and angular terms
    return body.inverse_mass + angular_term
}

// Apply an impulse to a rigid body at a specific point
apply_impulse :: proc(body: ^RigidBody, impulse: rl.Vector3, world_point: rl.Vector3) {
    if body.inverse_mass == 0 {
        return
    }
    
    // Apply linear impulse
    body.position += impulse * body.inverse_mass
    
    // Calculate angular impulse
    r := world_point - body.position
    angular_impulse := cross(r, impulse)
    angular_correction := linalg.matrix_mul_vector(body.inverse_inertia_tensor, angular_impulse)
    
    // Create quaternion from angular correction
    delta_q := Quaternion{
        angular_correction[0] * 0.5,
        angular_correction[1] * 0.5,
        angular_correction[2] * 0.5,
        0,
    }
    
    // Apply to orientation
    body.orientation = quaternion_multiply(delta_q, body.orientation)
    body.orientation = normalize_quaternion(body.orientation)
}

// Create a static ground plane
create_ground_plane :: proc(world: ^PhysicsWorld, y_position: f32 = 0.0, size: f32 = 50.0) -> int {
    // Create a large, flat box with infinite mass (static)
    ground_size := rl.Vector3{size, 0.1, size}
    ground_pos := rl.Vector3{0, y_position - 0.05, 0}  // Position slightly below the y_position to ensure contact
    
    ground_index := create_box(world, ground_pos, ground_size, 0)  // Mass of 0 means static/infinite mass
    
    // Set material properties for the ground
    set_material_properties(world, ground_index, 0.3, 0.8, 0.7, 0.0)
    
    return ground_index
}

// Create a chain of rigid bodies connected by distance constraints
create_chain :: proc(world: ^PhysicsWorld, start_pos: rl.Vector3, num_links: int, link_size: rl.Vector3, 
                    link_mass: f32, link_distance: f32, compliance: f32 = 0.001, unilateral: bool = false) -> []int {
    if num_links <= 0 {
        return nil
    }
    
    // Allocate array to store body indices
    body_indices := make([]int, num_links)
    
    // Create the first body
    current_pos := start_pos
    prev_body_index := create_box(world, current_pos, link_size, link_mass)
    body_indices[0] = prev_body_index
    
    // Set damping to prevent excessive oscillation
    world.bodies[prev_body_index].damping = 5.0
    
    // Create the rest of the chain
    for i := 1; i < num_links; i += 1 {
        // Position for the next link
        current_pos.y -= link_distance + link_size.y
        
        // Create the next body
        body_index := create_box(world, current_pos, link_size, link_mass)
        body_indices[i] = body_index
        
        // Set damping
        world.bodies[body_index].damping = 5.0
        
        // Create attachment points
        p1 := rl.Vector3{0, current_pos.y + 0.5 * link_size.y, 0}
        p2 := rl.Vector3{0, current_pos.y - link_distance + 0.5 * link_size.y, 0}
        
        // Create distance constraint between this body and the previous one
        create_distance_constraint(world, body_index, prev_body_index, p1, p2, 
                                  link_distance, compliance, unilateral)
        
        // Update previous body index for next iteration
        prev_body_index = body_index
    }
    
    return body_indices
}

// Create a mobile-like structure with bars and spheres
create_mobile :: proc(world: ^PhysicsWorld, start_pos: rl.Vector3, num_levels: int, 
                     bar_size: rl.Vector3, density: f32 = 1000.0, 
                     compliance: f32 = 0.0, unilateral: bool = true) -> []int {
    if num_levels <= 0 {
        return nil
    }
    
    // Allocate array to store body indices
    body_indices := make([]int, num_levels * 2)  // Each level has a bar and a sphere
    
    // Calculate base radius for the spheres
    base_radius :f32= 0.08
    
    // Calculate the distance between attachment points
    distance :f32= 0.5 * bar_size.x - 0.04  // Half bar length minus thickness
    height :f32= 0.3  // Height between levels
    
    // Start position
    bar_pos := start_pos
    prev_bar_index := -1
    
    // Create the mobile structure
    for i := 0; i < num_levels; i += 1 {
        // Create the bar
        bar_index := create_box(world, bar_pos, bar_size, density)
        body_indices[i*2] = bar_index
        
        // Set damping
        world.bodies[bar_index].damping = 0.5
        
        // If not the first level, connect to the previous bar
        if prev_bar_index >= 0 {
            // Create attachment points
            p0 := rl.Vector3{bar_pos.x, bar_pos.y + 0.5 * bar_size.y, bar_pos.z}
            p1 := rl.Vector3{bar_pos.x, bar_pos.y + height - 0.5 * bar_size.y, bar_pos.z}
            
            // Create distance constraint
            create_distance_constraint(world, bar_index, prev_bar_index, p0, p1, 
                                      height - bar_size.y, compliance, unilateral)
        }
        
        // Calculate sphere radius for this level
        // For simplicity, we'll use a formula that increases the radius for lower levels
        sphere_radius := base_radius * (1.0 + 0.5 * f32(num_levels - i - 1))
        
        // Create the sphere
        sphere_pos := rl.Vector3{bar_pos.x + distance, bar_pos.y - height, bar_pos.z}
        sphere_index := create_sphere(world, sphere_pos, sphere_radius, density)
        body_indices[i*2 + 1] = sphere_index
        
        // Set damping
        world.bodies[sphere_index].damping = 0.5
        
        // Create attachment points for the sphere
        p0 := rl.Vector3{sphere_pos.x, sphere_pos.y + 0.5 * sphere_radius, sphere_pos.z}
        p1 := rl.Vector3{sphere_pos.x, sphere_pos.y + height - 0.5 * bar_size.y, sphere_pos.z}
        
        // Create distance constraint between sphere and bar
        create_distance_constraint(world, sphere_index, bar_index, p0, p1, 
                                  height - bar_size.y, compliance, unilateral)
        
        // For the last level, add a second sphere on the other side for balance
        if i == num_levels - 1 {
            sphere_pos.x -= 2.0 * distance
            second_sphere_index := create_sphere(world, sphere_pos, sphere_radius, density)
            
            // Set damping
            world.bodies[second_sphere_index].damping = 0.5
            
            // Create attachment points
            p0.x -= 2.0 * distance
            p1.x -= 2.0 * distance
            
            // Create distance constraint
            create_distance_constraint(world, second_sphere_index, bar_index, p0, p1, 
                                      height - bar_size.y, compliance, unilateral)
        }
        
        // Update for next level
        prev_bar_index = bar_index
        bar_pos.y -= height
        bar_pos.x -= distance
    }
    
    return body_indices
}