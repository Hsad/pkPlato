package game

import rl "vendor:raylib"
import math "core:math"

Player :: struct {
	pos: rl.Vector3,
	vel: rl.Vector3,
	look_rot: rl.Vector3,
    look_target: rl.Vector3,

	foot_pos: rl.Vector3,
	foot_vel: rl.Vector3,
	foot_dist: f32,

	arm_pos: rl.Vector3,
	arm_vel: rl.Vector3,
	arm_dist: f32,
}


player_controller :: proc() {
    input := g_mem.input_intent

    // Update rotation based on look input
    look_sensitivity :: 2.0
	look_x := input.look_x
	look_y := input.look_y
	if math.abs(look_x) < 0.3 {look_x = 0}
	if math.abs(look_y) < 0.3 {look_y = 0}
    g_mem.player.look_rot.y += look_x * -look_sensitivity * rl.GetFrameTime() // Horizontal rotation
    g_mem.player.look_rot.x += look_y * look_sensitivity * rl.GetFrameTime() // Vertical rotation

    // Clamp vertical rotation to avoid over-rotation
    max_vertical_angle :: math.PI * 0.4 // About 72 degrees up/down
    g_mem.player.look_rot.x = clamp(g_mem.player.look_rot.x, -max_vertical_angle, max_vertical_angle)

    // Calculate look target position based on rotations
    look_distance :: 10.0
    
    // Calculate target offset using spherical coordinates
    horizontal := look_distance * math.cos(g_mem.player.look_rot.x)
    target_offset := rl.Vector3{
        horizontal * math.sin(g_mem.player.look_rot.y), // x component
        look_distance * math.sin(g_mem.player.look_rot.x), // y component
        horizontal * math.cos(g_mem.player.look_rot.y), // z component
    }
    
    g_mem.player.look_target = g_mem.player.pos + target_offset


	/* Movement */

	move_speed :: 1

	move_x := input.move_x
	move_z := input.move_z
	if math.abs(move_x) < 0.2 {move_x = 0}
	if math.abs(move_z) < 0.2 {move_z = 0}

	// Get forward vector from look rotation (ignoring vertical component)
	forward := rl.Vector3{
		math.sin(g_mem.player.look_rot.y),
		0,
		math.cos(g_mem.player.look_rot.y),
	}
	// Get right vector by rotating forward 90 degrees
	right := rl.Vector3{
		math.sin(g_mem.player.look_rot.y + math.PI/2),
		0,
		math.cos(g_mem.player.look_rot.y + math.PI/2),
	}

	// Combine forward/back and right/left movement
	move_direction := forward * -input.move_z + right * -input.move_x
	move_velocity := move_direction * move_speed
	g_mem.player.vel += move_velocity


    if input.button_a {
        g_mem.player.vel.y += 1
    }
}