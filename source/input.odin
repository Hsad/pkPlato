package game

import rl "vendor:raylib"
import math "core:math"
import linalg "core:math/linalg"
import fmt "core:fmt"
Input_Intent :: struct {
	move_x: f32,
	move_z: f32,
    right_stick: bool,

	look_x: f32,
	look_y: f32,
    left_stick: bool,

    trigger_left: f32,
    trigger_right: f32,

    bumper_left: bool,
    bumper_right: bool,

    button_a: bool,
    button_b: bool,
    button_x: bool,
    button_y: bool,

	bounce: bool,
}

get_input_intent :: proc() {
	g.input_previous = g.input_intent
    if rl.IsGamepadAvailable(0) {
        g.input_intent.move_x = rl.GetGamepadAxisMovement(0, .LEFT_X)
        g.input_intent.move_z = rl.GetGamepadAxisMovement(0, .LEFT_Y)

        g.input_intent.look_x = rl.GetGamepadAxisMovement(0, .RIGHT_X)
        g.input_intent.look_y = rl.GetGamepadAxisMovement(0, .RIGHT_Y)

        g.input_intent.right_stick = rl.IsGamepadButtonDown(0, .RIGHT_THUMB)
        g.input_intent.left_stick = rl.IsGamepadButtonDown(0, .LEFT_THUMB)

        g.input_intent.trigger_left = rl.GetGamepadAxisMovement(0, .LEFT_TRIGGER)
        g.input_intent.trigger_right = rl.GetGamepadAxisMovement(0, .RIGHT_TRIGGER)

        g.input_intent.bumper_left = rl.IsGamepadButtonDown(0, .LEFT_TRIGGER_1)
        g.input_intent.bumper_right = rl.IsGamepadButtonDown(0, .RIGHT_TRIGGER_1)

        g.input_intent.button_a = rl.IsGamepadButtonDown(0, .RIGHT_FACE_DOWN)
        g.input_intent.button_b = rl.IsGamepadButtonDown(0, .RIGHT_FACE_RIGHT)
        g.input_intent.button_x = rl.IsGamepadButtonDown(0, .RIGHT_FACE_LEFT)
        g.input_intent.button_y = rl.IsGamepadButtonDown(0, .RIGHT_FACE_UP)

		g.input_intent.bounce = false
    }

    if rl.IsKeyPressed(.SPACE) {
        g.camera.using_first_person = !g.camera.using_first_person
    }
}


controller_input :: proc() {
	input: rl.Vector3

	// Gamepad input (using first connected gamepad)
	if rl.IsGamepadAvailable(0) {

		// Right analog stick for looking
		look_x := rl.GetGamepadAxisMovement(0, .RIGHT_X)
		look_y := rl.GetGamepadAxisMovement(0, .RIGHT_Y)

		// Apply deadzone and sensitivity
		LOOK_DEADZONE :: 0.3
		LOOK_SENSITIVITY :: 2.0
		LOOK_DISTANCE :: 10.0  // Distance to keep the look target from player

		is_looking := abs(look_x) > LOOK_DEADZONE || abs(look_y) > LOOK_DEADZONE
		if is_looking {
			
			// Rotate the look target around the player
			current_target_offset := g.player.look_target - g.player.pos
			yaw := look_x * LOOK_SENSITIVITY * rl.GetFrameTime()
			
			// Apply horizontal rotation (yaw)
			cos_yaw := math.cos(yaw)
			sin_yaw := math.sin(yaw)
			new_offset := rl.Vector3{
				current_target_offset.x * cos_yaw - current_target_offset.z * sin_yaw,
				current_target_offset.y,
				current_target_offset.x * sin_yaw + current_target_offset.z * cos_yaw,
			}
			
			// Apply vertical look (pitch)
			VERTICAL_SENSITIVITY :: 1.0
			MAX_VERTICAL_ANGLE :: math.PI * 0.4 // Limit vertical look to about 72 degrees up/down
			
			// Calculate current vertical angle
			horizontal_dist := math.sqrt(new_offset.x * new_offset.x + new_offset.z * new_offset.z)
			current_angle := math.atan2(new_offset.y, horizontal_dist)
			
			// If actively looking up/down with stick, allow vertical movement
			if abs(look_y) > LOOK_DEADZONE {
				new_angle := current_angle + look_y * VERTICAL_SENSITIVITY * rl.GetFrameTime()
				current_angle = clamp(new_angle, -MAX_VERTICAL_ANGLE, MAX_VERTICAL_ANGLE)
			} 
			
			// Calculate new height using angle
			new_offset.y = horizontal_dist * math.tan(current_angle)
			
			// Maintain consistent distance from player
			g.player.look_target = g.player.pos + linalg.normalize(new_offset) * LOOK_DISTANCE
		}

		// Left analog stick movement (existing code)
		x := rl.GetGamepadAxisMovement(0, .LEFT_X)
		z := rl.GetGamepadAxisMovement(0, .LEFT_Y)

		fmt.println("x", x, "z", z)

		// Apply deadzone of 0.3 to handle controller drift
		is_moving := abs(x) > 0.3 || abs(z) > 0.3
		if is_moving {
			// Calculate direction vector from player to look target, projected onto xz plane
			look_dir := g.player.look_target - g.player.pos
			look_dir.y = 0 // Project onto xz plane
			look_dir = linalg.normalize(look_dir)
			
			// Forward/backward movement along look direction on xz plane
			input += look_dir * -z * 0.5
			
			// Strafe movement perpendicular to look direction on xz plane
			right_dir := rl.Vector3{look_dir.z, 0, -look_dir.x}
			input -= right_dir * x * 0.2
			
			//fmt.println("Input direction:", input)
		} else {
			g.pbd_world.points[0].velocity *= {0.95, 1, 0.95}
		}
		
		
		// adjust vertical look back to center while moving
		if is_moving && !is_looking {
			RETURN_TO_LEVEL_SPEED :: 2.0 // Speed at which view returns to level
			
			// Get current offset and recalculate angle
			current_offset := g.player.look_target - g.player.pos
			horizontal_dist := math.sqrt(current_offset.x * current_offset.x + current_offset.z * current_offset.z)
			current_angle := math.atan2(current_offset.y, horizontal_dist)
			
			// Lerp the angle back to 0
			new_angle := linalg.lerp(
				current_angle,
				0.0,
				min(1.0, rl.GetFrameTime() * RETURN_TO_LEVEL_SPEED),
			)
			
			// Construct new offset with updated height
			new_offset := rl.Vector3{
				current_offset.x,
				horizontal_dist * math.tan(new_angle),
				current_offset.z,
			}
			
			// Maintain consistent distance from player
			g.player.look_target = g.player.pos + linalg.normalize(new_offset) * LOOK_DISTANCE
		}

		// Get current velocity from physics system
		//current_vel := get_velocity(g.physics_world, g.player_physics_body_index)
		//physics_vel := rl.Vector3{current_vel.x, current_vel.y, current_vel.z}
		physics_vel := g.pbd_world.points[0].velocity
		//fmt.println("Current physics velocity before:", physics_vel)
		
		// Apply movement forces to physics body
		if linalg.length(input) > 0 {
			// Convert input to physics Vector3
			move_force := rl.Vector3{input.x, input.y, input.z}
			
			// Scale force for better control
			MOVE_FORCE_SCALE :: f32(100.0)  // Increased for more responsive movement
			move_force *= MOVE_FORCE_SCALE
			
			// Apply stronger horizontal damping to prevent excessive speed
			HORIZONTAL_DAMPING :: f32(0.8)
			physics_vel.x *= HORIZONTAL_DAMPING
			physics_vel.z *= HORIZONTAL_DAMPING
			
			// Add movement force to current velocity
			physics_vel.x += move_force.x
			physics_vel.z += move_force.z
			
			// Cap maximum horizontal speed
			MAX_HORIZONTAL_SPEED :: f32(50.0)
			horizontal_speed := math.sqrt(physics_vel.x * physics_vel.x + physics_vel.z * physics_vel.z)
			if horizontal_speed > MAX_HORIZONTAL_SPEED {
				scale_factor := MAX_HORIZONTAL_SPEED / horizontal_speed
				physics_vel.x *= scale_factor
				physics_vel.z *= scale_factor
			}
			
			//fmt.println("Applied force:", move_force, "New velocity:", physics_vel)
			
			// Apply velocity to physics body
			//set_velocity(g.physics_world, g.player_physics_body_index, physics_vel, {0, 0, 0})
			g.pbd_world.points[0].velocity = physics_vel
		}
		
		// Jump physics
		if rl.IsGamepadButtonPressed(0, .RIGHT_FACE_DOWN) {
			// Apply upward impulse
			JUMP_FORCE :: f32(15.0)  // Increased jump force
			physics_vel.y = JUMP_FORCE
			//fmt.println("Applying jump force:", JUMP_FORCE, "New velocity:", physics_vel)
			//set_velocity(g.physics_world, g.player_physics_body_index, physics_vel, {0, 0, 0})
			g.pbd_world.points[0].velocity.y = JUMP_FORCE
		}
	}
	
	// Update look target position based on player movement
	// This is done after physics update in the update function
	g.player.look_target += input * rl.GetFrameTime() * 100
}
