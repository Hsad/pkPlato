package game

import rl "vendor:raylib"
import linalg "core:math/linalg"


Camera :: struct {
	using_first_person: bool,
	frst_pos: rl.Vector3,
	thrd_pos: rl.Vector3,
	look_target: rl.Vector3,
}

game_camera :: proc() -> rl.Camera3D {
	if g.camera.using_first_person {
		look_dir := linalg.normalize(g.player.look_target - g.camera.frst_pos)
		return {
			position = g.camera.frst_pos - look_dir * 1,
			target = g.player.look_target,
			up = {0, 1, 0},
			fovy = 75,
			projection = .PERSPECTIVE,
		}
	} else {
		return {
			position = g.camera.thrd_pos,
			target = g.player.look_target,
			up = {0, 1, 0},
			fovy = 45,
			projection = .PERSPECTIVE,
		}
	}
}


calc_camera_1st_person :: proc() {
	// Set camera position to player position with a slight height offset for eye level
	g.camera.frst_pos = g.player.pos + rl.Vector3{0, 1.7, 0}  // Typical eye height
	
	// Convert player rotation to a point on unit sphere
	// Assuming g.player.rotation contains pitch (x-axis) and yaw (y-axis) in radians
	pitch := g.player.look_rot.x
	yaw := g.player.look_rot.y
	
	// Calculate direction vector on unit sphere
	x := linalg.sin(yaw) * linalg.cos(pitch) * 1
	y := linalg.sin(pitch)
	z := linalg.cos(yaw) * linalg.cos(pitch) * 1
	direction := rl.Vector3{x, y, z}
	
	// Set look target as position + direction
	g.camera.look_target = g.camera.frst_pos + direction
	g.player.look_target = g.camera.look_target
}

calc_camera_3rd_person :: proc() {
	// Camera constants
	CAMERA_DISTANCE :f32 = 20.0
	CAMERA_HEIGHT :f32 = 10.0
	HORIZONTAL_SMOOTHING :f32: 5.0 // Faster horizontal movement
	VERTICAL_SMOOTHING :f32: 5.0  // Slower vertical movement
	
	// Get direction from player to look target
	look_dir := linalg.normalize(g.player.look_target - g.player.pos)
	
	// Calculate ideal camera position on opposite side of player from look target
	base_camera_pos := g.player.pos - look_dir * CAMERA_DISTANCE
	
	// Add height offset inversely proportional to look target height
	vertical_offset := CAMERA_HEIGHT * (1.0 - look_dir.y)
	ideal_camera_pos := base_camera_pos + rl.Vector3{0, vertical_offset, 0}
	
	// Split current and target positions into horizontal and vertical components
	current_horizontal := rl.Vector3{g.camera.thrd_pos.x, 0, g.camera.thrd_pos.z}
	current_vertical := rl.Vector3{0, g.camera.thrd_pos.y, 0}
	
	target_horizontal := rl.Vector3{ideal_camera_pos.x, 0, ideal_camera_pos.z}
	target_vertical := rl.Vector3{0, ideal_camera_pos.y, 0}
	
	// Lerp horizontal and vertical components separately with different smoothing
	new_horizontal := linalg.lerp(
		current_horizontal,
		target_horizontal,
		min(1.0, rl.GetFrameTime() * HORIZONTAL_SMOOTHING),
	)
	
	new_vertical := linalg.lerp(
		current_vertical,
		target_vertical,
		min(1.0, rl.GetFrameTime() * VERTICAL_SMOOTHING),
	)
	
	// Combine components for final camera position
	g.camera.thrd_pos = new_horizontal + new_vertical
	
	g.debug.ideal_camera_pos = ideal_camera_pos
}