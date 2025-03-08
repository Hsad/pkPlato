/*
This file is the starting point of your game.

Some important procedures are:
- game_init_window: Opens the window
- game_init: Sets up the game state
- game_update: Run once per frame
- game_should_close: For stopping your game when close button is pressed
- game_shutdown: Shuts down game and frees memory
- game_shutdown_window: Closes window

The procs above are used regardless if you compile using the `build_release`
script or the `build_hot_reload` script. However, in the hot reload case, the
contents of this file is compiled as part of `build/hot_reload/game.dll` (or
.dylib/.so on mac/linux). In the hot reload cases some other procedures are
also used in order to facilitate the hot reload functionality:

- game_memory: Run just before a hot reload. That way game_hot_reload.exe has a
      pointer to the game's memory that it can hand to the new game DLL.
- game_hot_reloaded: Run after a hot reload so that the `g_mem` global
      variable can be set to whatever pointer it was in the old DLL.

NOTE: When compiled as part of `build_release`, `build_debug` or `build_web`
then this whole package is just treated as a normal Odin package. No DLL is
created.
*/

package game

import "core:fmt"
import "core:math/linalg"
import "core:math"
import rl "vendor:raylib"

PIXEL_WINDOW_HEIGHT :: 180

Debug_data :: struct {
	ideal_camera_pos: rl.Vector3,
}

Game_Memory :: struct {
	player_pos: rl.Vector3,
	player_texture: rl.Texture,
	camera_pos: rl.Vector3,
	player_look_target: rl.Vector3,
	some_number: int,
	run: bool,
	debug: Debug_data,
}

g_mem: ^Game_Memory

game_camera :: proc() -> rl.Camera3D {

	return {
		position = g_mem.camera_pos,
		target = g_mem.player_pos,
		up = {0, 1, 0},
		fovy = 45,
		projection = .PERSPECTIVE,
	}
}

ui_camera :: proc() -> rl.Camera2D {
	return {
		zoom = f32(rl.GetScreenHeight())/PIXEL_WINDOW_HEIGHT,
	}
}

update :: proc() {
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
			current_target_offset := g_mem.player_look_target - g_mem.player_pos
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
			g_mem.player_look_target = g_mem.player_pos + linalg.normalize(new_offset) * LOOK_DISTANCE
		}

		// Left analog stick movement (existing code)
		x := rl.GetGamepadAxisMovement(0, .LEFT_X)
		z := rl.GetGamepadAxisMovement(0, .LEFT_Y)

		// Apply deadzone of 0.3 to handle controller drift
		is_moving := abs(x) > 0.3 || abs(z) > 0.3
		if is_moving {
			// Calculate direction vector from player to look target, projected onto xz plane
			look_dir := g_mem.player_look_target - g_mem.player_pos
			look_dir.y = 0 // Project onto xz plane
			look_dir = linalg.normalize(look_dir)
			
			// Forward/backward movement along look direction on xz plane
			input += look_dir * -z
			
			// Strafe movement perpendicular to look direction on xz plane
			right_dir := rl.Vector3{look_dir.z, 0, -look_dir.x}
			input -= right_dir * x * 0.2
		}
		// adjust vertical look back to center while moving
		if is_moving && !is_looking {
			RETURN_TO_LEVEL_SPEED :: 2.0 // Speed at which view returns to level
			
			// Get current offset and recalculate angle
			current_offset := g_mem.player_look_target - g_mem.player_pos
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
			g_mem.player_look_target = g_mem.player_pos + linalg.normalize(new_offset) * LOOK_DISTANCE
		}

	}
	if rl.IsKeyDown(.UP) || rl.IsKeyDown(.W) {
		input += g_mem.player_look_target
	}
	if rl.IsKeyDown(.DOWN) || rl.IsKeyDown(.S) {
		input -= g_mem.player_look_target
	}
	if rl.IsKeyDown(.LEFT) || rl.IsKeyDown(.A) {
		input += {g_mem.player_look_target.z, 0, -g_mem.player_look_target.x}
	}
	if rl.IsKeyDown(.RIGHT) || rl.IsKeyDown(.D) {
		input -= {g_mem.player_look_target.z, 0, -g_mem.player_look_target.x}
	}
	

	
	g_mem.player_pos += input * rl.GetFrameTime() * 100
	g_mem.player_look_target += input * rl.GetFrameTime() * 100
	
	// Camera constants
	CAMERA_DISTANCE :f32 = 20.0
	CAMERA_HEIGHT :f32 = 10.0
	HORIZONTAL_SMOOTHING :f32: 5.0 // Faster horizontal movement
	VERTICAL_SMOOTHING :f32: 5.0  // Slower vertical movement
	
	// Get direction from player to look target
	look_dir := linalg.normalize(g_mem.player_look_target - g_mem.player_pos)
	
	// Calculate ideal camera position on opposite side of player from look target
	base_camera_pos := g_mem.player_pos - look_dir * CAMERA_DISTANCE
	
	// Add height offset inversely proportional to look target height
	vertical_offset := CAMERA_HEIGHT * (1.0 - look_dir.y)
	ideal_camera_pos := base_camera_pos + rl.Vector3{0, vertical_offset, 0}
	
	// Split current and target positions into horizontal and vertical components
	current_horizontal := rl.Vector3{g_mem.camera_pos.x, 0, g_mem.camera_pos.z}
	current_vertical := rl.Vector3{0, g_mem.camera_pos.y, 0}
	
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
	g_mem.camera_pos = new_horizontal + new_vertical
	
	g_mem.some_number += 1
	
	g_mem.debug.ideal_camera_pos = ideal_camera_pos

	//if rl.IsKeyPressed(.ESCAPE) {
	//	g_mem.run = false
	//}
}

draw :: proc() {
	rl.BeginDrawing()
	rl.ClearBackground(rl.BLACK)

	rl.BeginMode3D(game_camera())

	rl.DrawSphere(g_mem.player_pos, 1.0, rl.BLUE)
	rl.DrawPlane({0, -1, 0}, {100, 100}, rl.GRAY) // Center at y=-1, 10x10 size
	rl.DrawLine3D(g_mem.player_pos, g_mem.player_look_target, rl.YELLOW)
	rl.DrawSphere(g_mem.debug.ideal_camera_pos, 0.3, rl.GREEN)
	rl.DrawLine3D(g_mem.player_pos, g_mem.debug.ideal_camera_pos, rl.GREEN)
	
	// Draw the look target as a small yellow sphere
	rl.DrawSphere(g_mem.player_look_target, 0.3, rl.YELLOW)
	
	rl.EndMode3D()

	rl.BeginMode3D(rl.Camera3D{
		position = {40, 40, 40},
		target = {0, 0, 0},
		up = {0, 1, 0},
		fovy = 45,
		projection = .PERSPECTIVE,
	})
	rl.DrawSphere(g_mem.player_pos, 1.0, rl.RED)
	rl.DrawPlane({0, -1, 0}, {10, 10}, rl.GREEN) // Center at y=-1, 10x10 size
	rl.DrawLine3D(g_mem.player_pos, g_mem.player_look_target, rl.RED)
	rl.EndMode3D()

	rl.BeginMode2D(ui_camera())

	// NOTE: `fmt.ctprintf` uses the temp allocator. The temp allocator is
	// cleared at the end of the frame by the main application, meaning inside
	// `main_hot_reload.odin`, `main_release.odin` or `main_web_entry.odin`.
	rl.DrawText(fmt.ctprintf("some_number: %v\nplayer_pos: %v", g_mem.some_number, g_mem.player_pos), 5, 5, 8, rl.WHITE)

	rl.EndMode2D()

	rl.EndDrawing()
}

@(export)
game_update :: proc() {
	update()
	draw()
}

@(export)
game_init_window :: proc() {
	rl.SetConfigFlags({.WINDOW_RESIZABLE, .VSYNC_HINT})
	rl.InitWindow(1280, 720, "Odin + Raylib + Hot Reload template!")
	rl.SetWindowPosition(1200, 1200)
	rl.SetTargetFPS(500)
	rl.SetExitKey(nil)
}

@(export)
game_init :: proc() {
	g_mem = new(Game_Memory)

	g_mem^ = Game_Memory {
		run = true,
		some_number = 100,

		// You can put textures, sounds and music in the `assets` folder. Those
		// files will be part any release or web build.
		player_texture = rl.LoadTexture("assets/round_cat.png"),
		player_look_target = {10, 0, 0},  // Initialize some distance in front of default spawn
	}

	game_hot_reloaded(g_mem)
}

@(export)
game_should_run :: proc() -> bool {
	when ODIN_OS != .JS {
		// Never run this proc in browser. It contains a 16 ms sleep on web!
		if rl.WindowShouldClose() {
			return false
		}
	}

	return g_mem.run
}

@(export)
game_shutdown :: proc() {
	free(g_mem)
}

@(export)
game_shutdown_window :: proc() {
	rl.CloseWindow()
}

@(export)
game_memory :: proc() -> rawptr {
	return g_mem
}

@(export)
game_memory_size :: proc() -> int {
	return size_of(Game_Memory)
}

@(export)
game_hot_reloaded :: proc(mem: rawptr) {
	g_mem = (^Game_Memory)(mem)

	// Here you can also set your own global variables. A good idea is to make
	// your global variables into pointers that point to something inside
	// `g_mem`.
}

@(export)
game_force_reload :: proc() -> bool {
	return rl.IsKeyPressed(.F5)
}

@(export)
game_force_restart :: proc() -> bool {
	return rl.IsKeyPressed(.F6)
}

// In a web build, this is called when browser changes size. Remove the
// `rl.SetWindowSize` call if you don't want a resizable game.
game_parent_window_size_changed :: proc(w, h: int) {
	rl.SetWindowSize(i32(w), i32(h))
}
