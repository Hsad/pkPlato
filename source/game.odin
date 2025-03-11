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
	nearest_point: rl.Vector3,
}

Game_Memory :: struct {
	player_pos: rl.Vector3,
	player_vel: rl.Vector3,
	player_texture: rl.Texture,
	camera_pos: rl.Vector3,
	player_look_target: rl.Vector3,
	some_number: int,
	run: bool,
	debug: Debug_data,
	ground_model: rl.Model,
	test_ball_pos: rl.Vector3,
	test_ball_vel: rl.Vector3,
	ground_boxes: [dynamic]AABB,
	
	// Physics world
	//physics_world: ^PhysicsWorld,
	//physics_bodies: [dynamic]int,  // Indices into the physics world
	//player_physics_body_index: int, // Index of the player's physics body

	//simple pbd
	pbd_world: ^PBD_World,
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

@(export)
game_init :: proc() {
	g_mem = new(Game_Memory)

	ground_img := rl.GenImagePerlinNoise(256,256,0,0,1)
	defer rl.UnloadImage(ground_img)
	ground_mesh := rl.GenMeshPlane(100, 100, 1, 1)
	ground_model := rl.LoadModelFromMesh(ground_mesh)
	//ground_model.materials = rl.LoadMaterialDefault()
	ground_model.materials[0].maps[0].texture = rl.LoadTextureFromImage(ground_img)


	ground_boxes := [dynamic]AABB{}
	for _ in 0..<10 {
		for _ in 0..<10 {
			center := rl.Vector3{f32(rl.GetRandomValue(-50, 50)), 0, f32(rl.GetRandomValue(-50, 50))}
			append(&ground_boxes, AABB{min = {center.x - 1, 0, center.z - 1}, max = {center.x + 1, 5, center.z + 1}})
		}
	}


	// Initialize physics world
	//physics_world := create_physics_world()
	
	// Configure physics world for better stability
	// Increase iterations for more accurate simulation
	//configure_physics_world(physics_world, 20, {0, -9.81, 0})
	
	//physics_bodies := make([dynamic]int)
	
	// Create player physics body (a sphere)
	//player_radius := f32(1.0)
	//player_mass := f32(1.0)  // Even lighter for more responsive movement
	//player_compliance := f32(0.00001)  // Much stiffer for better control
	//player_physics_index := create_sphere(physics_world, player_start_pos, player_radius, player_mass, player_compliance)
	
	// Set player material properties for better control
	// Higher restitution for better bouncing, lower friction for smoother movement
	//set_material_properties(physics_world, player_physics_index, 0.5, 0.2, 0.1, player_compliance)
	
	// Add player body to the list of physics bodies
	//append(&physics_bodies, player_physics_index)
	
	// Create a ground plane (static body)
	//ground_index := create_box(physics_world, {0, -1, 0}, {100, 2.1, 100}, 0, 0.001)
	//append(&physics_bodies, ground_index)
	
	// Create some dynamic boxes with different compliance values
	//for i in 0..<5 {
	//	pos := rl.Vector3{f32(rl.GetRandomValue(-20, 20)), f32(50 + i * 2), f32(rl.GetRandomValue(-20, 20))}
	//	// Use slightly different compliance values for variety
	//	compliance := 0.01 + f32(i) * 0.001
	//	box_index := create_box(physics_world, pos, {1, 1, 1}, 1, compliance)
	//	//append(&physics_bodies, box_index)
	//}
	
	//// Create some dynamic spheres with different compliance values
	//for i in 0..<20 {
	//	pos := rl.Vector3{f32(rl.GetRandomValue(-20, 20)), f32(15 + i * 2), f32(rl.GetRandomValue(-20, 20))}
	//	// Use slightly different compliance values for variety
	//	compliance := 0.005 + f32(i) * 0.0005
	//	sphere_index := create_sphere(physics_world, pos, 1, 1, compliance)
	//	//append(&physics_bodies, sphere_index)
	//}

	pbd_world := pbd_init()
	player_start_pos := rl.Vector3{0, 5, 0}  // Start a bit higher to see if gravity works
	append(&pbd_world.points, Point{player_start_pos, player_start_pos, {0, 0, 0}, 1.0, 0})
	pbd_create_boxes(pbd_world)

	g_mem^ = Game_Memory {
		run = true,
		some_number = 100,

		// You can put textures, sounds and music in the `assets` folder. Those
		// files will be part any release or web build.
		player_texture = rl.LoadTexture("assets/round_cat.png"),
		player_pos = {player_start_pos.x, player_start_pos.y, player_start_pos.z},  // Initialize player position
		player_look_target = {player_start_pos.x + 10, player_start_pos.y, player_start_pos.z},  // Initialize some distance in front of player
		camera_pos = {player_start_pos.x - 10, player_start_pos.y + 5, player_start_pos.z},  // Initialize behind player
		ground_model = ground_model,
		ground_boxes = ground_boxes,
		//physics_world = physics_world,
		//physics_bodies = physics_bodies,
		//player_physics_body_index = player_physics_index,

		pbd_world = pbd_world,
	}
	//pbd_init()

	game_hot_reloaded(g_mem)
}

update :: proc() {
	// input_intent?
	//fmt.println("controller_input")
	controller_input()
	//fmt.println("pbd_simulate")
	pbd_simulate(rl.GetFrameTime())
	//fmt.println("pbd_simulate done")

	hight_change := g_mem.pbd_world.points[0].position.y - g_mem.player_pos.y
	g_mem.player_pos = g_mem.pbd_world.points[0].position
	g_mem.player_look_target.y += hight_change

	// Update physics
	//update_physics(g_mem.physics_world, rl.GetFrameTime())
	
	// Update player position from physics body
	//physics_pos := get_position(g_mem.physics_world, g_mem.player_physics_body_index)
	//physics_vel := get_velocity(g_mem.physics_world, g_mem.player_physics_body_index)
	
	// Debug print
	//fmt.println("Physics pos:", physics_pos, "Physics vel:", physics_vel)
	
	//g_mem.player_pos = {physics_pos.x, physics_pos.y, physics_pos.z}
	
	// Update camera after player position is updated
	//fmt.println("calc_camera")
	calc_camera()

	//fmt.println("g_mem.pbd_world.points: ", len(g_mem.pbd_world.points))
	//fmt.println("g_mem.pbd_world.springs: ", len(g_mem.pbd_world.springs))
	// Temporarily comment out AABB collision detection to see if it's interfering
	/*
	{
		// Check collisions between player and all ground boxes
		for box, i in g_mem.ground_boxes {
			result := point_in_aabb(g_mem.player_pos, box)
			if result.is_inside {
				// Color will be applied in draw proc
				g_mem.ground_boxes[i].is_colliding = true
				g_mem.debug.nearest_point = result.nearest_surface_point
			} else {
				g_mem.ground_boxes[i].is_colliding = false
			}
		}
	}
	*/


	g_mem.some_number += 1

	//if rl.IsKeyPressed(.ESCAPE) {
	//	g_mem.run = false
	//}
}

draw :: proc() {
	rl.BeginDrawing()
	rl.ClearBackground(rl.BLACK)

	rl.BeginMode3D(game_camera())

	rl.DrawModel(g_mem.ground_model, {0, 0, 0}, 1, rl.WHITE)

	// Draw the player as a small blue sphere
	rl.DrawSphere(g_mem.player_pos, 1.0, rl.BLUE)
	
	// Draw player velocity vector
	//physics_vel := get_velocity(g_mem.physics_world, g_mem.player_physics_body_index)
	//vel_end := rl.Vector3{
	//	g_mem.player_pos.x + physics_vel.x,
	//	g_mem.player_pos.y + physics_vel.y,
	//	g_mem.player_pos.z + physics_vel.z,
	//}
	//rl.DrawLine3D(g_mem.player_pos, vel_end, rl.MAGENTA)
	
	// Draw the ideal camera position as a small green sphere
	rl.DrawSphere(g_mem.debug.ideal_camera_pos, 0.3, rl.GREEN)
	rl.DrawLine3D(g_mem.player_pos, g_mem.debug.ideal_camera_pos, rl.GREEN)
	// Draw the look target as a small yellow sphere
	rl.DrawLine3D(g_mem.player_pos, g_mem.player_look_target, rl.YELLOW)
	rl.DrawSphere(g_mem.player_look_target, 0.3, rl.YELLOW)

	// simple pbd
	pbd_draw_points(g_mem.pbd_world.points)
	pbd_draw_springs(g_mem.pbd_world.springs)

	//debug
	rl.DrawSphere(g_mem.debug.nearest_point, 0.3, rl.RED)
	// Draw the ground boxes
	for box in g_mem.ground_boxes {
		center := (box.min + box.max) / 2
		size := box.max - box.min 
		if box.is_colliding {
			rl.DrawCubeV(center, size, rl.RED)
		} else {
			rl.DrawCubeV(center, size, rl.WHITE)
		}
	}

	//// Draw physics objects
	//for i := 0; i < len(g_mem.physics_world.bodies); i += 1 {
	//	body := g_mem.physics_world.bodies[i]
	//	
	//	// Convert Vector3 to rl.Vector3
	//	pos := rl.Vector3{body.position[0], body.position[1], body.position[2]}
	//	
	//	// Choose color based on body type
	//	color := rl.WHITE
	//	if body.inverse_mass == 0 {
	//		color = rl.GREEN  // Static bodies
	//	} else if body.shape_type == .Sphere {
	//		color = rl.BLUE   // Dynamic spheres
	//	} else {
	//		color = rl.RED    // Dynamic boxes
	//	}
	//	
	//	// Draw with wireframe for better visibility
	//	switch body.shape_type {
	//	case .Sphere:
	//		rl.DrawSphereWires(pos, body.shape_size[0], 8, 8, color)
	//		rl.DrawSphere(pos, body.shape_size[0], rl.ColorAlpha(color, 0.5))
	//	case .Box:
	//		size := rl.Vector3{
	//			body.shape_size.x * 2, // Multiply by 2 since shape_size is half-extents
	//			body.shape_size.y * 2,
	//			body.shape_size.z * 2,
	//		}
	//		rl.DrawCubeWires(pos, size.x, size.y, size.z, color)
	//		rl.DrawCubeV(pos, size, rl.ColorAlpha(color, 0.5))
	//	}
	//	
	//	// Draw velocity vector for debugging
	//	if body.inverse_mass > 0 {
	//		vel_end = rl.Vector3{
	//			pos.x + body.linear_velocity[0],
	//			pos.y + body.linear_velocity[1],
	//			pos.z + body.linear_velocity[2],
	//		}
	//		rl.DrawLine3D(pos, vel_end, rl.YELLOW)
	//	}
	//}
	
	rl.EndMode3D()

	//rl.BeginMode3D(rl.Camera3D{
		//position = {40, 40, 40},
		//target = {0, 0, 0},
		//up = {0, 1, 0},
		//fovy = 45,
		//projection = .PERSPECTIVE,
	//})
	//rl.DrawSphere(g_mem.player_pos, 1.0, rl.RED)
	//rl.DrawPlane({0, -1, 0}, {10, 10}, rl.GREEN) // Center at y=-1, 10x10 size
	//rl.DrawLine3D(g_mem.player_pos, g_mem.player_look_target, rl.RED)
	//rl.EndMode3D()

	rl.BeginMode2D(ui_camera())

	// NOTE: `fmt.ctprintf` uses the temp allocator. The temp allocator is
	// cleared at the end of the frame by the main application, meaning inside
	// `main_hot_reload.odin`, `main_release.odin` or `main_web_entry.odin`.
	//rl.DrawText(fmt.ctprintf("some_number: %v\nplayer_pos: %v", g_mem.some_number, g_mem.player_pos), 5, 5, 8, rl.WHITE)
	// frame time and frame rate
	rl.DrawText(fmt.ctprintf("ft: %v\nfps: %v", rl.GetFrameTime(), rl.GetFPS()), 5, 5, 8, rl.WHITE)
	//rl.DrawText(fmt.ctprintf("player: %v", g_mem.physics_world.bodies[0].position), 5, 30, 8, rl.WHITE)
	//rl.DrawText(fmt.ctprintf("player: %v", g_mem.physics_world.bodies[0].linear_velocity), 5, 45, 8, rl.WHITE)

	rl.EndMode2D()

	rl.EndDrawing()
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
			input += look_dir * -z * 0.5
			
			// Strafe movement perpendicular to look direction on xz plane
			right_dir := rl.Vector3{look_dir.z, 0, -look_dir.x}
			input -= right_dir * x * 0.2
			
			//fmt.println("Input direction:", input)
		} else {
			g_mem.pbd_world.points[0].velocity *= {0.95, 1, 0.95}
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

		// Get current velocity from physics system
		//current_vel := get_velocity(g_mem.physics_world, g_mem.player_physics_body_index)
		//physics_vel := rl.Vector3{current_vel.x, current_vel.y, current_vel.z}
		physics_vel := g_mem.pbd_world.points[0].velocity
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
			//set_velocity(g_mem.physics_world, g_mem.player_physics_body_index, physics_vel, {0, 0, 0})
			g_mem.pbd_world.points[0].velocity = physics_vel
		}
		
		// Jump physics
		if rl.IsGamepadButtonPressed(0, .RIGHT_FACE_DOWN) {
			// Apply upward impulse
			JUMP_FORCE :: f32(15.0)  // Increased jump force
			physics_vel.y = JUMP_FORCE
			//fmt.println("Applying jump force:", JUMP_FORCE, "New velocity:", physics_vel)
			//set_velocity(g_mem.physics_world, g_mem.player_physics_body_index, physics_vel, {0, 0, 0})
			g_mem.pbd_world.points[0].velocity.y = JUMP_FORCE
		}
	}
	
	// Update look target position based on player movement
	// This is done after physics update in the update function
	g_mem.player_look_target += input * rl.GetFrameTime() * 100
}

calc_camera :: proc() {
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
	
	g_mem.debug.ideal_camera_pos = ideal_camera_pos
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
	// Clean up physics resources
	//if g_mem.physics_world != nil {
	//	destroy_physics_world(g_mem.physics_world)
	//}
	//delete(g_mem.physics_bodies)
	
	if g_mem.pbd_world != nil {
		pbd_deinit(g_mem.pbd_world)
	}

	delete(g_mem.ground_boxes)
	
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