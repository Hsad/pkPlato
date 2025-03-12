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
import rl "vendor:raylib"

PIXEL_WINDOW_HEIGHT :: 180

Debug_data :: struct {
	ideal_camera_pos: rl.Vector3,
	nearest_point: rl.Vector3,
}

Game_Memory :: struct {
	run: bool,

	debug: Debug_data,

	input_intent: Input_Intent,
	player: Player,
	camera: Camera,

	ground_model: rl.Model,
	ground_grid: Ground_Grid,

	//simple pbd
	pbd_world: ^PBD_World,
}

g_mem: ^Game_Memory

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


	pbd_world := pbd_init()
	player_start_pos := rl.Vector3{0, 5, 0}  // Start a bit higher to see if gravity works
	append(&pbd_world.points, Point{player_start_pos, player_start_pos, {0, 0, 0}, 1.0, 0})
	pbd_create_boxes(pbd_world)

	ground_grid := init_ground_grid()

	camera := Camera{
		using_first_person = true,
		frst_pos = {player_start_pos.x, player_start_pos.y, player_start_pos.z},
		thrd_pos = {player_start_pos.x - 10, player_start_pos.y + 5, player_start_pos.z},
	}

	player := Player{
		pos = player_start_pos,
		look_target = player_start_pos + rl.Vector3{10, 0, 0},
	}

	g_mem^ = Game_Memory {
		run = true,

		// You can put textures, sounds and music in the `assets` folder. Those
		// files will be part any release or web build.
		player = player,

		ground_model = ground_model,

		ground_grid = ground_grid,

		camera = camera,

		pbd_world = pbd_world,
	}

	game_hot_reloaded(g_mem)
}

update :: proc() {

	//controller_input() // build input state
	get_input_intent()

	player_controller() // update player position
	g_mem.pbd_world.points[0].velocity = g_mem.player.vel

	pbd_simulate(rl.GetFrameTime()) // simulate physics

	// control look target
	hight_change := g_mem.pbd_world.points[0].position.y - g_mem.player.pos.y
	g_mem.player.look_target.y += hight_change
	// update player position
	g_mem.player.pos = g_mem.pbd_world.points[0].position
	g_mem.player.vel = g_mem.pbd_world.points[0].velocity

	calc_camera_3rd_person()
	calc_camera_1st_person()

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
	rl.DrawSphere(g_mem.player.pos, 1.0, rl.BLUE)
	
	// Draw the ideal camera position as a small green sphere
	rl.DrawSphere(g_mem.debug.ideal_camera_pos, 0.3, rl.GREEN)
	rl.DrawLine3D(g_mem.player.pos, g_mem.debug.ideal_camera_pos, rl.GREEN)
	// Draw the look target as a small yellow sphere
	rl.DrawLine3D(g_mem.player.pos, g_mem.player.look_target, rl.YELLOW)
	rl.DrawSphere(g_mem.player.look_target, 0.3, rl.YELLOW)

	// simple pbd
	pbd_draw_points(g_mem.pbd_world.points)
	pbd_draw_springs(g_mem.pbd_world.springs)

	//debug
	rl.DrawSphere(g_mem.debug.nearest_point, 0.3, rl.RED)

	draw_ground_grid()

	rl.EndMode3D()


	rl.BeginMode2D(ui_camera())

	// NOTE: `fmt.ctprintf` uses the temp allocator. The temp allocator is
	// cleared at the end of the frame by the main application, meaning inside
	// `main_hot_reload.odin`, `main_release.odin` or `main_web_entry.odin`.
	//rl.DrawText(fmt.ctprintf("some_number: %v\nplayer_pos: %v", g_mem.some_number, g_mem.player_pos), 5, 5, 8, rl.WHITE)
	// frame time and frame rate
	rl.DrawText(fmt.ctprintf("ft: %v\nfps: %v", rl.GetFrameTime(), rl.GetFPS()), 5, 5, 8, rl.WHITE)

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