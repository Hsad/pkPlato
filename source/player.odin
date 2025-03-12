package game

import rl "vendor:raylib"

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

    g_mem.input_intent = input
}