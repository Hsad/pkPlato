package game

import rl "vendor:raylib"
import math "core:math"
import "core:fmt"
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

	on_ground: bool,
	contact_point: rl.Vector3,
	contact_normal: rl.Vector3,

	upright: rl.Vector3,
	wall_run: rl.Vector3,
	tac_left: rl.Vector3,
	tac_right: rl.Vector3,
	climb_up: rl.Vector3,

	state: State,
	pbd: Player_PBD,
}

Player_PBD :: struct {
	Core: Point,
	Left_Leg: Point,
	Right_Leg: Point,
	Left_Arm: Point,
	Right_Arm: Point,

	Left_Leg_Spring: Spring,
	Right_Leg_Spring: Spring,
	Left_Arm_Spring: Spring,
	Right_Arm_Spring: Spring,
}

State :: struct {
	grounded: bool,
	hanging: bool, 
	//climbing: bool,
	rolling: bool,

	upright_contact: rl.Vector3,
	wall_run_contact: rl.Vector3,
	left_wall_contact: rl.Vector3,
	right_wall_contact: rl.Vector3,
	climb_up_contact: rl.Vector3,

	upright_normal: rl.Vector3,
	wall_run_normal: rl.Vector3,
	left_wall_normal: rl.Vector3,
	right_wall_normal: rl.Vector3,
	climb_up_normal: rl.Vector3,

	upright_hit: bool,
	wall_run_hit: bool,
	left_wall_hit: bool,
	right_wall_hit: bool,
	climb_up_hit: bool,
}


player_controller_presim :: proc() {
    input := g.input_intent

    // Update rotation based on look input
    look_sensitivity :: 2.0
	look_x := input.look_x
	look_y := input.look_y
	if math.abs(look_x) < 0.3 {look_x = 0}
	if math.abs(look_y) < 0.3 {look_y = 0}
    g.player.look_rot.y += look_x * -look_sensitivity * rl.GetFrameTime() // Horizontal rotation
    g.player.look_rot.x += look_y * look_sensitivity * rl.GetFrameTime() // Vertical rotation

    // Clamp vertical rotation to avoid over-rotation
    max_vertical_angle :: math.PI * 0.4 // About 72 degrees up/down
    g.player.look_rot.x = clamp(g.player.look_rot.x, -max_vertical_angle, max_vertical_angle)

	/* Movement */
	move_speed :: 1
	move_x := input.move_x
	move_z := input.move_z
	if math.abs(move_x) < 0.3 {move_x = 0}
	if math.abs(move_z) < 0.3 {move_z = 0}

	// Get forward vector from look rotation (ignoring vertical component)
	forward := rl.Vector3{
		math.sin(g.player.look_rot.y),
		0,
		math.cos(g.player.look_rot.y),
	}
	// Get right vector by rotating forward 90 degrees
	right := rl.Vector3{
		math.sin(g.player.look_rot.y + math.PI/2),
		0,
		math.cos(g.player.look_rot.y + math.PI/2),
	}

	// Combine forward/back and right/left movement
	move_direction := forward * -move_z + right * -move_x
	move_velocity := move_direction * move_speed
	g.player.vel += move_velocity

}

player_controller_postsim :: proc() {

    // Calculate look target position based on rotations
    look_distance :: 10.0
    
    // Calculate target offset using spherical coordinates
    //horizontal := look_distance * math.cos(g.player.look_rot.x)
    horizontal :f32= look_distance
    target_offset := rl.Vector3{
        horizontal * math.sin(g.player.look_rot.y), // x component
        look_distance * math.sin(g.player.look_rot.x), // y component
        horizontal * math.cos(g.player.look_rot.y), // z component
    }
    
    g.player.look_target = g.player.pos + target_offset


	// are the contact points hitting anything?
	// calculate position of points based on rotation and position
	wall_run_pos := vect_to_playerspace(g.player.wall_run)
	tac_left_pos := vect_to_playerspace(g.player.tac_left)
	tac_right_pos := vect_to_playerspace(g.player.tac_right)
	climb_up_pos := vect_to_playerspace(g.player.climb_up)
	upright_pos := vect_to_playerspace(g.player.upright)

	hit, vec, normal, height := check_grid_collision(wall_run_pos)
	g.player.state.wall_run_hit = hit
	g.player.state.wall_run_contact = vec
	g.player.state.wall_run_normal = normal
	
	hit, vec, normal, height = check_grid_collision(tac_left_pos)
	g.player.state.left_wall_hit = hit
	g.player.state.left_wall_contact = vec
	g.player.state.left_wall_normal = normal

	hit, vec, normal, height = check_grid_collision(tac_right_pos)
	g.player.state.right_wall_hit = hit
	g.player.state.right_wall_contact = vec
	g.player.state.right_wall_normal = normal

	hit, vec, normal, height = check_grid_collision(climb_up_pos)
	g.player.state.climb_up_hit = hit
	g.player.state.climb_up_contact = vec
	g.player.state.climb_up_normal = normal

	hit, vec, normal, height = check_grid_collision(upright_pos)
	g.player.state.upright_hit = hit
	g.player.state.upright_contact = vec
	g.player.state.upright_normal = normal

	//legs := g.player.upright.y + g.input_intent.trigger_left

	if g.player.state.upright_hit {
		g.player.state.grounded = true
		g.player.pos.y = height + g.player.upright.y
		g.player.vel.y = 0
		g.player.vel.x = g.player.vel.x * 0.9
		g.player.vel.z = g.player.vel.z * 0.9
	} else {
		g.player.state.grounded = false
	}

	if g.player.state.climb_up_hit && g.input_intent.button_y {
		g.player.state.hanging = true
	} else {
		g.player.state.hanging = false
	}

	if !g.player.state.grounded && !g.player.state.hanging {
		g.player.vel.y -= 9.8 * rl.GetFrameTime()
	}

	// Jump
	if g.player.state.grounded && g.input_intent.button_y && !g.input_previous.button_y && g.player.vel.y < 0.1 && g.player.vel.y > -0.1{
		g.player.vel.y += 15
	}
    if g.input_intent.button_b && !g.input_previous.button_b{
		toggle := check_map_toggle(0)
		if toggle {
			g.ground_grid.current_color = 0
		}
    }
    if g.input_intent.button_a && !g.input_previous.button_a{
		toggle := check_map_toggle(1)
		if toggle {
			g.ground_grid.current_color = 1
		}
    }
    if g.input_intent.button_x && !g.input_previous.button_x{
		toggle := check_map_toggle(2)
		if toggle {
			g.ground_grid.current_color = 2
		}
    }

	//g.player.pos += vect_to_playerspace(g.player.vel) * rl.GetFrameTime()
	g.player.pos += g.player.vel * rl.GetFrameTime()

	g.player.pos = g.player.pbd.Core.position
	g.player.foot_pos = g.player.pbd.Left_Leg.position
}


vect_to_playerspace :: proc(vect: rl.Vector3) -> rl.Vector3 {
    // Get rotation angles
    yaw := g.player.look_rot.y
    //pitch := g.player.look_rot.x

    // Rotate around Y axis first (yaw)
    cos_yaw := math.cos(yaw)
    sin_yaw := math.sin(yaw)
    rotated := rl.Vector3{
        vect.x * sin_yaw + vect.z * cos_yaw,
        vect.y,
        vect.x * cos_yaw - vect.z * sin_yaw,
    }

    // Then rotate around X axis (pitch)
    //cos_pitch := math.cos(pitch)
    //sin_pitch := math.sin(pitch)
    //final := rl.Vector3{
    //    rotated.x,
    //    rotated.y * cos_pitch + rotated.z * sin_pitch,
    //    rotated.y * sin_pitch - rotated.z * cos_pitch,
    //}

    return g.player.pos + rotated
}


draw_player :: proc() {
	rl.DrawSphere(g.player.pos, 0.5, rl.RED)
	if g.player.state.wall_run_hit {
		rl.DrawSphere(vect_to_playerspace(g.player.wall_run), 0.5, rl.GREEN)
	}
	if g.player.state.left_wall_hit {
		rl.DrawSphere(vect_to_playerspace(g.player.tac_left), 0.5, rl.BLUE)
	}
	if g.player.state.right_wall_hit {
		rl.DrawSphere(vect_to_playerspace(g.player.tac_right), 0.5, rl.YELLOW)
	}
	if g.player.state.climb_up_hit {
		rl.DrawSphere(vect_to_playerspace(g.player.climb_up), 0.5, rl.MAGENTA)
	}
	if g.player.state.upright_hit {
		rl.DrawSphere(vect_to_playerspace(g.player.upright), 0.5, rl.WHITE)
	}

	rl.DrawSphere(vect_to_playerspace(g.player.wall_run), 0.5, rl.GREEN)
	rl.DrawSphere(vect_to_playerspace(g.player.tac_left), 0.5, rl.BLUE)
	rl.DrawSphere(vect_to_playerspace(g.player.tac_right), 0.5, rl.YELLOW)
	rl.DrawSphere(vect_to_playerspace(g.player.climb_up), 0.5, rl.MAGENTA)
	rl.DrawSphere(vect_to_playerspace(g.player.upright), 0.5, rl.WHITE)

	rl.DrawLine3D(g.player.state.wall_run_contact, g.player.state.wall_run_contact + g.player.state.wall_run_normal, rl.RED)
	rl.DrawLine3D(g.player.state.left_wall_contact, g.player.state.left_wall_contact + g.player.state.left_wall_normal, rl.BLUE)
	rl.DrawLine3D(g.player.state.right_wall_contact, g.player.state.right_wall_contact + g.player.state.right_wall_normal, rl.YELLOW)
	rl.DrawLine3D(g.player.state.climb_up_contact, g.player.state.climb_up_contact + g.player.state.climb_up_normal, rl.MAGENTA)
	rl.DrawLine3D(g.player.state.upright_contact, g.player.state.upright_contact + g.player.state.upright_normal, rl.WHITE)
}

init_player_pbd :: proc(pbd_world: ^PBD_World, player_pos: rl.Vector3) -> Player_PBD {
	fmt.println("init_player_pbd", player_pos, player_pos)
	pbd := Player_PBD{}
	pbd.Core = pbd_create_point(pbd_world, player_pos)
	pbd.Left_Leg = pbd_create_point(pbd_world, player_pos + rl.Vector3{0.3, -1, 0})
	pbd.Right_Leg = pbd_create_point(pbd_world, player_pos + rl.Vector3{-0.3, -1, 0})
	pbd.Left_Arm = pbd_create_point(pbd_world, player_pos + rl.Vector3{0.3, 1, 0})
	pbd.Right_Arm = pbd_create_point(pbd_world, player_pos + rl.Vector3{-0.3, 1, 0})

	pbd.Left_Leg_Spring = pbd_create_spring(pbd_world, pbd.Core.id, pbd.Left_Leg.id, 1.0)
	pbd.Right_Leg_Spring = pbd_create_spring(pbd_world, pbd.Core.id, pbd.Right_Leg.id, 1.0)
	pbd.Left_Arm_Spring = pbd_create_spring(pbd_world, pbd.Core.id, pbd.Left_Arm.id, 1.0)
	pbd.Right_Arm_Spring = pbd_create_spring(pbd_world, pbd.Core.id, pbd.Right_Arm.id, 1.0)

	return pbd
}