
package game

import "core:fmt"
import rl "vendor:raylib"

PBD_Object :: struct {
	pos: rl.Vector3,
	vel: rl.Vector3,
}

Constraint :: struct{

}

Object :: struct{

}


// There are two classes of constraints:
// constrains that should always be enforced
// and constraints that only come into effect when the context is right, ie collisions

// The second type of contraints ought to be generated on the fly by the collision system

simulate_ball_on_floor :: proc() {
    fmt.printf("some number: %d\n", g_mem.some_number)
}

/*
ObjectPair :: struct {
    object1: int,
    object2: int,
}

findCollisions :: proc() -> []ObjectPair {
    //collisions = []
}

resolveCollision :: proc() {
    //resolveCollision()
}

playerStartStep :: proc() {
    //playerCore
    //playerFeet
}

playerConstraint :: proc() {
    //playerFeet
    //playerCore
}

playerResolveCollision :: proc() {
    //playerFeet
    //playerCore
}

playerEndStep :: proc() {
    //playerFeet
    //playerCore
}

{
    collisions := findCollisions()

    for i in 0..<g_mem.substeps {
        for j in 0..<g_mem.pbd_objects {
            playerStartStep()
            startStep() // gravity, prev pos, add vel to pos / sub_dt
        }
        for j in 0..<g_mem.constraints { // solve positions
            playerConstraint()
            constraint() // floor, etc
        }
        for i in collisions {
            playerResolveCollision()
            resolveCollision(i)
        }

        for j in 0..<g_mem.objects {
            playerEndStep()
            endStep() // vel = (new - old) / sub_dt
        }
        // velocity / friction step // solve velocities

    }
}
*/