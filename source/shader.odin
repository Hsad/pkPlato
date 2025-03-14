package game
//import rl "vendor:raylib"
/*
// Shader initialization function
init_shader :: proc() -> (rl.Shader, rl.Material, rl.Model) {
    // Load instancing shader
    shader := rl.LoadShader(
        "assets/cube.vs", 
        "assets/cube.fs",
    )
    
    // Create a default material that will use our shader
    material := rl.LoadMaterialDefault()
    material.shader = shader
    
    // Create a mesh for a cube if not already cached
    cube_mesh := rl.GenMeshCube(1.0, 1.0, 1.0)
    defer rl.UnloadMesh(cube_mesh)

    // Create a model from the mesh
    model := rl.LoadModelFromMesh(cube_mesh)

    // Set the material for the model
    model.materials[0] = material

    return shader, material, model
}

// Draw multiple cubes using instancing
draw_instanced_cube :: proc(model: rl.Model, positions: []rl.Vector3) {
    if len(positions) == 0 {
        return
    }
    
    
    // Create a buffer for instance transforms
    instance_count := i32(len(positions))
    
    when ODIN_OS == .JS {
        // For WebGL, we need to handle matrices differently
        // This is a simplified approach - you may need to adjust based on your raylib implementation
        transforms := make([]rl.Matrix, len(positions))
        defer delete(transforms)
        
        // Fill the transforms array with position matrices
        for i, pos in positions {
            transforms[i] = rl.MatrixTranslate(pos.x, pos.y, pos.z)
        }
        
        // Draw the instanced model - raylib should handle the matrix splitting internally
        rl.DrawMeshInstanced(model.meshes[0], model.materials[0], &transforms[0], instance_count)
    } else {
        // Desktop version - same as before
        transforms := make([]rl.Matrix, len(positions))
        defer delete(transforms)
        
        // Fill the transforms array with position matrices
        for pos, i in positions {
            transforms[i] = rl.MatrixTranslate(pos.x, pos.y, pos.z)
        }
        
        // Draw the instanced model
        rl.DrawMeshInstanced(model.meshes[0], model.materials[0], &transforms[0], instance_count)
    }
}
    */