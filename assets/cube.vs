#version 100

// Input vertex attributes
attribute vec3 vertexPosition;
attribute vec2 vertexTexCoord;
attribute vec3 vertexNormal;
attribute vec4 vertexColor;

// Input instance matrix (for WebGL 1.0, we need to split the matrix)
attribute vec4 instanceTransform0;
attribute vec4 instanceTransform1;
attribute vec4 instanceTransform2;
attribute vec4 instanceTransform3;

// Output vertex attributes (to fragment shader)
varying vec2 fragTexCoord;
varying vec4 fragColor;
varying vec3 fragNormal;

// Uniform locations
uniform mat4 mvp;

void main() {
    // Reconstruct instance transform matrix
    mat4 instanceTransform = mat4(
        instanceTransform0,
        instanceTransform1,
        instanceTransform2,
        instanceTransform3
    );
    
    // Calculate final vertex position
    mat4 instanceMVP = mvp * instanceTransform;
    gl_Position = instanceMVP * vec4(vertexPosition, 1.0);
    
    // Pass vertex attributes to fragment shader
    fragTexCoord = vertexTexCoord;
    fragColor = vertexColor;
    fragNormal = normalize(vec3(instanceTransform * vec4(vertexNormal, 0.0)));
}