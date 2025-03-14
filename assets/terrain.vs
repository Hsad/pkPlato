#version 330

// Input vertex attributes
in vec3 vertexPosition;
in vec2 vertexTexCoord;
in vec3 vertexNormal;
in mat4 instanceTransform;  // Instance transform matrix

// Input uniform values
uniform mat4 mvp;  // This will be view-projection matrix

// Output vertex attributes (to fragment shader)
out vec3 fragPosition;
out vec2 fragTexCoord;
out vec3 fragNormal;

void main() {
    // Calculate fragment position based on instance transform
    vec4 worldPosition = instanceTransform * vec4(vertexPosition, 1.0);
    fragPosition = worldPosition.xyz;
    fragTexCoord = vertexTexCoord;
    fragNormal = vertexNormal;
    
    // Calculate final vertex position
    gl_Position = mvp * worldPosition;
} 