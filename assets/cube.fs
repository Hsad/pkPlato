#version 100

precision mediump float;

// Input vertex attributes (from vertex shader)
varying vec2 fragTexCoord;
varying vec4 fragColor;
varying vec3 fragNormal;

// Uniform locations
uniform sampler2D texture0;
uniform vec4 colDiffuse;

void main() {
    // Calculate lighting (simple diffuse)
    vec3 lightDir = normalize(vec3(0.5, 1.0, 0.5));
    float diff = max(dot(fragNormal, lightDir), 0.2);
    
    // Calculate final fragment color
    vec4 texelColor = texture2D(texture0, fragTexCoord);
    gl_FragColor = texelColor * colDiffuse * fragColor * vec4(diff, diff, diff, 1.0);
}