#version 330

// Input vertex attributes (from vertex shader)
in vec3 fragPosition;
in vec2 fragTexCoord;
in vec3 fragNormal;

// Output fragment color
out vec4 finalColor;

void main() {
    // Define color stops for terrain
    vec4 waterColor = vec4(0.26, 0.50, 0.80, 1.0);    // Water blue
    vec4 sandColor = vec4(0.76, 0.70, 0.50, 1.0);    // Sandy brown
    vec4 grassColor = vec4(0.48, 0.62, 0.38, 1.0);   // Rainforest Green
    vec4 pineColor = vec4(0.20, 0.38, 0.27, 1.0);    // Dark pine
    vec4 stoneColor = vec4(0.50, 0.50, 0.50, 1.0);   // Stone grey
    vec4 snowColor = vec4(1.00, 1.00, 1.00, 1.0);    // Snow white

    float height = fragPosition.y;

    vec4 terrainColor;
    if (height < 1.5) {
        terrainColor = waterColor;
    } else if (height < 2.0) {
        terrainColor = mix(waterColor, sandColor, smoothstep(1.9, 2.0, height));
    } else if (height < 3.0) {
        terrainColor = mix(sandColor, grassColor, smoothstep(2.0, 3.0, height));
    } else if (height < 6.0) {
        terrainColor = mix(grassColor, pineColor, smoothstep(3.0, 6.0, height));
    } else if (height < 7.0) {
        terrainColor = mix(pineColor, stoneColor, smoothstep(6.0, 7.0, height));
    } else {
        terrainColor = mix(stoneColor, snowColor, smoothstep(7.0, 8.0, height));
    }

    // Add slight random variation to each triangle based on position
    float random = fract(sin(dot(fragPosition.xz, vec2(12.9898, 78.233))) * 43758.5453);
    terrainColor = mix(terrainColor, terrainColor * 0.9, random * 0.2);

    // Add edge highlighting
    vec3 dx = dFdx(fragPosition);
    vec3 dy = dFdy(fragPosition);
    vec3 normal = normalize(cross(dx, dy));
    float edgeFactor = 1.0 - abs(dot(normal, vec3(0.0, 1.0, 0.0)));
    vec4 edgeColor = vec4(0.0, 0.0, 0.0, 1.0);
    
    // Mix in edge color
    finalColor = mix(terrainColor, edgeColor, edgeFactor * 0.3);
}