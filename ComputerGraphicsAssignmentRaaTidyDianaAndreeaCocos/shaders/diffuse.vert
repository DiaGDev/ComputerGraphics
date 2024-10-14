#version 430
in vec4 vVert;
in vec4 vNorm;
layout(location = 0) uniform mat4 mvp;
layout(location = 1) uniform mat4 m;
layout(location = 2) uniform vec4 matDiff;
layout(location = 3) uniform vec4 lightDiff;
layout(location = 4) uniform vec4 lightPos;
out vec4 colour;
void main()
{
    vec3 norm=normalize(m*vNorm).xyz;
    vec3 lightP=(m*lightPos).xyz;
    vec3 vertP=(m*vVert).xyz;
    vec3 lightD=normalize(lightP-vertP);
    float NdotL= dot(norm, lightD);
    colour=lightDiff*matDiff*NdotL;
    colour[3]=1.0;
    gl_Position = mvp*vVert;
};