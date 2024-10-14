#version 430
precision highp float;
layout (std140, location=0) in vec4 vVert;
layout (std140, location=1) in vec4 vNorm;

layout (std140, location = 0) uniform Light
{
    vec4 ambient;
    vec4 diffuse;
    vec4 specular;
    vec4 position;
} light;

layout (std140, location = 16) uniform Material
{
    vec4 ambient;
    vec4 diffuse;
    vec4 specular;
} material;

layout (std140, location = 28) uniform Transforms
{
    mat4 mvp;
    mat4 m;
} transforms;

layout (std140, location = 60) uniform State
{
    vec4 states;
} state;

out vec3 n;
out vec3 l;
out vec3 v;

out vec4 amb;
out vec4 dif;
out vec4 spe;

void main()
{
    amb=light.ambient*material.ambient*state.states[0];
    dif=light.diffuse*material.diffuse*state.states[1];
    spe=light.specular*material.specular*state.states[2];

    vec3 p=(transforms.m*vVert).xyz;
    n = (transforms.m*vNorm).xyz;
    l= light.position.xyz-p;
    v = -p;

    gl_Position=transforms.mvp*vVert;
}