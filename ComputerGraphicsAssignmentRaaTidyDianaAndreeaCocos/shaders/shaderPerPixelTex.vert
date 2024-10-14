#version 430
precision highp float;
layout (std140, location=0) in vec4 vVert;
layout (std140, location=1) in vec4 vNorm;
layout (std140, location=2) in vec2 vText;

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

out vec4 ambl;
out vec4 difl;
out vec4 spel;
out vec4 ambm;
out vec4 difm;
out vec4 spem;
out vec2 text;
out vec4 lightStates;


void main()
{
    lightStates=state.states;
    ambl=light.ambient;
    difl=light.diffuse;
    spel=light.specular;
    ambm=material.ambient;
    difm=material.diffuse;
    spem=material.specular;

    vec3 p=(transforms.m*vVert).xyz;
    n = (transforms.m*vNorm).xyz;
    l= light.position.xyz-p;
    v = -p;

    text=vText;

    gl_Position=transforms.mvp*vVert;
}