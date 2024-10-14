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

out vec4 colour;

void main()
{
    vec4 amb=light.ambient*material.ambient*state.states[0];
    vec4 dif=vec4(0.0, 0.0, 0.0, 1.0);
    vec4 spe=vec4(0.0, 0.0, 0.0, 1.0);

    vec3 p=(transforms.m*vVert).xyz;
    vec3 n = normalize((transforms.m*vNorm).xyz);
    vec3 l= normalize(light.position.xyz-p);
    vec3 v = normalize(-p);

    vec3 r=normalize(reflect(-l, n));
    float NdotL=dot(n, l);
    float RdotV=dot(r, v);

    if(NdotL>0.0) dif=light.diffuse*material.diffuse*state.states[1]*max(NdotL, 0.0);
    if(RdotV>0.0) spe=light.specular*material.specular*state.states[2]*pow(max(RdotV, 0.0), 1000.0);

    colour=amb+clamp(dif, 0.0, 1.0)+clamp(spe, 0.0, 1.0);

    gl_Position=transforms.mvp*vVert;
}