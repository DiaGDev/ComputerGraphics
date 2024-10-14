#version 430
precision highp float;
layout (std140, location=0) in vec4 vVert;
layout (std140, location=1) in vec4 vNorm;

/*
struct _Lighting
{
    vec4 ambient;
    vec4 diffuse;
    vec4 specular;
    vec4 position;
} Lighting;

struct _Material
{
    vec4 ambient;
    vec4 diffuse;
    vec4 specular;
} Material;

struct _State
{
    vec4 states;
} State;

struct _Transforms
{
    mat4 mvp;
    mat4 m;
} Transforms;



layout (std140, location = 0) uniform Light light;
layout (std140, location = 1) uniform Material material;
layout (std140, location = 2) uniform State state;
layout (std140, location = 3) uniform Transforms transforms;
*/


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
    vec4 ambCol=light.ambient*material.ambient*state.states[0];
    ambCol[3]=1.0;

    vec4 diffCol=vec4(0.0, 0.0, 0.0, 1.0);
    vec3 norm=normalize(transforms.m*vNorm).xyz;
    vec3 lightP=(transforms.m*light.position).xyz;
    vec3 vertP=(transforms.m*vVert).xyz;
    vec3 lightD=normalize(vertP-lightP);
    float NdotL= dot(norm, lightD);
    if(NdotL>0.0)
    {
        diffCol=light.diffuse*material.diffuse*NdotL*state.states[1];
        diffCol[3]=1.0;
    }

    vec3 eye = -vec3(transforms.m[0][3], transforms.m[1][3], -transforms.m[2][3]);
    vec4 specCol=vec4(0.0, 0.0, 0.0, 1.0);
    vec3 vertToEye=normalize(eye-vVert.xyz);
    vec3 refl=vNorm.xyz*(-2.0*dot(lightD, vNorm.xyz))+lightD;
    float VdotL=max(dot(vertToEye, refl), 0.0);
    if (VdotL > 0.0) 
    {
		specCol = material.specular * light.specular * pow(VdotL,30.0) *state.states[2];
	}

    colour=clamp(ambCol+diffCol+specCol, 0.0, 1.0);
    colour[3]=1.0;
    gl_Position=transforms.mvp*vVert;
}