#version 430

//layout (std140, location = 0) uniform Light
layout (std140) uniform Light
{
    vec4 ambient;
    vec4 diffuse;
    vec4 specular;
    vec4 position;
} light;

//layout (std140, location = 16) uniform Material
layout (std140) uniform Material
{
    vec4 ambient;
    vec4 diffuse;
    vec4 specular;
    vec4 additional;
    vec4 texture;
} material;

//layout (std140, location = 28) uniform Transforms
layout (std140) uniform Transforms
{
    mat4 mvp;
    mat4 m;
} transforms;

//layout (std140, location = 60) uniform State
layout (std140) uniform State
{
    vec4 lightStates;
    vec4 lightingModelStates;
    vec4 normalState;
    vec4 textureStates;
} state;

layout (std140) uniform sampler2D imgA;
layout (std140) uniform sampler2D imgD;
layout (std140) uniform sampler2D imgS;
layout (std140) uniform sampler2D imgN;

in vec3 n;
in vec3 l;
in vec3 v;

in vec2 text;
in vec4 ambm;
in vec4 difm;
in vec4 spem;
in vec4 ambl;
in vec4 difl;
in vec4 spel;

out vec4 frag_color;

void main()
{
    if(state.lightingModelStates[0]>0.5)
    {
        vec4 am=mix(ambm, texture(imgA, text), state.textureStates[0]*material.texture[0]);
        vec4 dm=mix(difm, texture(imgD, text), state.textureStates[1]*material.texture[1]);
        vec4 sm=spem;
        vec4 al=ambl;
        vec4 dl=difl;
        vec4 sl=spel;

        frag_color=am*al+dm*dl+sm*sl;
    }
    else
    {
        vec4 pp=(mix(ambm,texture(imgA, text), state.textureStates[0]*material.texture[0]))*light.ambient*state.lightStates[0];

        vec3 nn=normalize(mix(n, texture(imgN, text).xyz,state.textureStates[3]));
        vec3 ll=normalize(l);
        vec3 vv=normalize(v);
        vec3 r=reflect(-ll, nn);

        float NdotL=dot(nn, ll);
        float RdotV=dot(r, vv);

        if(NdotL>0.0) pp+=(mix(material.diffuse,texture(imgD, text), state.textureStates[1]*material.texture[1]))*light.diffuse*max(NdotL, 0.0)*state.lightStates[1];
        if(RdotV>0.0) pp+=(mix(material.specular,texture(imgS, text), state.textureStates[2]*material.texture[2]))*light.specular*pow(max(RdotV, 0.0), material.additional[0])*state.lightStates[2];

        frag_color=pp;
    }
};
