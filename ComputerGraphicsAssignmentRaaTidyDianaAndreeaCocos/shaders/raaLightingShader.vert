#version 430
precision highp float;
layout (std140) in vec4 vVert;
layout (std140) in vec4 vModelledNorm;
layout (std140) in vec4 vFlatNorm;
layout (std140) in vec4 vSmoothNorm;
layout (std140) in vec2 vText;

layout (std140) uniform Light
{
    vec4 ambient;
    vec4 diffuse;
    vec4 specular;
    vec4 position;
} light;

layout (std140) uniform Material
{
    vec4 ambient;
    vec4 diffuse;
    vec4 specular;
    vec4 additional;
    vec4 texture;
} material;

layout (std140) uniform Transforms
{
    mat4 mvp;
    mat4 m;
} transforms;

layout (std140) uniform State
{
    vec4 lightStates;
    vec4 lightingModelStates;
    vec4 normalState;
    vec4 textureStates;
} state;

out vec3 n;
out vec3 l;
out vec3 v;
out vec2 text;
out vec4 ambm;
out vec4 difm;
out vec4 spem;
out vec4 ambl;
out vec4 difl;
out vec4 spel;

void main()
{
    vec4 norm=state.normalState[0]*vFlatNorm+state.normalState[1]*vSmoothNorm+state.normalState[2]*vModelledNorm;

    vec3 p=(transforms.m*vVert).xyz;
    n = (transforms.m*norm).xyz;
    l= light.position.xyz-p;
    v = -p;

    if(state.lightingModelStates[0]>0.5)
    {
        ambm=material.ambient;
        difm=material.diffuse;
        spem=material.specular;

        ambl=light.ambient*state.lightStates[0];
        difl=vec4(0.0, 0.0, 0.0, 1.0);
        spel=vec4(0.0, 0.0, 0.0, 1.0);

        n=normalize(n);
        l=normalize(l);
        v=normalize(v);
        vec3 r=normalize(reflect(-l, n));
        float NdotL=dot(n, l);
        float RdotV=dot(r, v);

        if(NdotL>0.0) difl=light.diffuse*max(NdotL, 0.0)*state.lightStates[1];
        if(RdotV>0.0) spel=light.specular*pow(max(RdotV, 0.0), material.additional[0])*state.lightStates[2];
    }

    text=vText;
    gl_Position=transforms.mvp*vVert;
}