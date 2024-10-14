#version 430
in vec3 n;
in vec3 l;
in vec3 v;

in vec4 ambl;
in vec4 difl;
in vec4 spel;
in vec4 ambm;
in vec4 difm;
in vec4 spem;
in vec2 text;

uniform sampler2D imgA;
uniform sampler2D imgD;
in vec4 lightStates;

out vec4 frag_color;

void main()
{
    float d=0;
    float s=0;

    vec3 nn=normalize(n);
    vec3 ll=normalize(l);
    vec3 vv=normalize(v);

    vec3 r=reflect(-ll, nn);

    float NdotL=dot(nn, ll);
    float RdotV=dot(r, vv);

    if(NdotL>0.0) d=max(NdotL, 0.0); 
    if(RdotV>0.0) s=pow(max(RdotV, 0.0), 10.0); 

/*
    vec4 amb=ambl*ambm;
    vec4 dif=difl*difm;
    vec4 spe=spel*spem;

    amb[3]=1.0;
    dif[3]=1.0;
    spe[3]=1.0;
    frag_color=lightStates[0]*clamp(vec4(mix((amb).xyz,texture(imgA, text).xyz, 0.5), amb[3]), 0.0, 1.0)
    +lightStates[1]*clamp(vec4(mix((d*dif).xyz,texture(imgD, text).xyz, 0.5), (d*dif)[3]), 0.0, 1.0)
    +lightStates[2]*clamp(s*spe, 0.0, 1.0);
*/
    vec4 amb=ambl*texture(imgA, text);
    vec4 dif=difl*texture(imgD, text);
    vec4 spe=spel*spem;
    amb[3]=1.0;
    dif[3]=1.0;
    spe[3]=1.0;

    frag_color=lightStates[0]*amb+lightStates[1]*d*dif+lightStates[2]*clamp(s*spe, 0.0, 1.0);

};
