#version 430
in vec3 n;
in vec3 l;
in vec3 v;

in vec4 amb;
in vec4 dif;
in vec4 spe;

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
    if(RdotV>0.0) s=pow(max(RdotV, 0.0), 1000.0); 

    frag_color=amb+clamp(d*dif, 0.0, 1.0)+clamp(s*spe, 0.0, 1.0);
};
