#version 430
in vec4 colour;

in vec2 text;
in vec4 amb;
in vec4 dif;
in vec4 spe;

uniform sampler2D imgA;
uniform sampler2D imgD;

out vec4 frag_color;
void main()
{
    frag_color=mix(amb, texture(imgD, text), 0.3)+mix(dif, texture(imgD, text), 0.3)+spe;
//    frag_color=amb+dif+spe;
//    frag_color=texture(img, text);
};
