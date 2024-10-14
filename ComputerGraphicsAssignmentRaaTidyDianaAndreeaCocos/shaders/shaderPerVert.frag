#version 430
in vec4 colour;
out vec4 frag_color;
void main()
{
    frag_color=colour;
//    frag_color=vec4(1.0, 1.0, 1.0, 1.0);
};
