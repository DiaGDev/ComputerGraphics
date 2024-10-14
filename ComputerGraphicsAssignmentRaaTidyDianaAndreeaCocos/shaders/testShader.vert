#version 460
//uniform layout(std140);
in vec4 vVert;

//uniform vec4 col;

layout(std140) uniform Colour
{
    vec4 col1;
    vec4 col2;
} colB;

layout(std140) uniform Transforms
{
    mat4 mvp;
} transforms;


//uniform mat4 mvp;




out vec4 colour;
void main()
{
    colour=colB.col1+colB.col2;
//    colour=vec4(1.0, 1.0, 0.0, 1.0);
    colour[3]=1.0;
    gl_Position = transforms.mvp*vVert;
//    gl_Position = mvp*vVert;
}