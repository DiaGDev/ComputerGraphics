//#version 330
varying vec2 texcoord;
uniform bvec4 StateControl;
uniform bvec4 StateControl2;

varying vec4 color;
varying vec4 glcolor;
varying vec4 position;
varying vec3 normal;


void main(void)
{
    color=vec4(0.0, 0.0, 0.0, 0.0);
    glcolor=gl_Color;
    texcoord = gl_MultiTexCoord0.xy;
    normal=normalize(gl_NormalMatrix * gl_Normal);
    gl_Position=position=gl_ModelViewProjectionMatrix * gl_Vertex;
}