#version 430

uniform vec4 CameraEye;

in VertexData{
    vec4 mColor;
    vec4 mVertex;
} VertexIn;

void main(void)
{
   gl_FragColor = vec4(0.33, 0.23, 0.3, 1.0);
//    gl_FragColor = VertexIn.mColor;
}