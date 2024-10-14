varying vec3 normal;
varying vec2 texcoord;

void main()
{
	normal = gl_Normal;
    texcoord = gl_MultiTexCoord0.xy;

	gl_Position = ftransform();
}