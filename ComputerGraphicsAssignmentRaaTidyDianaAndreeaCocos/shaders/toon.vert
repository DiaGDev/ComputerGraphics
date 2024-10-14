varying vec3 normal;
varying vec4 colour;

void main()
{
//	normal = gl_NormalMatrix * gl_Normal;
	normal = gl_Normal;
	colour=gl_Color;

	gl_Position = ftransform();
}