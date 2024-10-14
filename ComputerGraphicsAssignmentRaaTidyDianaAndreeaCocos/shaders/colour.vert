uniform int state;

varying vec4 colour;
varying vec3 normal;

void main()
{
	normal = gl_Normal;

    colour=vec4(1.0, 0.0, 1.0, 0.0);
    if(state==0) colour=vec4(0.2, 0.0, 0.0, 1.0);

	gl_Position = ftransform();
}