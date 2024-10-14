varying vec3 normal;
varying vec4 colour;

void main()
{
	float intensity;
	vec4 color;
	intensity = dot(normalize(gl_LightSource[0].position.xyz),normal);

    color=vec4(intensity*colour[0], intensity*colour[1], intensity*colour[2], 1.0);

	if (intensity > 0.35)
	    color=vec4(colour[0], colour[1], colour[2], 1.0);
		else
//	else if (intensity > 0.5)
//	    color=vec4(0.5*colour[0], 0.5*colour[1], 0.5*colour[2], 1.0);
//	else if (intensity > 0.25)
	    color=vec4(0.25*colour[0], 0.25*colour[1], 0.25*colour[2], 1.0);
//	else
//		discard;


	gl_FragColor = color;

}