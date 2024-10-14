varying vec3 normal;
uniform sampler2D texture0;
varying vec2 texcoord;

void main()
{
	float intensity;
	vec4 color;
	intensity = dot(normalize(gl_LightSource[0].position.xyz),normal);


	if (intensity > 0.95) intensity=1.0;
//	else if (intensity > 0.5) intensity=0.5;
//	else if (intensity > 0.25) intensity=0.25;
	else intensity=0.1;

    color = texture2D( texture0, texcoord)*intensity;
    color[3]=1.0;

	gl_FragColor = color;

}