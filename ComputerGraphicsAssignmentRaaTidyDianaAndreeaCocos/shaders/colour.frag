varying vec3 normal;
varying vec4 colour;




void main()
{
	float intensity;
	intensity = dot(normalize(gl_LightSource[0].position.xyz),normal);

	gl_FragColor = colour*intensity+colour*0.5;
}