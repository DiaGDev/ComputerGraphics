uniform sampler2D texture0;
uniform bvec4 StateControl;

varying vec2 texcoord;
varying vec4 basecolor;

void main(void)
{
    if(StateControl[3])
        gl_FragColor = texture2D( texture0, texcoord) * basecolor;
    else
        gl_FragColor = basecolor;
}
