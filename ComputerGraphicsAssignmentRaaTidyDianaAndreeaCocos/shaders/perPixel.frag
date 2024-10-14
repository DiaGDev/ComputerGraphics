//#version 330

uniform sampler2D texture0;
uniform bvec4 StateControl;
uniform bvec4 StateControl2;

varying vec2 texcoord;
varying vec4 color;
varying vec4 glcolor;
varying vec4 position;
varying vec3 normal;

float len(vec4 c)
{
//    return sqrt(c[0]*c[0]+c[1]*c[1]+c[2]*c[2]);
    return (c[0]+c[1]+c[2])/3.0;    
}

void directionalLight( int lightNum, inout vec4 col )
{
    if(StateControl[0])
    {
        col+=gl_FrontLightModelProduct.sceneColor*glcolor;        
    }

    float bumpVal=1.0;
    if(StateControl2[0]) bumpVal=len(texture2D( texture0, texcoord));
    bumpVal=clamp(bumpVal, 0.0, 1.0);

    if(StateControl[1])
    {
        float NdotL = dot( normalize(normal.xyz), normalize(gl_LightSource[lightNum].position.xyz) );
        NdotL = max( 0.0, NdotL );
        col+=gl_LightSource[lightNum].diffuse * NdotL*glcolor*bumpVal;        
    }

    if(StateControl[2])
    {
        float NdotHV = dot( normal, gl_LightSource[lightNum].halfVector.xyz );
        NdotHV = max( 0.0, NdotHV );
        col+=gl_FrontLightProduct[lightNum].specular * pow( NdotHV, gl_FrontMaterial.shininess )*bumpVal;        
    } 

    col*=1.5;   // fiddle factor
}

void main(void)
{
    vec4 col=color;
    directionalLight(0, col);

    if(StateControl[3])
        gl_FragColor = texture2D( texture0, texcoord) * col;
    else
        gl_FragColor = col;
}
