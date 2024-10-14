varying vec2 texcoord;
uniform bvec4 StateControl;

varying vec4 basecolor;

void directionalLight( int lightNum, vec3 normal, inout vec4 color )
{
    vec3 n = normalize(gl_NormalMatrix * normal);

    if(StateControl[0])
    {
//        color+=gl_LightSource[lightNum].ambient*gl_Color;        
        color+=gl_FrontLightModelProduct.sceneColor*gl_Color;        
    }

    if(StateControl[1])
    {
        float NdotL = dot( n, normalize(gl_LightSource[lightNum].position.xyz) );
        NdotL = max( 0.0, NdotL );
        color+=gl_LightSource[lightNum].diffuse * NdotL*gl_Color;        
    }

    if(StateControl[2])
    {
        float NdotHV = dot( n, gl_LightSource[lightNum].halfVector.xyz );
        NdotHV = max( 0.0, NdotHV );
        color+=gl_FrontLightProduct[lightNum].specular * pow( NdotHV, gl_FrontMaterial.shininess );        
    }
}

void main(void)
{
    basecolor=vec4(0.0, 0.0, 0.0, 0.0);
    directionalLight( 0, gl_Normal.xyz, basecolor);

    texcoord = gl_MultiTexCoord0.xy;
    gl_Position = gl_ModelViewProjectionMatrix * gl_Vertex;
}