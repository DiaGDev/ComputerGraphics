uniform float walkTime;
uniform sampler2D img;

void main()
{
    vec4 col=texture(img, gl_TexCoord[0].xy);
/*
    int count =0;
    if(col[0]>0.4) count++;
    if(col[1]>0.4) count++;
    if(col[2]>0.4) count++;
*/
//    col[3]=(col[0]+col[1]+col[2])/3.0;

    float c=col[0]+col[1]+col[2];


    if(c>0.7) gl_FragColor=col*walkTime;
    else discard;
}