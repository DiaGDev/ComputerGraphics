uniform int walkState;

void main()
{
    if(walkState==0) gl_TexCoord[0]=gl_MultiTexCoord0;
    else if(walkState==1) gl_TexCoord[0]=gl_MultiTexCoord1;
	gl_Position = ftransform();
}