#version 330

in vec4 vColor;

out vec4 fColor;

void main()
{
	fColor = vColor;
    //fColor = vec4(0.0f, 0.75f, 0.0f, 1.0f);
}
