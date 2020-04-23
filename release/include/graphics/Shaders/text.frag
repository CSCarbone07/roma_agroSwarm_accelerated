#version 330 core
in vec2 TexCoords;
out vec4 color;

uniform sampler2D text;
uniform vec3 inColor;

in vec4 vColor;

out vec4 fColor;

void main()
{    
    fColor = vColor;
    vec4 sampled = vec4(1.0, 1.0, 1.0, texture(text, TexCoords).r);
    color = fColor * sampled;
}  
