



#version 330  
                                                 
layout (location = 0) in vec3 pos;                            

out vec4 vColor;

uniform float xMove;
uniform mat4 model;
uniform mat4 projection;
uniform mat4 view;

void main()                                                    
{


                                                              
    //gl_Position = projection * model * vec4(pos.x, pos.y, pos.z, 1.0);      
    gl_Position = projection * view * model * vec4(pos, 1.0);      
	vColor = vec4(clamp(pos, 0.0f, 1.0f), 1.0f);
    //vColor = vec4(0.0f, 1.0f, 0.0f, 1.0f);

}                                                              
                                                               

