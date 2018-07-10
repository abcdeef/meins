#version 100
attribute highp vec4 a_posH;
attribute highp vec4 a_posL;
uniform mat4 u_mvpMatrix;
uniform vec2 u_offsetH;
uniform vec2 u_offsetL;
uniform vec2 u_mapOffset;
uniform vec3 u_color;
varying vec4 v_color;
varying vec4 v_Position;
void main() {
    gl_Position = a_posH;
    
    gl_Position.x += a_posL.x;
    gl_Position.y += a_posL.y;
    gl_Position.x -= float(u_offsetH.x);
    gl_Position.x -= u_offsetL.x;
    gl_Position.y = float(u_offsetH.y) - gl_Position.y;
    gl_Position.y += u_offsetL.y;
    
    //v_Position = gl_Position;

    gl_Position  *= u_mvpMatrix;
    gl_Position.x -= u_mapOffset.x;
    gl_Position.y -= u_mapOffset.y;
    v_color = vec4(u_color.x,u_color.y,u_color.z,1.0);
}