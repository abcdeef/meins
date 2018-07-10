#version 100
attribute vec4 a_position;
uniform vec2 u_mapOffset;
uniform float u_angle;
uniform vec3 u_color;
varying vec4 v_color;
float c;
float s;
void main() {
    c = cos(-u_angle);
    s = sin(-u_angle);
    gl_Position = a_position;
    float tmp_x = gl_Position.x;
    gl_Position.x = gl_Position.x * c - gl_Position.y * s;
    gl_Position.y = tmp_x * s + gl_Position.y * c;
    gl_Position.x -= u_mapOffset.x;
    gl_Position.y -= u_mapOffset.y;
    v_color = vec4(u_color.x,u_color.y,u_color.z,1.0);
}