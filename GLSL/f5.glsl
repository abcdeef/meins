#version 100
precision mediump float;
varying vec4 v_color;
varying vec4 v_Position;
void main() {
    gl_FragColor = v_color;
    //float dis = distance(vec2(gl_FragCoord.x, gl_FragCoord.y) , vec2(0.0, 0.0));
    
    //gl_FragColor = vec4(v_Position.x + 0.5, v_Position.y, 0.0, 1.0);
}