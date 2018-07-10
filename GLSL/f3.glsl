#version 100
precision mediump float;
varying vec2 v_texCoord;
uniform vec3 u_f;
void main() {
    float dis = distance(v_texCoord, vec2(0.5, 0.5));
    if (dis > 0.5) {
        gl_FragColor = vec4(0.0,0.0,0.0,0.0);
        return;
    }
    if (dis > 0.45) {
        gl_FragColor = vec4(0.0,0.0,0.0,1.0);
        return;
    }
    gl_FragColor = vec4(u_f.x,u_f.y,u_f.z,1.0);
}