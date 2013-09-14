attribute vec4 Position;

uniform mat4 ProjectionRealWorld;
uniform mat4 Projection;
uniform mat4 Modelview;

attribute vec2 TexCoordIn;
varying vec2 TexCoordOut;

void main(void) {
    vec4 posRW;
    posRW.x = Position.z*2.0*Position.x/ProjectionRealWorld[0][0];
    posRW.y = Position.z*2.0*Position.y/ProjectionRealWorld[1][1];
    posRW.z = Position.z;
    posRW.w = 1.0;
    
    gl_Position = Projection * Modelview * posRW;
    TexCoordOut = TexCoordIn;
}