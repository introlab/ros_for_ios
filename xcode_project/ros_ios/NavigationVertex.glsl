attribute vec4 Position;
attribute vec4 Source;

uniform mat4 Projection;
uniform mat4 Modelview;

varying vec4 Destination;

void main(void) {
    gl_Position = Projection * Modelview * Position;
    Destination = Source;
}