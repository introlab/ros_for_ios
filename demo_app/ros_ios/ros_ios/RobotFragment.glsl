varying lowp vec4 Destination;

void main(void) {
    gl_FragColor = vec4(1.0,0.5,0.5,5.0) * max(dot(Destination,vec4(0.0,1.0,0.5,1.0)), 0.0);
}