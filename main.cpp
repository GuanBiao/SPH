#include "fluid.hpp"

using namespace std;

Fluid fluid;
bool start = false;

void displayCallback( void ) {
    glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
    gluLookAt(0.2, 0.3, 0.6, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0);
    if (start)
        fluid.simulate();
    fluid.draw();

    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    gluPerspective(60.0, 1024.0 / 768.0, 0.05, 100.0);

    glutSwapBuffers();
}

void keyboardCallback( unsigned char key, int x, int y ) {
    if (key == 's') // Start to simulate
        start = true;
}

void setLighting( void ) {
    glEnable(GL_LIGHTING);
    glEnable(GL_LIGHT0);

    float lightPos[] = {0.0f, 0.4f, 1.0f, 0.0f};
    float lightAmb[] = {0.0f, 0.0f, 0.0f, 1.0f};
    float lightDif[] = {1.0f, 1.0f, 1.0f, 1.0f};
    float lightSpc[] = {1.0f, 1.0f, 1.0f, 1.0f};
    glLightfv(GL_LIGHT0, GL_POSITION, lightPos);
    glLightfv(GL_LIGHT0, GL_AMBIENT , lightAmb);
    glLightfv(GL_LIGHT0, GL_DIFFUSE , lightDif);
    glLightfv(GL_LIGHT0, GL_SPECULAR, lightSpc);

    float matAmb[] = {0.7f, 0.7f, 0.9f, 1.0f};
    float matDif[] = {0.7f, 0.7f, 0.9f, 1.0f};
    float matSpc[] = {0.0f, 0.0f, 0.0f, 1.0f};
    float matShi[] = {1.0f, 1.0f, 1.0f, 1.0f};
    glMaterialfv(GL_FRONT, GL_AMBIENT  , matAmb);
    glMaterialfv(GL_FRONT, GL_DIFFUSE  , matDif);
    glMaterialfv(GL_FRONT, GL_SPECULAR , matSpc);
    glMaterialfv(GL_FRONT, GL_SHININESS, matShi);
}

int main( int argc, char *argv[] ) {
    glutInit(&argc, argv);
    glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGBA | GLUT_DEPTH);
    glutInitWindowSize(1024, 768);
    glutCreateWindow("SPH");
    glutDisplayFunc(displayCallback);
    glutIdleFunc(displayCallback);
    glutKeyboardFunc(keyboardCallback);
    glEnable(GL_DEPTH_TEST);
    glShadeModel(GL_SMOOTH);
    setLighting();
    glutMainLoop();
    return 0;
}