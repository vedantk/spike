/*
 * spike.cc
 */

#include "scene.hh"

#define INIT_WIDTH 800
#define INIT_HEIGHT 600

struct Scene scene;

static void display()
{
    glEnable(GL_LIGHT0);
    glEnable(GL_LIGHTING);
    glEnable(GL_DEPTH_TEST);
    glShadeModel(GL_SMOOTH);

    glClearColor(0.0, 0.0, 0.0, 1.0);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    glOrtho(-10, 10, -10, 10, 10, -10);

    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();

    scene.orient();
    /*
    gluLookAt(scene.eye.x(),
              scene.eye.y(),
              scene.eye.z(),
              scene.lookAt.x(),
              scene.lookAt.y(),
              scene.lookAt.z(),
              0, 1, 0);
              */

    gluLookAt(0, 0, 0, 0, 0, -10, 0, 1, 0);

    cout << "scene.render()\n";
    print_vec3("eye", scene.eye);
    print_vec3("lookAt", scene.lookAt);
    cout << endl;
    scene.render();

    glFlush();
    glutSwapBuffers();
}

void reshape(int w, int h)
{
    glViewport(0, 0, w, h);
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    glOrtho(-10, 10, -10, 10, 10, -10);
}

static void handle_key(unsigned char key, int, int)
{
    static bool isFullscreen = false;

    switch (key) {
    case 'f':
        if (isFullscreen) {
            glutReshapeWindow(INIT_WIDTH, INIT_HEIGHT);
        } else {
            glutFullScreen();
        }
        isFullscreen = !isFullscreen;
        break;
    }

    return;
}

static void handle_key_special(int key, int, int)
{
    return;
}

static float flatSurface(float x, float z, float t)
{
    return 0;
}

static void init()
{
    scene.addThing(new Thing(0.5));
    scene.addSurface(flatSurface);
}

static void idle()
{
    glutPostRedisplay();
}

int main(int argc, char** argv)
{
    glutInit(&argc, argv);
    glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH);

    glutInitWindowSize(INIT_WIDTH, INIT_HEIGHT);
    glutInitWindowPosition(24, 24);
    glutCreateWindow("Spike");
    
    glutDisplayFunc(display);
    glutReshapeFunc(reshape);
    glutKeyboardFunc(handle_key);
    glutSpecialFunc(handle_key_special);
    glutIdleFunc(idle);

    init();

    glutMainLoop();

#ifdef _WIN32
    int tmp;
    cin >> tmp;
#endif
    
    return 0;
}
