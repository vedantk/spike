/*
 * spike.cc
 */

#include "arm.hh"
#include "scene.hh"

#define INIT_WIDTH 800
#define INIT_HEIGHT 600

struct Scene scene;

static void display()
{
    glClearColor(0.0, 0.0, 0.0, 1.0);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glClearDepth(1.0);
    glEnable(GL_DEPTH_TEST);
    glShadeModel(GL_SMOOTH);

    scene.reorient();
    scene.setupProjection();
    scene.render();

    glFlush();
    glutSwapBuffers();
}

void reshape(int w, int h)
{
    scene.vwidth = float(w);
    scene.vheight = float(h);
    glViewport(0, 0, w, h);
    scene.setupProjection();
}

static void handle_key(unsigned char key, int, int)
{
    const float camStep = 0.5;
    static bool isFullscreen = false;

    switch (key) {
    case 'q':
        exit(0);
        break;

    case 'f':
        if (isFullscreen) {
            glutReshapeWindow(INIT_WIDTH, INIT_HEIGHT);
        } else {
            glutFullScreen();
        }
        isFullscreen = !isFullscreen;
        break;

    case 'w':
        scene.cameraOffset(1) += camStep;
        break;

    case 'a':
        scene.cameraOffset(0) -= camStep;
        break;

    case 's':
        scene.cameraOffset(1) -= camStep;
        break;

    case 'd':
        scene.cameraOffset(0) += camStep;
        break;

    case '+':
        scene.cameraOffset(2) -= camStep;
        break;

    case '-':
        scene.cameraOffset(2) += camStep;
        break;

    default:
        cout << "Unhandled regular key: " << key << endl;
    }

    return;
}

static void handle_key_special(int key, int, int)
{

    switch (key) {
    case GLUT_KEY_UP:
        break;

    case GLUT_KEY_LEFT:
        break;

    case GLUT_KEY_DOWN:
        break;

    case GLUT_KEY_RIGHT:
        break;

    default:
        cout << "Unhandled special key: " << key << endl;
    }

    return;
}

static float flatSurface(float x, float z, float t)
{
    return 0;
}

static float waveSurface(float x, float z, float t)
{
    return 2 + sin(t + x) + cos(t + z);
}

static void init()
{
    scene.addThing(new Thing(0.7, Point3f(0, 6, -8)));
    scene.addSurface(waveSurface);
}

static void idle()
{
    glutPostRedisplay();
}

int main(int argc, char** argv)
{
    glutInit(&argc, argv);
    glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGBA | GLUT_DEPTH);

    glutInitWindowSize(INIT_WIDTH, INIT_HEIGHT);
    glutInitWindowPosition(-1, -1);
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
