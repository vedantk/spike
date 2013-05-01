/*
 * spike.cc
 */

#include "arm.hh"
#include "scene.hh"

#define INIT_WIDTH 800
#define INIT_HEIGHT 600

struct Scene scene;
Point3f origEye;

static float flatSurface(float x, float z, float t)
{
    (void) x;
    (void) z;
    (void) t;
    return 0.5;
}

static float waveSurface(float x, float z, float t)
{
    return 2 + sin(t + x) + cos(t + z);
}

static float timeInvariantWaveSurface(float x, float z, float t) 
{
    return 2 + sin(x) + cos(z);
}

static void display()
{
    glClearColor(0.0, 0.0, 0.0, 1.0);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glClearDepth(1.0);
    glEnable(GL_DEPTH_TEST);
    glShadeModel(GL_SMOOTH);

    // need to find a better setting for scene.eye
    GLfloat light_position[] = { expand_vec3(origEye), 0 };
    GLfloat light_ambient[] = { 0.1, 0.1, 0.1, 1.0 };
    GLfloat light_diffuse[] = { 0.3, 0.3, 0.3, 1 };
    GLfloat light_specular[] = { 1, 1, 1, 1 };

    glLightfv(GL_LIGHT0, GL_POSITION, light_position);
    glLightfv(GL_LIGHT0, GL_AMBIENT, light_ambient);
    glLightfv(GL_LIGHT0, GL_DIFFUSE, light_diffuse);
    glLightfv(GL_LIGHT0, GL_SPECULAR, light_specular);

    glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
    glEnable(GL_NORMALIZE);
    setNormalMaterial();

    scene.reorient();
    scene.setupProjection();
    scene.render();
    // scene.getFocusedThing()->touchSurfaceImmediately(scene.time);

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
    Point3f relativeEye;
    Matrix3f rotate;

    switch (key) {
    case 'o':
        for (size_t i=0; i < scene.things.size(); ++i) {
            scene.things[i]->surface = flatSurface;
        }
        break;

    case 'p':
        for (size_t i=0; i < scene.things.size(); ++i) {
            scene.things[i]->surface = timeInvariantWaveSurface;
        }
        break;

    case 'i':
        for (size_t i=0; i < scene.things.size(); ++i) {
            scene.things[i]->surface = waveSurface;
        }
        break;

    case 'Q':
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

    /* translations */
    case 'w': // "up"
        scene.eye += camStep * scene.up;
        scene.lookAt += camStep * scene.up;
        break;

    case 'a': // "left"
        scene.eye -= camStep * scene.right;
        scene.lookAt -= camStep * scene.right;
        break;

    case 's': // "down"
        scene.eye -= camStep * scene.up;
        scene.lookAt -= camStep * scene.up;
        break;

    case 'd': // "right"
        scene.eye += camStep * scene.right;
        scene.lookAt += camStep * scene.right;
        break;

    /* zoom */
    case 'q': // "zoom out"
        scene.eye -= camStep * scene.lookDir;
        break;

    case 'e': // "zoom out"
        scene.eye += camStep * scene.lookDir;
        break;

    /* rotations */
    case 'W': // rotate "up"
        scene.rotateEyeAboutRightAxis(true);
        break;

    case 'S':
        scene.rotateEyeAboutRightAxis(false);
        break;

    case 'D':
        scene.rotateEyeAboutYAxis(true);
        break;

    case 'A':
        scene.rotateEyeAboutYAxis(false);
        break;

    default:
        cout << "Unhandled regular key: " << key << endl;
    }

    return;
}

static void handle_key_special(int key, int, int)
{
    // up is in? i dunno, hack
    float up = -1; // fsign(scene.lookAt.z());
    float right = 1; // fsign(scene.lookAt.x());

    switch (key) {
    case GLUT_KEY_UP:
        scene.getFocusedThing()->moveTowards(make_pair(0.0f, up), scene.time);
        break;

    case GLUT_KEY_LEFT:
        scene.getFocusedThing()->moveTowards(make_pair(-right, 0.0f), scene.time);
        break;

    case GLUT_KEY_DOWN:
        scene.getFocusedThing()->moveTowards(make_pair(0.0f, -up), scene.time);
        break;

    case GLUT_KEY_RIGHT:
        scene.getFocusedThing()->moveTowards(make_pair(right, 0.0f), scene.time);
        break;

    default:
        cout << "Unhandled special key: " << key << endl;
    }

    return;
}

static void init()
{
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_LIGHTING);
    glEnable(GL_LIGHT0);
    scene.addThing(new Thing(waveSurface, Point3f(0, 5, -8)));
    scene.addThing(new Thing(waveSurface, Point3f(0, 5, -2)));
    scene.getFocusedThing()->touchSurfaceImmediately(scene.time);

    scene.setupEye();
    origEye = scene.eye;
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
    glutIdleFunc(display);

    init();

    glutMainLoop();

#ifdef _WIN32
    int tmp;
    cin >> tmp;
#endif
    
    return 0;
}
