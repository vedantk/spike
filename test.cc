/*
 * test.cc
 */

#include "util.hh"

static float vwidth = 400;
static float vheight = 400;
static float vw_center = vwidth / 2.0;
static float vh_center = vheight / 2.0;

GLfloat ctrlpoints[4][4][3] = {
    {{-1.5, -1.5, 4.0}, {-0.5, -1.5, 2.0}, 
        {0.5, -1.5, -1.0}, {1.5, -1.5, 2.0}}, 
    {{-1.5, -0.5, 1.0}, {-0.5, -0.5, 3.0}, 
        {0.5, -0.5, 0.0}, {1.5, -0.5, -1.0}}, 
    {{-1.5, 0.5, 4.0}, {-0.5, 0.5, 0.0}, 
        {0.5, 0.5, 3.0}, {1.5, 0.5, 4.0}}, 
    {{-1.5, 1.5, -2.0}, {-0.5, 1.5, -2.0}, 
        {0.5, 1.5, 0.0}, {1.5, 1.5, -1.0}}
};

void initlights(void)
{
    GLfloat ambient[] = { 0.2, 0.2, 0.2, 1.0 };
    GLfloat position[] = { 0.0, 0.0, 2.0, 1.0 };
    GLfloat mat_diffuse[] = { 0.6, 0.6, 0.6, 1.0 };
    GLfloat mat_specular[] = { 1.0, 1.0, 1.0, 1.0 };
    GLfloat mat_shininess[] = { 50.0 };

    glEnable(GL_LIGHTING);
    glEnable(GL_LIGHT0);

    glLightfv(GL_LIGHT0, GL_AMBIENT, ambient);
    glLightfv(GL_LIGHT0, GL_POSITION, position);
    glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, mat_diffuse);
    glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, mat_specular);
    glMaterialfv(GL_FRONT_AND_BACK,GL_SHININESS, mat_shininess);
}

void display(void)
{
    glClearColor (0.0,0.0,0.0,1.0);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glPushMatrix();
        glRotatef(85.0, 1.0, 1.0, 1.0);
        glEvalMesh2(GL_FILL, 0, 8, 0, 8);
    glPopMatrix();
    glFlush();
    glutSwapBuffers();
}

void myinit(void)
{
    glClearColor (0.0, 0.0, 0.0, 1.0);
    glEnable(GL_DEPTH_TEST);
    glMap2f(GL_MAP2_VERTEX_3, 0, 1, 3, 4,
        0, 1, 12, 4, &ctrlpoints[0][0][0]);
    glEnable(GL_MAP2_VERTEX_3);
    glEnable(GL_AUTO_NORMAL);
    glMapGrid2f(8, 0.0, 1.0, 8, 0.0, 1.0);
    initlights();
}

static void reshape(int w, int h) {
    vwidth = w;
    vheight = h;
    vw_center = w / 2.0;
    vh_center = h / 2.0;
    glViewport(0, 0, vwidth, vheight);
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
}

int main(int argc, char** argv) {
    glutInit(&argc, argv);
    glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH);

    glutInitWindowSize(400, 400);
    glutInitWindowPosition(100, 100);
    glutCreateWindow("Spike");
    
    myinit();

    glutDisplayFunc(display);
    glutReshapeFunc(reshape);
    // glutKeyboardFunc(handle_key);
    // glutSpecialFunc(handle_special_key);

    glutMainLoop();

    return 0;
} 
