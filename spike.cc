/*
 * spike.cc
 */

#include "iface.hh"

/* Viewport settings. */
static int vwidth = 800;
static int vheight = 600;
static float vw_center = vwidth / 2.0;
static float vh_center = vheight / 2.0;

int main(int argc, char** argv) {
    glutInit(&argc, argv);
    glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH);

    glutInitWindowSize(vwidth, vheight);
    glutInitWindowPosition(24, 24);
    glutCreateWindow("Spike");
    
    // glutFullScreen();

    glutDisplayFunc(display);
    glutReshapeFunc(reshape);
    glutKeyboardFunc(handle_key);
    glutSpecialFunc(handle_special_key);

    glutMainLoop();
    
    Torso torso = {
        .radius = 5.0,
        .theta = .3,
        .phi= .3,
        .centroid = Point3f(0, 0, 0),
    };

    Arm::Vector mat;
    mat << .3, .3, .3, .3, .3, .3, .5, .5;
    Arm arm(mat);

    cout << arm.getPincerEnd(&torso) << endl;
    
    /*
    arm.getLength(0) = 3.0;
    cout << arm.getPincerEnd(&torso) << endl;
    */

    return 0;
}
