/*
 * scene.hh
 */

#pragma once

#include "arm.hh"

#define NR_ARMS     6

typedef float (*Surface)(float x, float z, float t);

void setNormalMaterial() {
   GLfloat mat_ambient[] = {0.4,0.2,0.2,1};
   GLfloat mat_diffuse[] = {0.4,0.2,0.2,1};
   GLfloat mat_specular[] = {1,0,0,1};

   glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT, mat_ambient);
   glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, mat_diffuse);
   glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, mat_specular);
   glMateriali(GL_FRONT_AND_BACK, GL_SHININESS, 10);

   glEnable(GL_COLOR_MATERIAL);
   glColor3f(1, 0, 0);
}

void setSurfaceMaterial() {
   GLfloat mat_ambient[]  = {0.2,0.2,0.2,1};
   GLfloat mat_diffuse[]  = {0.2,0.2,0.8,1};
   GLfloat mat_specular[] = {0.2,0.2,0.8,1};

   glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT, mat_ambient);
   glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, mat_diffuse);
   glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, mat_specular);
   glMateriali(GL_FRONT_AND_BACK, GL_SHININESS, 10);

   glEnable(GL_COLOR_MATERIAL);
   glColor3f(0.5, 0.5, 0.5);
}

void renderSurface(Surface fn, float t, float x0, float xf, float z0, float zf)
{
    const float step = 0.1;
    for (float x=x0; x < xf; x += step) {
        for (float z=zf; z < z0; z += step) {
            float ll = fn(x, z + step, t);
            float lr = fn(x + step, z + step, t);
            float ul = fn(x, z, t);
            float ur = fn(x + step, z, t);

            // glColor3f(sin(x) + cos(z), 0, 0);

            Point3f ulp(x, ul, z);
            Point3f urp(x + step, ur, z);
            Point3f lrp(x + step, lr, z + step);
            Point3f llp(x, ll, z + step);

            // glColor3f(sin(x) + sin(z), 0, 0);
            glBegin(GL_POLYGON);
                glnorm3f((urp - llp).cross(ulp - lrp));
                glVertex3f(x, ul, z);
                glVertex3f(x + step, ur, z);
                glVertex3f(x + step, lr, z + step);
                glVertex3f(x, ll, z + step);
            glEnd();
        }
    }
}

struct MoveData {
    bool moveOddLegs;
    Vector3f direction;
    float stepSize;
    float numDeltas;
    Point3f newTorsoLocation;

    MoveData(float step) : stepSize(step) {
        moveOddLegs = true; 
        numDeltas = 10;
    }
};

struct Thing {
    Torso* torso;
    Arm* arms[NR_ARMS];
    Surface surface;
    /* if we're implementing arcing, the thing will need to know where 
       on the arc it is, and what direction it was/is moving in */
    MoveData *moveData;
    /* which part of a step each arm is on */
    float deltas[NR_ARMS];

    Thing(Surface surf, Point3f centroid)
        : surface(surf)
    {
        torso = new Torso(centroid);
        moveData = new MoveData(2);

        for (int i=0; i < NR_ARMS; ++i) {
            /* Place arms radially in a circle on the xz plane. */
            float Stheta = 2.0 * M_PI * i / float(NR_ARMS);

            /* Place arms closer together at offsets on the xy plane. */
            float Sphi = 6.5 * M_PI / 8.0;

            /* All arms start off parallel to their torso joint vector. */
            arms[i] = new Arm(torso, Stheta, Sphi, 0, 0, 0,
                              0, M_PI/8, 0);
        }
    }

    ~Thing()
    {
        delete torso;
        for (int i=0; i < NR_ARMS; ++i) {
            delete arms[i];
        }
    }
    
    void render()
    {
        torso->render();
        for (int i=0; i < NR_ARMS; ++i) {
            arms[i]->render();
        }
    }

    inline Point3f getCentroid()
    {
        return torso->centroid;
    }

    inline void updateCentroid(Vector3f& delta) {
        Point3f updatedPoint = torso->centroid + delta;
        setCentroid(updatedPoint);
    }

    inline void setCentroid(Point3f& pos) {
        torso->centroid = pos;
        for (Arm* arm : arms) {
            while (!arm->IKUpdate());
            arm->endEffector = arm->getPincerEnd(); 
        }
    }

    inline Point3f calculateNewCentroid(float time) {
        Point3f c(0,0,0);
        for (Arm *arm : arms) {
            c += arm->endEffector;
        }
        c /= NR_ARMS;
        c[1] = surface(c[0], c[2], time) + (torso->radius * 3.5);
        return c;
    }

    void moveTowards(FloatPair dir, float time)
    {
        Vector3f direction(dir.first, 0, dir.second);
        direction.normalized();

        /* goal is direction vector on the xz plane */
        Vector3f stepSize = direction * moveData->stepSize;
        Vector3f deltaSize = stepSize / moveData->numDeltas;

        /* have all the arms completed a full step? */
        bool completedStep = true;
        bool directionChanged = direction != moveData->direction;

        setCentroid(calculateNewCentroid(time));

        for (int i = int(moveData->moveOddLegs); i < NR_ARMS; i+=2) {

            Arm *arm = arms[i];

            bool reachedGoal = arms[i]->getError() == 0;

            /* if we've changed direction, or if the arm has reached it's 
               previous delta (but hasn't finished a step), update the goal */
            if (directionChanged || 
                (reachedGoal && deltas[i] < moveData->numDeltas)) {

                // reset number of deltas if we've changed direction
                deltas[i] = directionChanged ? 1 : deltas[i] + 1;
                arms[i]->goal = arms[i]->getPincerEnd() + deltaSize;
                clampToSurface(arms[i]->goal, surface, time);

                completedStep = false;
            } else {
                if (deltas[i] != moveData->numDeltas || !reachedGoal) {
                    arms[i]->IKUpdate();
                    completedStep = false;
                }
            }
        }

        moveData->direction = direction;
        if (completedStep) {
            moveData->moveOddLegs = !moveData->moveOddLegs;
            for (int i = 0; i < NR_ARMS; i++) {
                deltas[i] = 0; 
            }
        }
        return;
    }

    void touchSurfaceImmediately(float time) {
        for (Arm* arm : arms) {
            Point3f pos = arm->getPincerEnd();
            arm->goal = Point3f(pos.x(), surface(pos.x(), pos.z(), time), pos.z());
            while (!arm->IKUpdate()) {};
        }
    }

};

struct Scene {
    float vwidth;
    float vheight;

    float time;

    Point3f eye;
    Point3f lookAt;
    Vector3f lookDir;

    Vector3f up;
    Vector3f right;

    Matrix3f rightRotate;
    Matrix3f yRotate;
    bool updateRightRotateMatrix;

    float rotationStep;

    int focusedThing;
    vector<Thing*> things;

    Scene()
        : vwidth(800), vheight(600), time(0), focusedThing(0), updateRightRotateMatrix(true)
    {}

    inline void addThing(Thing* thing)
    {
        things.push_back(thing);
    }

    inline Thing* getFocusedThing()
    {
        return things[focusedThing];
    }

    inline void cycleFocus()
    {
        focusedThing = (focusedThing + 1) % things.size();
    }

    void reorient() {
        const float step = 0.03;
        time += step;
    }

    void setupEye() {
        lookAt = getFocusedThing()->getCentroid();
        eye = lookAt + Point3f(0, 15, 5);
        updateLookVectors();

        // setup rotations
        rotationStep = M_PI / 64.0f;
        Vector3f y(0,1,0);
        yRotate = AngleAxisf(rotationStep, y).toRotationMatrix();
    }

    void updateLookVectors() {
        updateLookDir();
        updateRight();
        updateUp();
    }
    
    inline void updateLookDir() {
        lookDir = (lookAt - eye).normalized();
    }

    inline void updateRight() {
        right = (Vector3f(0,-1,0).cross(lookDir)).normalized();
        updateRightRotateMatrix = true;
    }

    inline void updateUp() { 
        up = (right.cross(lookDir)).normalized();
    }

    void rotateEyeAboutYAxis(bool clockwise) {
        eye = ((clockwise ? yRotate : yRotate.inverse()) * (eye - lookAt)) + lookAt;
        updateLookDir();
        updateRight();
        updateUp();
    }

    void rotateEyeAboutRightAxis(bool clockwise) {
        /*
            for some reason, clockwise becomes counterclockwise

            note:   we can't use a negative in front of right in 
                    true 184-style, since it has to be a const reference
        */
        clockwise = !clockwise;
        if (updateRightRotateMatrix) {
            rightRotate = AngleAxisf(rotationStep, right).toRotationMatrix();
        }

        eye = ((clockwise ? rightRotate : rightRotate.inverse()) * (eye - lookAt)) + lookAt;
        print_vec3("Eye: ", eye);
        updateLookDir();
        updateUp();
    }

    void printLookVectors()
    {
        print_vec3("Eye: ", eye);
        print_vec3("Look at: ", lookAt);
        print_vec3("Right: ", right);
        print_vec3("Up: ", up);
    }

    void setupProjection()
    {
        glMatrixMode(GL_PROJECTION);
        glLoadIdentity();
        gluPerspective(45, vwidth / vheight, 0.001, 1000);

        glMatrixMode(GL_MODELVIEW);
        glLoadIdentity();

        gluLookAt(eye.x(), eye.y(), eye.z(), lookAt.x(), lookAt.y(), lookAt.z(),
                  up.x(), up.y(), up.z());
    }

    void render()
    {
        setNormalMaterial();
        for (size_t i=0; i < things.size(); ++i) {
            things[i]->render();
        }

        float x0 = min(lookAt.x(), eye.x()) - 12;
        float xf = max(lookAt.x(), eye.x()) + 12;
        float z0 = max(lookAt.z(), eye.z()) + 12;
        float zf = min(lookAt.z(), eye.z()) - 12;
        setSurfaceMaterial();
        renderSurface(getFocusedThing()->surface, time, x0, xf, z0, zf);
    }
};
