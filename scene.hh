/*
 * scene.hh
 */

#pragma once

#include "arm.hh"

#define NR_ARMS     6

typedef float (*Surface)(float x, float z, float t);

void renderSurface(Surface fn, float t, float x0, float xf, float z0, float zf)
{
    const float step = 0.1;
    for (float x=x0; x < xf; x += step) {
        for (float z=zf; z < z0; z += step) {
            float ll = fn(x, z + step, t);
            float lr = fn(x + step, z + step, t);
            float ul = fn(x, z, t);
            float ur = fn(x + step, z, t);

            glcol3f(Point3f(fabs(x*z), 0, ur).normalized());

            glBegin(GL_POLYGON);
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

    void updateCentroid(Vector3f& delta) {
        torso->centroid += delta;
        for (Arm* arm : arms) {
            arm->endEffector = arm->getPincerEnd(); 
        }
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

        if (directionChanged) {
            moveData->newTorsoLocation = torso->centroid;
        }

        float torsoError = (torso->centroid - moveData->newTorsoLocation).norm();
        if (torsoError > 1.0e-3) {
            for (Arm *arm : arms) {
                while(!arm->IKUpdate());
            }
            Point3f torsoDelta = deltaSize / 50;
            updateCentroid(torsoDelta);
            return;
        }

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
                arms[i]->goal[1] = surface(arms[i]->goal.x(),
                                           arms[i]->goal.z(),
                                           time);

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
            cout << "===============" << endl;
            cout << "Completed step." << endl;
            cout << "===============" << endl;
            moveData->newTorsoLocation = torso->centroid + stepSize/2;
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
    Vector3f cameraOffset;
    bool lockCamera;

    int focusedThing;
    vector<Thing*> things;

    Scene()
        : vwidth(800), vheight(600), time(0),
          cameraOffset(Point3f(0, 0, 0)), lockCamera(false), focusedThing(0)
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

    inline void shiftCamera(float dx, float dy, float dz)
    {
        if (!lockCamera) {
            cameraOffset[0] += dx;
            cameraOffset[1] += dy;
            cameraOffset[2] += dz;
        }
    }

    void reorient()
    {
        const float step = 0.03;
        time += step;

        lookAt = getFocusedThing()->getCentroid();
        eye = lookAt + Point3f(0, 15, 5) + cameraOffset;
    }

    void setupProjection()
    {
        glMatrixMode(GL_PROJECTION);
        glLoadIdentity();
        gluPerspective(45, vwidth / vheight, 0.001, 1000);

        glMatrixMode(GL_MODELVIEW);
        glLoadIdentity();

        gluLookAt(eye.x(), eye.y(), eye.z(), lookAt.x(), lookAt.y(), lookAt.z(),
                  0, 1, 0);
    }

    void render()
    {
        for (size_t i=0; i < things.size(); ++i) {
            things[i]->render();
        }

        float x0 = min(lookAt.x(), eye.x()) - 12;
        float xf = max(lookAt.x(), eye.x()) + 12;
        float z0 = max(lookAt.z(), eye.z()) + 12;
        float zf = min(lookAt.z(), eye.z()) - 12;
        renderSurface(getFocusedThing()->surface, time, x0, xf, z0, zf);
    }
};
