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

            glColor3f(ur - floor(ur),
                      ul - floor(ul),
                      ll - floor(ll));
            glBegin(GL_POLYGON);
                glVertex3f(x, ul, z);
                glVertex3f(x + step, ur, z);
                glVertex3f(x + step, lr, z + step);
                glVertex3f(x, ll, z + step);
            glEnd();
        }
    }
}

struct Thing {
    Torso* torso;
    Arm* arms[NR_ARMS];
    Surface surface;

    Thing(Surface surf, float radius, Point3f centroid)
        : surface(surf)
    {
        torso = new Torso(radius, centroid);

        for (int i=0; i < NR_ARMS; ++i) {
            /* Place arms radially in a circle on the xz plane. */
            float Stheta = 2.0 * M_PI * i / float(NR_ARMS);

            /* Place arms closer together at offsets on the xy plane. */
            float Sphi = M_PI / 4.0;

            /* All arms start off parallel to their torso joint vector. */
            arms[i] = new Arm(torso, Stheta, Sphi, 0, 0, 0, 0, 0, 0);
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

    void moveTowards(FloatPair direction)
    {
        /* goal is direction vector on the xz plane */

        return;
    }
};

struct Scene {
    float vwidth;
    float vheight;

    float time;

    Point3f eye;
    Point3f lookAt;
    Vector3f cameraOffset;

    int focusedThing;
    vector<Thing*> things;

    Scene()
        : vwidth(800), vheight(600), time(0),
          cameraOffset(Point3f(0, 0, 0)), focusedThing(0)
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

        float x0 = min(lookAt.x(), eye.x()) - 10;
        float xf = max(lookAt.x(), eye.x()) + 10;
        float z0 = max(lookAt.z(), eye.z()) + 10;
        float zf = min(lookAt.z(), eye.z()) - 10;
        renderSurface(getFocusedThing()->surface, time, x0, xf, z0, zf);
    }
};
