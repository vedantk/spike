/*
 * scene.hh
 */

#pragma once

#include "arm.hh"

/* The construction of Thing objects is intimately tied to the number of arms
 * attached to the torso. This is simply a convenience macro. */
#define NR_ARMS     16

typedef float (*Surface)(float x, float z, float t);

void renderSurface(Surface fn, float t, float x0, float xf, float z0, float zf)
{
    const float step = 0.01;
    for (float x=x0; x < xf; x += step) {
        for (float z=zf; z < z0; z += step) {
            float ll = fn(x, z, t);
            float lr = fn(x + step, z, t);
            float ul = fn(x, z + step, t);
            float ur = fn(x + step, z + step, t);

            glColor3f(1.0, 0, 0);
            glBegin(GL_POLYGON);
                glVertex3f(x, ll, z);
                glVertex3f(x + step, lr, z);
                glVertex3f(x, ul, z + step);
                glVertex3f(x + step, ur, z + step);
            glEnd();
        }
    }
}

struct Thing {
    Torso* torso;
    Arm* arms[NR_ARMS];

    Thing(float radius) {
        torso = new Torso(radius, Point3f(0, 0, 0));

        for (int i=0; i < NR_ARMS; ++i) {
            /* Place arms radially in a circle on the xz plane. */
            float Stheta = (i % 4) * (M_PI / 2);

            /* Place arms closer together at offsets on the xy plane. */
            float Sphi = ((2*i / 4) + 1) * (M_PI / 8);

            /* All arms start off parallel to their torso joint vector. */
            arms[i] = new Arm(torso, Stheta, Sphi,
                              0, 0, 0, 0, 0, 0, 1.0, 1.0);
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

    void moveTowards(Point3f goal)
    {
        return;
    }
};

struct Scene {
    float time;

    Point3f eye;
    Point3f lookAt;

    int focusedThing;
    vector<Thing*> things;

    vector<Surface> surfaces;

    float x0, xf, y0, yf, z0, zf;

    Scene()
        : time(0), focusedThing(0),
          x0(-10), xf(10), y0(-10), yf(10), z0(10), zf(-10)
    {}

    inline void addThing(Thing* thing)
    {
        things.push_back(thing);
    }

    inline void addSurface(Surface fn)
    {
        surfaces.push_back(fn);
    }

    inline Thing* getFocusedThing()
    {
        return things[focusedThing];
    }

    inline void cycleFocus()
    {
        ++focusedThing;
        focusedThing %= things.size();
    }

    void orient()
    {
        const float step = 0.1;
        time += step;

        /*
        lookAt = getFocusedThing()->getCentroid();
        eye = Point3f((x0 + xf) / 2, yf, z0);
        */
        lookAt = Point3f(0, 0, -8);
        eye = Point3f(0, 3, 3);
    }

    void render()
    {
        for (size_t i=0; i < things.size(); ++i) {
            things[i]->render();
        }
        for (size_t i=0; i < surfaces.size(); ++i) {
            renderSurface(surfaces[i], time, x0, xf, z0, zf);
        }
    }

    void setupOrtho()
    {
        glMatrixMode(GL_PROJECTION);
        glLoadIdentity();
        glOrtho(x0, xf, y0, yf, z0, zf);
    }
};
