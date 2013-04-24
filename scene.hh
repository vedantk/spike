/*
 * scene.hh
 */

#pragma once

#include "arm.hh"

/* The construction of Thing objects is intimately tied to the number of arms
 * attached to the torso. This is simply a convenience macro. */
#define NR_ARMS     16

typedef float (*Surface)(float x, float z, float t);

struct Thing {
    Torso* torso;
    Arm* arms[NR_ARMS];
    Surface currentSurface;

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
    Point3f eye;
    Point3f lookAt;
    int focusedThing;
    vector<Thing*> things;
    vector<Surface> surfaces;

    Scene()
        : focusedThing(0)
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
        ++focusedThing;
        focusedThing %= things.size();
    }

    void orient()
    {
        lookAt = getFocusedThing()->getCentroid();

        /* Place yourself above and closer to origin from the Thing. */
        eye = lookAt.cwiseProduct(Point3f(0.75, 1.5, 0.75));
    }
};
