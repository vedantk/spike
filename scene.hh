/*
 * scene.hh
 */

#pragma once

#include "arm.hh"

#define NR_ARMS     (4 * 4)

typedef float (*Surface)(float x, float z, float t);

struct Torso {
    float radius;
    Point3f centroid;
    
    void render()
    {
        glPushMatrix();
    	glTranslatef(centroid.x(), centroid.y(), centroid.z());
    	glutSolidSphere(radius, 100, 100);
    	glPopMatrix();
    }
};

struct Thing {
    Torso torso;
    Arm arms[NR_ARMS];
    Surface currentSurface;

    Thing(float radius) {
        torso.radius = radius;
        torso.centroid = Point3f(0, 0, 0);

        for (int i=0; i < NR_ARMS; ++i) {
            float Stheta = (i % 4) * (M_PI / 2);
            float Sphi = ((i / 4) + 1) * (M_PI / 5);
            arms[i] = Arm(&torso, 
        }
    }
    
    void render()
    {
        torso.render();
        for (int i=0; i < NR_ARMS; ++i) {
            arms[i].render();
        }
    }

    void moveTowards(Point3f goal)
    {
        return;
    }
};

struct Scene {
    vector<Thing*> things;
    vector<Surface> surfaces;
};


