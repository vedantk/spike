/*
 * arm.hh
 */

#pragma once

#include "util.hh"

#define jointRadius 0.10

Vector3f getDirection(float theta, float phi) {
    return Vector3f(sin(phi) * cos(theta), cos(phi), sin(phi) * sin(theta));
}

struct Torso {
    float radius;
    Point3f centroid;

    Torso(float _radius, Point3f _centroid)
        : radius(_radius), centroid(_centroid)
    {}
    
    void render()
    {
        glPushMatrix();
    	glTranslatef(centroid.x(), centroid.y(), centroid.z());
    	glutSolidSphere(radius, 10, 10);
    	glPopMatrix();
    }
};

struct Arm {
    typedef Matrix<float, 6, 1> Param;
    typedef Matrix<float, 3, 6> Jacobian;

    Arm(Torso* _torso, float _Stheta, float _Sphi,
        float theta0, float theta1, float theta2,
        float phi0, float phi1, float phi2)
        : torso(_torso), Stheta(_Stheta), Sphi(_Sphi)
    {
        values(0) = theta0;
        values(1) = theta1;
        values(2) = theta2;
        values(3) = phi0;
        values(4) = phi1;
        values(5) = phi2;

        endEffector = getPincerEnd();
    }

    inline float getTheta(int n)
    {
        return values(n);
    }

    inline float getPhi(int n)
    {
        return values(n + 3);
    }

    inline float getSegmentLength()
    {
        return 1.2;
    }

    inline float getPincerLength()
    {
        return 0.12;
    }

    inline Vector3f getArrow(int n)
    {
        float theta = Stheta;
        float phi = Sphi;
        for (int i=0; i < n; ++i) {
            theta += getTheta(i);
            phi += getPhi(i);
        }
        return getDirection(theta, phi).normalized();
    }

    inline Point3f getTorsoEnd()
    {
        return torso->centroid + (torso->radius * getArrow(0));
    }

    inline Point3f getShoulderEnd()
    {
        float segLen = getSegmentLength() + (2 * jointRadius);
        return getTorsoEnd() + (segLen * getArrow(1));
    }

    inline Point3f getForearmEnd()
    {
        float segLen = getSegmentLength() + (2 * jointRadius);
        return getShoulderEnd() + (segLen * getArrow(2));
    }

    inline Point3f getPincerEnd()
    {
        float segLen = getPincerLength() + (2 * jointRadius);
        return getForearmEnd() + (segLen * getArrow(3));
    }

#if 0
    void computeJacobian()
    {
        // indexed (c, r)

        float phi1 = getPhi(0);
        float phi2 = getPhi(1);
        float phi3 = getPhi(2);

        float theta1 = getTheta(0);
        float theta2 = getTheta(1);
        float theta3 = getTheta(2);

        float L = torso->radius;
        float l1 = getLength(0);
        float l2 = getLength(1);

        // lol sage
        J(0,0) = sin(Sphi + phi1)*cos(Stheta + theta1);
        J(1,0) = sin(Sphi + phi1 + phi2)*cos(Stheta + theta1 + theta2);
        J(2,0) = L*cos(Stheta + theta1 + theta2 + theta3)*cos(Sphi + phi1 + phi2 + phi3)
                    + l1*cos(Stheta + theta1)*cos(Sphi + phi1)
                    + l2*cos(Stheta + theta1 + theta2)*cos(Sphi + phi1 + phi2);
        J(3,0) = L*cos(Stheta + theta1 + theta2 + theta3)*cos(Sphi + phi1 + phi2 + phi3)
                    + l2*cos(Stheta + theta1 + theta2)*cos(Sphi + phi1 + phi2);
        J(4,0) = L*cos(Stheta + theta1 + theta2 + theta3)*cos(Sphi + phi1 + phi2 + phi3);
        J(5,0) = -L*sin(Stheta + theta1 + theta2 + theta3)*sin(Sphi + phi1 + phi2 + phi3)
                   - l1*sin(Stheta + theta1)*sin(Sphi + phi1)
                   - l2*sin(Stheta + theta1 + theta2)*sin(Sphi + phi1 + phi2);
        J(6,0) = -L*sin(Stheta + theta1 + theta2 + theta3)*sin(Sphi + phi1 + phi2 + phi3) 
                   - l2*sin(Stheta + theta1 + theta2)*sin(Sphi + phi1 + phi2);
        J(7,0) = -L*sin(Stheta + theta1 + theta2 + theta3)*sin(Sphi + phi1 + phi2 + phi3);

        J(0,1) = cos(Sphi + phi1);
        J(1,1) = cos(Sphi + phi1 + phi2);
        J(2,1) = -L*sin(Sphi + phi1 + phi2 + phi3) 
                    - l1*sin(Sphi + phi1) 
                    - l2*sin(Sphi + phi1 + phi2);
        J(3,1) = -L*sin(Sphi + phi1 + phi2 + phi3) - l2*sin(Sphi + phi1 + phi2);
        J(4,1) = -L*sin(Sphi + phi1 + phi2 + phi3);
        J(5,1) = 0;
        J(6,1) = 0;
        J(7,1) = 0;

        J(0,2) = sin(Stheta + theta1)*sin(Sphi + phi1);
        J(1,2) = sin(Stheta + theta1 + theta2)*sin(Sphi + phi1 + phi2);
        J(2,2) = L*sin(Stheta + theta1 + theta2 + theta3)*cos(Sphi + phi1 + phi2 + phi3)
                    + l1*sin(Stheta + theta1)*cos(Sphi + phi1) 
                    + l2*sin(Stheta + theta1 + theta2)*cos(Sphi + phi1 + phi2);
        J(3,2) = L*sin(Stheta + theta1 + theta2 + theta3)*cos(Sphi + phi1 + phi2 + phi3) 
                    + l2*sin(Stheta + theta1 + theta2)*cos(Sphi + phi1 + phi2);
        J(4,2) = L*sin(Stheta + theta1 + theta2 + theta3)*cos(Sphi + phi1 + phi2 + phi3);
        J(5,2) = L*sin(Sphi + phi1 + phi2 + phi3)*cos(Stheta + theta1 + theta2 + theta3)
                    + l1*sin(Sphi + phi1)*cos(Stheta + theta1) 
                    + l2*sin(Sphi + phi1 + phi2)*cos(Stheta + theta1 + theta2);
        J(6,2) = L*sin(Sphi + phi1 + phi2 + phi3)*cos(Stheta + theta1 + theta2 + theta3) 
                    + l2*sin(Sphi + phi1 + phi2)*cos(Stheta + theta1 + theta2);
        J(7,2) = L*sin(Sphi + phi1 + phi2 + phi3)*cos(Stheta + theta1 + theta2 + theta3);
        /* J */
        return;
    }
    
    inline float getError(Point3f goal)
    {
        return (endEffector - goal).norm();
    }
    
    inline void updatePosition(Param delta)
    {
        values += delta;
        endEffector = getPincerEnd();
    }

    bool IKUpdate(Point3f goal)
    {
        const float tolerance = 1.0e-5;
        const float posTolerance = 1.0e-2;
        const int maxSplits = 8;
        
        float error = getError(goal);
        
        if (error < posTolerance) {
            return true;
        }
        
        computeJacobian();
        
        JacobiSVD<Jacobian> svd = J.jacobiSvd(ComputeThinU | ComputeThinV);
        JacobiSVD<Jacobian>::SingularValuesType inv = svd.singularValues();

        for (long i=0; i < J.cols(); ++i) {
            inv(i) = (inv(i) > tolerance) ? (1.0 / inv(i)) : 0.0;
        }

        Jacobian Jinv = svd.matrixV() *
                        inv.asDiagonal() *
                        svd.matrixU().transpose();

        /* σ = (J+)x * Δp */
        Param vdelta = Jinv * (goal - endEffector);
        
        updatePosition(vdelta);
        float currentError;
        int nrSplits = 0;
        while ((currentError = getError(goal)) > error
               && (nrSplits++ < maxSplits))
        {
            vdelta /= 2;
            updatePosition(-vdelta);
        }
        return currentError < posTolerance;
    }
#endif
 
    Point3f drawJoint(Point3f start, Vector3f arrow)
    {
    	Point3f center = start + jointRadius * arrow;
    	glColor3f(1.0, 0.0, 0.0);
        glPushMatrix();
    	glTranslatef(center.x(), center.y(), center.z());
    	glutSolidSphere(jointRadius, 10, 10);
    	glPopMatrix();
    	return center + jointRadius * arrow;
    }   

    void drawTetrahedron(Point3f center, Vector3f arrow, float length)
    {
        /* Actually this is technically a square pyramid. Whoops. */

        const float sideLength = 0.25;
        const float sideHalved = sideLength / 2.0;

        Point3f right = arrow.cross(Vector3f(0, 1, 0)).normalized();
        Point3f up = right.cross(arrow).normalized();

        Point3f ll = center - sideHalved * (right + up);
        Point3f lr = center + (sideHalved * right) - (sideHalved * up);
        Point3f ul = center - (sideHalved * right) + (sideHalved * up);
        Point3f ur = center + sideHalved * (right + up);
        Point3f tip = center + length * arrow;

        /* Draw the base. */
        glBegin(GL_POLYGON);
            glvtx3f(ll);
            glvtx3f(lr);
            glvtx3f(ur);
            glvtx3f(ul);
        glEnd();

        /* Draw each face. */
        glBegin(GL_TRIANGLES);
            glvtx3f(ll);
            glvtx3f(lr);
            glvtx3f(tip);

            glvtx3f(ll);
            glvtx3f(ul);
            glvtx3f(tip);

            glvtx3f(ur);
            glvtx3f(ul);
            glvtx3f(tip);

            glvtx3f(ur);
            glvtx3f(lr);
            glvtx3f(tip);
        glEnd();
    }

    void render()
    {
    	Point3f center; 

    	/* Draw the torso joint. */
        center = drawJoint(getTorsoEnd(), getArrow(0));

        /* Draw the shoulder tetrahedron. */
        glColor3f(0.0, 0.5, 0.5);
        drawTetrahedron(center, getArrow(0), getSegmentLength());

        /* Draw the elbow joint. */
        center = drawJoint(getShoulderEnd(), getArrow(1));

        /* Draw the forearm tetrahedron. */
        glColor3f(0.0, 0.1, 0.9);
        drawTetrahedron(center, getArrow(1), getSegmentLength());

        /* Draw the wrist joint. */
        center = drawJoint(getForearmEnd(), getArrow(2));

        /* Draw the pincer. */
        glColor3f(0.0, 0.9, 0.1);
        drawTetrahedron(center, getArrow(2), getSegmentLength() / 5.0);
    }

private:
    Torso* torso;
    
    float Stheta;
    float Sphi;
    
    /* <Theta1, Theta2, Theta3, Phi1, Phi2, Phi3> */
    Param values;
    
    Jacobian J;
    
    Point3f endEffector;
};

/*
struct Surface {
    typedef float (* SurfaceFn) ( float x, float z, float t);
    SurfaceFn mSurfaceFn;
    Surface(SurfaceFn surfaceFn);

    void render();
};
*/
