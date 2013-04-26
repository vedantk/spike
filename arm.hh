/*
 * arm.hh
 */

#pragma once

#include "util.hh"

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
    	glutSolidSphere(radius, 100, 100);
    	glPopMatrix();
    }
};

struct Arm {
    typedef Matrix<float, 8, 1> Param;
    typedef Matrix<float, 3, 8> Jacobian;

    Arm(Torso* _torso, float _Stheta, float _Sphi,
        float theta0, float theta1, float theta2,
        float phi0, float phi1, float phi2,
        float l1, float l2)
        : torso(_torso), Stheta(_Stheta), Sphi(_Sphi)
    {
        values(0) = theta0;
        values(1) = theta1;
        values(2) = theta2;
        values(3) = phi0;
        values(4) = phi1;
        values(5) = phi2;
        values(6) = l1;
        values(7) = l2;

        endEffector = getPincerEnd();
    }

    inline float getTheta(int n) const {
        return values(n);
    }

    inline float getPhi(int n) const {
        return values(n + 3);
    }

    inline float getLength(int n) const {
        return values(n + 6);
    }

    inline float getPincerLength() const {
        return 1.0;
    }

    inline Point3f getTorsoPoint()
    {
        Vector3f arrow = getDirection(Stheta, Sphi);
        return torso->centroid + (torso->radius * arrow);
    }

    inline Point3f getShoulderEnd()
    {
        Vector3f arrow = getDirection(Stheta + getTheta(0), Sphi + getPhi(0));
        return getTorsoPoint() + (getLength(0) * arrow);
    }

    inline Point3f getForearmEnd()
    {
        Vector3f arrow = getDirection(Stheta + getTheta(0) + getTheta(1),
                                      Sphi + getPhi(0) + getPhi(1));
        return getShoulderEnd() + (getLength(1) * arrow);
    }

    inline Point3f getPincerEnd()
    {
        Vector3f arrow = getDirection(Stheta + getTheta(0) +
                                      getTheta(1) + getTheta(2),
                                      Sphi + getPhi(0) +
                                      getPhi(1) + getPhi(2));
        return getForearmEnd() + (getPincerLength() * arrow);
    }

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
#if 0
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
        while ((currentError = getError(goal)) > error && (nrSplits++ < maxSplits)) {
            vdelta /= 2;
            updatePosition(-vdelta);
        }
        return currentError < posTolerance;
#endif
        return 0;
    }
    
    void render()
    {
        float radius;
    	const float segmentVol = 4.0;

    	glColor3f(0.0, 1.0, 0.0);
    	
    	/* Draw a segment that's connected to the torso. */
    	Point3f torsoPoint = getTorsoPoint();
    	radius = sqrt(segmentVol / (M_PI * getLength(0)));
    	glPushMatrix();
    	glScalef(1.0, getLength(0) / radius, 1.0);
    	glRotatef(Stheta + getTheta(0), 0, 1, 0);
    	glRotatef(Sphi + getPhi(0), 1, 0, 0);
    	glTranslatef(torsoPoint.x(), torsoPoint.y(), torsoPoint.z());
    	glutSolidSphere(radius, 100, 100);
    	glPopMatrix();
    	
    	/* Now attach the forearm to the shoulder and draw it. */
    	Point3f elbowPoint = getShoulderEnd();
    	radius = sqrt(segmentVol / (M_PI * getLength(1)));
    	glPushMatrix();
    	glScalef(1.0, getLength(1) / radius, 1.0);
    	glRotatef(Stheta + getTheta(0) + getTheta(1), 0, 1, 0);
    	glRotatef(Sphi + getPhi(0) + getPhi(1), 1, 0, 0);
    	glTranslatef(elbowPoint.x(), elbowPoint.y(), elbowPoint.z());
    	glutSolidSphere(radius, 100, 100);
    	glPopMatrix();
    	
    	/* Now draw the pincer. */
    	Point3f wristPoint = getForearmEnd();
    	radius = sqrt((segmentVol / 6.0) / (M_PI * getPincerLength()));
    	glPushMatrix();
    	glScalef(1.0, getPincerLength() / radius, 1.0);
    	glRotatef(Stheta + getTheta(0) + getTheta(1) + getTheta(2), 0, 1, 0);
    	glRotatef(Sphi + getPhi(0) + getPhi(1) + getPhi(2), 1, 0, 0);
    	glTranslatef(wristPoint.x(), wristPoint.y(), wristPoint.z());
    	glutSolidSphere(radius, 10, 10);
    	glPopMatrix();
    }

private:
    Torso* torso;
    
    float Stheta;
    float Sphi;
    
    /* XXX: Contraint solving. */
    /* <Theta1, Theta2, Theta3, Phi1, Phi2, Phi3, L1, L2> */
    Param values;
    
    Jacobian J;
    
    Point3f endEffector;
};

struct Surface {
    typedef float (* SurfaceFn) ( float x, float z, float t);
    SurfaceFn mSurfaceFn;
    Surface(SurfaceFn surfaceFn);

    void render();
};
