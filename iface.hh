/*
 * iface.hh
 */

#include "util.hh"

Vector3f getDirection(float theta, float phi) {
    return Vector3f(sin(phi) * cos(theta), cos(phi), sin(phi) * sin(theta));
}

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

struct Arm {
    typedef Matrix<float, 8, 1> Param;
    typedef Matrix<float, 3, 8> Jacobian;

    Arm(Torso* _torso, Param& _values, float _Stheta, float _Sphi)
        : torso(_torso), values(_values), Stheta(_Stheta), Sphi(_Sphi)
    {
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
        /* p(v) = T_0 +
         * Sr * <sin(Sphi) * cos(Stheta), cos(Sphi), sin(Sphi) * sin(Stheta)> +
         * l1 * <sin(Sphi+phi1)*cos(Stheta+theta1), cos(Sphi+phi1), sin(Sphi+phi1)*sin(Stheta+theta1)> +
         * l2 * <sin(Sphi+phi1+phi2)*cos(Stheta+theta1+theta2), cos(Sphi+phi1+phi2), sin(Sphi+phi1+phi2)*sin(Stheta+theta1+theta2)> +
         * L * <sin(Sphi+phi1+phi2+phi3)*cos(Stheta+theta1+theta2+theta3), cos(Sphi+phi1+phi2+phi3), sin(Sphi+phi1+phi2+phi3)*sin(Stheta+theta1+theta2+theta3)> 
         */

        Vector3f arrow = getDirection(Stheta + getTheta(0) +
                                      getTheta(1) + getTheta(2),
                                      Sphi + getPhi(0) +
                                      getPhi(1) + getPhi(2));
        return getForearmEnd() + (getPincerLength() * arrow);
    }

    void computeJacobian()
    {
        /* d(px)/d(theta1) */
        J(0, 0) = ;

        /* d(px)/d(theta2) */
        J(0, 1) = ;
    }
    
    inline float getError(Point3f goal)
    {
        return (endEffector - goal).normalized();
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

        Jacobian Jinv = (svd.matrixV() *
                         inv.asDiagonal() *
                         svd.matrixU().transpose()).transpose();

        /* σ = (J+)x * Δp */
        Param vdelta = Jinv * (goal - endEffector);
        
        updatePosition(vdelta);
        float currentError;
        while ((currentError = getError(goal)) > error && (nrSplits++ < maxSplits)) {
            vdelta /= 2;
            updatePosition(-vdelta);
        }
        return currentError < posTolerance;
    }
    
    void render()
    {
        float radius;
    	const float segmentVol = 8.0;
    	
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
    	radius = sqrt((segmentVol / 4.0) / (M_PI * getPincerLength()));
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
}
