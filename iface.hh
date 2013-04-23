/*
 * iface.hh
 */

#include "util.hh"

Vector3f getDirection(float theta, float phi) {
    return Vector3f(sin(phi) * cos(theta), cos(phi), sin(phi) * sin(theta));
}

struct Torso {
    float radius;
    float theta;
    float phi;
    Point3f centroid;
};

struct Arm {
    typedef Matrix<float, 8, 1> Param;
    typedef Matrix<float, 3, 8> Jacobian;

    Arm(Torso* _torso, Param& _values)
        : torso(_torso), values(_values)
    {}

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
        Vector3f arrow = getDirection(torso->theta, torso->phi);
        return torso->centroid + (torso->radius * arrow);
    }

    inline Point3f getShoulderEnd()
    {
        Vector3f arrow = getDirection(torso->theta + getTheta(0),
                                      torso->phi + getPhi(0));
        return getTorsoPoint() + (getLength(0) * arrow);
    }

    inline Point3f getForearmEnd()
    {
        Vector3f arrow = getDirection(torso->theta + getTheta(0) + getTheta(1),
                                      torso->phi + getPhi(0) + getPhi(1));
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

        Vector3f arrow = getDirection(torso->theta + getTheta(0) +
                                      getTheta(1) + getTheta(2),
                                      torso->phi + getPhi(0) +
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
    
    float getError(Point3f goal)
    {
        return (getPincerEnd() - goal).normalized();
    }

    bool IKUpdate(Point3f goal)
    {
        /* σ = (J+)x * Δp */

        const float tolerance = 1.0e-5;
        const float posTolerance = 1.0e-2;
        const int maxSplits = 8;
        
        float error = getError(goal);
        
        if (error < posTolerance) {
            return true;
        }
        
        Point3f loc = getPincerEnd();
        computeJacobian();
        
        JacobiSVD<Jacobian> svd = J.jacobiSvd(ComputeThinU | ComputeThinV);
        JacobiSVD<Jacobian>::SingularValuesType inv = svd.singularValues();

        for (long i=0; i < J.cols(); ++i) {
            inv(i) = (inv(i) > tolerance) ? (1.0 / inv(i)) : 0.0;
        }

        Jacobian Jinv = (svd.matrixV() *
                         inv.asDiagonal() *
                         svd.matrixU().transpose()).transpose();

        Param vdelta = Jinv * (goal - loc);
        
        values += vdelta;
        float currentError;
        while ((currentError = getError(goal)) > error && (nrSplits++ < maxSplits)) {
            vdelta /= 2;
            values -= vdelta;
        }
        return currentError < posTolerance;
    }

private:
    Torso* torso;
    
    /* <Theta1, Theta2, Theta3, Phi1, Phi2, Phi3, L1, L2> */
    Vector values;
    
    Jacobian J;
};
