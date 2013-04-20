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

class Arm {
public:
    typedef Matrix<float, 8, 1> Vector;
    typedef Matrix<float, 3, 8> Jacobian;

    Arm(Vector& params)
        : values(params)
    {}

    float getTheta(int n) const {
        return values(n);
    }

    float getPhi(int n) const {
        return values(n + 3);
    }

    float getLength(int n) const {
        return values(n + 6);
    }

    float getPincerLength() const {
        return 1.0;
    }

    Point3f getTorsoPoint(Torso* torso)
    {
        Vector3f arrow = getDirection(torso->theta, torso->phi);
        return torso->centroid + (torso->radius * arrow);
    }

    Point3f getShoulderEnd(Torso* torso)
    {
        Vector3f arrow = getDirection(torso->theta + getTheta(0),
                                      torso->phi + getPhi(0));
        return getTorsoPoint(torso) + (getLength(0) * arrow);
    }

    Point3f getForearmEnd(Torso* torso)
    {
        Vector3f arrow = getDirection(torso->theta + getTheta(0) + getTheta(1),
                                      torso->phi + getPhi(0) + getPhi(1));
        return getShoulderEnd(torso) + (getLength(1) * arrow);
    }

    Point3f getPincerEnd(Torso* torso)
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
        return getForearmEnd(torso) + (getPincerLength() * arrow);
    }

    void computeJacobian(Torso* torso, Jacobian& J)
    {
        /* d(px)/d(theta1) */
        J(0, 0) = ;

        /* d(px)/d(theta2) */
        J(0, 1) = ;
    }

    void IKUpdate(Point3f goal)
    {
        /* p(goal) = p(loc) + J(vcur) * delta(vec)
         * delta(p) = J(vcur) * delta(vec)
         * delta(vec) = J^+(vcur) * delta(p)
         */

        const float tolerance = 1.0e-5;
        JacobiSVD<Jacobian> svd = J.jacobiSvd(ComputeThinU | ComputeThinV);
        JacobiSVD<Jacobian>::SingularValuesType inv = svd.singularValues();

        for (long i=0; i < J.cols(); ++i) {
            inv(i) = (inv(i) > tolerance) ? (1.0 / inv(i)) : 0.0;
        }

        Jacobian Jinv = (svd.matrixV() *
                         inv.asDiagonal() *
                         svd.matrixU().transpose()).transpose();

        Vector vdelta = Jinv * (goal - loc);

        values += vdelta;
        if (c)

    }

private:
    /* <Theta1, Theta2, Theta3, Phi1, Phi2, Phi3, L1, L2> */
    Vector values;
};
