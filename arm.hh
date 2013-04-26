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
    typedef Matrix<float, 8, 1> Param;
    typedef Matrix<float, 3, 8> Jacobian;

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
        values(6) = 1.2;
        values(7) = 1.5;

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

    inline float getLength(int n)
    {
        return values(n + 6);
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
        float segLen = getLength(0) + (2 * jointRadius);
        return getTorsoEnd() + (segLen * getArrow(1));
    }

    inline Point3f getForearmEnd()
    {
        float segLen = getLength(1) + (2 * jointRadius);
        return getShoulderEnd() + (segLen * getArrow(2));
    }

    inline Point3f getPincerEnd()
    {
        float segLen = getPincerLength() + (2 * jointRadius);
        return getForearmEnd() + (segLen * getArrow(3));
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
        J(0,1) = sin(Sphi + phi1 + phi2)*cos(Stheta + theta1 + theta2);
        J(0,2) = L*cos(Stheta + theta1 + theta2 + theta3)*cos(Sphi + phi1 + phi2 + phi3)
                    + l1*cos(Stheta + theta1)*cos(Sphi + phi1)
                    + l2*cos(Stheta + theta1 + theta2)*cos(Sphi + phi1 + phi2);
        J(0,3) = L*cos(Stheta + theta1 + theta2 + theta3)*cos(Sphi + phi1 + phi2 + phi3)
                    + l2*cos(Stheta + theta1 + theta2)*cos(Sphi + phi1 + phi2);
        J(0,4) = L*cos(Stheta + theta1 + theta2 + theta3)*cos(Sphi + phi1 + phi2 + phi3);
        J(0,5) = -L*sin(Stheta + theta1 + theta2 + theta3)*sin(Sphi + phi1 + phi2 + phi3)
                   - l1*sin(Stheta + theta1)*sin(Sphi + phi1)
                   - l2*sin(Stheta + theta1 + theta2)*sin(Sphi + phi1 + phi2);
        J(0,6) = -L*sin(Stheta + theta1 + theta2 + theta3)*sin(Sphi + phi1 + phi2 + phi3) 
                   - l2*sin(Stheta + theta1 + theta2)*sin(Sphi + phi1 + phi2);
        J(0,7) = -L*sin(Stheta + theta1 + theta2 + theta3)*sin(Sphi + phi1 + phi2 + phi3);

        J(1,0) = cos(Sphi + phi1);
        J(1,1) = cos(Sphi + phi1 + phi2);
        J(1,2) = -L*sin(Sphi + phi1 + phi2 + phi3) 
                    - l1*sin(Sphi + phi1) 
                    - l2*sin(Sphi + phi1 + phi2);
        J(1,3) = -L*sin(Sphi + phi1 + phi2 + phi3) - l2*sin(Sphi + phi1 + phi2);
        J(1,4) = -L*sin(Sphi + phi1 + phi2 + phi3);
        J(1,5) = 0;
        J(1,6) = 0;
        J(1,7) = 0;

        J(2,0) = sin(Stheta + theta1)*sin(Sphi + phi1);
        J(2,1) = sin(Stheta + theta1 + theta2)*sin(Sphi + phi1 + phi2);
        J(2,2) = L*sin(Stheta + theta1 + theta2 + theta3)*cos(Sphi + phi1 + phi2 + phi3)
                    + l1*sin(Stheta + theta1)*cos(Sphi + phi1) 
                    + l2*sin(Stheta + theta1 + theta2)*cos(Sphi + phi1 + phi2);
        J(2,3) = L*sin(Stheta + theta1 + theta2 + theta3)*cos(Sphi + phi1 + phi2 + phi3) 
                    + l2*sin(Stheta + theta1 + theta2)*cos(Sphi + phi1 + phi2);
        J(2,4) = L*sin(Stheta + theta1 + theta2 + theta3)*cos(Sphi + phi1 + phi2 + phi3);
        J(2,5) = L*sin(Sphi + phi1 + phi2 + phi3)*cos(Stheta + theta1 + theta2 + theta3)
                    + l1*sin(Sphi + phi1)*cos(Stheta + theta1) 
                    + l2*sin(Sphi + phi1 + phi2)*cos(Stheta + theta1 + theta2);
        J(2,6) = L*sin(Sphi + phi1 + phi2 + phi3)*cos(Stheta + theta1 + theta2 + theta3) 
                    + l2*sin(Sphi + phi1 + phi2)*cos(Stheta + theta1 + theta2);
        J(2,7) = L*sin(Sphi + phi1 + phi2 + phi3)*cos(Stheta + theta1 + theta2 + theta3);
        /* J */
        return;
    }
    
    inline float getError(const Point3f& goal)
    {
        return (endEffector - goal).norm();
    }
    
    inline void updatePosition(Param delta)
    {
        values += delta;
        endEffector = getPincerEnd();
    }

    bool IKUpdate(const Point3f& goal)
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

        for (long i=0; i < J.rows(); ++i) {
            inv(i) = (inv(i) > tolerance) ? (1.0 / inv(i)) : 0.0;
        }

        /*
        cout << "matrix u cols: " << svd.matrixU().cols() << "; rows: " << svd.matrixU().rows() << endl;
        cout << "matrix v cols: " << svd.matrixV().cols() << "; rows: " << svd.matrixV().rows() << endl;
        cout << "matrix inv as diagonal cols: " << inv.asDiagonal().cols() << "; rows: " << svd.matrixV().rows() << endl;
        */

        // svd.matrixV() => 8x3
        // inv.asDiagonal() => 3x3
        // svd.matrixU().transpose() => (3x3).transpose() => (3x3)
        // (3x3) * (3x3) => (3x3)
        // (3x3) * (3x3) * (3x8) => (3x8)
        // Matrix<float, 8, 3> Jinv = 

        /* σ = (J+)x * Δp */
        MatrixXf tmp = svd.matrixV() * inv.asDiagonal();
        Param vdelta = (tmp * svd.matrixU().adjoint()) * (goal - endEffector);
        // Param vdelta = J.transpose() * (goal - endEffector);
        
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
 
    Point3f drawJoint(const Point3f& start, const Vector3f& arrow)
    {
    	Point3f center = start + jointRadius * arrow;
    	glColor3f(1.0, 0.0, 0.0);
        glPushMatrix();
    	glTranslatef(center.x(), center.y(), center.z());
    	glutSolidSphere(jointRadius, 10, 10);
    	glPopMatrix();
    	return center + jointRadius * arrow;
    }   

    void drawTetrahedron(const Point3f& center, const Vector3f& arrow, float length)
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
        drawTetrahedron(center, getArrow(0), getLength(0));

        /* Draw the elbow joint. */
        center = drawJoint(getShoulderEnd(), getArrow(1));

        /* Draw the forearm tetrahedron. */
        glColor3f(0.0, 0.1, 0.9);
        drawTetrahedron(center, getArrow(1), getLength(1));

        /* Draw the wrist joint. */
        center = drawJoint(getForearmEnd(), getArrow(2));

        /* Draw the pincer. */
        glColor3f(0.0, 0.9, 0.1);
        drawTetrahedron(center, getArrow(2), getPincerLength());
    }

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

// private:
    Torso* torso;
    
    float Stheta;
    float Sphi;
    
    /* <Theta1, Theta2, Theta3, Phi1, Phi2, Phi3, l1, l2> */
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
