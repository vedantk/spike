/*
 * arm.hh
 */

#pragma once

#include "util.hh"
#include <FreeImage.h>

#define jointRadius 0.02

inline Vector3f getDirection(float theta, float phi)
{
    return Vector3f(sin(phi) * cos(theta), cos(phi), sin(phi) * sin(theta));
}

inline float getDirectionComponent(float theta, float phi, int k)
{
    if (k == 0) {
        return sin(phi) * cos(theta);
    } else if (k == 1) {
        return cos(phi);
    } else if (k == 2) {
        return sin(phi) * sin(theta);
    }
    return 0.0;
}

struct Torso {
    float radius;
    Point3f centroid;
    GLUquadricObj* quadric;

    /* http://web.engr.oregonstate.edu/~mjb/cs553/Handouts/Texture/texture.pdf */
    GLuint texID;

    Torso(Point3f _centroid)
        : radius(0.1), centroid(_centroid)
    {
        FreeImage_Initialise();
        FIBITMAP* bitmap = FreeImage_Load(FIF_BMP,
                                          "data/obrien-many-texture.bmp",
                                          BMP_DEFAULT);
        if (!bitmap) {
            cout << "Torso::Torso(): Could not load bitmap.\n";
            exit(0);
        }

        uint32_t bm_width = FreeImage_GetWidth(bitmap);
        uint32_t bm_height = FreeImage_GetHeight(bitmap);
        uint8_t* texture = (uint8_t*) FreeImage_GetBits(bitmap);

        glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
        glGenTextures(1, &texID);
        glBindTexture(GL_TEXTURE_2D, texID);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
        glTexEnvf(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_REPLACE); 
        glTexImage2D(GL_TEXTURE_2D, 0, 3, bm_width, bm_height,
                     0, GL_RGB, GL_UNSIGNED_BYTE, texture);

        FreeImage_Unload(bitmap);
        FreeImage_DeInitialise();

        /* http://acidleaf.com/texture-mapping-the-glusphere/ */
        quadric = gluNewQuadric();
        gluQuadricNormals(quadric, GLU_SMOOTH);
        gluQuadricTexture(quadric, GL_TRUE);
    }
    
    void render()
    {
        glEnable(GL_TEXTURE_2D);
        glBindTexture(GL_TEXTURE_2D, texID);
        glMatrixMode(GL_TEXTURE);
        glLoadIdentity();
        glTranslatef(0.5, 0, 0);
        glRotatef(180, 1, 0, 0);

        glMatrixMode(GL_MODELVIEW);
        glShadeModel(GL_SMOOTH);
        glPushMatrix();
    	glTranslatef(centroid.x(), centroid.y(), centroid.z());
    	gluSphere(quadric, radius, 30, 30);
    	glPopMatrix();

        glDisable(GL_TEXTURE_2D);
    }

};

struct Arm {
    typedef Matrix<float, 8, 1> Param;

    Torso* torso;
    
    float Stheta;
    float Sphi;
    
    /* <Theta1, Theta2, Theta3, Phi1, Phi2, Phi3, l1, l2> */
    Param values;
    
    MatrixXf J;
    
    Point3f endEffector;
    Point3f goal;

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
        values(6) = 0.5;
        values(7) = 0.7;

        J = MatrixXf(3, 8);
        endEffector = getPincerEnd();
        goal = endEffector;
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

    inline float getArrowComponent(int n, int k)
    {
        float theta = Stheta;
        float phi = Sphi;
        for (int i=0; i < n; ++i) {
            theta += getTheta(i);
            phi += getPhi(i);
        }
        return getDirectionComponent(theta, phi, k);
    }

    inline Point3f getTorsoEnd()
    {
        return torso->centroid + (torso->radius * getArrow(0));
    }

    inline float getTorsoEndComponent(int k)
    {
        return torso->centroid[k] + (torso->radius * getArrowComponent(0, k));
    }

    inline Point3f getShoulderEnd()
    {
        float segLen = getLength(0) + (2 * jointRadius);
        return getTorsoEnd() + (segLen * getArrow(1));
    }

    inline float getShoulderEndComponent(int k)
    {
        float segLen = getLength(0) + (2 * jointRadius);
        return getTorsoEndComponent(k) + (segLen * getArrowComponent(1, k));
    }

    inline Point3f getForearmEnd()
    {
        float segLen = getLength(1) + (2 * jointRadius);
        return getShoulderEnd() + (segLen * getArrow(2));
    }

    inline float getForearmEndComponent(int k)
    {
        float segLen = getLength(1) + (2 * jointRadius);
        return getShoulderEndComponent(k) + (segLen * getArrowComponent(2, k));
    }

    inline Point3f getPincerEnd()
    {
        float segLen = getPincerLength() + (2 * jointRadius);
        endEffector = getForearmEnd() + (segLen * getArrow(3));
        return endEffector;
    }

    inline float getPincerEndComponent(int k)
    {
        float segLen = getPincerLength() + (2 * jointRadius);
        return getForearmEndComponent(k) + (segLen * getArrowComponent(3, k));
    }

    inline float getPincerDelta(int direction, int k, float delta)
    {
        values[k] += delta;
        return getPincerEndComponent(direction);
    }

    float partialDerivative(int direction, int parameter)
    {
        const float delta = 0.001;
        return (getPincerDelta(direction, parameter, delta) -
                getPincerDelta(direction, parameter, -delta)) / delta;
    }

    void computeJacobian()
    {
        for (int direction = 0; direction < 3; ++direction) {
            for (int parameter = 0; parameter < 8; ++parameter) {
                J(direction, parameter) = partialDerivative(direction, parameter);
            }
        }
    }
    
    inline void updatePosition(const Param& delta)
    {
        values += delta;
        endEffector = getPincerEnd();
    }

    // return error if less than tolerance, else return 0
    inline float getError(float posTolerance = 1.0e-1) {
        float error = (endEffector - goal).norm();
        if (error < posTolerance) {
            return 0;
        } else {
            return error;
        }
    }

    bool IKUpdate(float posTolerance = 1.0e-1)
    {
        const float tolerance = 1.0e-5;
        const int maxSplits = 8;
        
        float error = getError(posTolerance);
        if (error == 0) return true;
        
        computeJacobian();
        
        JacobiSVD<MatrixXf> svd = J.jacobiSvd(ComputeThinU | ComputeThinV);
        JacobiSVD<MatrixXf>::SingularValuesType inv = svd.singularValues();

        for (long i=0; i < J.rows(); ++i) {
            inv(i) = (inv(i) > tolerance) ? (1.0 / inv(i)) : 0.0;
        }

        /* σ = (J+)x * Δp */
        MatrixXf Jinv = svd.matrixV() * inv.asDiagonal() * svd.matrixU().adjoint();
        Param vdelta = (Jinv) * (goal - endEffector) / 3;
        
        updatePosition(vdelta);
        float currentError;
        int nrSplits = 0;
        while ((currentError = getError()) > error
               && (nrSplits++ < maxSplits))
        {
            vdelta /= 2;
            updatePosition(-vdelta);
        }
        return currentError <= 0;
    }
 
    // draws spherical joint, returns center of joint
    Point3f drawJoint(const Point3f& segmentEnd, const Vector3f& arrow)
    {
    	Point3f center = segmentEnd + jointRadius * arrow;
    	// glColor3f(0.4, 0.1, 0.1);
        glPushMatrix();
    	glTranslatef(center.x(), center.y(), center.z());
    	glutSolidSphere(jointRadius, 20, 20);
    	glPopMatrix();
        return center;
    }   

    // draw a tetrahedron with base at center, in the direction of arrow
    void drawTetrahedron(const Point3f& center, const Vector3f& arrow, float length)
    {
        /* Actually this is technically a square pyramid. Whoops. */

        const float sideLength = 0.07;
        const float sideHalved = sideLength / 2.0;

        // get one other point on the plane (plane defined by arrow.dot((x - center)) = 0)
        Vector3f linearlyIndependent;
        if (arrow.x() == 0 && arrow.z() == 0) {
            // vector points in y direction
            linearlyIndependent = Vector3f(1,0,0);
        } else {
            linearlyIndependent = Vector3f(0,1,0);
        }

        Point3f right = arrow.cross(linearlyIndependent).normalized();
        Point3f up = right.cross(arrow).normalized();

        Point3f ll = center - sideHalved * (right + up);
        Point3f lr = center + (sideHalved * right) - (sideHalved * up);
        Point3f ul = center - (sideHalved * right) + (sideHalved * up);
        Point3f ur = center + sideHalved * (right + up);
        Point3f tip = center + length * arrow;

        /* Draw the base. */
        glnorm3f((ur - ll).cross(lr - ul));
        glBegin(GL_POLYGON);
            glvtx3f(ll);
            glvtx3f(lr);
            glvtx3f(ur);
            glvtx3f(ul);
        glEnd();

        /* Draw each face. */
        glBegin(GL_TRIANGLES);
            glnorm3f((lr - ll).cross(tip - ll));
            glvtx3f(ll);
            glvtx3f(lr);
            glvtx3f(tip);

            glnorm3f((ul - ll).cross(tip - ll));
            glvtx3f(ll);
            glvtx3f(ul);
            glvtx3f(tip);

            glnorm3f((ul - ur).cross(tip - ur));
            glvtx3f(ur);
            glvtx3f(ul);
            glvtx3f(tip);

            glnorm3f((lr - ur).cross(tip - ur));
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
        center += getArrow(1) * jointRadius;

        /* Draw the shoulder tetrahedron. */
        // glColor3f(0xff / 255.0, 0x9e / 255.0, 0.0);
        drawTetrahedron(center, getArrow(1), getLength(0));

        /* Draw the elbow joint. */
        center = drawJoint(getShoulderEnd(), getArrow(1));
        center += getArrow(2) * jointRadius;

        /* Draw the forearm tetrahedron. */
        // glColor3f(0x09 / 255.0, 0x66 / 255.0, 0xde / 255.0);
        drawTetrahedron(center, getArrow(2), getLength(1));

        /* Draw the wrist joint. */
        center = drawJoint(getForearmEnd(), getArrow(2));
        center += getArrow(3) * jointRadius;

        /* Draw the pincer. */
        // glColor3f(1.0, 0.0, 0.0);
        drawTetrahedron(center, getArrow(3), getPincerLength());

        // debug goals
        glColor3f(0, 1, 0);
        glPushMatrix();
        glTranslatef(goal.x(), goal.y(), goal.z());
        glutSolidSphere(jointRadius, 20, 20);
        glColor3f(1, 0, 0);
        glPopMatrix();
    }

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
};
