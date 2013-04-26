/*
 * util.hh
 */

#pragma once

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <assert.h>

#include <string>
#include <vector>
#include <utility>
#include <iostream>
#include <algorithm>

#include <GL/glut.h>
#include <GL/glu.h>

#include <Eigen/SVD>
#include <Eigen/Dense>
#include <Eigen/StdVector>

#ifdef _WIN32
#include <windows.h>
#else
#include <sys/time.h>
#endif

using namespace std;
using namespace Eigen;

typedef Vector3f Point3f;
typedef Vector3i Point3i;
typedef Vector3f Color3f;
typedef pair<float, float> FloatPair;
typedef pair<Vector3f, Vector3f> Vec3fPair;

#define expand_vec3(vec) \
    vec(0), vec(1), vec(2)

#define cout_expand_vec3(vec) \
    "<" << vec(0) << ", " << vec(1) << ", " << vec(2) << ">"

inline float fsign(float v0)
{
    if (v0 >= 0) {
        return 1.0;
    } else {
        return -1.0;
    }
}

inline bool fequal(float lhs, float rhs)
{
    return fabs(lhs - rhs) < 0.001;
}

inline float square(float a) {
    return a * a;
}

inline FloatPair addfp(FloatPair lhs, FloatPair rhs)
{
    return make_pair(lhs.first + rhs.first, lhs.second + rhs.second);
}

inline FloatPair scalefp(float c, FloatPair p)
{
    return make_pair(c * p.first, c * p.second);
}

inline void glvtx3f(Vector3f vec)
{
    glVertex3f(expand_vec3(vec));
}

inline void glcol3f(Vector3f vec)
{
    glColor3f(expand_vec3(vec));
}

inline float remap(float v0,
                   float min0, float max0,
                   float minf, float maxf)
{
    /* Remap a signal in [min0, max0] to [minf, maxf] linearly. */
    return ((maxf - minf) / (max0 - min0)) * (v0 - min0);
}

inline float clamp(float v0, float min0, float max0)
{
    if (v0 < min0) return min0;
    if (v0 > max0) return max0;
    return v0;
}

inline Vector3f clampv(const Vector3f& v0, float min0, float max0)
{
    return Vector3f(clamp(v0(0), min0, max0),
                    clamp(v0(1), min0, max0),
                    clamp(v0(2), min0, max0));
}

inline Vector4f homogenize(const Vector3f& v, bool location)
{
    return Vector4f(v(0), v(1), v(2), float(location));
}

inline Vector3f dehomogenize(const Vector4f& v)
{
    return Vector3f(v(0), v(1), v(2));
}

inline void print_vec3(string name, Vector3f& vec)
{
    cout << name << ": " << cout_expand_vec3(vec) << endl;
}

template<typename T>
inline void print_vector(vector<T>& lst, int beg, int end)
{
    cout << "[";
    for (int a = beg; a < end; ++a) {
        cout << lst[a];
        if (a < (end - 1)) {
            cout << ", ";
        }
    }
    cout << "]" << endl;
}
