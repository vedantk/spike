/*
 * spike.cc
 */

#include "iface.hh"

int main() {
    Torso torso = {
        .radius = 5.0,
        .theta = .3,
        .phi= .3,
        .centroid = Point3f(0, 0, 0),
    };

    Arm::Vector mat;
    mat << .3, .3, .3, .3, .3, .3, .5, .5;
    Arm arm(mat);

    cout << arm.getPincerEnd(&torso) << endl;
    
    /*
    arm.getLength(0) = 3.0;
    cout << arm.getPincerEnd(&torso) << endl;
    */

    return 0;
}
