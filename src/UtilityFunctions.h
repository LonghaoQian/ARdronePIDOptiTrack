// a file contains all utility functions
#include <Eigen/Eigen>
#include <math.h>
#include <math_utils.h>

#ifndef UTILITYFUNCTIONS_H
#define UTILITYFUNCTIONS_H

using namespace std;
namespace namespace_mathauxiliary {
    void Veemap(const Matrix3d& cross_matrix, Vector3d& vector){
        vector(0) = -cross_matrix(1,2);
        vector(1) = cross_matrix(0,2);
        vector(2) = -cross_matrix(0,1);
    }
    void Hatmap(const Vector3d& vector, Matrix3d& cross_matrix){
    /*
    r^x = [0 -r3 r2;
           r3 0 -r1;
          -r2 r1 0]
    */
    cross_matrix(0,0) = 0.0;
    cross_matrix(0,1) = - vector(2);
    cross_matrix(0,2) = vector(1);

    cross_matrix(1,0) = vector(2);
    cross_matrix(1,1) = 0.0;
    cross_matrix(1,2) = - vector(0);

    cross_matrix(2,0) = - vector(1);
    cross_matrix(2,1) = vector(0);
    cross_matrix(2,2) = 0.0;
    }
}
#endif