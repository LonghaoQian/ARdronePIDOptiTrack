// a file contains all utility functions
#include <Eigen/Eigen>

#ifndef UTILITYFUNCTIONS_H
#define UTILITYFUNCTIONS_H

using namespace std;
using namespace Eigen;
namespace mathauxiliary{
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

    void GetEulerAngleFromQuaterion(const Vector4d& quaterion,Vector3d& Euler){
    /* Normal means the following https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
    */
    // roll (x-axis rotation)
    double sinr_cosp = +2.0 * (quaterion(0) * quaterion(1) + quaterion(2) * quaterion(3));
    double cosr_cosp = +1.0 - 2.0 * (quaterion(1) * quaterion(1) + quaterion(2) * quaterion(2));
    double roll = atan2(sinr_cosp, cosr_cosp);

    // pitch (y-axis rotation)
    double sinp = +2.0 * (quaterion(0) *quaterion(2) - quaterion(3) * quaterion(1));
    double pitch;
    if (fabs(sinp) >= 1)
           pitch = copysign(M_PI / 2, sinp); // use 90 degrees if out of range
    else
           pitch = asin(sinp);

    // yaw (z-axis rotation)
    double siny_cosp = +2.0 * (quaterion(0) * quaterion(3) + quaterion(1) * quaterion(2));
    double cosy_cosp = +1.0 - 2.0 * (quaterion(2)* quaterion(2) + quaterion(3) * quaterion(3));
    double yaw = atan2(siny_cosp, cosy_cosp);
    Euler(0) = roll;
    Euler(1) = pitch;
    Euler(2) = yaw;}

}
#endif