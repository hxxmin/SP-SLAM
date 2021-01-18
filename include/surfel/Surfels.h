#ifndef SURFELS_H_
#define SURFELS_H_

#include "Thirdparty/g2o/g2o/types/se3quat.h"

#include <Eigen/Eigen>
#include <vector>
#include <Frame.h>

using namespace Eigen;

typedef Eigen::Matrix<double, 3, 8> Matrix38d;
typedef Eigen::Matrix<double, 10, 1> Vector10d;
typedef Eigen::Matrix<double, 6, 1> Vector6d;
typedef Eigen::Matrix<double, 7, 1> Vector7d;
typedef Eigen::Matrix<double, 4, 1> Vector4d;
typedef Eigen::Matrix<double, 6, 6> Matrix6d;

namespace SP_SLAM
{

class Frame;


/** \brief surfel representation. **/
class Surfel {
public:
    const static Vector3d zAxis;
    const static float MAXRADIUS;
    const static float MINRADIUS;

    const Matrix3d Normal2RotM(Vector3d vNormal) const;
    Matrix3d getRotM() const;
public:
    Surfel();
    Surfel(const Surfel &surfel); // Copy constructor.

    bool isNaN() const;

    Eigen::Vector3d getNormal() const;
    Eigen::Vector4d getQuat() const;
    Eigen::Vector3d getPosition() const;
    void setPosition(const cv::Mat x3D);
    void setPosition(const float _x, const float _y, const float _z);
    void setPosition(const Eigen::Vector3d pos);
    void setNormal(const cv::Mat x3Dn);
    void setNormal(const float nx, const float ny, const float nz);
    void setNormal(const Eigen::Vector3d n);
    void setNormal(const Eigen::Quaterniond quat);
    void setColor(const float _r, const float _g, const float _b);

    Surfel getSurfel() const;
    Vector6d getSurfel6d();
    Vector7d getSurfel7d() const;

//private:
    float px, py, pz;
    float radius/*, color*/, weight;
    int r, g, b;
    int update_times;
    int last_update;
    float nx, ny, nz;

public:
    // xyz nx ny nz
    void fromVector(const Eigen::Matrix<double, 7, 1> &v);
    void print();

    void printPos()
    {
        cout << px << ", " << py << ", " << pz << endl;
    }

    // g2o 용

    Surfel exp_update(const Eigen::Matrix<double, 6, 1> &update);   
    Vector6d surfel_log_error(const Surfel &newone);
    Vector6d surfel_plane_error(const Surfel &newone);

    // xyz nx ny nz
    inline void fromMinimalVector(const Eigen::Matrix<double, 6, 1> &v)
    {
            setPosition(v(0), v(1), v(2));
            setNormal(v(3), v(4), v(5));
    }

    // xyz nx ny nz
    inline Eigen::Matrix<double, 6, 1> toMinimalVector() const
    {
            Vector6d v;
            v(0) = px; v(1) = py; v(2) = pz;
            v(3) = nx; v(4) = ny; v(5) = nz;
            return v;
    }


};

}
#endif /* SURFELS_H_ */
