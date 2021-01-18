#include "surfel/Surfels.h"
namespace SP_SLAM
{

const float Surfel::MAXRADIUS = 0.10;
const float Surfel::MINRADIUS = 0.01;

const Vector3d Surfel::zAxis = Vector3d(0.0, 0.0, 1.0);

float calcDistanceBTSurfels(Surfel Sf1, Surfel Sf2)
{

    float dx = Sf1.px - Sf2.px;
    float dy = Sf1.py - Sf2.py;
    float dz = Sf1.pz - Sf2.pz;

    return sqrt(dx*dx+dy*dy+dz*dz);
}

Surfel::Surfel()
{

    px = 0.f; py = 0.f; pz = 0.f;
    nx = 0.f; ny = 0.f; nz = 0.f;
    radius = 0.f;
    //color = 0.f;
    weight = 0.f;

    r= 0; g=0; b=0;

    update_times = -1;
    last_update = -1;
}

//Copy Constructor
Surfel::Surfel(const Surfel &surfel)
    :px(surfel.px),py(surfel.py),pz(surfel.pz),
      nx(surfel.nx),ny(surfel.ny),nz(surfel.nz),
      radius(surfel.radius),weight(surfel.weight),
      r(surfel.r),g(surfel.g),b(surfel.b)
{
}
bool Surfel::isNaN() const
{
    return isnan(this->px)||isnan(this->py) ||isnan(this->pz);
}

Surfel Surfel::getSurfel() const
{
    return *this;
}

void Surfel::setPosition(const cv::Mat x3D)
{
    const float* x3Dptr = x3D.ptr<float>(0);

    px = x3Dptr[0];
    py = x3Dptr[1];
    pz = x3Dptr[2];
}

void Surfel::setPosition(const float _x, const float _y, const float _z)
{
    px = _x;
    py = _y;
    pz = _z;
}

void Surfel::setPosition(Eigen::Vector3d pos)
{
    px = pos(0);
    py = pos(1);
    pz = pos(2);
}

void Surfel::setNormal(const cv::Mat x3Dn)
{
    const float* x3Dnptr = x3Dn.ptr<float>(0);

    float _nx = x3Dnptr[0];
    float _ny = x3Dnptr[1];
    float _nz = x3Dnptr[2];
    float norm = sqrt(_nx*_nx + _ny*_ny + _nz*_nz);
    nx = _nx/norm;
    ny = _ny/norm;
    nz = _nz/norm;
}

void Surfel::setNormal(const float _nx, const float _ny, const float _nz)
{
    float norm = sqrt(_nx*_nx + _ny*_ny + _nz*_nz);
    nx = _nx/norm;
    ny = _ny/norm;
    nz = _nz/norm;
}

void Surfel::setNormal(const Vector3d n)
{
    this->nx= n(0);
    this->ny= n(1);
    this->nz= n(2);
}

void Surfel::setNormal(const Quaterniond quat)
{
    Matrix3d rotM(quat);
    Vector3d v = rotM * zAxis;
    this->setNormal(v);
}

void Surfel::setColor(const float _r, const float _g, const float _b)
{
    r = _r;
    g = _g;
    b = _b;
}

void Surfel::print()
{
    cout << "pos: (" << px << "," << py << "," << pz <<")"
         << ", normal: (" << nx << "," << ny << "," << nz << ")"
         << ", radius: " << radius << endl;
}


}

