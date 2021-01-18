#include "surfel/SurfelMatcher.h"


using namespace std;

namespace SP_SLAM
{

// TUM/ICL
const float Surfelmatcher::SF_TH_DIST = 0.2; 
const float Surfelmatcher::SF_TH_ANGLE = 20.0 * M_PI / 180;

const float Surfelmatcher::SF_CENTERDIST_RATIO = 1.5;
const float Surfelmatcher::SF_PLANEDIST_RATIO = 0.7;

Surfelmatcher::Surfelmatcher()//: mfNNratio(nnratio), mbCheckOrientation(checkOri)
{
}

float Surfelmatcher::calcDistanceBTSurfels(Surfel Sf1, Surfel Sf2)
{

    float dx = Sf1.px - Sf2.px;
    float dy = Sf1.py - Sf2.py;
    float dz = Sf1.pz - Sf2.pz;

    return sqrt(dx*dx+dy*dy+dz*dz);
}

float Surfelmatcher::calcPlaneDistanceBTSurfels(Surfel source, Surfel target)
{
    // translation은 중점에서 plane까지로
    float dist = abs(target.nx*(source.px - target.px)
            +target.ny*(source.py - target.py)
            +target.nz*(source.pz - target.pz));

    float dist2 = abs(source.nx*(target.px - source.px)
            +source.ny*(target.py - source.py)
            +source.nz*(target.pz - source.pz));

    return (abs(dist) + abs(dist2))/2;
}

//calcAngleBTSurfels(SfMap, SfKF);
float Surfelmatcher::calcAngleBTSurfels(Surfel Sf1, Surfel Sf2)
{
    Eigen::Vector3f vSf1(Sf1.nx, Sf1.ny, Sf1.nz);
    Eigen::Vector3f vSf2(Sf2.nx, Sf2.ny, Sf2.nz);

    float cosTh = vSf1.dot(vSf2) / (vSf1.norm() * vSf2.norm());
    // for safe acos
    if(cosTh < -1.0) cosTh = -1.0;
    else if(cosTh > 1.0) cosTh = 1.0;

    return acos(cosTh);
}

float Surfelmatcher::calcColorDiffBTSurfels(Surfel Sf1, Surfel Sf2)
{
    Eigen::Vector3f vSf1(Sf1.r, Sf1.g, Sf1.b);
    Eigen::Vector3f vSf2(Sf2.r, Sf2.g, Sf2.b);

    Eigen::Vector3f rgb_diff = vSf1 - vSf2;

    return rgb_diff.norm();
}


} //namespace ORB_SLAM

