
#ifndef SURFELMATCHER_H
#define SURFELMATCHER_H

#include<vector>
#include<opencv2/core/core.hpp>
#include<opencv2/features2d/features2d.hpp>

#include "MapSurfel.h"
#include "KeyFrame.h"
#include "Frame.h"

namespace SP_SLAM
{


class Surfelmatcher
{
public:
    static const float SF_TH_DIST;
    static const float SF_TH_ANGLE;
    static const float SF_TH_COLOR;

    static const float SF_CENTERDIST_RATIO;
    static const float SF_PLANEDIST_RATIO;

    Surfelmatcher();
    int SearchByCenterDistance(KeyFrame *pKF1, KeyFrame *pKF2,
                   vector<pair<size_t, size_t> > &vMatchedPairs, const bool bOnlyStereo);
    int SearchByCenterDistance(KeyFrame &KF, const vector<MapSurfel*> &vpMapSurfels);

    int SearchByDistAngle(KeyFrame *pKF1, KeyFrame *pKF2,
                   vector<pair<size_t, size_t> > &vMatchedPairs, const bool bOnlyStereo);
    int SearchByDistAngle(KeyFrame *pKF, const vector<MapSurfel*> &vpMapSurfels);
    int SearchByDistAngleMulti(KeyFrame *pKF, const vector<MapSurfel*> &vpMapSurfels);
    int SearchWithNeighbor(KeyFrame &CurrentKF);
    int SearchWithKF(Frame &CurrentFrame, KeyFrame *pKFKeyFrame);

    static float calcDistanceBTSurfels(Surfel Sf1, Surfel Sf2);
    static float calcPlaneDistanceBTSurfels(Surfel source, Surfel target);
    static float calcAngleBTSurfels(Surfel Sf1, Surfel Sf2);
    static float calcColorDiffBTSurfels(Surfel Sf1, Surfel Sf2);

    int AddObservation(KeyFrame *pKF, const vector<MapSurfel *> &vpMapSurfels, const float th);

};
}

#endif // SURFELMATCHER_H
