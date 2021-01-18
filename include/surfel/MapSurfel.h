#ifndef MAPSURFEL_H
#define MAPSURFEL_H

#include"KeyFrame.h"
#include"Frame.h"
#include"Map.h"

#include<opencv2/core/core.hpp>
#include<mutex>

#include "Surfels.h"

namespace SP_SLAM
{

class KeyFrame;
class Map;
class Frame;
class Surfel;

class MapSurfel
{
public:
    MapSurfel(const Surfel &Sf, KeyFrame* pRefKF, Map* pMap);
    MapSurfel(const Surfel &Sf,  Map* pMap, Frame* pFrame, const int &idxF);
    MapSurfel(const cv::Mat &Pos, KeyFrame* pRefKF, Map* pMap);

    void setSurfel(const Surfel &Sf);
    Surfel getSurfel();

    KeyFrame* GetReferenceKeyFrame();

    std::map<KeyFrame*,size_t> GetObservations();
    int Observations();

    void AddObservation(KeyFrame* pKF, size_t idx);
    void EraseObservation(KeyFrame* pKF);

    int GetIndexInKeyFrame(KeyFrame* pKF);
    bool IsInKeyFrame(KeyFrame* pKF);

    void SetBadFlag();
    bool isBad();

    void Replace(MapSurfel* pMP);
    MapSurfel* GetReplaced();

    void IncreaseVisible(int n=1);
    void IncreaseFound(int n=1);
    float GetFoundRatio();
    inline int GetFound(){
        return mnFound;
    }

    void UpdateRadius();

public:
    long unsigned int mnId;
    static long unsigned int nNextId;
    long int mnFirstKFid;
    long int mnFirstFrame;
    int nObs;

    // Variables used by the tracking
    float mTrackProjX;
    float mTrackProjY;
    float mTrackProjXR;
    bool mbTrackInView;
    int mnTrackScaleLevel;
    float mTrackViewCos;
    long unsigned int mnTrackReferenceForFrame;
    long unsigned int mnLastFrameSeen;

    // Variables used by local mapping
    long unsigned int mnBALocalForKF;
    long unsigned int mnFuseCandidateForKF;

    // Variables used by loop closing
    long unsigned int mnLoopSurfelForKF;
    long unsigned int mnCorrectedByKF;
    long unsigned int mnCorrectedReference;
    //cv::Mat mPosGBA;
    Surfel* mSfGBA;
    long unsigned int mnBAGlobalForKF;

    static std::mutex mGlobalMutex;

protected:    
    Surfel* mWorldSurfel;

    // Keyframes observing the surfel and associated index in keyframe
    std::map<KeyFrame*,size_t> mObservations;

    // Mean viewing direction
    cv::Mat mNormalVector;

    // Reference KeyFrame
    KeyFrame* mpRefKF;

    // Tracking counters
    int mnVisible;
    int mnFound;

     // Bad flag (we do not currently erase MapSurfel from memory)
     bool mbBad;
     MapSurfel* mpReplaced;

     // Scale invariance distances
     float mfMinDistance;
     float mfMaxDistance;

     Map* mpMap;

     std::mutex mMutexPos;
     std::mutex mMutexFeatures;
};

} 

#endif // MAPSURFEL_H
