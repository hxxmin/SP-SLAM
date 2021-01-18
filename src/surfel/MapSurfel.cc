
#include "surfel/MapSurfel.h"

#include<mutex>

namespace SP_SLAM
{

long unsigned int MapSurfel::nNextId=0;
mutex MapSurfel::mGlobalMutex;

MapSurfel::MapSurfel(const Surfel &Sf, KeyFrame *pRefKF, Map* pMap):
    mnFirstKFid(pRefKF->mnId), mnFirstFrame(pRefKF->mnFrameId), nObs(0), mnTrackReferenceForFrame(0),
    mnLastFrameSeen(0), mnBALocalForKF(0), mnFuseCandidateForKF(0), mnLoopSurfelForKF(0), mnCorrectedByKF(0),
    mnCorrectedReference(0), mnBAGlobalForKF(0), mpRefKF(pRefKF), mnVisible(1), mnFound(1), mbBad(false),
    mpReplaced(static_cast<MapSurfel*>(NULL)), mfMinDistance(0), mfMaxDistance(0), mpMap(pMap)
{
    mWorldSurfel=new Surfel(Sf);

    // MapSurfels can be created from Tracking and Local Mapping. This mutex avoid conflicts with id.
    unique_lock<mutex> lock(mpMap->mMutexPointCreation);
    mnId=nNextId++;

}

void MapSurfel::setSurfel(const Surfel &Sf)
{
    unique_lock<mutex> lock2(mGlobalMutex);
    unique_lock<mutex> lock(mMutexPos);
        //mWorldSurfel = Sf;
    delete mWorldSurfel;
    mWorldSurfel = new Surfel(Sf);
}

Surfel MapSurfel::getSurfel()
{
    unique_lock<mutex> lock(mMutexPos);
    return *mWorldSurfel;
}

KeyFrame* MapSurfel::GetReferenceKeyFrame()
{
    unique_lock<mutex> lock(mMutexFeatures);
    return mpRefKF;
}

void MapSurfel::AddObservation(KeyFrame* pKF, size_t idx)
{
    unique_lock<mutex> lock(mMutexFeatures);
    if(mObservations.count(pKF))
        return;
    mObservations[pKF]=idx;

    if(pKF->mvuRight[idx]>=0)
        nObs+=2;
    else
        nObs++;
}

void MapSurfel::EraseObservation(KeyFrame* pKF)
{
    bool bBad=false;
    {
        unique_lock<mutex> lock(mMutexFeatures);
        if(mObservations.count(pKF))
        {
            int idx = mObservations[pKF];
            if(pKF->mvuRight[idx]>=0)
                nObs-=2;
            else
                nObs--;

            mObservations.erase(pKF);

            if(mpRefKF==pKF)
                mpRefKF=mObservations.begin()->first;

            // If only 2 observations or less, discard point
            if(nObs<=2)
                bBad=true;
        }
    }

    if(bBad)
        SetBadFlag();
}

map<KeyFrame*, size_t> MapSurfel::GetObservations()
{
    unique_lock<mutex> lock(mMutexFeatures);
    return mObservations;
}

int MapSurfel::Observations()
{
    unique_lock<mutex> lock(mMutexFeatures);
    return nObs;
}

void MapSurfel::SetBadFlag()
{
    map<KeyFrame*,size_t> obs;
    {
        unique_lock<mutex> lock1(mMutexFeatures);
        unique_lock<mutex> lock2(mMutexPos);
        mbBad=true;
        obs = mObservations;
        mObservations.clear();
    }
    for(map<KeyFrame*,size_t>::iterator mit=obs.begin(), mend=obs.end(); mit!=mend; mit++)
    {
        KeyFrame* pKF = mit->first;
        pKF->EraseMapSurfelMatch(mit->second);
    }

    mpMap->EraseMapSurfel(this);
}

MapSurfel* MapSurfel::GetReplaced()
{
    unique_lock<mutex> lock1(mMutexFeatures);
    unique_lock<mutex> lock2(mMutexPos);
    return mpReplaced;
}

void MapSurfel::Replace(MapSurfel* pMS)
{
    if(pMS->mnId==this->mnId)
        return;

    int nvisible, nfound;
    map<KeyFrame*,size_t> obs;
    {
        unique_lock<mutex> lock1(mMutexFeatures);
        unique_lock<mutex> lock2(mMutexPos);
        obs=mObservations;
        mObservations.clear();
        mbBad=true;
        nvisible = mnVisible;
        nfound = mnFound;
        mpReplaced = pMS;
    }

    for(map<KeyFrame*,size_t>::iterator mit=obs.begin(), mend=obs.end(); mit!=mend; mit++)
    {
        // Replace measurement in keyframe
        KeyFrame* pKF = mit->first;

        if(!pMS->IsInKeyFrame(pKF))
        {
            pKF->ReplaceMapSurfelMatch(mit->second, pMS);
            pMS->AddObservation(pKF,mit->second);
        }
        else
        {
            pKF->EraseMapSurfelMatch(mit->second);
        }
    }
    pMS->IncreaseFound(nfound);
    pMS->IncreaseVisible(nvisible);

    mpMap->EraseMapSurfel(this);
}

bool MapSurfel::isBad()
{
    unique_lock<mutex> lock(mMutexFeatures);
    unique_lock<mutex> lock2(mMutexPos);
    return mbBad;
}

void MapSurfel::IncreaseVisible(int n)
{
    unique_lock<mutex> lock(mMutexFeatures);
    mnVisible+=n;
}

void MapSurfel::IncreaseFound(int n)
{
    unique_lock<mutex> lock(mMutexFeatures);
    mnFound+=n;
}

float MapSurfel::GetFoundRatio()
{
    unique_lock<mutex> lock(mMutexFeatures);
    return static_cast<float>(mnFound)/mnVisible;
}

int MapSurfel::GetIndexInKeyFrame(KeyFrame *pKF)
{
    unique_lock<mutex> lock(mMutexFeatures);
    if(mObservations.count(pKF))
        return mObservations[pKF];
    else
        return -1;
}

bool MapSurfel::IsInKeyFrame(KeyFrame *pKF)
{
    unique_lock<mutex> lock(mMutexFeatures);
    return (mObservations.count(pKF));
}

void MapSurfel::UpdateRadius()
{
    map<KeyFrame*,size_t> observations;
        if(mbBad)
            return;
        observations=mObservations;

    if(observations.empty())
        return;

    for(map<KeyFrame*,size_t>::iterator mit=observations.begin(), mend=observations.end(); mit!=mend; mit++)
    {
        KeyFrame* pKF = mit->first;

        Surfel sf_meas = pKF->getWorldSurfel(mit->second);

        // update MapSurfel Size
        float new_radius = calcDistanceBTSurfels(this->getSurfel(), sf_meas) + sf_meas.radius;
        if (new_radius > Surfel::MAXRADIUS)
            new_radius = Surfel::MAXRADIUS;

        if(this->mWorldSurfel->radius < new_radius)
        {
            unique_lock<mutex> lock(mMutexPos);
            this->mWorldSurfel->radius = new_radius;
        }
    }

}


} //namespace ORB_SLAM
