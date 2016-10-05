/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef MAPPOINT_H
#define MAPPOINT_H

#include"KeyFrame.h"
#include"Frame.h"
#include"Map.h"
#include "CVSerializationHelper.h"
#include "BowSerializationHelper.h"

#include<opencv2/core/core.hpp>
#include <boost/serialization/map.hpp>
#include<mutex>

namespace ORB_SLAM2
{

class KeyFrame;
class Map;
class Frame;


class MapPoint
{
public:
    MapPoint(const cv::Mat &Pos, KeyFrame* pRefKF, Map* pMap);
    MapPoint(const cv::Mat &Pos,  Map* pMap, Frame* pFrame, const int &idxF);
    MapPoint();

    void SetWorldPos(const cv::Mat &Pos);
    cv::Mat GetWorldPos();

    cv::Mat GetNormal();
    KeyFrame* GetReferenceKeyFrame();

    std::map<KeyFrame*,size_t> GetObservations();
    int Observations();

    void AddObservation(KeyFrame* pKF,size_t idx);
    void EraseObservation(KeyFrame* pKF);

    int GetIndexInKeyFrame(KeyFrame* pKF);
    bool IsInKeyFrame(KeyFrame* pKF);

    void SetBadFlag();
    bool isBad();

    void Replace(MapPoint* pMP);    
    MapPoint* GetReplaced();

    void IncreaseVisible(int n=1);
    void IncreaseFound(int n=1);
    float GetFoundRatio();
    inline int GetFound(){
        return mnFound;
    }

    void ComputeDistinctiveDescriptors();

    cv::Mat GetDescriptor();

    void UpdateNormalAndDepth();

    float GetMinDistanceInvariance();
    float GetMaxDistanceInvariance();
    int PredictScale(const float &currentDist, const float &logScaleFactor);

    bool equals(MapPoint* other);

public:
    long unsigned int mnId = 0;
    static long unsigned int nNextId;
    long int mnFirstKFid = 0;
    long int mnFirstFrame = 0;
    int nObs = 0;

    // Variables used by the tracking
    float mTrackProjX = 0;
    float mTrackProjY = 0;
    float mTrackProjXR = 0;
    bool mbTrackInView = false;
    int mnTrackScaleLevel = 0;
    float mTrackViewCos = 0;
    long unsigned int mnTrackReferenceForFrame = 0;
    long unsigned int mnLastFrameSeen = 0;

    // Variables used by local mapping
    long unsigned int mnBALocalForKF = 0;
    long unsigned int mnFuseCandidateForKF = 0;

    // Variables used by loop closing
    long unsigned int mnLoopPointForKF = 0;
    long unsigned int mnCorrectedByKF = 0;
    long unsigned int mnCorrectedReference = 0;
    cv::Mat mPosGBA;
    long unsigned int mnBAGlobalForKF = 0;


    static std::mutex mGlobalMutex;

protected:    

     // Position in absolute coordinates
     cv::Mat mWorldPos;

     // Keyframes observing the point and associated index in keyframe
     std::map<KeyFrame*,size_t> mObservations;

     // Mean viewing direction
     cv::Mat mNormalVector;

     // Best descriptor to fast matching
     cv::Mat mDescriptor;

     // Reference KeyFrame
     KeyFrame* mpRefKF = NULL;

     // Tracking counters
     int mnVisible  = 0;
     int mnFound = 0;

     // Bad flag (we do not currently erase MapPoint from memory)
     bool mbBad = false;
     MapPoint* mpReplaced;

     // Scale invariance distances
     float mfMinDistance = 0;
     float mfMaxDistance = 0;

     Map* mpMap = NULL;

     std::mutex mMutexPos;
     std::mutex mMutexFeatures;
private:
    // Boost serialization
    friend class boost::serialization::access;

    template<class Archive>
    void serialize(Archive & ar, const unsigned int version)
    {
        ar & mnId;
        ar & nNextId;
        ar & mnFirstKFid;
        ar & mnFirstFrame;
        ar & nObs;

        ar & mTrackProjX;
        ar & mTrackProjY;
        ar & mTrackProjXR;
        ar & mbTrackInView;
        ar & mnTrackScaleLevel;
        ar & mTrackViewCos;
        ar & mnTrackReferenceForFrame;
        ar & mnLastFrameSeen;

        ar & mnBALocalForKF;
        ar & mnFuseCandidateForKF;

        ar & mnLoopPointForKF;
        ar & mnCorrectedByKF;
        ar & mnCorrectedReference;
        ar & mPosGBA;
        ar & mnBAGlobalForKF;

        ar & mWorldPos;
        ar & mObservations;
        ar & mNormalVector;
        ar & mDescriptor;
        ar & mpRefKF;
        ar & mnVisible;
        ar & mnFound;
        ar & mbBad;
        ar & mpReplaced;
        ar & mfMinDistance;
        ar & mfMaxDistance;
        ar & mpMap;
    }
};

} //namespace ORB_SLAM

#endif // MAPPOINT_H
