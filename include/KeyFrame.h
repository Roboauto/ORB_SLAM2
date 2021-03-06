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

#ifndef KEYFRAME_H
#define KEYFRAME_H

#include "MapPoint.h"
#include "DBoW2/BowVector.h"
#include "DBoW2/FeatureVector.h"
#include "ORBVocabulary.h"
#include "ORBextractor.h"
#include "Frame.h"
#include "KeyFrameDatabase.h"

#include <mutex>
#include <boost/serialization/vector.hpp>
#include <boost/serialization/set.hpp>
#include <boost/serialization/map.hpp>


namespace ORB_SLAM2
{

class Map;
class MapPoint;
class Frame;
class KeyFrameDatabase;

class KeyFrame
{
public:
    KeyFrame(Frame &F, Map* pMap, KeyFrameDatabase* pKFDB);

    //DO NOT USE. ONLY FOR BOOST SERIALIZATION
    KeyFrame();

    // Pose functions
    void SetPose(const cv::Mat &Tcw);
    cv::Mat GetPose();
    cv::Mat GetPoseInverse();
    cv::Mat GetCameraCenter();
    cv::Mat GetStereoCenter();
    cv::Mat GetRotation();
    cv::Mat GetTranslation();

    // Bag of Words Representation
    void ComputeBoW();

    // Covisibility graph functions
    void AddConnection(KeyFrame* pKF, const int &weight);
    void EraseConnection(KeyFrame* pKF);
    void UpdateConnections();
    void UpdateBestCovisibles();
    std::set<KeyFrame *> GetConnectedKeyFrames();
    std::vector<KeyFrame* > GetVectorCovisibleKeyFrames();
    std::vector<KeyFrame*> GetBestCovisibilityKeyFrames(const int &N);
    std::vector<KeyFrame*> GetCovisiblesByWeight(const int &w);
    int GetWeight(KeyFrame* pKF);

    // Spanning tree functions
    void AddChild(KeyFrame* pKF);
    void EraseChild(KeyFrame* pKF);
    void ChangeParent(KeyFrame* pKF);
    std::set<KeyFrame*> GetChilds();
    KeyFrame* GetParent();
    bool hasChild(KeyFrame* pKF);

    // Loop Edges
    void AddLoopEdge(KeyFrame* pKF);
    std::set<KeyFrame*> GetLoopEdges();

    // MapPoint observation functions
    void AddMapPoint(MapPoint* pMP, const size_t &idx);
    void EraseMapPointMatch(const size_t &idx);
    void EraseMapPointMatch(MapPoint* pMP);
    void ReplaceMapPointMatch(const size_t &idx, MapPoint* pMP);
    std::set<MapPoint*> GetMapPoints();
    std::vector<MapPoint*> GetMapPointMatches();
    int TrackedMapPoints(const int &minObs);
    MapPoint* GetMapPoint(const size_t &idx);

    // KeyPoint functions
    std::vector<size_t> GetFeaturesInArea(const float &x, const float  &y, const float  &r) const;
    cv::Mat UnprojectStereo(int i);

    // Image
    bool IsInImage(const float &x, const float &y) const;

    // Enable/Disable bad flag changes
    void SetNotErase();
    void SetErase();

    // Set/check bad flag
    void SetBadFlag();
    bool isBad();

    // Compute Scene Depth (q=2 median). Used in monocular.
    float ComputeSceneMedianDepth(const int q);

    static bool weightComp( int a, int b){
        return a>b;
    }

    static bool lId(KeyFrame* pKF1, KeyFrame* pKF2){
        return pKF1->mnId<pKF2->mnId;
    }

    bool equals(KeyFrame* other);

    void SetVocabulary(ORBVocabulary *voc);
    const ORBVocabulary* GetVocabulary() const;


    // The following variables are accesed from only 1 thread or never change (no mutex needed).
public:

    static long unsigned int nNextId;
    long unsigned int mnId = 0;
    const long unsigned int mnFrameId;

    const double mTimeStamp;

    // Grid (to speed up feature matching)
    const int mnGridCols;
    const int mnGridRows;
    const float mfGridElementWidthInv;
    const float mfGridElementHeightInv;

    // Variables used by the tracking
    long unsigned int mnTrackReferenceForFrame = 0;
    long unsigned int mnFuseTargetForKF = 0;

    // Variables used by the local mapping
    long unsigned int mnBALocalForKF = 0;
    long unsigned int mnBAFixedForKF = 0;

    // Variables used by the keyframe database
    long unsigned int mnLoopQuery = 0;
    int mnLoopWords = 0;
    float mLoopScore = 0;
    long unsigned int mnRelocQuery = -1;
    int mnRelocWords = 0;
    float mRelocScore = 0;

    // Variables used by loop closing
    cv::Mat mTcwGBA;
    cv::Mat mTcwBefGBA;
    long unsigned int mnBAGlobalForKF = 0;

    // Calibration parameters
    const float fx, fy, cx, cy, invfx, invfy, mbf, mb, mThDepth;

    // Number of KeyPoints
    const int N;

    // KeyPoints, stereo coordinate and descriptors (all associated by an index)
    const std::vector<cv::KeyPoint> mvKeys;
    const std::vector<cv::KeyPoint> mvKeysUn;
    const std::vector<float> mvuRight; // negative value for monocular points
    const std::vector<float> mvDepth; // negative value for monocular points
    const cv::Mat mDescriptors;

    //BoW
    DBoW2::BowVector mBowVec;
    DBoW2::FeatureVector mFeatVec;

    // Pose relative to parent (this is computed when bad flag is activated)
    cv::Mat mTcp;

    // Scale
    const int mnScaleLevels;
    const float mfScaleFactor;
    const float mfLogScaleFactor;
    const std::vector<float> mvScaleFactors;
    const std::vector<float> mvLevelSigma2;
    const std::vector<float> mvInvLevelSigma2;

    // Image bounds and calibration
    const int mnMinX;
    const int mnMinY;
    const int mnMaxX;
    const int mnMaxY;
    const cv::Mat mK;


    // The following variables need to be accessed trough a mutex to be thread safe.
protected:

    // SE3 Pose and camera center
    cv::Mat Tcw;
    cv::Mat Twc;
    cv::Mat Ow;

    cv::Mat Cw; // Stereo middel point. Only for visualization

    // MapPoints associated to keypoints
    std::vector<MapPoint*> mvpMapPoints;

    // BoW
    KeyFrameDatabase* mpKeyFrameDB = NULL;
    ORBVocabulary* mpORBvocabulary = NULL;

    // Grid over the image to speed up feature matching
    std::vector< std::vector <std::vector<size_t> > > mGrid;

    std::map<KeyFrame*,int> mConnectedKeyFrameWeights;
    std::vector<KeyFrame*> mvpOrderedConnectedKeyFrames;
    std::vector<int> mvOrderedWeights;

    // Spanning Tree and Loop Edges
    bool mbFirstConnection = false;
    KeyFrame* mpParent = NULL;
    std::set<KeyFrame*> mspChildrens;
    std::set<KeyFrame*> mspLoopEdges;

    // Bad flags
    bool mbNotErase = false;
    bool mbToBeErased = false;
    bool mbBad = false;

    float mHalfBaseline = 0; // Only for visualization

    Map* mpMap = NULL;

    std::mutex mMutexPose;
    std::mutex mMutexConnections;
    std::mutex mMutexFeatures;

private:
    // Boost serialization
    friend class boost::serialization::access;

    template<class Archive>
    void serialize(Archive & ar, const unsigned int version)
    {
        ar & nNextId;
        ar & mnId;
        ar & const_cast<long unsigned int &>(mnFrameId);
        ar & const_cast<double &>(mTimeStamp);

        ar & const_cast<int &>(mnGridCols);
        ar & const_cast<int &>(mnGridRows);
        ar & const_cast<float &>(mfGridElementWidthInv);
        ar & const_cast<float &>(mfGridElementHeightInv);

        ar & mnTrackReferenceForFrame;
        ar & mnFuseTargetForKF;

        ar & mnBALocalForKF;
        ar & mnBAFixedForKF;

        ar & mnLoopQuery;
        ar & mnLoopWords;
        ar & mLoopScore;
        ar & mnRelocQuery;
        ar & mnRelocWords;
        ar & mRelocScore;

        ar & mTcwGBA;
        ar & mTcwBefGBA;
        ar & mnBAGlobalForKF;

        ar & const_cast<float &>(fx);
        ar & const_cast<float &>(fy);
        ar & const_cast<float &>(cx);
        ar & const_cast<float &>(cy);
        ar & const_cast<float &>(invfx);
        ar & const_cast<float &>(invfy);
        ar & const_cast<float &>(mbf);
        ar & const_cast<float &>(mb);
        ar & const_cast<float &>(mThDepth);

        ar & const_cast<int &>(N);

        ar & const_cast<std::vector<cv::KeyPoint> &>(mvKeys);
        ar & const_cast<std::vector<cv::KeyPoint> &>(mvKeysUn);
        ar & const_cast<std::vector<float> &>(mvuRight);
        ar & const_cast<std::vector<float> &>(mvDepth);
        ar & const_cast<cv::Mat &>(mDescriptors);

        ar & mBowVec;
        ar & mFeatVec;

        ar & mTcp;

        ar & const_cast<int &>(mnScaleLevels);
        ar & const_cast<float &>(mfScaleFactor);
        ar & const_cast<float &>(mfLogScaleFactor);
        ar & const_cast<std::vector<float> &>(mvScaleFactors);
        ar & const_cast<std::vector<float> &>(mvLevelSigma2);
        ar & const_cast<std::vector<float> &>(mvInvLevelSigma2);

        ar & const_cast<int &>(mnMinX);
        ar & const_cast<int &>(mnMinY);
        ar & const_cast<int &>(mnMaxX);
        ar & const_cast<int &>(mnMaxY);
        ar & const_cast<cv::Mat &>(mK);

        ar & Tcw;
        ar & Twc;
        ar & Ow;

        ar & Cw;

        ar & mvpMapPoints;

        //ar & mpKeyFrameDB; //TODO: is not serialized in git version //Probably not necessary here
        //ar & mpORBvocabulary; //TODO: is not serialized in git version //Probably not necessary here???

        ar & mGrid;

        ar & mConnectedKeyFrameWeights;
        ar & mvpOrderedConnectedKeyFrames;
        ar & mvOrderedWeights;

        ar & mbFirstConnection;
        ar & mpParent;
        ar & mspChildrens;
        ar & mspLoopEdges;

        ar & mbNotErase;
        ar & mbToBeErased;
        ar & mbBad;

        ar & mHalfBaseline;

        ar & mpMap;
    }
};

} //namespace ORB_SLAM

#endif // KEYFRAME_H
