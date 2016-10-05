/**
 * Created by Matous Kolarik on 10/5/16.
 */

#ifndef ORB_SLAM2_BOWSERIALIZATIONHELPER_H
#define ORB_SLAM2_BOWSERIALIZATIONHELPER_H

#include <boost/serialization/split_free.hpp>
#include <boost/serialization/vector.hpp>
#include "DBoW2/BowVector.h"
#include "DBoW2/FeatureVector.h"

BOOST_SERIALIZATION_SPLIT_FREE(DBoW2::BowVector)
namespace boost {
    namespace serialization {

        /**
         * Serialization support for DBoW2::BowVector
         */
        template<class Archive>
        void save(Archive &ar, const DBoW2::BowVector &m, const unsigned int version) {
            DBoW2::WordId key;
            DBoW2::WordValue value;
            int itemsCount = m.size();

            ar & itemsCount;

            for (DBoW2::BowVector::const_iterator it = m.begin(); it != m.end(); ++it) {
                key = it->first;
                ar & key;
                value = it->second;
                ar & value;
            }
        }

        template<class Archive>
        void load(Archive &ar, DBoW2::BowVector &m, const unsigned int version) {
            DBoW2::WordId key;
            DBoW2::WordValue value;
            int itemsCount;

            ar & itemsCount;
            m.clear();
            for (int i = 0; i < itemsCount; ++i) {
                ar & key;
                ar & value;
                m.addIfNotExist(key, value);
            }
        }
    }
}


BOOST_SERIALIZATION_SPLIT_FREE(DBoW2::FeatureVector)
namespace boost {
    namespace serialization {

        /**
         * Serialization support for DBoW2::FeatureVector
         */
        template<class Archive>
        void save(Archive &ar, const DBoW2::FeatureVector &m, const unsigned int version) {
                DBoW2::NodeId key;
                unsigned int value;
                int itemsCount = m.size();

                ar & itemsCount;

                for (DBoW2::FeatureVector::const_iterator it = m.begin(); it != m.end(); ++it) {
                    itemsCount = it->second.size();
                    ar & itemsCount;
                    key = it->first;
                    ar & key;
                    for (int i = 0; i < itemsCount; i++) {
                        value = it->second[i];
                        ar & value;
                    }
                }
        }

        template<class Archive>
        void load(Archive &ar, DBoW2::FeatureVector &m, const unsigned int version) {
                DBoW2::NodeId key;
                unsigned int value;
                int itemsCount, itemsCount2;

                ar & itemsCount;
                m.clear();
                for (int i = 0; i < itemsCount; ++i) {
                    ar & itemsCount2;
                    ar & key;
                    for (int j = 0; j < itemsCount2; j++) {
                        ar & value;
                        m.addFeature(key, value);
                    }
                }
        }
    }
}
#endif //ORB_SLAM2_BOWSERIALIZATIONHELPER_H
