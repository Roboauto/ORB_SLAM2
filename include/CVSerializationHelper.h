/**
 * Created by Matous Kolarik on 09/14/16.
 */

#ifndef ORB_SLAM2_CVSERIALIZATIONHELPER_H
#define ORB_SLAM2_CVSERIALIZATIONHELPER_H

#include <opencv2/core/core.hpp>
#include <boost/serialization/split_free.hpp>
#include <boost/serialization/vector.hpp>

BOOST_SERIALIZATION_SPLIT_FREE(cv::Mat)
namespace boost {
    namespace serialization {

        /**
         * Serialization support for cv::Mat
         */
        template<class Archive>
        void save(Archive &ar, const cv::Mat &m, const unsigned int version) {
            size_t elem_size = m.elemSize();
            size_t elem_type = m.type();

            ar & m.cols;
            ar & m.rows;
            ar & elem_size;
            ar & elem_type;

            const size_t data_size = m.cols * m.rows * elem_size;
            ar & boost::serialization::make_array(m.ptr(), data_size);
        }

        template<class Archive>
        void load(Archive &ar, cv::Mat &m, const unsigned int version) {
            int cols, rows;
            size_t elem_size, elem_type;

            ar & cols;
            ar & rows;
            ar & elem_size;
            ar & elem_type;

            m.create(rows, cols, elem_type);

            size_t data_size = m.cols * m.rows * elem_size;
            ar & boost::serialization::make_array(m.ptr(), data_size);
        }
    }
}

BOOST_SERIALIZATION_SPLIT_FREE(cv::KeyPoint)
namespace boost {
    namespace serialization {
        /**
         * Serialization support for cv::KeyPoint
         */
        template<class Archive>
        void save(Archive &ar, const cv::KeyPoint &k, const unsigned int version) {
            ar & k.size;
            ar & k.angle;
            ar & k.response;
            ar & k.octave;
            ar & k.class_id;
            ar & k.pt.x;
            ar & k.pt.y;
        }

        template<class Archive>
        void load(Archive &ar, cv::KeyPoint &k, const unsigned int version) {
            ar & k.size;
            ar & k.angle;
            ar & k.response;
            ar & k.octave;
            ar & k.class_id;
            ar & k.pt.x;
            ar & k.pt.y;
        }
    }
}

//bool equal    Mats(cv::Mat mat1, cv::Mat mat2) {
//    return true;
//}

#endif //ORB_SLAM2_CVSERIALIZATIONHELPER_H
