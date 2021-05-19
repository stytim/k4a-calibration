#pragma once

#include <cstdint>
#include <vector>

#include <Eigen/Eigen>

#include <apriltag.h>
#include <tagStandard41h12.h>
#include <apriltag_pose.h>
#include <opencv2/aruco/charuco.hpp>

#include <cereal/archives/json.hpp>
#include <open3d/geometry/PointCloud.h>

namespace cereal {
    template<class Archive>
    void serialize(Archive & archive,
                    Eigen::Quaternionf& m)
    {
        archive( cereal::make_nvp("x", m.x()), cereal::make_nvp("y", m.y()),
                    cereal::make_nvp("z", m.z()), cereal::make_nvp("w", m.w()) );
    };

    template<class Archive>
    void serialize(Archive & archive,
                    Eigen::Vector3f& t)
    {
        archive( cereal::make_nvp("m00", t.x()), cereal::make_nvp("m10", t.y()),
                    cereal::make_nvp("m20", t.z()) );
    };
}

struct  CameraExtrinsics
{
    Eigen::Vector3f translation;
    Eigen::Quaternionf rotation;

    template <class Archive>
    void serialize( Archive & ar )
    {
        ar(cereal::make_nvp("translation", translation), cereal::make_nvp("rotation", rotation));
    }

};

struct CameraIntrinsics
{
    // Sensor resolution
    int32_t Width, Height;

    // Intrinsics
    float cx, cy;
    float fx, fy;
    float k[6];
    float codx, cody;
    float p1, p2;
};

struct CameraCalibration
{
    // Intrinsics for each camera
    CameraIntrinsics Color, Depth;

    // Extrinsics transform from 3D depth camera point to 3D point relative to color camera
    float RotationFromDepth[3*3];
    float TranslationFromDepth[3];
};

//------------------------------------------------------------------------------
// Registration

struct FrameInfo
{
    CameraCalibration Calibration{};

    // Accelerometer reading for extrinsics calibration
    float Accelerometer[3];

    // Color image
    std::vector<uint8_t> ColorImage;
    int ColorWidth = 0, ColorHeight = 0, ColorStride = 0;

    // Depth image
    std::vector<uint16_t> DepthImage;
    int DepthWidth = 0, DepthHeight = 0, DepthStride = 0;

    // Point cloud data
    std::vector<int16_t> PointCloudData;

    uint32_t CameraIndex;
    int FrameNumber;
    char *filename;
};


struct AlignmentTransform
{
    float Transform[16];
    bool Identity = true;
    inline void operator=(const Eigen::Matrix4f& src)
    {
        Identity = src.isIdentity();
        for (int row = 0; row < 4; ++row) {
            for (int col = 0; col < 4; ++col) {
                Transform[row * 4 + col] = src(row, col);
            }
        }
    }
    inline void Set(Eigen::Matrix4f& dest) const
    {
        if (Identity) {
            dest = Eigen::Matrix4f::Identity();
        } else {
            for (int row = 0; row < 4; ++row) {
                for (int col = 0; col < 4; ++col) {
                    dest(row, col) = Transform[row * 4 + col];
                }
            }
        }
    }
};

class ExtrinsicsCalibration
{
    public:

        apriltag_family_t *tf;
        apriltag_detector_t* td;
        cv::Ptr<cv::aruco::Dictionary> dictionary;
        cv::Ptr<cv::aruco::CharucoBoard> board;
        cv::Ptr<cv::aruco::DetectorParameters> params;

        ExtrinsicsCalibration()
        {
            tf = tagStandard41h12_create();
            td = apriltag_detector_create();
            apriltag_detector_add_family_bits(td, tf, 1);
            td->quad_decimate = 1.f;
            td->quad_sigma = 0.8f;
            td->nthreads = 8;
            td->refine_edges = 1;
            td->decode_sharpening = 0.25;

            dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);
            board = cv::aruco::CharucoBoard::create(5, 7, 0.04f, 0.02f, dictionary);
            params = cv::aruco::DetectorParameters::create();
        }

        ~ExtrinsicsCalibration()
        {
            tagStandard41h12_destroy(tf);
            apriltag_detector_destroy(td);
        }

        bool CalculateExtrinsics(
            const std::vector<FrameInfo>& frames,
            std::vector<AlignmentTransform>& extrinsics);

        bool RefineExtrinsics(
            const std::vector<FrameInfo>& frames,
            std::vector<AlignmentTransform>& extrinsics);
    private:
        std::vector<std::shared_ptr<open3d::geometry::PointCloud>> full_cloud;
        bool GenerateFullCloudFromFrames(FrameInfo frame);
        bool DownSample(std::shared_ptr<open3d::geometry::PointCloud>& cloud, const double voxel_size, const int idx);


};
