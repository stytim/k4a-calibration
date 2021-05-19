#include "camera_extrinsics.hpp"

#include <fstream>
#include <experimental/filesystem>

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/calib3d.hpp>


#include <open3d/pipelines/registration/Registration.h>
#include <open3d/pipelines/registration/ColoredICP.h>
#include <open3d/utility/Console.h>
#include <open3d/io/PointCloudIO.h>


namespace fs = std::experimental::filesystem;
using namespace std;
//------------------------------------------------------------------------------
// Registration
bool ExtrinsicsCalibration::GenerateFullCloudFromFrames(
    FrameInfo frame)
{
    const int idx = frame.CameraIndex;
    if (!frame.PointCloudData.data()) {
        return false;
    }
    cout <<"Generating Cloud From Frames"<< endl;
    full_cloud[idx] = std::make_shared<open3d::geometry::PointCloud>();
    const int count = frame.ColorWidth * frame.ColorHeight;
    const int coord_stride = 1;

    full_cloud[idx]->points_.reserve(count);
    full_cloud[idx]->colors_.reserve(count);
    for (int i = 0; i < count; i += coord_stride) {
        Eigen::Vector3d q(frame.PointCloudData.data()[3 * i + 0] / 1000.0f, frame.PointCloudData.data()[3 * i + 1] / 1000.0f, frame.PointCloudData.data()[3 * i + 2] / 1000.0f);
        if (q.z() == 0)
        {
            continue;
        }
        // BGR -> RGB
        Eigen::Vector3d color{};
        color.z() = (double)frame.ColorImage.data()[4 * i + 0] / 255.0;
        color.y() = (double)frame.ColorImage.data()[4 * i + 1] / 255.0;
        color.x() = (double)frame.ColorImage.data()[4 * i + 2] / 255.0;

        if (color.x() == 0 && color.y() == 0 && color.z() == 0)
        {
            continue;
        }
        full_cloud[idx]->points_.push_back(q);
        full_cloud[idx]->colors_.push_back(color);
    }

    cout <<"Constructed Open3D PointCloud for Camera ID: "<< idx <<endl;

    return true;
}

bool ExtrinsicsCalibration::DownSample(std::shared_ptr<open3d::geometry::PointCloud>& cloud, const double voxel_size, const int idx)
{
     // Downsample the point cloud
    cloud = full_cloud[idx]->VoxelDownSample(voxel_size);;
    if (!cloud) {
        cout  <<"VoxelDownSample failed"<< endl;
        return false;
    }
    // Estimate normals with full resolution point cloud
    const double normal_radius = voxel_size * 2.0;
    open3d::geometry::KDTreeSearchParamHybrid normals_params(normal_radius, 30);
    const bool fast_normal_computation = true;
    cloud->EstimateNormals(normals_params, fast_normal_computation);

    // Incorporate the assumption that normals should be pointed towards the camera
    cloud->OrientNormalsTowardsCameraLocation(Eigen::Vector3d(0, 0, 0));

    return true;

}

bool ExtrinsicsCalibration::CalculateExtrinsics(
    const std::vector<FrameInfo>& frames,
    std::vector<AlignmentTransform>& output)
{
    output.clear();
    full_cloud.clear();
    if (frames.empty()) {
        cout  <<"No images provided to registration"<< endl;
        return false;
    }
    open3d::utility::SetVerbosityLevel(open3d::utility::VerbosityLevel::Debug);

    const int camera_count = static_cast<int>( frames.size() );
    output.resize(camera_count);
    full_cloud.resize(camera_count);
  
    // Estimate camera poses from AprilTag or ChArUco board:
    std::vector<Eigen::Matrix4f> tag_poses(camera_count);
    std::vector<Eigen::Matrix4d> current_transform(camera_count);
    float l = 0.38f;
    for (int camera_index = 0; camera_index < camera_count; ++camera_index)
    {
        cv::Mat gray;
        cv::Mat image(frames[camera_index].ColorHeight, frames[camera_index].ColorWidth, CV_8UC4,
                                                        (void*)frames[camera_index].ColorImage.data(),
                                                        cv::Mat::AUTO_STEP);
        cv::cvtColor(image, gray, cv::COLOR_BGRA2GRAY);
        
        image_u8_t orig { .width = gray.cols,
            .height = gray.rows,
            .stride = gray.cols,
            .buf = gray.data
        };
        
        zarray_t* detections = apriltag_detector_detect(td, &orig);
        cout <<"Detected " << zarray_size(detections) << " fiducial markers" << endl;
        bool found = false;
        CameraCalibration calibration = frames[camera_index].Calibration;
      
        for (int i = 0; i < zarray_size(detections); i++)
        {
            apriltag_detection_t *det;
            zarray_get(detections, i, &det);
            
            cout <<"Camera "<< camera_index << " detected marker ID : " << det->id << endl;
            if (det->id != 0) {
                cout <<"Camera "<< camera_index <<" detected incorrect marker #"<< det->id <<endl;;
                continue;
            }

            apriltag_detection_info_t info;
            info.det = det;
            info.cx = calibration.Color.cx; // pixels
            info.cy = calibration.Color.cy;
            info.fx = calibration.Color.fx; // mm
            info.fy = calibration.Color.fy;
            info.tagsize = 0.22f; // in meters

            apriltag_pose_t pose;
            double err = estimate_tag_pose(&info, &pose);

            const double* tr = &pose.R->data[0];
            const double* tt = &pose.t->data[0];

            cout <<"Object-space error = " << err << endl;
            cout <<"R = [ " << tr[0] << ", " << tr[1] << ", " << tr[2] << endl;
            cout <<"      " << tr[3] << ", " << tr[4] << ", " << tr[5] << endl;
            cout <<"      " << tr[6] << ", " << tr[7] << ", " << tr[8] << " ]" << endl;
            cout <<"t = [ " << tt[0] << ", " << tt[1] << ", " << tt[2] << " ]" << endl;

            Eigen::Matrix4f transform;
            Eigen::Matrix3f rotmat;
            for (int row = 0; row < 3; ++row) {
                for (int col = 0; col < 3; ++col) {
                    transform(row, col) = static_cast<float>( tr[row * 3 + col] );
                    rotmat(row, col) = static_cast<float>( tr[row * 3 + col] );
                }
            }
            for (int row = 0; row < 3; ++row) {
                transform(row, 3) = static_cast<float>( tt[row] );
            }
            for (int col = 0; col < 3; ++col) {
                transform(3, col) = 0.f;
            }
            transform(3, 3) = 1.f;
/*
            Eigen::Matrix4f cube_transform = Eigen::Matrix4f::Identity();
            switch (det->id)
            {
          
            case 0: cube_transform << 0, 1, 0, 0,
                                        1, 0, 0, 0,
                                        0, 0, -1, l/2,
                                        0, 0, 0, 1;
                break;
            case 1: cube_transform << 0, 0, -1, l/2,
                                      1, 0, 0, 0,
                                      0, -1, 0, 0,
                                      0, 0, 0, 1;
                break;
            case 2: cube_transform << -1, 0, 0, 0,
                                       0, 0, -1, l/2,
                                       0, -1, 0, 0,
                                       0, 0, 0, 1;

                break;
            case 3: cube_transform << 0, 0, 1, -l/2,
                                        -1, 0, 0, 0,
                                        0, -1, 0, 0,
                                        0, 0, 0, 1;
                break;
            case 4: cube_transform << 1, 0, 0, 0,
                                        0, 0, 1, -l/2,
                                        0, -1, 0, 0,
                                        0, 0, 0, 1;
                break;
            default:
                break;
            }
*/           
            tag_poses[camera_index] = transform; // * cube_transform.inverse();
            const Eigen::Matrix4f tag_pose = tag_poses[0] * tag_poses[camera_index].inverse();
            current_transform[camera_index] = tag_pose.cast<double>();
            found = true;
        }

        // Try ChArUco board
        if (!found) {
            cout << "No AprilTag detected, trying ChArUco board" << endl;
            std::vector<int> markerIds;
            std::vector<std::vector<cv::Point2f> > markerCorners;
            cv::aruco::detectMarkers(gray, board->dictionary, markerCorners, markerIds, params);
            if (markerIds.size() > 0) {
                std::vector<cv::Point2f> charucoCorners;
                std::vector<int> charucoIds;
                cv::Mat camMatrix = (cv::Mat_<float>(3,3) << calibration.Color.fx, 0, calibration.Color.cx, 
                                                            0, calibration.Color.fy, calibration.Color.cy,
                                                            0, 0, 1);
                cv::Mat distCoeffs = (cv::Mat_<float>(8,1) << calibration.Color.k[0],calibration.Color.k[1], calibration.Color.p1, calibration.Color.p2,
                                                            calibration.Color.k[2],calibration.Color.k[3],calibration.Color.k[4],calibration.Color.k[5]);
                cv::aruco::interpolateCornersCharuco(markerCorners, markerIds, gray, board, charucoCorners, charucoIds, camMatrix, distCoeffs);
                // if at least one charuco corner detected
                if (charucoIds.size() > 0) {
                    cv::Vec3d rvec, tvec;
                    cout << "Detected ChArUco board with " << charucoIds.size() << " corners" << endl;
                    bool valid = cv::aruco::estimatePoseCharucoBoard(charucoCorners, charucoIds, board, camMatrix, distCoeffs, rvec, tvec);
                    if (valid) {
                        cv::Mat rmat;
                        cv::Rodrigues(rvec, rmat);
                        Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();

                        for (int row = 0; row < 3; ++row) {
                            for (int col = 0; col < 3; ++col) {
                                transform(row, col) = rmat.at<double>(row,col);
                            }
                        }
                        for (int row = 0; row < 3; ++row) {
                            transform(row, 3) = static_cast<float>( tvec[row] );
                        }
                        cout << "Pose: " << endl;
                        cout << transform << endl;
                        tag_poses[camera_index] = transform;
                        const Eigen::Matrix4f tag_pose = tag_poses[0] * tag_poses[camera_index].inverse();
                        current_transform[camera_index] = tag_pose.cast<double>();
                        found = true;
                    }
                }

            }

        }

        // cv::imshow(to_string(camera_index), image);
        // if (cv::waitKey(30) >= 0)
        //    break;

        if (!found) {
            cout  <<"Camera "<< camera_index << " did not observe the fiducial marker - Waiting for the next frame" << endl;
            return false;
        }
    }

    cout <<"All cameras observed the fiducial marker"<< endl;

    // Calculate scene yaw relative to marker:

#ifndef M_PI_FLOAT
# define M_PI_FLOAT 3.14159265f
#endif

    auto& pose0 = tag_poses[0];
    Eigen::Vector3f euler0 = pose0.block<3, 3>(0, 0).eulerAngles(0, 1, 2);
    float yaw = euler0.z();
    cout <<"Detected marker yaw = " << yaw * 180.f / M_PI_FLOAT << "degrees" << endl;
    Eigen::AngleAxis<float> yaw_rot(-yaw, Eigen::Vector3f(0.f, 1.f, 0.f));
    Eigen::Matrix4f yaw_transform = Eigen::Affine3f(yaw_rot).matrix();

    // Center scene on marker:
    Eigen::Vector3f marker_offset_0(pose0(0, 3), pose0(1, 3), pose0(2, 3));

    // Correct camera tilt based on accelerometer:
    Eigen::Matrix4f tilt_transform = Eigen::Matrix4f::Identity();

    // Use first camera as reference
    auto& accel = frames[0].Accelerometer;
    if (accel[0] == 0.f && accel[1] == 0.f && accel[2] == 0.f) {
        cout  <<"IMU acceleration reading not available for tilt correction"<< endl;
    }
    else
    {
        cout <<"Correcting tilt of primary camera using gravity down-vector [ " <<  accel[0] << ", "<< accel[1] << ", "<<  accel[2] << " ]" << endl;

        // Accelerometer frame: (x, y, z) = (+forward, +right, +up)
        // Pointcloud frame:    (x, y, z) = (+right,   -up,    +forward)
        Eigen::Quaternionf q;
        q.setFromTwoVectors(
            Eigen::Vector3f(accel[1], accel[2], accel[0]),
            Eigen::Vector3f(0.f, -1.f, 0.f));
        Eigen::Matrix3f tilt_r = q.toRotationMatrix();
        tilt_transform.block<3, 3>(0, 0) = tilt_r.transpose();

        marker_offset_0 = tilt_r.inverse() * marker_offset_0;
    }

    Eigen::Translation3f translation(-marker_offset_0);
    Eigen::Matrix4f translation_transform = Eigen::Affine3f(translation).matrix();

    const Eigen::Matrix4f center_transform =  yaw_transform* translation_transform* tilt_transform;

    cout <<"===========================================================" << endl;
    cout <<"!!! Starting extrinsics calibration for " << camera_count << " cameras..." << endl;

    output[0] = center_transform;
    for (int camera_index = 0; camera_index < camera_count; ++camera_index)
    {
        if (!GenerateFullCloudFromFrames(frames[camera_index])) {
            cout  <<"GenerateCloudFromFrames failed for i = "<< camera_index <<endl;
            return false;
        }
        std::shared_ptr<open3d::geometry::PointCloud> cloud_i;
        if (!DownSample(cloud_i, 0.01, camera_index)) {
            cout  <<"DownSample failed for i = " << camera_index << endl;
            return false;
        }
        cloud_i->Transform(center_transform.cast<double>() * current_transform[camera_index]);
        string filename = "cloud_" + to_string(camera_index) + ".ply";
        open3d::io::WritePointCloudToPLY(filename, *cloud_i, true);
    }

#if 1
    // Muti-stage Colored ICP
    std::vector<double> voxel_radius{0.04, 0.02, 0.01};
    std::vector<int> max_iter{50, 30, 14};
    for(int stage = 0; stage < 3; ++stage)
    {
        const double radius = voxel_radius[stage];
        const int iter = max_iter[stage];
        cout <<"voxel radius: "<< voxel_radius[stage]<< " Max iterations: "<<max_iter[stage] <<endl;
    
        std::shared_ptr<open3d::geometry::PointCloud> cloud_0;
        if (!DownSample(cloud_0, radius, 0)) {
            cout  <<"DownSample failed for i=0"<< endl;
            return false;
        }

        for (int camera_index = 1; camera_index < camera_count; ++camera_index)
        {
            std::shared_ptr<open3d::geometry::PointCloud> cloud_i;
            if (!DownSample(cloud_i, radius, camera_index)) {
                cout  <<"DownSample failed for i = " << camera_index << endl;
                return false;
            }

            open3d::pipelines::registration::ICPConvergenceCriteria criteria(1e-6, 1e-6, iter);

            // How much it tends towards using the geometry instead of the color
            const double lambda_geometric = 0.968;
            open3d::pipelines::registration::TransformationEstimationForColoredICP transform_estimate(lambda_geometric);

            auto result = open3d::pipelines::registration::RegistrationColoredICP(
            *cloud_i,
            *cloud_0,
            voxel_radius[stage],
            current_transform[camera_index],
            transform_estimate,
            criteria);
            
            current_transform[camera_index] = result.transformation_.cast<double>();

            cout <<"==========================================================="<< endl;
            cout <<"Color ICP refinement for "<< camera_index <<" -> 0" <<endl;;

            // cloud_i->Transform(current_transform[camera_index]);
            // open3d::visualization::DrawGeometries({cloud_0, cloud_i}, "icp", 1600,900);

            output[camera_index] = center_transform * current_transform[camera_index].cast<float>();
        } 
    }
#endif
    // Save to PLY and JSON after ICP
    for (int camera_index = 0; camera_index < camera_count; ++camera_index)
    {
        std::shared_ptr<open3d::geometry::PointCloud> cloud_i;
        if (!DownSample(cloud_i, 0.01, camera_index)) {
            cout  <<"DownSample failed for i = " << camera_index << endl;
            return false;
        }

        const Eigen::Matrix4f cloud_transform = center_transform * current_transform[camera_index].cast<float>();
        cloud_i->Transform(cloud_transform.cast<double>());
        string filename = "icp_cloud_" + to_string(camera_index) + ".ply";
        open3d::io::WritePointCloudToPLY(filename, *cloud_i, true);

        cout <<"Point cloud saved"<< endl;

        Eigen::Affine3f t = Eigen::Affine3f::Identity();
        t.matrix() = cloud_transform;

        // Transform to Depth camera coordinate
        CameraCalibration calibration = frames[camera_index].Calibration;
        Eigen::Matrix3f r_depth = Eigen::Map<Eigen::Matrix3f>(calibration.RotationFromDepth, 3, 3).transpose(); // row major
        Eigen::Vector3f t_depth = Eigen::Map<Eigen::Vector3f>(calibration.TranslationFromDepth, 3, 1) / 1000.0f; // in meters
        t.rotate(r_depth);
        t.translate(t_depth);

        // Flip y and z for OpenGL coordinate system
        Eigen::Matrix4f yz_transform = Eigen::Matrix4f::Identity();
        yz_transform(1,1) = -1.0;
        yz_transform(2,2) = -1.0;
        CameraExtrinsics extrinsicsmat;
        t.matrix() = (yz_transform * t.matrix() * yz_transform);
        extrinsicsmat.translation = t.translation();
        extrinsicsmat.rotation = Eigen::Quaternionf(t.rotation());

        // Save JSON file
        std::ofstream file;
        fs::path path = fs::path(frames[camera_index].filename).parent_path();
        path /= "cn0" + to_string(camera_count - camera_index) + ".json";
        file.open(path, std::ofstream::out);
        {
            cereal::JSONOutputArchive archive( file );
            archive(extrinsicsmat);
        }
        file.flush();
        file.close();
    }

    cout <<"==========================================================="<< endl;
    
    return true;
}

bool ExtrinsicsCalibration::RefineExtrinsics(
    const std::vector<FrameInfo>& frames,
    std::vector<AlignmentTransform>& extrinsics)
{
    if (extrinsics.size() != frames.size()) {
        cout  <<"Invalid input"<< endl;
        return false;
    }

    const int camera_count = static_cast<int>( frames.size() );

    cout <<"==========================================================="<< endl;
    cout <<"!!! Starting extrinsics calibration for " << camera_count << " cameras" << endl;

    Eigen::Matrix4f center_transform;
    extrinsics[0].Set(center_transform);

    Eigen::Matrix4f inv_center_transform = center_transform.inverse();

    std::shared_ptr<open3d::geometry::PointCloud> cloud_0;
    if (!GenerateFullCloudFromFrames(frames[0])) {
        cout  <<"GenerateCloudFromFrames failed for i=0"<< endl;
        return false;
    }

    for (int camera_index = 1; camera_index < camera_count; ++camera_index)
    {

        std::shared_ptr<open3d::geometry::PointCloud> cloud_i;
        if (!GenerateFullCloudFromFrames(frames[camera_index])) {
            cout  <<"GenerateCloudFromFrames failed for i= " << camera_index <<endl;
            return false;
        }

        cout <<"==========================================================="<< endl;

        const double max_distance = 0.01; // meters
        open3d::pipelines::registration::ICPConvergenceCriteria criteria(1e-16, 1e-16, 500);

        // How much it tends towards using the geometry instead of the color
        const double lambda_geometric = 0.96;
        open3d::pipelines::registration::TransformationEstimationForColoredICP transform_estimate(lambda_geometric);

        Eigen::Matrix4f transform_i;
        extrinsics[camera_index].Set(transform_i);

        // Left multiply to undo the "center transform" from full registration,
        // leaving just the prior transform from cloud_i to cloud_0
        Eigen::Matrix4f current_transform_f = inv_center_transform * transform_i;
        Eigen::Matrix4d current_transform_d = current_transform_f.cast<double>();

        auto result = open3d::pipelines::registration::RegistrationColoredICP(
            *cloud_i,
            *cloud_0,
            max_distance,
            current_transform_d,
            transform_estimate,
            criteria);

        cout <<"==========================================================="<< endl;

        auto transform4x4 = result.transformation_.cast<float>();

        extrinsics[camera_index] = center_transform * transform4x4;
    } // next image

    return true;
}
