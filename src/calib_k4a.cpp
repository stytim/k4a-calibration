#include <stdio.h>
#include <malloc.h>
#include <iostream>
#include <k4a/k4a.hpp>
#include <k4arecord/playback.hpp>
#include "calib_k4a.hpp"

using namespace std;

static void print_calibration(k4a_calibration_t& calibration)
    {
        {
            cout << "Depth camera:" << endl;
            auto calib = calibration.depth_camera_calibration;

            cout << "resolution width: " << calib.resolution_width << endl;
            cout << "resolution height: " << calib.resolution_height << endl;
            cout << "principal point x: " << calib.intrinsics.parameters.param.cx << endl;
            cout << "principal point y: " << calib.intrinsics.parameters.param.cy << endl;
            cout << "focal length x: " << calib.intrinsics.parameters.param.fx << endl;
            cout << "focal length y: " << calib.intrinsics.parameters.param.fy << endl;
            cout << "radial distortion coefficients:" << endl;
            cout << "k1: " << calib.intrinsics.parameters.param.k1 << endl;
            cout << "k2: " << calib.intrinsics.parameters.param.k2 << endl;
            cout << "k3: " << calib.intrinsics.parameters.param.k3 << endl;
            cout << "k4: " << calib.intrinsics.parameters.param.k4 << endl;
            cout << "k5: " << calib.intrinsics.parameters.param.k5 << endl;
            cout << "k6: " << calib.intrinsics.parameters.param.k6 << endl;
            cout << "center of distortion in Z=1 plane, x: " << calib.intrinsics.parameters.param.codx << endl;
            cout << "center of distortion in Z=1 plane, y: " << calib.intrinsics.parameters.param.cody << endl;
            cout << "tangential distortion coefficient x: " << calib.intrinsics.parameters.param.p1 << endl;
            cout << "tangential distortion coefficient y: " << calib.intrinsics.parameters.param.p2 << endl;
            cout << "metric radius: " << calib.intrinsics.parameters.param.metric_radius << endl;
        }

        {
            cout << "Color camera:" << endl;
            auto calib = calibration.color_camera_calibration;

            cout << "resolution width: " << calib.resolution_width << endl;
            cout << "resolution height: " << calib.resolution_height << endl;
            cout << "principal point x: " << calib.intrinsics.parameters.param.cx << endl;
            cout << "principal point y: " << calib.intrinsics.parameters.param.cy << endl;
            cout << "focal length x: " << calib.intrinsics.parameters.param.fx << endl;
            cout << "focal length y: " << calib.intrinsics.parameters.param.fy << endl;
            cout << "radial distortion coefficients:" << endl;
            cout << "k1: " << calib.intrinsics.parameters.param.k1 << endl;
            cout << "k2: " << calib.intrinsics.parameters.param.k2 << endl;
            cout << "k3: " << calib.intrinsics.parameters.param.k3 << endl;
            cout << "k4: " << calib.intrinsics.parameters.param.k4 << endl;
            cout << "k5: " << calib.intrinsics.parameters.param.k5 << endl;
            cout << "k6: " << calib.intrinsics.parameters.param.k6 << endl;
            cout << "center of distortion in Z=1 plane, x: " << calib.intrinsics.parameters.param.codx << endl;
            cout << "center of distortion in Z=1 plane, y: " << calib.intrinsics.parameters.param.cody << endl;
            cout << "tangential distortion coefficient x: " << calib.intrinsics.parameters.param.p1 << endl;
            cout << "tangential distortion coefficient y: " << calib.intrinsics.parameters.param.p2 << endl;
            cout << "metric radius: " << calib.intrinsics.parameters.param.metric_radius << endl;
        }

        auto extrinsics = calibration.extrinsics[K4A_CALIBRATION_TYPE_DEPTH][K4A_CALIBRATION_TYPE_COLOR];
        cout << "depth2color translation: (" << extrinsics.translation[0] << "," << extrinsics.translation[1] << "," << extrinsics.translation[2] << ")" << endl;
        cout << "depth2color rotation: |" << extrinsics.rotation[0] << "," << extrinsics.rotation[1] << "," << extrinsics.rotation[2] << "|" << endl;
        cout << "                      |" << extrinsics.rotation[3] << "," << extrinsics.rotation[4] << "," << extrinsics.rotation[5] << "|" << endl;
        cout << "                      |" << extrinsics.rotation[6] << "," << extrinsics.rotation[7] << "," << extrinsics.rotation[8] << "|" << endl;

    }


typedef struct
{
    char *filename;
    k4a_playback_t handle;
    k4a_record_configuration_t record_config;
    k4a_calibration_t k4a_calibration;
    k4a_transformation_t k4a_transformation;
    CameraCalibration calibration{};
    k4a_capture_t capture;
} recording_t;

static uint64_t first_capture_timestamp(k4a_capture_t capture)
{
    uint64_t min_timestamp = (uint64_t)-1;
    k4a_image_t images[3];
    images[0] = k4a_capture_get_color_image(capture);
    images[1] = k4a_capture_get_depth_image(capture);
    images[2] = k4a_capture_get_ir_image(capture);

    for (int i = 0; i < 3; i++)
    {
        if (images[i] != NULL)
        {
            uint64_t timestamp = k4a_image_get_device_timestamp_usec(images[i]);
            if (timestamp < min_timestamp)
            {
                min_timestamp = timestamp;
            }
            k4a_image_release(images[i]);
            images[i] = NULL;
        }
    }

    return min_timestamp;
}

static FrameInfo process_capture(recording_t *file)
{
    FrameInfo frame;
    k4a_image_t images[2];
    images[0] = k4a_capture_get_color_image(file->capture);
    images[1] = k4a_capture_get_depth_image(file->capture);

    // Copy color
    frame.ColorWidth = k4a_image_get_width_pixels(images[0]);
    frame.ColorHeight = k4a_image_get_height_pixels(images[0]);
    frame.ColorStride = k4a_image_get_stride_bytes(images[0]);
    const uint8_t* color_image = \
        reinterpret_cast<const uint8_t*>( k4a_image_get_buffer(images[0]) );
    const size_t color_size = k4a_image_get_size(images[0]);
    frame.ColorImage.resize(color_size);
    memcpy(frame.ColorImage.data(), color_image, color_size);

    // Copy depth
    frame.DepthWidth = k4a_image_get_width_pixels(images[1]);
    frame.DepthHeight = k4a_image_get_height_pixels(images[1]);
    frame.DepthStride = k4a_image_get_stride_bytes(images[1]);
    const unsigned depth_size = frame.DepthStride * frame.DepthHeight;
    frame.DepthImage.resize(depth_size);
    const uint16_t* depth_image = \
        reinterpret_cast<const uint16_t*>( k4a_image_get_buffer(images[1]) );
    memcpy(frame.DepthImage.data(), depth_image, depth_size * 2);

    k4a_image_t transformed_depth_image = NULL;
    if (K4A_RESULT_SUCCEEDED != k4a_image_create(K4A_IMAGE_FORMAT_DEPTH16,
                                                 frame.ColorWidth,
                                                 frame.ColorHeight,
                                                 frame.ColorWidth * (int)sizeof(uint16_t),
                                                 &transformed_depth_image))
    {
        cout << "Failed to create transformed depth image" <<endl;
    }

    k4a_image_t point_cloud_image = NULL;
    if (K4A_RESULT_SUCCEEDED != k4a_image_create(K4A_IMAGE_FORMAT_CUSTOM,
                                                 frame.ColorWidth,
                                                 frame.ColorHeight,
                                                 frame.ColorWidth * 3 * (int)sizeof(int16_t),
                                                 &point_cloud_image))
    {
        printf("Failed to create point cloud image\n");
    }
    if (K4A_RESULT_SUCCEEDED != k4a_transformation_depth_image_to_color_camera(file->k4a_transformation, images[1], transformed_depth_image))
    {
        cout<< "Failed to compute transformed depth image" << endl;
    }
    if (K4A_RESULT_SUCCEEDED != k4a_transformation_depth_image_to_point_cloud(file->k4a_transformation, transformed_depth_image, K4A_CALIBRATION_TYPE_COLOR, point_cloud_image))
    {
        cout << "Failed to compute point cloud image" << endl;
    }
    
    // Copy point cloud
    const size_t cloud_size = k4a_image_get_size(point_cloud_image);
    const int16_t* point_cloud_image_data = \
        reinterpret_cast<const int16_t*>( k4a_image_get_buffer(point_cloud_image) );
    frame.PointCloudData.resize(cloud_size);
    memcpy(frame.PointCloudData.data(), point_cloud_image_data, cloud_size );

    k4a_image_release(transformed_depth_image);
    k4a_image_release(point_cloud_image);


    printf("%-32s", file->filename);
    for (int i = 0; i < 2; i++)
    {
        if (images[i] != NULL)
        {
            uint64_t timestamp = k4a_image_get_device_timestamp_usec(images[i]);
            printf("  %7ju usec", timestamp);
            k4a_image_release(images[i]);
            images[i] = NULL;
        }
        else
        {
            printf("  %12s", "");
        }
    }
    printf("\n");

    return frame;
}

int main(int argc, char **argv)
{
    if (argc < 3)
    {
        printf("Usage: ./calib_k4a <master.mkv> <sub1.mkv>...\n");
        return 1;
    }

    size_t file_count = (size_t)(argc - 1);
    bool master_found = false;
    k4a_result_t result = K4A_RESULT_SUCCEEDED;

    // Allocate memory to store the state of N recordings.
    recording_t *files = reinterpret_cast<recording_t*>(malloc(sizeof(recording_t) * file_count));
    if (files == NULL)
    {
        printf("Failed to allocate memory for playback (%zu bytes)\n", sizeof(recording_t) * file_count);
        return 1;
    }
    memset(files, 0, sizeof(recording_t) * file_count);

    // Open each recording file and validate they were recorded in master/subordinate mode.
    for (size_t i = 0; i < file_count; i++)
    {
        files[i].filename = argv[i + 1];

        result = k4a_playback_open(files[i].filename, &files[i].handle);
        if (result != K4A_RESULT_SUCCEEDED)
        {
            printf("Failed to open file: %s\n", files[i].filename);
            break;
        }

        result = k4a_playback_get_record_configuration(files[i].handle, &files[i].record_config);
        if (result != K4A_RESULT_SUCCEEDED)
        {
            printf("Failed to get record configuration for file: %s\n", files[i].filename);
            break;
        }

        // assert( files[i].record_config.color_format == k4a_image_format_t::K4A_IMAGE_FORMAT_COLOR_MJPG );

        result = k4a_playback_get_calibration(files[i].handle, &files[i].k4a_calibration);
        if (result != K4A_RESULT_SUCCEEDED)
        {
            printf("Failed to get record configuration for file: %s\n", files[i].filename);
            break;
        }
        print_calibration(files[i].k4a_calibration);
        CalibrationFromK4a(files[i].k4a_calibration, files[i].calibration);
        files[i].k4a_transformation = k4a_transformation_create(&files[i].k4a_calibration);

        if (files[i].record_config.wired_sync_mode == K4A_WIRED_SYNC_MODE_MASTER)
        {
            printf("Opened master recording file: %s\n", files[i].filename);
            if (master_found)
            {
                printf("ERROR: Multiple master recordings listed!\n");
                result = K4A_RESULT_FAILED;
                break;
            }
            else
            {
                master_found = true;
            }
        }
        else if (files[i].record_config.wired_sync_mode == K4A_WIRED_SYNC_MODE_SUBORDINATE)
        {
            printf("Opened subordinate recording file: %s\n", files[i].filename);
        }
        else
        {
            printf("ERROR: Recording file was not recorded in master/sub mode: %s\n", files[i].filename);
            result = K4A_RESULT_FAILED;
            break;
        }

        result = k4a_playback_set_color_conversion(files[i].handle, K4A_IMAGE_FORMAT_COLOR_BGRA32);

        for (int j = 0; j< 30; j++)
        {
            // Skip the first 30 capture of each recording.
            k4a_stream_result_t stream_result = k4a_playback_get_next_capture(files[i].handle, &files[i].capture);
            if (stream_result == K4A_STREAM_RESULT_EOF)
            {
                printf("ERROR: Recording file is empty: %s\n", files[i].filename);
                result = K4A_RESULT_FAILED;
                break;
            }
            else if (stream_result == K4A_STREAM_RESULT_FAILED)
            {
                printf("ERROR: Failed to read first capture from file: %s\n", files[i].filename);
                result = K4A_RESULT_FAILED;
                break;
            }
            k4a_capture_release(files[i].capture);
        }
    }

    if (result == K4A_RESULT_SUCCEEDED)
    {
        printf("%-32s  %12s  %12s  %12s\n", "Source file", "COLOR", "DEPTH", "IR");
        printf("==========================================================================\n");

        bool terminated = false;

        std::shared_ptr<ExtrinsicsCalibration> extrinsicsCalib = std::make_shared<ExtrinsicsCalibration>();

        while (!terminated) {
         
                // uint64_t min_timestamp = (uint64_t)-1;
                // recording_t *min_file = NULL;

                std::vector<FrameInfo> frames;
                std::vector<AlignmentTransform> extrinsics;

                k4a_stream_result_t stream_result;

                // Find the lowest timestamp out of each of the current captures.
                for (size_t i = 0; i < file_count; i++)
                {
                    stream_result = k4a_playback_get_next_capture(files[i].handle, &files[i].capture);
                    if (K4A_STREAM_RESULT_SUCCEEDED == stream_result)
                    {

                        recording_t * min_file = &files[i];
                        FrameInfo frame = process_capture(min_file);
                        frame.Calibration = files[i].calibration;
                        k4a_imu_sample_t imu_sample;
                        stream_result = k4a_playback_get_next_imu_sample(files[i].handle, &imu_sample);
                        for (int j = 0; j < 3; ++j) {
                            frame.Accelerometer[j] = imu_sample.acc_sample.v[j];
                        }
                        frame.CameraIndex = i;
                        frame.filename = files[i].filename;

                        frames.push_back(frame);

                        AlignmentTransform transform;
                        transform.Identity = true;
                        extrinsics.push_back(transform);

                        k4a_capture_release(min_file->capture);
                        min_file->capture = NULL;
                    }
                    else {
                        terminated = true;
                        break;
                    }
                }
                cout << "Calculating Extrisincs" << endl;              
                if (!extrinsicsCalib->CalculateExtrinsics(frames, extrinsics)) {
                    cout << "Full registration failed" << endl;
                }
                else{
                    break;
                }
        }
    }

    for (size_t i = 0; i < file_count; i++)
    {
        if (files[i].handle != NULL)
        {
            k4a_playback_close(files[i].handle);
            files[i].handle = NULL;
        }
    }
    free(files);
    return result == K4A_RESULT_SUCCEEDED ? 0 : 1;
}
