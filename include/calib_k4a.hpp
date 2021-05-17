#pragma once
#include "camera_extrinsics.hpp"

static void CopyIntrinsics(
    const k4a_calibration_camera_t& from,
    CameraIntrinsics& to)
{
    to.Width = from.resolution_width;
    to.Height = from.resolution_height;

    const k4a_calibration_intrinsic_parameters_t& params = from.intrinsics.parameters;
    to.cx = params.param.cx;
    to.cy = params.param.cy;
    to.fx = params.param.fx;
    to.fy = params.param.fy;
    to.k[0] = params.param.k1;
    to.k[1] = params.param.k2;
    to.k[2] = params.param.k3;
    to.k[3] = params.param.k4;
    to.k[4] = params.param.k5;
    to.k[5] = params.param.k6;
    to.codx = params.param.codx;
    to.cody = params.param.cody;
    to.p1 = params.param.p1;
    to.p2 = params.param.p2;
}

void CalibrationFromK4a(
    const k4a_calibration_t& from,
    CameraCalibration& to)
{
 
    CopyIntrinsics(from.depth_camera_calibration, to.Depth);
    CopyIntrinsics(from.color_camera_calibration, to.Color);
    // Extrinsics from depth to color camera
    const k4a_calibration_extrinsics_t* extrinsics = &from.extrinsics[K4A_CALIBRATION_TYPE_DEPTH][K4A_CALIBRATION_TYPE_COLOR];
    for (int i = 0; i < 9; ++i) {
        to.RotationFromDepth[i] = extrinsics->rotation[i]; 
    }
    for (int i = 0; i < 3; ++i) {
        to.TranslationFromDepth[i] = extrinsics->translation[i]; 
    }
}