/***
 * @description: 统一 C API 实现 (单编译单元, 避免 logging.hpp 的重复定义);
 *               包含跟踪器 + 检测器的所有 C API;
 * @Copyright (c) 2026 by gxs, All Rights Reserved.
 */
#include "detector_c_api.hpp"
#include "tracker_c_api.hpp"

// ================================================================
// 所有跟踪器依赖 (header-only)
// ================================================================
#include <cstring>
#include <string>
#include <vector>

#include <opencv2/opencv.hpp>

// YoloObject.h 依赖 NetConfig.hpp 中的 yolo::TaskType, 需提前包含;
#include "NetConfig.hpp"
#include "detector/RunTime.hpp"
#include "detector/YoloObject.h"
#include "tracker/TrackerRuntime.hpp"
#include "tracker/version.hpp"

// ================================================================
// Tracker API Implementation
// ================================================================

using tracker::TrackerRuntime;

TrackerHandle tracker_create()
{
    try
    {
        TrackerRuntime* runtime = new TrackerRuntime();
        return static_cast<TrackerHandle>(runtime);
    }
    catch (const std::exception& e)
    {
        return nullptr;
    }
}

void tracker_destroy(TrackerHandle handle)
{
    if (handle == nullptr)
        return;
    TrackerRuntime* runtime = static_cast<TrackerRuntime*>(handle);
    delete runtime;
}

int tracker_init(TrackerHandle handle, const char* ini_path, int max_history)
{
    if (handle == nullptr || ini_path == nullptr)
        return -1;
    TrackerRuntime* runtime = static_cast<TrackerRuntime*>(handle);
    try
    {
        runtime->init(std::string(ini_path), max_history);
        return 0;
    }
    catch (const std::exception& e)
    {
        return -1;
    }
}

int tracker_update(TrackerHandle handle, const float* dets_data, int n_dets, float* results_data, int max_results,
                   int* n_results)
{
    if (handle == nullptr || dets_data == nullptr || results_data == nullptr || n_results == nullptr)
        return -1;
    TrackerRuntime* runtime = static_cast<TrackerRuntime*>(handle);
    try
    {
        std::vector<yolo::YoloObject> detections;
        detections.reserve(static_cast<size_t>(n_dets));
        for (int i = 0; i < n_dets; i++)
        {
            yolo::YoloObject obj;
            int base = i * 6;
            obj.box.x1 = dets_data[base + 0];
            obj.box.y1 = dets_data[base + 1];
            obj.box.x2 = dets_data[base + 2];
            obj.box.y2 = dets_data[base + 3];
            obj.box.score = dets_data[base + 4];
            obj.box.cls_id = static_cast<uint32>(dets_data[base + 5]);
            detections.push_back(obj);
        }
        std::vector<tracker::TrackResult> results = runtime->update(detections);

        int actual = static_cast<int>(results.size());
        int out_count = (actual > max_results) ? max_results : actual;
        *n_results = out_count;
        for (int i = 0; i < out_count; i++)
        {
            const tracker::TrackResult& r = results[static_cast<size_t>(i)];
            int base = i * 8;
            results_data[base + 0] = static_cast<float>(r.track_id);
            results_data[base + 1] = static_cast<float>(r.det_index);
            results_data[base + 2] = static_cast<float>(r.cls_id);
            results_data[base + 3] = r.score;
            results_data[base + 4] = r.ltwh[0];
            results_data[base + 5] = r.ltwh[1];
            results_data[base + 6] = r.ltwh[2];
            results_data[base + 7] = r.ltwh[3];
        }
        return 0;
    }
    catch (const std::exception& e)
    {
        *n_results = 0;
        return -1;
    }
}

int tracker_reset(TrackerHandle handle)
{
    if (handle == nullptr)
        return -1;
    TrackerRuntime* runtime = static_cast<TrackerRuntime*>(handle);
    try
    {
        runtime->reset();
        return 0;
    }
    catch (const std::exception& e)
    {
        return -1;
    }
}

int tracker_active_count(TrackerHandle handle)
{
    if (handle == nullptr)
        return -1;
    TrackerRuntime* runtime = static_cast<TrackerRuntime*>(handle);
    try
    {
        return static_cast<int>(runtime->active_track_count());
    }
    catch (const std::exception& e)
    {
        return -1;
    }
}

int tracker_total_count(TrackerHandle handle)
{
    if (handle == nullptr)
        return -1;
    TrackerRuntime* runtime = static_cast<TrackerRuntime*>(handle);
    try
    {
        return static_cast<int>(runtime->total_track_count());
    }
    catch (const std::exception& e)
    {
        return -1;
    }
}

int tracker_is_initialized(TrackerHandle handle)
{
    if (handle == nullptr)
        return -1;
    TrackerRuntime* runtime = static_cast<TrackerRuntime*>(handle);
    return runtime->is_initialized() ? 1 : 0;
}

const char* tracker_version()
{
    return tracker::version_string();
}

// ================================================================
// Detector API Implementation
// ================================================================

DetectorHandle detector_create(const char* ini_path, const char* onnx_path, int device)
{
    try
    {
        yolo::ModelPathParams param;
        param.onnx_path = std::string(onnx_path);
        yolo::RunTime* runtime = new yolo::RunTime(std::string(ini_path), param, yolo::ModelBench::OpenCV, device);
        return static_cast<DetectorHandle>(runtime);
    }
    catch (const std::exception& e)
    {
        return nullptr;
    }
}

void detector_destroy(DetectorHandle handle)
{
    if (handle == nullptr)
        return;
    yolo::RunTime* runtime = static_cast<yolo::RunTime*>(handle);
    delete runtime;
}

int detector_detect(DetectorHandle handle, const char* image_path, float* out_data, int max_dets, int* n_dets)
{
    if (handle == nullptr || image_path == nullptr || out_data == nullptr || n_dets == nullptr)
        return -1;
    yolo::RunTime* runtime = static_cast<yolo::RunTime*>(handle);
    try
    {
        cv::Mat img = cv::imread(std::string(image_path));
        if (img.empty())
        {
            *n_dets = 0;
            return -1;
        }
        std::vector<cv::Mat> batch_input = {img};
        std::vector<std::vector<yolo::YoloObject>> batch_output;
        (*runtime)(batch_input, batch_output);
        const std::vector<yolo::YoloObject>& dets = batch_output[0];
        int actual = static_cast<int>(dets.size());
        int out_count = (actual > max_dets) ? max_dets : actual;
        *n_dets = out_count;
        for (int i = 0; i < out_count; i++)
        {
            const yolo::YoloObject& obj = dets[static_cast<size_t>(i)];
            int base = i * 6;
            out_data[base + 0] = obj.box.x1;
            out_data[base + 1] = obj.box.y1;
            out_data[base + 2] = obj.box.x2;
            out_data[base + 3] = obj.box.y2;
            out_data[base + 4] = obj.box.score;
            out_data[base + 5] = static_cast<float>(obj.box.cls_id);
        }
        return 0;
    }
    catch (const std::exception& e)
    {
        *n_dets = 0;
        return -1;
    }
}