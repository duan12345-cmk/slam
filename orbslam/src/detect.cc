/**
* This file is part of SegAndLocatingSLAM.
*
* Copyright (C) 2023 HongWeiLiu (University of Northeast Forestry)
*
*/

#include "detect.h"
#include <iostream>
#include <fstream>
#include <iomanip>
#include <dirent.h>
#include <errno.h>
#include <locale>
#include <codecvt>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <Python.h>
#include <numpy/arrayobject.h>
//#include <arrayobject.h>
#include <opencv2/opencv.hpp>
using namespace std;
namespace ORB_SLAM3
{
   
   //
// Created by yuwenlu on 2022/3/14.
//
#include <YoloDetect.h>

YoloDetection::YoloDetection() {
    torch::jit::setTensorExprFuserEnabled(false);
    mModule = torch::jit::load("yolov8n.pt"); 
    std::ifstream f("voc.names");
    std::string name = "";
    while (std::getline(f, name))
    {
        mClassnames.push_back(name);
    } 
    mvDynamicNames = {"person", "car", "motorbike", "bus", "train", "bicycle", 
                      "aeroplane", "boat", "bird", "cat", "dog", "horse", "sheep", "cow"};
}

YoloDetection::~YoloDetection()
{

}
std::vector<cv::KeyPoint> YoloDetection::removeDynamicFeatures(
    const std::vector<cv::KeyPoint>& currentFrameFeatures,
    const std::vector<cv::Rect2i>& knownDynamicBoxes,
    const std::vector<cv::Rect2i>& potentialDynamicBoxes,
    float motionThreshold)
{
    // Initialize D and S sets
    std::vector<cv::KeyPoint> dynamicPoints;  // D set
    std::vector<cv::KeyPoint> staticPoints;   // S set
    
    // For each point p in current frame's feature points
    for (const auto& p : currentFrameFeatures)
    {
        bool isDynamic = false;
        
        // Check if point is in potential dynamic boxes
        for (const auto& box : potentialDynamicBoxes)
        {
            if (p.pt.x >= box.x && p.pt.x <= box.x + box.width &&
                p.pt.y >= box.y && p.pt.y <= box.y + box.height)
            {
                isDynamic = true;
                break;
            }
        }
        
        // Check if point is in known dynamic boxes
        if (!isDynamic)
        {
            for (const auto& box : knownDynamicBoxes)
            {
                if (p.pt.x >= box.x && p.pt.x <= box.x + box.width &&
                    p.pt.y >= box.y && p.pt.y <= box.y + box.height)
                {
                    isDynamic = true;
                    break;
                }
            }
        }
        
        // Add to appropriate set
        if (isDynamic)
        {
            dynamicPoints.push_back(p);
        }
        else
        {
            staticPoints.push_back(p);
        }
    }
    
    // For each point in static points, compute Lucas-Kanade motion
    std::vector<cv::KeyPoint> pointsToRemove;
    
    for (const auto& p : staticPoints)
    {
        // Compute Lucas-Kanade optical flow for point p
        // In a real implementation, we would need previous frame data
        // and use cv::calcOpticalFlowPyrLK
        
        // Here we assume computeMotion() function calculates (u,v) = LK(p)
        cv::Point2f motion = computeMotion(p);
        float m = std::sqrt(motion.x * motion.x + motion.y * motion.y);
        
        // If motion exceeds threshold, move point from S to D
        if (m > motionThreshold)
        {
            dynamicPoints.push_back(p);
            pointsToRemove.push_back(p);
        }
    }
    
    // Remove points from staticPoints that were moved to dynamicPoints
    for (const auto& p : pointsToRemove)
    {
        staticPoints.erase(std::remove_if(staticPoints.begin(), staticPoints.end(),
            [&p](const cv::KeyPoint& kp) {
                return kp.pt.x == p.pt.x && kp.pt.y == p.pt.y;
            }), staticPoints.end());
    }
    
    // Return the set of static feature points
    return staticPoints;
}

cv::Point2f YoloDetection::computeMotion(const cv::KeyPoint& point)
{
    // In a real implementation, this function would:
    // 1. Use the previous frame
    // 2. Apply Lucas-Kanade optical flow
    // 3. Return the motion vector (u,v)
    
    // For this example, we'll return a placeholder motion
    // This should be replaced with actual Lucas-Kanade implementation
    return cv::Point2f(0.0f, 0.0f);
}

bool YoloDetection::Detect(const std::vector<cv::KeyPoint>& currentFrameFeatures)
{
    cv::Mat img;

    if(mRGB.empty())
    {
        std::cout << "Read RGB failed!" << std::endl;
        return false;
    }

    // Preparing input tensor
    cv::resize(mRGB, img, cv::Size(640, 380));
    cv::cvtColor(img, img, cv::COLOR_BGR2RGB);
    torch::Tensor imgTensor = torch::from_blob(img.data, {img.rows, img.cols,3},torch::kByte);
    imgTensor = imgTensor.permute({2,0,1});
    imgTensor = imgTensor.toType(torch::kFloat);
    imgTensor = imgTensor.div(255);
    imgTensor = imgTensor.unsqueeze(0);

    // Run YOLO detection
    torch::Tensor preds = mModule.forward({imgTensor}).toTuple()->elements()[0].toTensor();
    std::vector<torch::Tensor> dets = YoloDetection::non_max_suppression(preds, 0.4, 0.5);
    
    // Clear previous detection results
    mmDetectMap.clear();
    mvDynamicArea.clear();
    
    std::vector<cv::Rect2i> potentialDynamicBoxes;
    std::vector<cv::Rect2i> knownDynamicBoxes; // This should be populated from previous detections
    
    if (dets.size() > 0)
    {
        // Process detection results
        for (size_t i=0; i < dets[0].sizes()[0]; ++i)
        {
            float left = dets[0][i][0].item().toFloat() * mRGB.cols / 640;
            float top = dets[0][i][1].item().toFloat() * mRGB.rows / 384;
            float right = dets[0][i][2].item().toFloat() * mRGB.cols / 640;
            float bottom = dets[0][i][3].item().toFloat() * mRGB.rows / 384;
            int classID = dets[0][i][5].item().toInt();

            cv::Rect2i detectArea(left, top, (right - left), (bottom - top));
            mmDetectMap[mClassnames[classID]].push_back(detectArea);

            // Add to potential dynamic boxes if it's a class in mvDynamicNames
            if (std::find(mvDynamicNames.begin(), mvDynamicNames.end(), mClassnames[classID]) != mvDynamicNames.end())
            {
                potentialDynamicBoxes.push_back(detectArea);
            }
        }
    }
    
    // Apply dynamic feature point removal algorithm
    // Motion threshold is set to 1.5 pixels as specified
    const float motionThreshold = 1.5f;
    std::vector<cv::KeyPoint> staticFeatures = removeDynamicFeatures(
        currentFrameFeatures, knownDynamicBoxes, potentialDynamicBoxes, motionThreshold);
    
    // Update dynamic areas (boxes containing dynamic objects)
    mvDynamicArea = potentialDynamicBoxes;
    
    // If no dynamic objects were detected, add a placeholder
    if (mvDynamicArea.size() == 0)
    {
        cv::Rect2i tDynamicArea(1, 1, 1, 1);
        mvDynamicArea.push_back(tDynamicArea);
    }
    
    return true;
}

vector<torch::Tensor> YoloDetection::non_max_suppression(torch::Tensor preds, float score_thresh, float iou_thresh)
{
    std::vector<torch::Tensor> output;
    for (size_t i=0; i < preds.sizes()[0]; ++i)
    {
        torch::Tensor pred = preds.select(0, i);

        // Filter by scores
        torch::Tensor scores = pred.select(1, 4) * std::get<0>( torch::max(pred.slice(1, 5, pred.sizes()[1]), 1));
        pred = torch::index_select(pred, 0, torch::nonzero(scores > score_thresh).select(1, 0));
        if (pred.sizes()[0] == 0) continue;

        // (center_x, center_y, w, h) to (left, top, right, bottom)
        pred.select(1, 0) = pred.select(1, 0) - pred.select(1, 2) / 2;
        pred.select(1, 1) = pred.select(1, 1) - pred.select(1, 3) / 2;
        pred.select(1, 2) = pred.select(1, 0) + pred.select(1, 2);
        pred.select(1, 3) = pred.select(1, 1) + pred.select(1, 3);

        // Computing scores and classes
        std::tuple<torch::Tensor, torch::Tensor> max_tuple = torch::max(pred.slice(1, 5, pred.sizes()[1]), 1);
        pred.select(1, 4) = pred.select(1, 4) * std::get<0>(max_tuple);
        pred.select(1, 5) = std::get<1>(max_tuple);

        torch::Tensor  dets = pred.slice(1, 0, 6);

        torch::Tensor keep = torch::empty({dets.sizes()[0]});
        torch::Tensor areas = (dets.select(1, 3) - dets.select(1, 1)) * (dets.select(1, 2) - dets.select(1, 0));
        std::tuple<torch::Tensor, torch::Tensor> indexes_tuple = torch::sort(dets.select(1, 4), 0, 1);
        torch::Tensor v = std::get<0>(indexes_tuple);
        torch::Tensor indexes = std::get<1>(indexes_tuple);
        int count = 0;
        while (indexes.sizes()[0] > 0)
        {
            keep[count] = (indexes[0].item().toInt());
            count += 1;

            // Computing overlaps
            torch::Tensor lefts = torch::empty(indexes.sizes()[0] - 1);
            torch::Tensor tops = torch::empty(indexes.sizes()[0] - 1);
            torch::Tensor rights = torch::empty(indexes.sizes()[0] - 1);
            torch::Tensor bottoms = torch::empty(indexes.sizes()[0] - 1);
            torch::Tensor widths = torch::empty(indexes.sizes()[0] - 1);
            torch::Tensor heights = torch::empty(indexes.sizes()[0] - 1);
            for (size_t i=0; i<indexes.sizes()[0] - 1; ++i)
            {
                lefts[i] = std::max(dets[indexes[0]][0].item().toFloat(), dets[indexes[i + 1]][0].item().toFloat());
                tops[i] = std::max(dets[indexes[0]][1].item().toFloat(), dets[indexes[i + 1]][1].item().toFloat());
                rights[i] = std::min(dets[indexes[0]][2].item().toFloat(), dets[indexes[i + 1]][2].item().toFloat());
                bottoms[i] = std::min(dets[indexes[0]][3].item().toFloat(), dets[indexes[i + 1]][3].item().toFloat());
                widths[i] = std::max(float(0), rights[i].item().toFloat() - lefts[i].item().toFloat());
                heights[i] = std::max(float(0), bottoms[i].item().toFloat() - tops[i].item().toFloat());
            }
            torch::Tensor overlaps = widths * heights;


            torch::Tensor ious = overlaps / (areas.select(0, indexes[0].item().toInt()) + torch::index_select(areas, 0, indexes.slice(0, 1, indexes.sizes()[0])) - overlaps);
            indexes = torch::index_select(indexes, 0, torch::nonzero(ious <= iou_thresh).select(1, 0) + 1);
        }
        keep = keep.toType(torch::kInt64);
        output.push_back(torch::index_select(dets, 0, keep.slice(0, 0, count)));
    }
    return output;
}

void YoloDetection::GetImage(cv::Mat &RGB)
{
    mRGB = RGB;
}

void YoloDetection::ClearImage()
{
    mRGB = 0;
}

void YoloDetection::ClearArea()
{
    mvPersonArea.clear();
}





}





















