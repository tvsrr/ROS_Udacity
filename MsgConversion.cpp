/*
Copyright (c) 2010-2016, Mathieu Labbe - IntRoLab - Universite
de Sherbrooke
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
    * ...
    * ...
    * ...

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. ...
*/

#include "rtabmap_conversions/MsgConversion.h"

#include <opencv2/highgui/highgui.hpp>
#include <zlib.h>
#include <ros/ros.h>
#include <rtabmap/core/util3d.h>
#include <rtabmap/core/util3d_transforms.h>
#include <rtabmap/core/util3d_filtering.h>
#include <rtabmap/core/Compression.h>
#include <rtabmap/utilite/UStl.h>
#include <rtabmap/utilite/ULogger.h>
#include <rtabmap/utilite/UTimer.h>
#include <pcl_conversions/pcl_conversions.h>
#include <eigen_conversions/eigen_msg.h>
#include <tf_conversions/tf_eigen.h>
#include <image_geometry/pinhole_camera_model.h>
#include <image_geometry/stereo_camera_model.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <laser_geometry/laser_geometry.h>
#include <rtabmap/core/util3d_surface.h>

namespace rtabmap_conversions {

void transformToTF(const rtabmap::Transform & transform, tf::Transform & tfTransform)
{
    if(!transform.isNull())
    {
        tf::transformEigenToTF(transform.toEigen3d(), tfTransform);
    }
    else
    {
        tfTransform = tf::Transform();
    }
}

rtabmap::Transform transformFromTF(const tf::Transform & transform)
{
    Eigen::Affine3d eigenTf;
    tf::transformTFToEigen(transform, eigenTf);
    return rtabmap::Transform::fromEigen3d(eigenTf);
}

void transformToGeometryMsg(const rtabmap::Transform & transform, geometry_msgs::Transform & msg)
{
    if(!transform.isNull())
    {
        tf::transformEigenToMsg(transform.toEigen3d(), msg);
        // normalize quaternion
        long double recipNorm = 1.0 / sqrt(msg.rotation.x * msg.rotation.x +
                                           msg.rotation.y * msg.rotation.y +
                                           msg.rotation.z * msg.rotation.z +
                                           msg.rotation.w * msg.rotation.w);
        msg.rotation.x *= recipNorm;
        msg.rotation.y *= recipNorm;
        msg.rotation.z *= recipNorm;
        msg.rotation.w *= recipNorm;
    }
    else
    {
        msg = geometry_msgs::Transform();
    }
}

rtabmap::Transform transformFromGeometryMsg(const geometry_msgs::Transform & msg)
{
    if(msg.rotation.w == 0 &&
       msg.rotation.x == 0 &&
       msg.rotation.y == 0 &&
       msg.rotation.z == 0)
    {
        return rtabmap::Transform();
    }
    Eigen::Affine3d tfTransform;
    tf::transformMsgToEigen(msg, tfTransform);
    return rtabmap::Transform::fromEigen3d(tfTransform);
}

void transformToPoseMsg(const rtabmap::Transform & transform, geometry_msgs::Pose & msg)
{
    if(!transform.isNull())
    {
        tf::poseEigenToMsg(transform.toEigen3d(), msg);
    }
    else
    {
        msg = geometry_msgs::Pose();
    }
}

rtabmap::Transform transformFromPoseMsg(const geometry_msgs::Pose & msg, bool ignoreRotationIfNotSet)
{
    if(msg.orientation.w == 0 &&
       msg.orientation.x == 0 &&
       msg.orientation.y == 0 &&
       msg.orientation.z == 0)
    {
        if(ignoreRotationIfNotSet)
        {
            return rtabmap::Transform(msg.position.x, msg.position.y, msg.position.z, 0, 0, 0);
        }
        return rtabmap::Transform();
    }
    Eigen::Affine3d tfPose;
    tf::poseMsgToEigen(msg, tfPose);
    return rtabmap::Transform::fromEigen3d(tfPose);
}

void toCvCopy(const rtabmap_msgs::RGBDImage & image,
              cv_bridge::CvImagePtr & rgb,
              cv_bridge::CvImagePtr & depth)
{
    if(!image.rgb.data.empty())
    {
        rgb = cv_bridge::toCvCopy(image.rgb);
    }
    else if(!image.rgb_compressed.data.empty())
    {
#ifdef CV_BRIDGE_HYDRO
        ROS_ERROR("Unsupported compressed image copy, upgrade to at least ROS Indigo.");
#else
        rgb = cv_bridge::toCvCopy(image.rgb_compressed);
#endif
    }
    if(!image.depth.data.empty())
    {
        depth = cv_bridge::toCvCopy(image.depth);
    }
    else if(!image.depth_compressed.data.empty())
    {
        cv_bridge::CvImagePtr ptr = boost::make_shared<cv_bridge::CvImage>();
        ptr->header = image.depth_compressed.header;
        ptr->image = rtabmap::uncompressImage(image.depth_compressed.data);
        ROS_ASSERT(ptr->image.empty() ||
                   ptr->image.type() == CV_32FC1 ||
                   ptr->image.type() == CV_16UC1);
        ptr->encoding = ptr->image.empty() ? "" :
                        (ptr->image.type() == CV_32FC1 ?
                         sensor_msgs::image_encodings::TYPE_32FC1 :
                         sensor_msgs::image_encodings::TYPE_16UC1);
        depth = ptr;
    }
}

void toCvShare(const rtabmap_msgs::RGBDImageConstPtr & image,
               cv_bridge::CvImageConstPtr & rgb,
               cv_bridge::CvImageConstPtr & depth)
{
    toCvShare(*image, image, rgb, depth);
}

void toCvShare(const rtabmap_msgs::RGBDImage & image,
               const boost::shared_ptr<void const>& trackedObject,
               cv_bridge::CvImageConstPtr & rgb,
               cv_bridge::CvImageConstPtr & depth)
{
    if(!image.rgb.data.empty())
    {
        rgb = cv_bridge::toCvShare(image.rgb, trackedObject);
    }
    else if(!image.rgb_compressed.data.empty())
    {
#ifdef CV_BRIDGE_HYDRO
        ROS_ERROR("Unsupported compressed image copy, upgrade to at least ROS Indigo.");
#else
        rgb = cv_bridge::toCvCopy(image.rgb_compressed);
#endif
    }
    if(!image.depth.data.empty())
    {
        depth = cv_bridge::toCvShare(image.depth, trackedObject);
    }
    else if(!image.depth_compressed.data.empty())
    {
        if(image.depth_compressed.format.compare("jpg") == 0)
        {
#ifdef CV_BRIDGE_HYDRO
            ROS_ERROR("Unsupported compressed image copy, upgrade to at least ROS Indigo.");
#else
            depth = cv_bridge::toCvCopy(image.depth_compressed);
#endif
        }
        else
        {
            cv_bridge::CvImagePtr ptr = boost::make_shared<cv_bridge::CvImage>();
            ptr->header = image.depth_compressed.header;
            ptr->image = rtabmap::uncompressImage(image.depth_compressed.data);
            ROS_ASSERT(ptr->image.empty() ||
                       ptr->image.type() == CV_32FC1 ||
                       ptr->image.type() == CV_16UC1);
            ptr->encoding = ptr->image.empty() ? "" :
                            (ptr->image.type() == CV_32FC1 ?
                             sensor_msgs::image_encodings::TYPE_32FC1 :
                             sensor_msgs::image_encodings::TYPE_16UC1);
            depth = ptr;
        }
    }
}

void rgbdImageToROS(const rtabmap::SensorData & data,
                    rtabmap_msgs::RGBDImage & msg,
                    const std::string & sensorFrameId)
{
    std_msgs::Header header;
    header.frame_id = sensorFrameId;
    header.stamp = ros::Time(data.stamp());
    rtabmap::Transform localTransform;
    if(data.cameraModels().size() > 1)
    {
        UERROR("Cannot convert multi-camera data to RGBDImage.");
        return;
    }
    if(data.cameraModels().size() == 1)
    {
        rtabmap_conversions::cameraModelToROS(data.cameraModels().front(), msg.rgb_camera_info);
        msg.rgb_camera_info.header = header;
        localTransform = data.cameraModels().front().localTransform();
    }
    if(!data.imageRaw().empty())
    {
        cv_bridge::CvImage cvImg;
        cvImg.header = header;
        cvImg.image = data.imageRaw();
        UASSERT(data.imageRaw().type() == CV_8UC1 || data.imageRaw().type() == CV_8UC3);
        cvImg.encoding = data.imageRaw().type() == CV_8UC1 ?
                         sensor_msgs::image_encodings::MONO8 :
                         sensor_msgs::image_encodings::BGR8;
        cvImg.toImageMsg(msg.rgb);
    }
    else if(!data.imageCompressed().empty())
    {
        ROS_ERROR("Conversion of compressed SensorData to RGBDImage not implemented...");
    }
    if(!data.depthOrRightRaw().empty())
    {
        cv_bridge::CvImage cvDepth;
        cvDepth.header = header;
        cvDepth.image = data.depthOrRightRaw();
        UASSERT(data.depthOrRightRaw().type() == CV_8UC1 ||
                data.depthOrRightRaw().type() == CV_16UC1 ||
                data.depthOrRightRaw().type() == CV_32FC1);
        cvDepth.encoding =
            data.depthOrRightRaw().type() == CV_8UC1 ?
                sensor_msgs::image_encodings::MONO8 :
                (data.depthOrRightRaw().type() == CV_16UC1 ?
                 sensor_msgs::image_encodings::TYPE_16UC1 :
                 sensor_msgs::image_encodings::TYPE_32FC1);
        cvDepth.toImageMsg(msg.depth);
    }
    else if(!data.depthOrRightCompressed().empty())
    {
        ROS_ERROR("Conversion of compressed SensorData to RGBDImage not implemented...");
    }
    if(!data.keypoints().empty())
    {
        rtabmap_conversions::keypointsToROS(data.keypoints(), msg.key_points);
    }
    if(!data.keypoints3D().empty())
    {
        rtabmap_conversions::points3fToROS(data.keypoints3D(), msg.points, localTransform.inverse());
    }
    if(!data.descriptors().empty())
    {
        msg.descriptors = rtabmap::compressData(data.descriptors());
    }
    if(!data.globalDescriptors().empty())
    {
        rtabmap_conversions::globalDescriptorToROS(data.globalDescriptors().front(), msg.global_descriptor);
        msg.global_descriptor.header = header;
    }
}

rtabmap::SensorData rgbdImageFromROS(const rtabmap_msgs::RGBDImageConstPtr & image)
{
    rtabmap::SensorData data;
    cv_bridge::CvImageConstPtr imageMsg;
    cv_bridge::CvImageConstPtr depthMsg;
    toCvShare(image, imageMsg, depthMsg);
    ros::Time higherStamp;
    int imageWidth = imageMsg->image.cols;
    int imageHeight = imageMsg->image.rows;
    int depthWidth = depthMsg->image.cols;
    int depthHeight = depthMsg->image.rows;
    UASSERT_MSG(
        imageWidth / depthWidth == imageHeight / depthHeight,
        uFormat("rgb=%dx%d depth=%dx%d", imageWidth, imageHeight, depthWidth, depthHeight).c_str());
    if(!(imageMsg->encoding.compare(sensor_msgs::image_encodings::TYPE_8UC1) == 0 ||
         imageMsg->encoding.compare(sensor_msgs::image_encodings::MONO8) == 0 ||
         imageMsg->encoding.compare(sensor_msgs::image_encodings::MONO16) == 0 ||
         imageMsg->encoding.compare(sensor_msgs::image_encodings::BGR8) == 0 ||
         imageMsg->encoding.compare(sensor_msgs::image_encodings::RGB8) == 0 ||
         imageMsg->encoding.compare(sensor_msgs::image_encodings::BGRA8) == 0 ||
         imageMsg->encoding.compare(sensor_msgs::image_encodings::RGBA8) == 0 ||
         imageMsg->encoding.compare(sensor_msgs::image_encodings::BAYER_GRBG8) == 0) ||
       !(depthMsg->encoding.compare(sensor_msgs::image_encodings::TYPE_16UC1) == 0 ||
         depthMsg->encoding.compare(sensor_msgs::image_encodings::TYPE_32FC1) == 0 ||
         depthMsg->encoding.compare(sensor_msgs::image_encodings::MONO16) == 0))
    {
        ROS_ERROR("Input type must be image=mono8,mono16,rgb8,bgr8,bgra8,rgba8 and image_depth=32FC1,16UC1,mono16. Current rgb=%s depth=%s",
                  imageMsg->encoding.c_str(), depthMsg->encoding.c_str());
        return data;
    }
    cv_bridge::CvImageConstPtr ptrImage = imageMsg;
    if(imageMsg->encoding.compare(sensor_msgs::image_encodings::TYPE_8UC1) == 0 ||
       imageMsg->encoding.compare(sensor_msgs::image_encodings::MONO8) == 0 ||
       imageMsg->encoding.compare(sensor_msgs::image_encodings::BGR8) == 0)
    {
        // do nothing
    }
    else if(imageMsg->encoding.compare(sensor_msgs::image_encodings::MONO16) == 0)
    {
        ptrImage = cv_bridge::cvtColor(imageMsg, "mono8");
    }
    else
    {
        ptrImage = cv_bridge::cvtColor(imageMsg, "bgr8");
    }
    cv_bridge::CvImageConstPtr ptrDepth = depthMsg;
    data = rtabmap::SensorData(
        ptrImage->image,
        ptrDepth->image,
        rtabmap_conversions::cameraModelFromROS(image->rgb_camera_info),
        0,
        rtabmap_conversions::timestampFromROS(image->header.stamp));
    return data;
}

void compressedMatToBytes(const cv::Mat & compressed, std::vector<unsigned char> & bytes)
{
    UASSERT(compressed.empty() || compressed.type() == CV_8UC1);
    bytes.clear();
    if(!compressed.empty())
    {
        bytes.resize(compressed.cols * compressed.rows);
        memcpy(bytes.data(), compressed.data, bytes.size());
    }
}

cv::Mat compressedMatFromBytes(const std::vector<unsigned char> & bytes, bool copy)
{
    cv::Mat out;
    if(bytes.size())
    {
        out = cv::Mat(1, (int)bytes.size(), CV_8UC1, (void*)bytes.data());
        if(copy)
        {
            out = out.clone();
        }
    }
    return out;
}

void infoFromROS(const rtabmap_msgs::Info & info, rtabmap::Statistics & stat)
{
    stat.setExtended(true);
    stat.setRefImageId(info.refId);
    stat.setLoopClosureId(info.loopClosureId);
    stat.setProximityDetectionId(info.proximityDetectionId);
    stat.setStamp(info.header.stamp.toSec());
    stat.setLoopClosureTransform(rtabmap_conversions::transformFromGeometryMsg(info.loopClosureTransform));
    stat.setWmState(info.wmState);
    std::map<int, float> mapIntFloat;
    for(unsigned int i = 0; i < info.posteriorKeys.size() &&
                            i < info.posteriorValues.size(); ++i)
    {
        mapIntFloat.insert(std::make_pair(info.posteriorKeys.at(i), info.posteriorValues.at(i)));
    }
    stat.setPosterior(mapIntFloat);
    mapIntFloat.clear();
    for(unsigned int i = 0; i < info.likelihoodKeys.size() &&
                            i < info.likelihoodValues.size(); ++i)
    {
        mapIntFloat.insert(std::make_pair(info.likelihoodKeys.at(i), info.likelihoodValues.at(i)));
    }
    stat.setLikelihood(mapIntFloat);
    mapIntFloat.clear();
    for(unsigned int i = 0; i < info.rawLikelihoodKeys.size() &&
                            i < info.rawLikelihoodValues.size(); ++i)
    {
        mapIntFloat.insert(std::make_pair(info.rawLikelihoodKeys.at(i), info.rawLikelihoodValues.at(i)));
    }
    stat.setRawLikelihood(mapIntFloat);
    std::map<int, int> mapIntInt;
    for(unsigned int i = 0; i < info.weightsKeys.size() &&
                            i < info.weightsValues.size(); ++i)
    {
        mapIntInt.insert(std::make_pair(info.weightsKeys.at(i), info.weightsValues.at(i)));
    }
    stat.setWeights(mapIntInt);
    std::map<int, std::string> mapIntStr;
    for(unsigned int i = 0; i < info.labelsKeys.size() &&
                            i < info.labelsValues.size(); ++i)
    {
        mapIntStr.insert(std::make_pair(info.labelsKeys.at(i), info.labelsValues.at(i)));
    }
    stat.setLabels(mapIntStr);
    stat.setLocalPath(info.localPath);
    stat.setCurrentGoalId(info.currentGoalId);
    for(unsigned int i = 0; i < info.statsKeys.size() &&
                            i < info.statsValues.size(); i++)
    {
        stat.addStatistic(info.statsKeys.at(i), info.statsValues.at(i));
    }
}

void infoToROS(const rtabmap::Statistics & stats, rtabmap_msgs::Info & info)
{
    info.refId = stats.refImageId();
    info.loopClosureId = stats.loopClosureId();
    info.proximityDetectionId = stats.proximityDetectionId();
    rtabmap_conversions::transformToGeometryMsg(stats.loopClosureTransform(), info.loopClosureTransform);
    if(stats.extended())
    {
        info.wmState = stats.wmState();
        info.posteriorKeys = uKeys(stats.posterior());
        info.posteriorValues = uValues(stats.posterior());
        info.likelihoodKeys = uKeys(stats.likelihood());
        info.likelihoodValues = uValues(stats.likelihood());
        info.rawLikelihoodKeys = uKeys(stats.rawLikelihood());
        info.rawLikelihoodValues = uValues(stats.rawLikelihood());
        info.weightsKeys = uKeys(stats.weights());
        info.weightsValues = uValues(stats.weights());
        info.labelsKeys = uKeys(stats.labels());
        info.labelsValues = uValues(stats.labels());
        info.localPath = stats.localPath();
        info.currentGoalId = stats.currentGoalId();
        info.statsKeys = uKeys(stats.data());
        info.statsValues = uValues(stats.data());
    }
}

rtabmap::Link linkFromROS(const rtabmap_msgs::Link & msg)
{
    cv::Mat information = cv::Mat(6, 6, CV_64FC1, (void*)msg.information.data()).clone();
    return rtabmap::Link(msg.fromId, msg.toId, (rtabmap::Link::Type)msg.type,
                         transformFromGeometryMsg(msg.transform), information);
}

void linkToROS(const rtabmap::Link & link, rtabmap_msgs::Link & msg)
{
    msg.fromId = link.from();
    msg.toId = link.to();
    msg.type = link.type();
    if(link.infMatrix().type() == CV_64FC1 &&
       link.infMatrix().cols == 6 && link.infMatrix().rows == 6)
    {
        memcpy(msg.information.data(), link.infMatrix().data, 36 * sizeof(double));
    }
    transformToGeometryMsg(link.transform(), msg.transform);
}

cv::KeyPoint keypointFromROS(const rtabmap_msgs::KeyPoint & msg)
{
    return cv::KeyPoint(msg.pt.x, msg.pt.y, msg.size, msg.angle, msg.response, msg.octave, msg.class_id);
}

void keypointToROS(const cv::KeyPoint & kpt, rtabmap_msgs::KeyPoint & msg)
{
    msg.angle = kpt.angle;
    msg.class_id = kpt.class_id;
    msg.octave = kpt.octave;
    msg.pt.x = kpt.pt.x;
    msg.pt.y = kpt.pt.y;
    msg.response = kpt.response;
    msg.size = kpt.size;
}

std::vector<cv::KeyPoint> keypointsFromROS(const std::vector<rtabmap_msgs::KeyPoint> & msg)
{
    std::vector<cv::KeyPoint> v(msg.size());
    for(unsigned int i = 0; i < msg.size(); ++i)
    {
        v[i] = keypointFromROS(msg[i]);
    }
    return v;
}

void keypointsFromROS(const std::vector<rtabmap_msgs::KeyPoint> & msg,
                      std::vector<cv::KeyPoint> & kpts, int xShift)
{
    size_t outCurrentIndex = kpts.size();
    kpts.resize(kpts.size() + msg.size());
    for(unsigned int i = 0; i < msg.size(); ++i)
    {
        kpts[outCurrentIndex + i] = keypointFromROS(msg[i]);
        kpts[outCurrentIndex + i].pt.x += xShift;
    }
}

void keypointsToROS(const std::vector<cv::KeyPoint> & kpts,
                    std::vector<rtabmap_msgs::KeyPoint> & msg)
{
    msg.resize(kpts.size());
    for(unsigned int i = 0; i < msg.size(); ++i)
    {
        keypointToROS(kpts[i], msg[i]);
    }
}

rtabmap::GlobalDescriptor globalDescriptorFromROS(const rtabmap_msgs::GlobalDescriptor & msg)
{
    return rtabmap::GlobalDescriptor(msg.type,
                                     rtabmap::uncompressData(msg.data),
                                     rtabmap::uncompressData(msg.info));
}

void globalDescriptorToROS(const rtabmap::GlobalDescriptor & desc,
                           rtabmap_msgs::GlobalDescriptor & msg)
{
    msg.type = desc.type();
    msg.info = rtabmap::compressData(desc.info());
    msg.data = rtabmap::compressData(desc.data());
}

std::vector<rtabmap::GlobalDescriptor> globalDescriptorsFromROS(
    const std::vector<rtabmap_msgs::GlobalDescriptor> & msg)
{
    if(!msg.empty())
    {
        std::vector<rtabmap::GlobalDescriptor> v(msg.size());
        for(unsigned int i = 0; i < msg.size(); ++i)
        {
            v[i] = globalDescriptorFromROS(msg[i]);
        }
        return v;
    }
    return std::vector<rtabmap::GlobalDescriptor>();
}

void globalDescriptorsToROS(const std::vector<rtabmap::GlobalDescriptor> & desc,
                            std::vector<rtabmap_msgs::GlobalDescriptor> & msg)
{
    msg.clear();
    if(!desc.empty())
    {
        msg.resize(desc.size());
        for(unsigned int i = 0; i < msg.size(); ++i)
        {
            globalDescriptorToROS(desc[i], msg[i]);
        }
    }
}

rtabmap::EnvSensor envSensorFromROS(const rtabmap_msgs::EnvSensor & msg)
{
    return rtabmap::EnvSensor((rtabmap::EnvSensor::Type)msg.type,
                              msg.value,
                              timestampFromROS(msg.header.stamp));
}

void envSensorToROS(const rtabmap::EnvSensor & sensor,
                    rtabmap_msgs::EnvSensor & msg)
{
    msg.type = sensor.type();
    msg.value = sensor.value();
    msg.header.stamp = ros::Time(sensor.stamp());
}

rtabmap::EnvSensors envSensorsFromROS(const std::vector<rtabmap_msgs::EnvSensor> & msg)
{
    rtabmap::EnvSensors v;
    if(!msg.empty())
    {
        for(unsigned int i = 0; i < msg.size(); ++i)
        {
            rtabmap::EnvSensor s = envSensorFromROS(msg[i]);
            v.insert(std::make_pair(s.type(), s));
        }
    }
    return v;
}

void envSensorsToROS(const rtabmap::EnvSensors & sensors,
                     std::vector<rtabmap_msgs::EnvSensor> & msg)
{
    msg.clear();
    if(!sensors.empty())
    {
        msg.resize(sensors.size());
        int i = 0;
        for(rtabmap::EnvSensors::const_iterator iter = sensors.begin();
            iter != sensors.end(); ++iter)
        {
            envSensorToROS(iter->second, msg[i++]);
        }
    }
}

cv::Point2f point2fFromROS(const rtabmap_msgs::Point2f & msg)
{
    return cv::Point2f(msg.x, msg.y);
}

void point2fToROS(const cv::Point2f & kpt, rtabmap_msgs::Point2f & msg)
{
    msg.x = kpt.x;
    msg.y = kpt.y;
}

std::vector<cv::Point2f> points2fFromROS(const std::vector<rtabmap_msgs::Point2f> & msg)
{
    std::vector<cv::Point2f> v(msg.size());
    for(unsigned int i = 0; i < msg.size(); ++i)
    {
        v[i] = point2fFromROS(msg[i]);
    }
    return v;
}

void points2fToROS(const std::vector<cv::Point2f> & kpts,
                   std::vector<rtabmap_msgs::Point2f> & msg)
{
    msg.resize(kpts.size());
    for(unsigned int i = 0; i < msg.size(); ++i)
    {
        point2fToROS(kpts[i], msg[i]);
    }
}

cv::Point3f point3fFromROS(const rtabmap_msgs::Point3f & msg)
{
    return cv::Point3f(msg.x, msg.y, msg.z);
}

void point3fToROS(const cv::Point3f & pt, rtabmap_msgs::Point3f & msg)
{
    msg.x = pt.x;
    msg.y = pt.y;
    msg.z = pt.z;
}

std::vector<cv::Point3f> points3fFromROS(const std::vector<rtabmap_msgs::Point3f> & msg,
                                         const rtabmap::Transform & transform)
{
    bool transformPoints = !transform.isNull() && !transform.isIdentity();
    std::vector<cv::Point3f> v(msg.size());
    for(unsigned int i = 0; i < msg.size(); ++i)
    {
        v[i] = point3fFromROS(msg[i]);
        if(transformPoints)
        {
            v[i] = rtabmap::util3d::transformPoint(v[i], transform);
        }
    }
    return v;
}

void points3fFromROS(const std::vector<rtabmap_msgs::Point3f> & msg,
                     std::vector<cv::Point3f> & points3,
                     const rtabmap::Transform & transform)
{
    size_t currentIndex = points3.size();
    points3.resize(points3.size() + msg.size());
    bool transformPoint = !transform.isNull() && !transform.isIdentity();
    for(unsigned int i = 0; i < msg.size(); ++i)
    {
        points3[currentIndex + i] = point3fFromROS(msg[i]);
        if(transformPoint)
        {
            points3[currentIndex + i] = rtabmap::util3d::transformPoint(points3[currentIndex + i], transform);
        }
    }
}

void points3fToROS(const std::vector<cv::Point3f> & pts,
                   std::vector<rtabmap_msgs::Point3f> & msg,
                   const rtabmap::Transform & transform)
{
    msg.resize(pts.size());
    bool transformPoints = !transform.isNull() && !transform.isIdentity();
    for(unsigned int i = 0; i < msg.size(); ++i)
    {
        if(transformPoints)
        {
            cv::Point3f pt = rtabmap::util3d::transformPoint(pts[i], transform);
            point3fToROS(pt, msg[i]);
        }
        else
        {
            point3fToROS(pts[i], msg[i]);
        }
    }
}

rtabmap::CameraModel cameraModelFromROS(const sensor_msgs::CameraInfo & camInfo,
                                        const rtabmap::Transform & localTransform)
{
    cv::Mat K, D, R, P;
    if(!camInfo.K.empty())
    {
        UASSERT(camInfo.K.size() == 9);
        K = cv::Mat(3, 3, CV_64FC1);
        memcpy(K.data, camInfo.K.elems, 9*sizeof(double));
    }
    if(camInfo.D.size())
    {
        D = cv::Mat(1, (int)camInfo.D.size(), CV_64FC1);
        memcpy(D.data, camInfo.D.data(), D.cols*sizeof(double));
    }
    if(!camInfo.R.empty())
    {
        UASSERT(camInfo.R.size() == 9);
        R = cv::Mat(3, 3, CV_64FC1);
        memcpy(R.data, camInfo.R.elems, 9*sizeof(double));
    }
    if(!camInfo.P.empty())
    {
        UASSERT(camInfo.P.size() == 12);
        P = cv::Mat(3, 4, CV_64FC1);
        memcpy(P.data, camInfo.P.elems, 12*sizeof(double));
    }
    cv::Size size(camInfo.width, camInfo.height);
    rtabmap::CameraModel model;
    if(!K.empty() && !R.empty() && !P.empty() && size.width > 0 && size.height > 0)
    {
        model = rtabmap::CameraModel("", size, K, D, R, P, localTransform);
    }
    else if(!K.empty() && size.width > 0 && size.height > 0)
    {
        double fx = K.at<double>(0,0);
        double fy = K.at<double>(1,1);
        double cx = K.at<double>(0,2);
        double cy = K.at<double>(1,2);
        model = rtabmap::CameraModel(fx, fy, cx, cy, localTransform, 0.0, size);
    }
    return model;
}

void cameraModelToROS(const rtabmap::CameraModel & model,
                      sensor_msgs::CameraInfo & camInfo)
{
    if(model.isValidForProjection())
    {
        camInfo.width = model.imageWidth();
        camInfo.height = model.imageHeight();
        camInfo.distortion_model = "plumb_bob";
        camInfo.D.resize(model.D_raw().cols);
        for(int i = 0; i < model.D_raw().cols; ++i)
        {
            camInfo.D[i] = model.D_raw().at<double>(0, i);
        }
        // Avoid extended initializer lists if not using C++11:
        for(int i = 0; i < 9; ++i)
            camInfo.K[i] = 0.0;
        const cv::Mat & K = model.K_raw();
        if(!K.empty() && K.cols == 3 && K.rows == 3 && K.type() == CV_64FC1)
        {
            for(int i = 0; i < 9; ++i)
            {
                camInfo.K[i] = ((double*)K.data)[i];
            }
        }
        for(int i = 0; i < 9; ++i)
            camInfo.R[i] = 0.0;
        const cv::Mat & R = model.R();
        if(!R.empty() && R.cols == 3 && R.rows == 3 && R.type() == CV_64FC1)
        {
            for(int i = 0; i < 9; ++i)
            {
                camInfo.R[i] = ((double*)R.data)[i];
            }
        }
        for(int i = 0; i < 12; ++i)
            camInfo.P[i] = 0.0;
        const cv::Mat & P = model.P();
        if(!P.empty() && P.cols == 4 && P.rows == 3 && P.type() == CV_64FC1)
        {
            for(int i = 0; i < 12; ++i)
            {
                camInfo.P[i] = ((double*)P.data)[i];
            }
        }
    }
}

// --- New stub definitions (ensure these appear only once) ---

void userDataToROS(const cv::Mat & data,
                   rtabmap_msgs::UserData & msg,
                   bool compress)
{
    msg.rows = data.rows;
    msg.cols = data.cols;
    msg.type = data.type();
    // No assignment to "msg.compressed" (field does not exist in Kinetic)
    if(!data.empty())
    {
        if(compress)
        {
            msg.data = rtabmap::compressData(data);
        }
        else
        {
            msg.data.resize(data.total() * data.elemSize());
            memcpy(msg.data.data(), data.data, msg.data.size());
        }
    }
    else
    {
        msg.data.clear();
    }
}

void mapDataFromROS(const rtabmap_msgs::MapData & msg,
                    std::map<int, rtabmap::Transform> & poses,
                    std::multimap<int, rtabmap::Link> & links,
                    std::map<int, rtabmap::Signature> & signatures,
                    rtabmap::Transform & mapToOdom)
{
    // Kinectic's MapData has no mapToOdom field
    mapToOdom = rtabmap::Transform(); 
    
    // Convert poses (if nodes are available)
    poses.clear();
    for(size_t i = 0; i < msg.nodes.size(); ++i)
    {
        poses.insert(std::make_pair(
            msg.nodes[i].id,
            transformFromGeometryMsg(msg.nodes[i].pose)
        ));
    }
    
    // Convert links
    links.clear();
    for(size_t i = 0; i < msg.links.size(); ++i)
    {
        links.insert(std::make_pair(
            msg.links[i].fromId,
            linkFromROS(msg.links[i])
        ));
    }
    
    // Signatures not supported in Kinetic
    signatures.clear(); 
}

} // namespace rtabmap_conversions
