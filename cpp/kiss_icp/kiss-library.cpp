/*

 Copyright (c) 2023 University of Edinburgh, Imperial College, University of Manchester.
 Developed in the PAMELA project, EPSRC Programme Grant EP/K008730/1

 This code is licensed under the MIT License.

*/


#include <SLAMBenchAPI.h>
#include <io/SLAMFrame.h>
#include <io/sensor/LidarSensor.h>
#include <io/sensor/GroundTruthSensor.h>
#include <chrono>
#include <Eigen/Eigen>
#include <iostream>
#include <string>

#include <chrono>
#include <filesystem>
#include <functional>
#include <thread>
#include <kiss_icp/pipeline/KissICP.hpp>


const std::string default_yaml_path = "/deps/legoloam/configs/configs.yaml";

// Parameters
std::string yaml_path;
bool show_point_cloud;
int point_cloud_ratio;
// extern parameters in utility.h
int N_SCAN;
int Horizon_SCAN;
float ang_res_x;
float ang_res_y;
float ang_bottom;
int groundScanInd;
float segmentAlphaX;
float segmentAlphaY;
int skipFrameNum;
double mappingProcessInterval;
std::string dataset_name;

// Sensors
slambench::io::LidarSensor *lidar_sensor;

slambench::TimeStamp last_frame_timestamp;
double current_timestamp;
double fake_timestamp = 1000.0;

// Outputs
slambench::outputs::Output *pose_output;
slambench::outputs::Output *pointcloud_output;

// contains rotation only
Eigen::Matrix4f velo_2_lgrey_kitti = (Eigen::Matrix4f() << 9.999728e-01f,  7.027479e-03f, -2.255075e-03f,  0.000000e+00f,
                                                          -7.027555e-03f,  9.999753e-01f, -2.599616e-05f,  0.000000e+00f,
                                                           2.254837e-03f,  4.184312e-05f,  9.999975e-01f,  0.000000e+00f,
                                                           0.000000e+00f,  0.000000e+00f,  0.000000e+00f,  1.000000e+00f).finished();

Eigen::Matrix4f align_mat = (Eigen::Matrix4f() << -1.0,  0.0, 0.0, 0.0,
                                                   0.0, -1.0, 0.0, 0.0,
                                                   0.0,  0.0, 1.0, 0.0,
                                                   0.0,  0.0, 0.0, 1.0).finished();

static std::vector<Eigen::Vector3d> lidar_frame;
static kiss_icp::pipeline::KissICP * library;

bool sb_new_slam_configuration(SLAMBenchLibraryHelper * slam_settings) {

    slam_settings->addParameter(TypedParameter<std::string>("configs", "configuration", "path to configuration YAML file", &yaml_path, &default_yaml_path));

    return true;
}


bool sb_init_slam_system(SLAMBenchLibraryHelper *slam_settings) {
    
    // Declare Outputs
    pose_output = new slambench::outputs::Output("Pose", slambench::values::VT_POSE, true);

    pointcloud_output = new slambench::outputs::Output("PointCloud", slambench::values::VT_POINTCLOUD, true);
    pointcloud_output->SetKeepOnlyMostRecent(true);

    slam_settings->GetOutputManager().RegisterOutput(pose_output);
    slam_settings->GetOutputManager().RegisterOutput(pointcloud_output);

    // Inspect sensors
    lidar_sensor = (slambench::io::LidarSensor*)slam_settings->get_sensors().GetSensor(slambench::io::LidarSensor::kLidarType);
    if (lidar_sensor == nullptr) {
        std::cerr << "Invalid sensors found, Lidar not found." << std::endl;
        return false;
    }

    

    if (dataset_name == "KITTI") {
        std::cout << "Use KITTI dataset" << std::endl;
        align_mat = align_mat * velo_2_lgrey_kitti;
    }
    kiss_icp::pipeline::KISSConfig libConfig;
    library = new kiss_icp::pipeline::KissICP(libConfig);
    
    return true;
}


bool sb_update_frame(SLAMBenchLibraryHelper *slam_settings , slambench::io::SLAMFrame *s) {
    
	if (s->FrameSensor == lidar_sensor) {

        last_frame_timestamp = s->Timestamp;
        current_timestamp = static_cast<double>(s->Timestamp.S) + static_cast<double>(s->Timestamp.Ns) / 1e9;
        // LeGO-LOAM doesn't fit KITTI's timestamp.
        // fake_timestamp = fake_timestamp + 0.2;
        // legoloam.IP_->laserCloudInMetadata.timestamp = current_timestamp;
        lidar_frame.clear();
        float *fdata = static_cast<float*>(s->GetData());
        int count = s->GetSize()/(4 * sizeof(float));

        for(int i = 0; i < count; ++i) {
            float x = fdata[i*4];
            float y = fdata[i*4+1];
            float z = fdata[i*4+2];
            float intensity = 0.0;
            lidar_frame.push_back(Eigen::Vector3d(x, y, z));
        //     pcl::PointXYZI point;
        //     point.x = x;
        //     point.y = y;
        //     point.z = z;
        //     point.intensity = intensity;
        //     // legoloam.IP_->laserCloudIn->points.push_back(point);
        }
        // legoloam.IP_->laserCloudIn->width = legoloam.IP_->laserCloudIn->points.size();
        // legoloam.IP_->laserCloudIn->height = 1;

        return true;
	}
	
	return false;
}


bool sb_process_once(SLAMBenchLibraryHelper *slam_settings) {
    
    // legoloam.Run();

    // if (legoloam.MO_->cloudKeyPoses6D->points.size() == 0) {
    //     pose = Eigen::Matrix4f::Identity();
    //     return true;
    // }

    // PointXYZIRPYT pose6d = legoloam.MO_->cloudKeyPoses6D->points.back();

    // Eigen::AngleAxisf rollAngle(pose6d.roll, Eigen::Vector3f::UnitX());
    // Eigen::AngleAxisf pitchAngle(pose6d.pitch, Eigen::Vector3f::UnitY());
    // Eigen::AngleAxisf yawAngle(pose6d.yaw, Eigen::Vector3f::UnitZ());
    // Eigen::Matrix3f rotationMatrix = (yawAngle * pitchAngle * rollAngle).matrix();

    // // Create the translation vector
    // Eigen::Vector3f translation(pose6d.x, pose6d.y, pose6d.z);

    // Combine the rotation matrix and translation vector into a Matrix4f pose
    // pose.block<3, 3>(0, 0) = rotationMatrix;
    // pose.block<3, 1>(0, 3) = translation;
    std::vector<double> timestampVector(lidar_frame.size(), current_timestamp);
    assert(lidar_frame.size()==timestampVector.size());
    library->RegisterFrame(lidar_frame, timestampVector);
    return true;
}


bool sb_update_outputs(SLAMBenchLibraryHelper *lib, const slambench::TimeStamp *ts_p) {
    (void)lib;

    slambench::TimeStamp ts = *ts_p;

    if (pose_output->IsActive()) {
        std::lock_guard<FastLock> lock (lib->GetOutputManager().GetLock());
        
        Sophus::SE3d last_pose = library->poses().back();
        Eigen::Matrix4d pose = Eigen::Matrix4d::Identity();
    
    // Set the rotation block
        pose.block<3, 3>(0, 0) = last_pose.rotationMatrix();
        
        // Set the translation block
        pose.block<3, 1>(0, 3) = last_pose.translation();
		pose_output->AddPoint(ts, new slambench::values::PoseValue(pose.cast<float>()));
    }
    
    // if (pointcloud_output->IsActive() && show_point_cloud) {

    //     pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_out_trans(new pcl::PointCloud<pcl::PointXYZI>);

    //     pcl::transformPointCloud(*(legoloam.MO_->cloudOutMetadata.cloud), *cloud_out_trans, align_mat);

    //     auto slambench_point_cloud = new slambench::values::PointCloudValue();
    //     int count = 0;
    //     for(const auto &p : *cloud_out_trans) {
    //         if (count % point_cloud_ratio == 0) slambench_point_cloud->AddPoint(slambench::values::Point3DF(p.x, p.y, p.z));
    //         count++;
    //     }

    //     // Take lock only after generating the map
    //     std::lock_guard<FastLock> lock (lib->GetOutputManager().GetLock());
    //     pointcloud_output->AddPoint(ts, slambench_point_cloud);
    // }

    return true;
}


bool sb_clean_slam_system() {
    delete pose_output;
    delete pointcloud_output;
    delete lidar_sensor;
    return true;
}