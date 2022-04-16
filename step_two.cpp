// Author: jike5
// Email: zhenxinzhu163@163.com
// Date: 2022 04 16

/* 生成AirSim仿真数据集第二步
* 采集参考轨迹上的Image数据
* Image: 30Hz
* 需要将json切换为Computer Vision模式
*/

#include <string>
#include <vector>
#include <fstream>
#include <thread>
#include <time.h>
#include <assert.h>
#include <iomanip>
#include <chrono>
#include "common/common_utils/StrictMode.hpp"
STRICT_MODE_OFF
#ifndef RPCLIB_MSGPACK
#define RPCLIB_MSGPACK clmdep_msgpack
#endif // !RPCLIB_MSGPACK
#include "rpc/rpc_error.h"
STRICT_MODE_ON
#include "vehicles/multirotor/api/MultirotorRpcLibClient.hpp"
#include "common/common_utils/FileSystem.hpp"

typedef msr::airlib::ImageCaptureBase::ImageRequest ImageRequest;
typedef msr::airlib::ImageCaptureBase::ImageResponse ImageResponse;
typedef msr::airlib::ImageCaptureBase::ImageType ImageType;

int main()
{
    msr::airlib::MultirotorRpcLibClient client_image_;
    std::string camera_name_ = "0";
    std::string path_image_data_;
    std::string path_image_csv_;
    vector<ImageRequest> image_request_;
    image_request_ = { ImageRequest(camera_name_, ImageType::Scene) };

    client_image_.confirmConnection();
    
    std::string path_time_stamp_ = "D:\\AirSim\\dataset\\2022-03-21-17-33-16"; // enter your data dir name
    path_image_data_ = path_time_stamp_ + "\\mav0" + "\\cam0" + "\\data";
    path_image_csv_ = path_time_stamp_ + "\\mav0" + "\\cam0";

    // create the cam0/data dir
    string command = "mkdir -p " + path_image_data_; // mkdir -p
    std::cout << command << std::endl;
    system(command.c_str());

    // create .csv file
    std::ofstream fout_image_;
    fout_image_.open(path_image_csv_ + "\\data.csv", std::ios::out);
    if (!fout_image_.is_open()) {
        std::cerr << "cannot open the file: " << path_image_csv_ << std::endl;
    }
    else {
        fout_image_ << "#timestamp[ns], filename" << std::endl;
    }

    // read pose ground
    std::ifstream fin_pose_;
    std::string path_pose_data_ = path_time_stamp_ + "\\mav0" + "\\state_groundtruth_estimate0";
    fin_pose_.open(path_pose_data_ + "\\data.csv", std::ios::in);
    if (!fin_pose_.is_open()) {
        std::cerr << "failed to open pose ground file" << std::endl;
        return -1;
    }
    std::vector<std::string> vStrNames; // 存放时间戳
    vStrNames.reserve(5000);
    std::vector<msr::airlib::Pose> vPoses; // 存放对应的位姿
    vPoses.reserve(5000);

    std::string sTmp_line;
    char tmp_dot;
    float p_x, p_y, p_z, q_w, q_x, q_y, q_z;
    double tmp_name_num;
    std::string tmp_name;
    bool first_line_flag = true;
    while (std::getline(fin_pose_, sTmp_line) && !sTmp_line.empty()) {
        std::istringstream ssTmpData(sTmp_line);
        if (first_line_flag) {
            first_line_flag = false;
            continue;
        }
        ssTmpData >> tmp_name_num >> tmp_dot >> p_x >> tmp_dot >> p_y >> tmp_dot >> p_z >>
            tmp_dot >> q_w >> tmp_dot >> q_x >> tmp_dot >> q_y >> tmp_dot >> q_z;
        msr::airlib::Vector3r position(p_x, p_y, p_z);
        msr::airlib::Quaternionr orientation(q_w, q_x, q_y, q_z);
        msr::airlib::Pose tmpPose(position, orientation);
        std::stringstream stream;
        stream << std::fixed << std::setprecision(0) << tmp_name_num;
        tmp_name = stream.str();
        vStrNames.push_back(tmp_name);
        vPoses.push_back(tmpPose);
    }

    // 获取图像
    for (int i = 0; i < vStrNames.size(); i++) {
        msr::airlib::Pose tmpPose = vPoses[i];
        client_image_.simSetVehiclePose(tmpPose, true);
        std::this_thread::sleep_for(std::chrono::milliseconds(500)); // ms

        const vector<ImageResponse>& response = client_image_.simGetImages(image_request_);
        if (response.size() > 0) {
            for (const ImageResponse& image_info : response) {
                std::string file_path = common_utils::FileSystem::combine(path_image_data_, vStrNames[i]);
                //std::cout << file_path << std::endl;
                std::ofstream file(file_path + ".png", std::ios::binary);
                file.write(reinterpret_cast<const char*>(image_info.image_data_uint8.data()), image_info.image_data_uint8.size());
                file.close();
                fout_image_ << vStrNames[i] << ',' << vStrNames[i] + ".png" << std::endl;
            }
        }

        std::cout << vStrNames[i] << std::endl;
        std::cout << vPoses[i].position.x() << "," << vPoses[i].position.y() << "," << vPoses[i].position.z() << "," << vPoses[i].orientation.w() << "," << vPoses[i].orientation.x() << "," << vPoses[i].orientation.y() << "," << vPoses[i].orientation.z() << "," << std::endl;
        std::cout << std::endl;
    }

    fout_image_.close();
    return 0;
}