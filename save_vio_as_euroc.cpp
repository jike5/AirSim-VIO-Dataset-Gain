#include <string>
#include <vector>
#include <fstream>
#include <thread>
#include <time.h>
#include <assert.h>
#include <iomanip>
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


class AirSimVISave
{
public:
    AirSimVISave();
    ~AirSimVISave();
    void save_image_thread();
    void save_imu_thread();
    void save_poseground();

    msr::airlib::MultirotorRpcLibClient client_image_;
    msr::airlib::MultirotorRpcLibClient client_imu_;
    msr::airlib::MultirotorRpcLibClient client_poseground_;

    std::string camera_name_;
    std::string path_image_data_;
    std::string path_image_csv_;
    std::string path_imu_data_;
    std::string path_poseground_;
    std::string path_time_stamp_;
    vector<ImageRequest> image_request_;
    std::ofstream fout_image_;
    std::ofstream fout_imu_;
    std::ofstream fout_poseground_;
    bool isStop_;
    //double hz_image_;
    //double hz_imu_;
    //double duration_image_;  //  1000 / hz_image_  (ms)
    //double duration_imu_;    //  1000 / hz_imu_    (ms)
};

int main() {
    AirSimVISave* pairsimsave = new AirSimVISave();

    std::thread image_thread(&AirSimVISave::save_image_thread, pairsimsave);
    std::thread imu_thread(&AirSimVISave::save_imu_thread, pairsimsave);
    std::thread poseground(&AirSimVISave::save_poseground, pairsimsave);
    std::string isStop;
    std::cout << "is stop?(y/yes)" << std::endl;
    std::cin >> isStop;
    if (isStop == "y" || isStop == "yes")
        pairsimsave->isStop_ = true;

    image_thread.join();
    std::cout << "image_thread join" << std::endl;
    //pairsimsave->save_image_thread();
    //pairsimsave->save_imu_thread();
    
    // pairsimsave->isStop_ = true;

    delete pairsimsave;
    return 0;
}

AirSimVISave::AirSimVISave() {
    try {
        camera_name_ = "front_center_custom";
        client_image_.confirmConnection();
        client_imu_.confirmConnection();
        client_poseground_.confirmConnection();
        // use current time to mkdir a folder to save data
        time_t t = time(0);
        char ch[64];
        strftime(ch, sizeof(ch), "%Y-%m-%d-%H-%M-%S", localtime(&t)); //年-月-日-时-分-秒
        path_time_stamp_ = "D:\\AirSim\\dataset\\" + std::string(ch);
        
        image_request_ = { ImageRequest(camera_name_, ImageType::Scene) };
        isStop_ = false;
    }
    catch (rpc::rpc_error& e) {
        std::string msg = e.get_error().as<std::string>();
        std::cout << "Exception raised by the API, something went wrong." << std::endl
                  << msg << std::endl;
    }
    path_image_data_ = path_time_stamp_ + "\\mav0" + "\\cam0" + "\\data";
    path_image_csv_ = path_time_stamp_ + "\\mav0" + "\\cam0";
    path_imu_data_ = path_time_stamp_ + "\\mav0" + "\\imu0";
    path_poseground_ = path_time_stamp_ + "\\mav0" + "\\leica0";

    string command = "mkdir -p " + path_image_data_; // mkdir -p 
    std::cout << command << std::endl;
    system(command.c_str());

    command = "mkdir -p " + path_imu_data_;
    system(command.c_str());
    std::cout << command << std::endl;

    command = "mkdir -p " + path_poseground_;
    system(command.c_str());
    std::cout << command << std::endl;

    // open .csv file
    fout_image_.open(path_image_csv_ + "\\data.csv", std::ios::out);
    if (!fout_image_.is_open()) {
        std::cerr << "cannot open the file: " << path_image_csv_ << std::endl;
    }
    else {
        fout_image_ << "#timestamp[ns], filename" << std::endl;
    }

    fout_imu_.open(path_imu_data_ + "\\data.csv", std::ios::out);
    if (!fout_imu_.is_open()) {
        std::cerr << "cannot open the file: " << path_imu_data_ << std::endl;
    }
    else {
        fout_imu_ << "#timestamp [ns],w_RS_S_x [rad s^-1],w_RS_S_y [rad s^-1],w_RS_S_z [rad s^-1],a_RS_S_x [m s^-2],a_RS_S_y [m s^-2],a_RS_S_z [m s^-2]" << std::endl;
    }

    fout_poseground_.open(path_poseground_ + "\\data.csv", std::ios::out);
    if (!fout_poseground_.is_open()) {
        std::cerr << "cannot open the file: " << path_poseground_ << std::endl;
    }
    else {
        fout_poseground_ << "#timestamp [ns],p_RS_R_x [m],p_RS_R_y [m],p_RS_R_z [m]" << std::endl;
    }
    // duration_image_ = 1000 / hz_image_;
    // duration_imu_ = 1000 / hz_imu_;
}


AirSimVISave::~AirSimVISave(){
    fout_image_.close();
    fout_imu_.close();
    fout_poseground_.close();
}

void AirSimVISave::save_image_thread() {
    std::cout << "Request image" << std::endl;
    time_t start, end;
    while(!isStop_) { // TODO::isStop seems not work, isStop is modified in main stack but this thread still run
        time(&start);
        const vector<ImageResponse>& response = client_image_.simGetImages(image_request_);
        if (response.size() > 0) {
            for (const ImageResponse& image_info : response) {
                std::string file_path = common_utils::FileSystem::combine(path_image_data_, std::to_string(image_info.time_stamp));
                //std::cout << file_path << std::endl;               
                std::ofstream file(file_path + ".png", std::ios::binary);
                file.write(reinterpret_cast<const char*>(image_info.image_data_uint8.data()), image_info.image_data_uint8.size());
                file.close();
                fout_image_ << std::to_string(image_info.time_stamp) << ',' << std::to_string(image_info.time_stamp) + ".png" << std::endl;
            }
        }
        time(&end);
        //double duration = difftime(end, start);

        //std::cout << "image cost time: " << std::fixed << std::setprecision(9) << duration << std::endl;
        std::this_thread::sleep_for(std::chrono::milliseconds(30));
    }
    
}

void AirSimVISave::save_imu_thread() {
    std::cout << "Request imu" << std::endl;
    while (!isStop_) {
        auto imu_data = client_imu_.getImuData();
        string time_stamp = std::to_string(imu_data.time_stamp);
        fout_imu_ << time_stamp << "," << imu_data.angular_velocity.x() << ","
             << imu_data.angular_velocity.y() << ","
             << imu_data.angular_velocity.z() << ","
             << imu_data.linear_acceleration.x() << ","
             << imu_data.linear_acceleration.y() << ","
             << imu_data.linear_acceleration.z() << std::endl;
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
    
}


void AirSimVISave::save_poseground(){
    std::cout << "Request poseground" << std::endl;
    msr::airlib::MultirotorState state;
    while (!isStop_) {
        state = client_poseground_.getMultirotorState();
        string time_stamp = std::to_string(state.timestamp);
        fout_poseground_ << time_stamp << "," 
            << state.getPosition().x() << ","
            << state.getPosition().y() << ","
            << state.getPosition().z()
            << std::endl;
        std::this_thread::sleep_for(std::chrono::milliseconds(30));
    }
}