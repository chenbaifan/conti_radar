#ifndef RADAR_CONTROLLER_H
#define RADAR_CONTROLLER_H

#include<thread>
#include<mutex>
#include<chrono>

#include<ros/ros.h>
#include<macro.h>
#include<can_interface/kvaser_interface.h>

namespace radar_driver{

class RadarController{
    public:
        RadarController(){};
        virtual ~RadarController();
        /**
        * @brief init can_reader, can_writer, RecvThreadFunc, publisher & subscribers 
        */
        virtual void init(ros::NodeHandle &nh) = 0;
        /**
        * @brief Overide this function in derived Class
        * called from can_recv_thread_ to read can frames from can bus
        */
        virtual void RecvThreadFunc() = 0;
        /**
        * @brief Called this function to send data to canbus
        * used after encoding data to can frame
        */
        void SendFunc(const CanFrame* const frame);
    protected:
        /**
        * @brief Utill function to print can frame data
        */
        void printCanFrameData(const CanFrame& frame);
        /**
        * @brief Check for thread running
        */
        bool IsRunning() const;
        
        //CAN interface parameter
        CANCardParameter param_;
        
        //CAN reader and writers, init in derived class
        boost::shared_ptr<radar_driver::KvaserInterface> can_reader_, can_writer_;
        
        //CAN recieve thread pointer
        std::unique_ptr<std::thread> can_recv_thread_ = nullptr;
        
        //ros publisher
        ros::Publisher can_fbk_;
        
        //ros Subscriber
        ros::Subscriber can_cmd_;
        ros::Subscriber can_cmd_aux_;
        
        //bool variable to check for thread running
        bool is_running_ = false;
    private:
        /**
        * @brief Close the can_reader_, can_writer_ and can_recv_thread_
        */
    void Stop(); 
};
}


#endif 