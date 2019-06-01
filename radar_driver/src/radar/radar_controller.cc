#include "radar_controller.h"

namespace radar {
  RadarController::~RadarController(){
        ros::waitForShutdown();
        Stop();
  }
  void RadarController::Stop(){
      if(IsRunning())
      {
          is_running_ = false;
          if(can_recv_thread_ != nullptr && can_recv_thread_->joinable())
          {
              can_recv_thread_->join();
          }
          can_recv_thread_.reset();
      }
      if(can_reader_->IsOpen())
      {
          can_reader_->Stop();
      }
      if(can_writer_->IsOpen())
      {
          can_writer_->Stop();
      }
  }
  void RadarController::SendFunc(const CanFrame* const frame){
      return_statuses ret;

      if (can_writer_->IsOpen())
      {
          ret = can_writer_->write(frame);
      }
  }

  bool RadarController::IsRunning() const{
    return is_running_;
  }

  void RadarController::printCanFrameData(const CanFrame& frame){
    
    std::cout<<"Frmaes ID: "<< frame.id << " DLC: "<< frame.dlc<< " Flag: " << frame.is_extended <<std::endl;
    for(int i =0 ;i<8;i++)
        std::cout<< static_cast<unsigned int>(frame.data[i])<<" ";
    std::cout<< std::endl;
  }
}
