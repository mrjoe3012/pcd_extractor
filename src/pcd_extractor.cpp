#include <iostream>
#include <string>
#include <mutex>
#include <thread>
#include <stdexcept>
#include <functional>
#include <deque>

#include <rclcpp/rclcpp.hpp>
#include "sensor_msgs/msg/point_cloud2.hpp"
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/io/pcd_io.h>

// huge queue so we don't drop any msgs
const int MSG_QUEUE_SIZE = 1000;

typedef pcl::PCLPointCloud2 PointCloud;

std::string outputDirectory;

std::deque<PointCloud> cloudQueue;
std::mutex mCloudQueue;

std::exception_ptr ePtr;

bool killWorkerThread = false;

class PCDExtractor : public rclcpp::Node
{

public:
  PCDExtractor()
  : Node("pcd_extractor")
  {
    // topic to listen to
    this->declare_parameter<std::string>("topic", "/velodyne_points");
    // output directory
    this->declare_parameter<std::string>("output", "./");

    std::string topicName = this->get_parameter("topic").as_string();
    pSub = this->create_subscription<sensor_msgs::msg::PointCloud2>(topicName, MSG_QUEUE_SIZE, std::bind(&PCDExtractor::callback, this, std::placeholders::_1));

    outputDirectory = this->get_parameter("output").as_string();

    std::cout << "topicName : " << topicName << std::endl;
    std::cout << "outputDirectory : " << outputDirectory << std::endl;

  }

  void callback(const sensor_msgs::msg::PointCloud2::SharedPtr pPc2)
  {
    pcl::PCLPointCloud2 pclpc2;
    pcl_conversions::toPCL(*pPc2, pclpc2);
    mCloudQueue.lock();
    cloudQueue.push_back(pclpc2);
    mCloudQueue.unlock();
  }


private:
	rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pSub;

};

// constantly attempts to pull a point cloud off of the queue and write it to the disk
void writeCloudToDisk()
{

  try
  {

    int i = 0;

    while(!killWorkerThread)
    {

      if(mCloudQueue.try_lock())
      {
        size_t size = cloudQueue.size();
        if(size > 0)
        {
          PointCloud cloud = cloudQueue.front();
          cloudQueue.pop_front();
          mCloudQueue.unlock();
          
          pcl::io::savePCDFile(outputDirectory + "/frame_" + std::to_string(i++) + ".pcd", cloud, Eigen::Vector4f::Zero(), Eigen::Quaternionf::Identity(), true);        
        }
        else
          mCloudQueue.unlock();      
      }

    }

  }
  catch(...)
  {
    ePtr = std::current_exception();
  }

}

int main(int argc, char** argv)
{
  std::thread workerThread(writeCloudToDisk);

  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PCDExtractor>());
  rclcpp::shutdown();

  killWorkerThread = true;
  workerThread.join();

  // error-handling for worker thread
  if(ePtr)
  {
    std::rethrow_exception(ePtr);
  }

  return 0;
}