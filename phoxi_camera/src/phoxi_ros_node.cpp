#include <rclcpp/rclcpp.hpp>
// #include <rclcpp/node.hpp>
// #include <rclcpp/time.hpp>
// #include <rclcpp/logging.hpp>
// #include <rclcpp/parameter.hpp>

#include <cv_bridge/cv_bridge.h>
#include <Eigen/Geometry>
#include <Eigen/Dense>
#include <tf2/convert.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/buffer_interface.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/pcd_io.h>
#include <tf2_eigen/tf2_eigen.h>

#include <phoxi_camera/PhoXiInterface.h>

#include <phoxi_msgs/srv/get_device_list.hpp>
#include <phoxi_msgs/srv/connect_camera.hpp>
#include <phoxi_msgs/srv/is_connected.hpp>
#include <phoxi_msgs/srv/is_acquiring.hpp>
#include <phoxi_msgs/srv/get_bool.hpp>
#include <phoxi_msgs/srv/empty.hpp>
#include <phoxi_msgs/srv/trigger_image.hpp>
#include <phoxi_msgs/srv/get_frame.hpp>
#include <phoxi_msgs/srv/save_frame.hpp>
#include <phoxi_msgs/srv/save_last_frame.hpp>
#include <phoxi_msgs/srv/get_hardware_identification.hpp>
#include <phoxi_msgs/srv/get_supported_capturing_modes.hpp>
#include <phoxi_msgs/srv/set_coordinates_space.hpp>
#include <phoxi_msgs/srv/set_transformation_matrix.hpp>
#include <phoxi_msgs/srv/get_string.hpp>

namespace phoxi_camera
{

class PhoxiRosNode : public rclcpp::Node
{
public:
  // Constructor
  PhoxiRosNode(const rclcpp::NodeOptions & node_options = rclcpp::NodeOptions())
    : Node("phoxi_ros_node", node_options)
  {
    // // Get parameters
    // this->declare_parameter<bool>("start_acquisition_", true);
    // this->declare_parameter<bool>("stop_acquisition_", false);

    // Create service servers
    get_device_list_srv_ = this->create_service<phoxi_msgs::srv::GetDeviceList>("phoxi_camera/get_device_list", std::bind(&PhoxiRosNode::getDeviceList, this, std::placeholders::_1, std::placeholders::_2));

    // // Create publishers
    // cloudPub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/phoxi_camera/points", 1);
    // normalMapPub_ = this->create_publisher<sensor_msgs::msg::Image>("/phoxi_camera/normal_map", topic_queue_size_);
    // confidenceMapPub_ = this->create_publisher<sensor_msgs::msg::Image>("/phoxi_camera/confidence_map", topic_queue_size_);
    // rawTexturePub_ = this->create_publisher<sensor_msgs::msg::Image>("/phoxi_camera/texture", topic_queue_size_);
    // rgbTexturePub_ = this->create_publisher<sensor_msgs::msg::Image>("/phoxi_camera/rgb_texture", topic_queue_size_);
    // depthMapPub_ = this->create_publisher<sensor_msgs::msg::Image>("/phoxi_camera/depth_map", topic_queue_size_);

    RCLCPP_INFO(this->get_logger(), "Ready to get_device_list");
  }

private:
  /////////////////////////////////////////////////////////////////////////////
  /// Variable Declarations
  /////////////////////////////////////////////////////////////////////////////
  // PhoxiInterface object
  phoxi_camera::PhoXiInterface phoxi_;

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloudPub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr normalMapPub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr confidenceMapPub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr rawTexturePub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr rgbTexturePub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr depthMapPub_;

  rclcpp::Service<phoxi_msgs::srv::GetDeviceList>::SharedPtr get_device_list_srv_;

  /////////////////////////////////////////////////////////////////////////////
  /// Function Declarations
  /////////////////////////////////////////////////////////////////////////////
  bool getDeviceList(const std::shared_ptr<phoxi_msgs::srv::GetDeviceList::Request> req,
                     std::shared_ptr<phoxi_msgs::srv::GetDeviceList::Response> res)
  {
    try {
      res->out_id = phoxi_.cameraList();
      // phoXiDeviceInforamtionToRosMsg(PhoXiInterface::deviceList(), res->device_information_list);
      res->len = res->out_id.size();
      res->success = true;
      res->message = "Ok";
    } catch (PhoXiInterfaceException& e) {
      res->success = false;
      res->message = e.what();
    }
    return true;
  }

};  // class PhoxiRosNode

}  // namespace phoxi_camera

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<phoxi_camera::PhoxiRosNode>());
  rclcpp::shutdown();
  return 0;
}
