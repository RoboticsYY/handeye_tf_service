#include <tf2_ros/buffer.h>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_listener.h>
#include <handeye_tf_service/srv/handeye_tf.hpp>
#include <geometry_msgs/msg/transform_stamped.h>

using HandeyeTF = handeye_tf_service::srv::HandeyeTF;

class ServerNode : public rclcpp::Node
{
public:
  explicit ServerNode(const rclcpp::NodeOptions & options)
  : Node("handeye_tf_server", options)
  {
    // Init tf listener
    rclcpp::Clock::SharedPtr clock = std::make_shared<rclcpp::Clock>(RCL_SYSTEM_TIME);
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(clock);
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    auto handle_service =
      [this](const std::shared_ptr<rmw_request_id_t> request_header,
        const std::shared_ptr<HandeyeTF::Request> request,
        std::shared_ptr<HandeyeTF::Response> response) -> void
      {
        (void)request_header;
        RCLCPP_INFO(this->get_logger(), "Incoming request\nframe_id: %s child_frame_id: %s",
          request->frame_id.data(), request->child_frame_id.data());

        try
        {
          response->transform = tf_buffer_->lookupTransform(request->frame_id, request->child_frame_id, tf2::get_now());
        }
        catch (tf2::TransformException &ex)
        {
          std::string temp = ex.what();
          RCLCPP_WARN(this->get_logger(), "%s", temp.c_str());
        }
      };
    // Create a service that will use the callback function to handle requests.
    srv_ = create_service<HandeyeTF>("handeye_tf_service", handle_service);
    RCLCPP_INFO(this->get_logger(), "Handeye TF service created.");
  }

private:
  rclcpp::Service<HandeyeTF>::SharedPtr srv_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ServerNode>(rclcpp::NodeOptions());
  rclcpp::spin(node);
  rclcpp::shutdown();
  node = nullptr;
  return 0;
}