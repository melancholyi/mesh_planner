#include <rclcpp/rclcpp.hpp>    

namespace mesh_bgk{
  class MeshBGK: public rclcpp::Node{
    public:
    MeshBGK(const rclcpp::NodeOptions& options): Node("mesh_bgk_node", options){
      RCLCPP_INFO(this->get_logger(),"mesh_bgk_node starting!!");
    }

    private:

  } ;
}   
