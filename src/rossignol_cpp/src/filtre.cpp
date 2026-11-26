#include <array>
#include <cmath>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/pose2_d.hpp"

// noeud du filtre
class Filtre : public rclcpp::Node
{
public:
  Filtre()
  : Node("filtre")
  {
    publisher_ = create_publisher<geometry_msgs::msg::Pose2D>("position_filtree", 10);
    pub_pos_cmd_ = create_publisher<geometry_msgs::msg::Pose2D>("position_commande", 10);

    sub_pos_ = create_subscription<geometry_msgs::msg::Pose2D>(
      "position_arUco_plan", 10,
      std::bind(&Filtre::cb_position, this, std::placeholders::_1));

    sub_cmd_ = create_subscription<std_msgs::msg::String>(
      "commande_roue", 10,
      std::bind(&Filtre::cb_cmd, this, std::placeholders::_1));

    RCLCPP_INFO(get_logger(), "Noeud filtre (C++) demarre");
  }

private:
  void cb_cmd(const std_msgs::msg::String::SharedPtr msg)
  {
    // avec les commandes des roues on obtient la vitesse lineaire et la vitesse angulaire
    const std::string & s = msg->data;
    const size_t virgule = s.find(',');
    if (virgule == std::string::npos) {
      RCLCPP_WARN(get_logger(), "Commande mal formee : '%s'", s.c_str());
      return;
    }

    double vg = 0.0;
    double vd = 0.0;
    try {
      vg = std::stod(s.substr(0, virgule));
      vd = std::stod(s.substr(virgule + 1));
    } catch (const std::exception & e) {
      RCLCPP_WARN(get_logger(), "Conversion impossible : '%s' (%s)", s.c_str(), e.what());
      return;
    }

    const double v = (vg + vd) / 2.0;
    const double w = (vd - vg) / d_;

    // frequence d'envoi des commandes en sortie du reseau de neurones
    const double delta_t = 0.05;

    // pose estimee via la camera
    const double x_cam = z_[0];
    const double y_cam = z_[1];
    const double theta_cam = z_[2];

    // positions calculees via l'odometrie
    const double odomT = theta_ + w * delta_t;
    const double odomX = x_ + v * std::cos(theta_) * delta_t;
    const double odomY = y_ + v * std::sin(theta_) * delta_t;

    geometry_msgs::msg::Pose2D pose_msg_cmd;
    pose_msg_cmd.x = odomX;
    pose_msg_cmd.y = odomY;
    pose_msg_cmd.theta = odomT;
    pub_pos_cmd_->publish(pose_msg_cmd);

    // si l'ArUco n'est pas visible alors z = -1 pour toutes ses composantes :
    // les donnees de la camera ne sont pas prises en compte.
    // Sinon on fait la moyenne odometrie / vision.
    if (z_[0] != -1.0 && z_[1] != -1.0) {
      theta_ = (odomT + theta_cam) / 2.0;
      x_ = (odomX + x_cam) / 2.0;
      y_ = (odomY + y_cam) / 2.0;
    } else {
      theta_ = odomT;
      x_ = odomX;
      y_ = odomY;
    }

    // publication des coordonnees
    geometry_msgs::msg::Pose2D pose_msg;
    pose_msg.x = x_;
    pose_msg.y = y_;
    pose_msg.theta = theta_;
    publisher_->publish(pose_msg);
  }

  void cb_position(const geometry_msgs::msg::Pose2D::SharedPtr msg)
  {
    // lorsqu'on recoit une position via la camera
    z_ = {msg->x, msg->y, msg->theta};
  }

  rclcpp::Publisher<geometry_msgs::msg::Pose2D>::SharedPtr publisher_;
  rclcpp::Publisher<geometry_msgs::msg::Pose2D>::SharedPtr pub_pos_cmd_;
  rclcpp::Subscription<geometry_msgs::msg::Pose2D>::SharedPtr sub_pos_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_cmd_;

  // Parametres 
  const double d_ = 10.0;                        
  std::array<double, 3> z_ = {-1.0, -1.0, -1.0};   
  double x_ = -0.4;
  double y_ = 1.5;
  double theta_ = 0.0;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Filtre>());
  rclcpp::shutdown();
  return 0;
}
