#include "ros/ros.h"
#include "sensor_msgs/Joy.h"

namespace joy {
struct PS5ButtonMap {
  enum Button {
    Cross = 0,
    Circle = 1,
    Triangle = 2,
    Square = 3,
    L1 = 4,
    R1 = 5,
    L2 = 6,
    R2 = 7,
    Share = 8,
    Options = 9,
    PS = 10,
    L3 = 11,
    R3 = 12,

  };
  enum Axis {
    L3Horizontal = 0,
    L3Vertical = 1,
    L2Pressure = 2,
    R3Horizontal = 3,
    R3Vertical = 4,
    R2Pressure = 5,
    Left = 6,
    Right = 6,
    Up = 7,
    Down = 7
  };
};
class PS5Joy {
public:
  PS5Joy(ros::NodeHandle &nh) {
    joy_sub_ = nh.subscribe("/joy", 10, &PS5Joy::joyCallback, this);
    buttons_.fill(false);
    axes_.fill(0.0);
  }

  bool getButtonState(PS5ButtonMap::Button button) const {
    return buttons_[button];
  }

  double getAxisValue(PS5ButtonMap::Axis axis) const { return axes_[axis]; }

private:
  ros::Subscriber joy_sub_;
  std::array<bool, 13>
      buttons_; // Corresponds to the number of buttons in PS5ButtonMap::Button
  std::array<double, 8>
      axes_; // Corresponds to the number of axes in PS5ButtonMap::Axis

  void joyCallback(const sensor_msgs::JoyConstPtr &msg) {
    for (size_t i = 0; i < buttons_.size(); ++i) {
      buttons_[i] = msg->buttons[i] == 1; // Update button states
    }

    for (size_t i = 0; i < axes_.size(); ++i) {
      axes_[i] = msg->axes[i]; // Update axis values
    }
  }
};
} // namespace joy

int main(int argc, char **argv) {
  ros::init(argc, argv, "ps5_joy_test");
  ros::NodeHandle nh;

  joy::PS5Joy ps5joy(nh);

  ros::Rate loop_rate(10); // Set a suitable rate to check the joystick status

  while (ros::ok()) {
    // Print button states
    std::cout << "Button States:" << std::endl;
    std::cout << "Cross: "
              << (ps5joy.getButtonState(joy::PS5ButtonMap::Cross) ? "Pressed"
                                                                  : "Released")
              << std::endl;
    std::cout << "Circle: "
              << (ps5joy.getButtonState(joy::PS5ButtonMap::Circle) ? "Pressed"
                                                                   : "Released")
              << std::endl;
    std::cout << "Triangle: "
              << (ps5joy.getButtonState(joy::PS5ButtonMap::Triangle)
                      ? "Pressed"
                      : "Released")
              << std::endl;
    std::cout << "Square: "
              << (ps5joy.getButtonState(joy::PS5ButtonMap::Square) ? "Pressed"
                                                                   : "Released")
              << std::endl;

    // Print axis values
    std::cout << "Axis Values:" << std::endl;
    std::cout << "L3 Horizontal: "
              << ps5joy.getAxisValue(joy::PS5ButtonMap::L3Horizontal)
              << std::endl;
    std::cout << "L3 Vertical: "
              << ps5joy.getAxisValue(joy::PS5ButtonMap::L3Vertical)
              << std::endl;
    std::cout << "L2 Pressure: "
              << ps5joy.getAxisValue(joy::PS5ButtonMap::L2Pressure)
              << std::endl;

    ros::spinOnce(); // Handle callbacks
    loop_rate.sleep();
  }

  return 0;
}
