#include "base_node/base_publisher.hpp"

void ExitFcn();


int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    // Erstellen des Knoten
    auto node = std::make_shared<KinematicsPublisher>();
    std::atexit(ExitFcn);
    
    try
    {
        // ROS2 spinnt den Knoten
        rclcpp::spin(node);
    }
    catch(const std::runtime_error& e)
    {
        // Fehlerbehandlung in ROS2
        RCLCPP_ERROR(rclcpp::get_logger(node->get_name()), "Exiting with error:\n%s\n", e.what());
        return 1; // Exit mit Fehlercode
    }

    // Aufräumarbeiten bei normalem Abschluss
    rclcpp::shutdown();
}

// ExitFcn gibt den Knotennamen aus, wenn das Programm beendet wird
void ExitFcn()
{
    RCLCPP_ERROR(rclcpp::get_logger("Kinematics"), "Exiting Node: Kinematics");
}
