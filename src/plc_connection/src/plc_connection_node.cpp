#include "plc_connection/plc_connection.hpp"

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    try
    {
        auto node = std::make_shared<PlcConnectionNode>();
        rclcpp::spin(node);
    }
    catch (const std::exception& e)
    {
        RCLCPP_ERROR(
            rclcpp::get_logger("PLC_Connection"),
            "PLC connection node exited with exception: %s",
            e.what());

        if (rclcpp::ok())
        {
            rclcpp::shutdown();
        }

        return 1;
    }

    if (rclcpp::ok())
    {
        rclcpp::shutdown();
    }

    return 0;
}
