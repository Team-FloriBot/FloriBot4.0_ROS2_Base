#include "plc_connection/plc_connection.hpp"
#include "network/udp/udp_socket.hpp"
#include "network/socket/socket.hpp"
#include "rclcpp/rclcpp.hpp"

#include <cstdlib>
#include <memory>
#include <cmath>

// Konstruktor des Nodes
PlcConnectionNode::PlcConnectionNode()
: Node("PlcConnectionNode"),
  ConnectionTimeout_(rclcpp::Duration::from_seconds(1.5))
{
    seq_ = 0;

    // Datenstruktur vollständig initialisieren
    Data_ = {};

    // Parameter lesen
    ReadParams();

    // Timeout nach dem Lesen der Parameter setzen
    ConnectionTimeout_ = rclcpp::Duration::from_seconds(PLCTimeout_);

    // Socket initialisieren, Subscriber und Publisher erstellen
    InitializeSocket();
    Subscribe();
    CreatePublisher();

    // Service zum Abrufen des Zählerstands erstellen
    CountServer_ = this->create_service<plc_connection::srv::GetCount>(
        "sensors/bodyAngle/getCounter",
        std::bind(
            &PlcConnectionNode::GetCountService,
            this,
            std::placeholders::_1,
            std::placeholders::_2));

    // Timer für zyklisches Senden und Empfangen erstellen
    SendRecvTimer_ = this->create_wall_timer(
        std::chrono::duration<double>(readWritePeriod_),
        std::bind(&PlcConnectionNode::SendRecv, this));
}

// Destruktor des Nodes
PlcConnectionNode::~PlcConnectionNode() {}

// Initialisiert den UDP-Socket für die Kommunikation mit der PLC
void PlcConnectionNode::InitializeSocket()
{
    OwnUDP::Address tmpAddress;

    ConnectionTimeout_ = rclcpp::Duration::from_seconds(PLCTimeout_);

    RCLCPP_INFO(
        this->get_logger(),
        "PLC Timeout is set to %f seconds",
        PLCTimeout_);

    Target_.IP.IP = strTargetIP_;
    Target_.IP.Port = TargetPort_;

    RCLCPP_INFO(
        this->get_logger(),
        "PLC IP is %s:%i",
        strTargetIP_.c_str(),
        TargetPort_);

    Target_.LastID = 0;
    Target_.ComOk = false;
    Target_.LastMsgTime = this->get_clock()->now();

    tmpAddress.IP = strOwnIP_;
    tmpAddress.Port = OwnPort_;

    RCLCPP_INFO(this->get_logger(), "Creating UDP Socket");

    try
    {
        PLC_Socket_.bindAddress(&tmpAddress);

        RCLCPP_INFO(
            this->get_logger(),
            "UDP Socket created on Port %i",
            OwnPort_);

        PLC_Socket_.setReceiveTime(ReceiveTimeoutUsec_, ReceiveTimeoutSec_);

        RCLCPP_INFO(
            this->get_logger(),
            "Receive Timeout for UDP Socket is set to %i seconds and %i usec",
            ReceiveTimeoutSec_,
            ReceiveTimeoutUsec_);
    }
    catch (const std::runtime_error& e)
    {

    RCLCPP_ERROR(
        this->get_logger(),
        "Error while creating UDP Socket on Port %i: %s",
        OwnPort_,
        e.what());

        throw;
    }
}

// Liest die Parameter aus der ROS2-Parameterliste
void PlcConnectionNode::ReadParams()
{
    this->declare_parameter<std::string>("PLC_IP", "192.168.0.43");
    this->declare_parameter<std::string>("Xavier_IP", "");
    this->declare_parameter<int>("PLC_Port", 5000);
    this->declare_parameter<int>("Xavier_Port", 5000);
    this->declare_parameter<double>("PLC_Timeout", 1.5);

    this->declare_parameter<int>("Receive_Timeout_sec", 0);

    // 500 us ist für UDP-Kommunikation sehr knapp.
    // 50000 us = 50 ms.
    this->declare_parameter<int>("Receive_Timeout_usec", 50000);

    this->declare_parameter<double>("Period_Send_Read", 0.05);
    this->declare_parameter<int>("ZeroCount_Encoder", 0);
    this->declare_parameter<float>("CountPerRotation_Encoder", 20000.0);
    this->declare_parameter<float>("Engine_Acceleration", 0.0);
    this->declare_parameter<float>("Engine_Jerk", 0.0);

    this->get_parameter("PLC_IP", strTargetIP_);
    this->get_parameter("Xavier_IP", strOwnIP_);
    this->get_parameter("PLC_Port", TargetPort_);
    this->get_parameter("Xavier_Port", OwnPort_);
    this->get_parameter("PLC_Timeout", PLCTimeout_);
    this->get_parameter("Receive_Timeout_sec", ReceiveTimeoutSec_);
    this->get_parameter("Receive_Timeout_usec", ReceiveTimeoutUsec_);
    this->get_parameter("Period_Send_Read", readWritePeriod_);
    this->get_parameter("ZeroCount_Encoder", zeroCount_);

    this->get_parameter("CountPerRotation_Encoder", countPerRotation_);
    this->get_parameter("Engine_Acceleration", Data_.To.Accelleration);
    this->get_parameter("Engine_Jerk", Data_.To.Jerk);

    if (readWritePeriod_ <= 0.0)
    {
        RCLCPP_WARN(
            this->get_logger(),
            "Period_Send_Read <= 0. Using fallback value 0.05 s.");

        readWritePeriod_ = 0.05;
    }

    if (PLCTimeout_ <= 0.0)
    {
        RCLCPP_WARN(
            this->get_logger(),
            "PLC_Timeout <= 0. Using fallback value 1.5 s.");

        PLCTimeout_ = 1.5;
    }

    if (countPerRotation_ == 0.0f)
    {
        RCLCPP_WARN(
            this->get_logger(),
            "CountPerRotation_Encoder is 0. Using fallback value 20000.");

        countPerRotation_ = 20000.0f;
    }

    ConnectionTimeout_ = rclcpp::Duration::from_seconds(PLCTimeout_);
}

// Erstellt die Subscriber für Sollgeschwindigkeit und Modus
void PlcConnectionNode::Subscribe()
{
    SpeedSubscriber_ = this->create_subscription<base::msg::Wheels>(
        "engine/targetSpeed",
        1,
        std::bind(
            &PlcConnectionNode::SpeedCallback,
            this,
            std::placeholders::_1));

    ModeSubscriber_ = this->create_subscription<std_msgs::msg::UInt32>(
        "engine/mode",
        1,
        std::bind(
            &PlcConnectionNode::ModeCallback,
            this,
            std::placeholders::_1));
}

// Erstellt die Publisher für Istgeschwindigkeit und Knickwinkel
void PlcConnectionNode::CreatePublisher()
{
    SpeedPublisher_ = this->create_publisher<base::msg::Wheels>(
        "engine/actualSpeed",
        1);

    AnglePublisher_ = this->create_publisher<base::msg::Angle>(
        "sensors/bodyAngle",
        1);
}

// Callback-Funktion für Sollgeschwindigkeit
void PlcConnectionNode::SpeedCallback(const base::msg::Wheels::SharedPtr msg)
{
    Data_.To.Speed[0] = msg->front_right;
    Data_.To.Speed[1] = msg->front_left;
    Data_.To.Speed[2] = msg->rear_right;
    Data_.To.Speed[3] = msg->rear_left;
}

// Callback-Funktion für Betriebsmodus
void PlcConnectionNode::ModeCallback(const std_msgs::msg::UInt32::SharedPtr msg)
{
    Data_.To.Mode = msg->data;
}

// Service-Funktion zum Abrufen des Encoder-Zählerstands
bool PlcConnectionNode::GetCountService(
    const std::shared_ptr<plc_connection::srv::GetCount::Request> request,
    std::shared_ptr<plc_connection::srv::GetCount::Response> response)
{
    (void)request;

    response->count = Data_.From.Angle;
    return true;
}

// Zyklisches Senden und Empfangen von PLC-Daten
void PlcConnectionNode::SendRecv()
{
    try
    {
        SendData();
        ReadData();
    }
    catch (const std::runtime_error& e)
    {
        RCLCPP_WARN_THROTTLE(
            this->get_logger(),
            *this->get_clock(),
            1000,
            "PLC SendRecv failed: %s",
            e.what());
    }

    // Letzte gültige Daten werden weiterhin publiziert.
    PublishData();
}

// Sendet Daten an die PLC
void PlcConnectionNode::SendData()
{
    PLC_Data tmpData{};

    htonPLC(&tmpData, &Data_);

    PLC_Socket_.write(
        reinterpret_cast<uint8_t*>(&tmpData.To),
        sizeof(Data_.To),
        &Target_.IP);
}

// Empfängt Daten von der PLC
bool PlcConnectionNode::ReadData()
{
    PLC_Data tmpData{};
    OwnUDP::Address tmpAddress{};

    PLC_Socket_.read(
        reinterpret_cast<uint8_t*>(&tmpData.From),
        sizeof(Data_.From),
        &tmpAddress);

    const uint32_t receivedMessageId = ntohl(tmpData.From.MessageID);

    if (receivedMessageId == Target_.LastID)
    {
        if ((this->get_clock()->now() - Target_.LastMsgTime).seconds() >
            ConnectionTimeout_.seconds())
        {
            if (Target_.ComOk == true)
            {
                RCLCPP_ERROR(this->get_logger(), "No Connection to PLC");
            }

            Target_.ComOk = false;
        }

        return false;
    }

    ntohPLC(&Data_, &tmpData);

    Target_.ComOk = true;
    Target_.LastID = Data_.From.MessageID;
    Target_.LastMsgTime = this->get_clock()->now();

    return true;
}

// Veröffentlicht Istgeschwindigkeit und Knickwinkel
void PlcConnectionNode::PublishData()
{
    base::msg::Angle AngleMsg;
    base::msg::Wheels SpeedMsg;

    const float angle =
        (((static_cast<float>(Data_.From.Angle) - static_cast<float>(zeroCount_)) /
          countPerRotation_) *
         2.0f *
         static_cast<float>(M_PI));

    const auto now = this->get_clock()->now();

    SpeedMsg.header.stamp = now;
    AngleMsg.header.stamp = now;

    SpeedMsg.front_right = Data_.From.Speed[0];
    SpeedMsg.front_left  = Data_.From.Speed[1];
    SpeedMsg.rear_right  = Data_.From.Speed[2];
    SpeedMsg.rear_left   = Data_.From.Speed[3];

    AngleMsg.angle = angle;

    SpeedPublisher_->publish(SpeedMsg);
    AnglePublisher_->publish(AngleMsg);
}

// Konvertiert Netzwerkdaten in Host-Daten
void ntohPLC(PLC_Data* Host, PLC_Data* Network)
{
    if (Host == nullptr || Network == nullptr)
    {
        return;
    }

    Host->From.MessageID = ntohl(Network->From.MessageID);
    Host->From.Mode = ntohl(Network->From.Mode);
    Host->From.Angle = ntohl(Network->From.Angle);
    Host->From.Voltage = OwnSocket::ntohf(Network->From.Voltage);
    Host->From.HomingError = ntohl(Network->From.HomingError);
    Host->From.Angle_rad = OwnSocket::ntohf(Network->From.Angle_rad);

    for (int i = 0; i < 4; i++)
    {
        Host->From.Speed[i] = OwnSocket::ntohf(Network->From.Speed[i]);
        Host->From.SpeedError[i] = ntohl(Network->From.SpeedError[i]);
        Host->From.ResetError[i] = ntohl(Network->From.ResetError[i]);
    }
}

// Konvertiert Host-Daten in Netzwerkdaten
void htonPLC(PLC_Data* Network, PLC_Data* Host)
{
    if (Host == nullptr || Network == nullptr)
    {
        return;
    }

    Network->To.MessageID = htonl(Host->To.MessageID++);
    Network->To.Mode = htonl(Host->To.Mode);
    Network->To.Jerk = OwnSocket::htonf(Host->To.Jerk);
    Network->To.Accelleration = OwnSocket::htonf(Host->To.Accelleration);

    for (int i = 0; i < 4; i++)
    {
        Network->To.Speed[i] = OwnSocket::htonf(Host->To.Speed[i]);
        Network->To.Dummy[i] = OwnSocket::htonf(Host->To.Dummy[i]);
    }
}
