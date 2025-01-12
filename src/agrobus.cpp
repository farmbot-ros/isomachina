#include "isobus/hardware_integration/can_hardware_interface.hpp"
#include "isobus/hardware_integration/socket_can_interface.hpp"
#include "isobus/isobus/can_NAME.hpp"
#include "isobus/isobus/can_network_manager.hpp"
#include "isobus/isobus/can_partnered_control_function.hpp"
#include "rclcpp/rclcpp.hpp"
#include <memory>
#include <optional>
#include <spdlog/spdlog.h>

namespace echo = spdlog;
using namespace std::chrono_literals;
using namespace std::placeholders;

class CanBus {
  private:
    std::string interface_;
    bool started_ = false;
    std::shared_ptr<isobus::NAME> isocont;
    std::shared_ptr<isobus::SocketCANInterface> bus;
    std::shared_ptr<isobus::InternalControlFunction> ecu;

  public:
    CanBus(std::string interface = "vcan0", bool autostart = false) : interface_(interface) {
        setup();
        if (autostart) {
            started_ = start();
        }
        echo::info("ISOBUS at {} initialized", interface_);
    }

    void stop() { isobus::CANHardwareInterface::stop(); }

    void setup() {
        echo::info("setting up ISOBUS control function");
        isocont = std::make_shared<isobus::NAME>(0);
        isocont->set_arbitrary_address_capable(true);
        isocont->set_industry_group(1);
        isocont->set_device_class(0);
        isocont->set_function_code(static_cast<std::uint8_t>(isobus::NAME::Function::SteeringControl));
        isocont->set_identity_number(2);
        isocont->set_ecu_instance(0);
        isocont->set_function_instance(0);
        isocont->set_device_class_instance(0);
        isocont->set_manufacturer_code(1407);

        bus = std::make_shared<isobus::SocketCANInterface>(interface_);
        isobus::CANHardwareInterface::set_number_of_can_channels(1);
        isobus::CANHardwareInterface::assign_can_channel_frame_handler(0, bus);
    }

    bool start() {
        if ((!isobus::CANHardwareInterface::start()) || (!bus->get_is_valid())) {
            echo::warn("Failed to start CAN hardware interface");
            return false;
        }
        ecu = isobus::CANNetworkManager::CANNetwork.create_internal_control_function(*isocont, 0);
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
        echo::info("Starting CAN bus");
        return true;
    }

    std::optional<bool> send_message(int address = 0xEF00, std::array<std::uint8_t, isobus::CAN_DATA_LENGTH> message = {0}) {
        if (!started_) {
            echo::warn("CAN bus not started");
            return false;
        }
        isobus::CANNetworkManager::CANNetwork.send_can_message(address, message.data(), isobus::CAN_DATA_LENGTH, ecu);
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        echo::info("Message sent");
        return true;
    }
};

class AgroBus {
  private:
    std::string namespace_;
    rclcpp::Node::SharedPtr node;
    std::shared_ptr<CanBus> canbus;
    rclcpp::TimerBase::SharedPtr timer_;

  public:
    AgroBus(rclcpp::Node::SharedPtr node) : node(node) {
        namespace_ = node->get_namespace();
        canbus = std::make_shared<CanBus>("vcan0", true);

        timer_ = node->create_wall_timer(1s, std::bind(&AgroBus::send_message, this));
    }

    ~AgroBus() { canbus->stop(); }

  private:
    void send_message() { canbus->send_message(); }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 4);

    rclcpp::NodeOptions options_0;
    options_0.allow_undeclared_parameters(true);
    options_0.automatically_declare_parameters_from_overrides(true);

    auto node_0 = rclcpp::Node::make_shared("agrobus", options_0);
    auto parser = AgroBus(node_0);
    executor.add_node(node_0);

    executor.spin();
    rclcpp::shutdown();
    return 0;
}
