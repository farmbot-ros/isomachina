#include "isobus/hardware_integration/can_hardware_interface.hpp"
#include "isobus/hardware_integration/socket_can_interface.hpp"
#include "isobus/isobus/can_NAME.hpp"
#include "isobus/isobus/can_network_manager.hpp"
#include "isobus/isobus/can_partnered_control_function.hpp"

#include "rclcpp/rclcpp.hpp"
#include <memory>

class AgroBus {
    private:
        std::string namespace_;
        rclcpp::Node::SharedPtr node;
        std::shared_ptr<isobus::NAME> isocont;
    public:
        AgroBus(rclcpp::Node::SharedPtr node, std::shared_ptr<isobus::NAME> isocont) : node(node), isocont(isocont) {
            namespace_ = node->get_namespace();
            setup_isocont();
        }

        void setup_isocont() {
            isocont->set_arbitrary_address_capable(true);
            isocont->set_industry_group(1);
            isocont->set_device_class(0);
            isocont->set_function_code(static_cast<std::uint8_t>(isobus::NAME::Function::SteeringControl));
            isocont->set_identity_number(2);
            isocont->set_ecu_instance(0);
            isocont->set_function_instance(0);
            isocont->set_device_class_instance(0);
            isocont->set_manufacturer_code(1407);
        }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 4);

    rclcpp::NodeOptions options_0;
    options_0.allow_undeclared_parameters(true);
    options_0.automatically_declare_parameters_from_overrides(true);
    auto node_0 = rclcpp::Node::make_shared("rerun", options_0);
    std::shared_ptr<isobus::NAME> isocont_0 = std::make_shared<isobus::NAME>(0);

    auto parser = AgroBus(node_0, isocont_0);
    executor.add_node(node_0);

    executor.spin();
    rclcpp::shutdown();
    return 0;
}
