/// joint_reset — Zero all joint commands in one process.
/// Usage: joint_reset <model_name>
/// Publishes 0.0 to all JointPositionController and JointController
/// topics for the named model, then exits.

#include <gz/transport/Node.hh>
#include <gz/msgs/double.pb.h>
#include <chrono>
#include <string>
#include <thread>
#include <vector>

int main(int argc, char* argv[])
{
    std::string model = (argc > 1) ? argv[1] : "jetacker";
    std::string prefix = "/model/" + model + "/joint/";

    // JointPositionController topics (PID-driven, /0/cmd_pos)
    std::vector<std::string> pos_joints = {
        "turret_joint",
        "front_steering_joint",
        "front_left_wheel_steering_joint",
        "front_right_wheel_steering_joint",
    };

    // JointController topics (direct velocity, /cmd_vel)
    std::vector<std::string> vel_joints = {
        "rear_left_wheel_joint",
        "rear_right_wheel_joint",
    };

    gz::transport::Node node;
    gz::msgs::Double msg;
    msg.set_data(0.0);

    std::vector<gz::transport::Node::Publisher> pubs;

    for (const auto& j : pos_joints)
    {
        auto pub = node.Advertise<gz::msgs::Double>(prefix + j + "/0/cmd_pos");
        pubs.push_back(pub);
    }
    for (const auto& j : vel_joints)
    {
        auto pub = node.Advertise<gz::msgs::Double>(prefix + j + "/cmd_vel");
        pubs.push_back(pub);
    }

    // Brief pause for advertisement propagation
    std::this_thread::sleep_for(std::chrono::milliseconds(50));

    for (auto& pub : pubs)
    {
        pub.Publish(msg);
    }

    // Brief pause for message delivery
    std::this_thread::sleep_for(std::chrono::milliseconds(50));

    return 0;
}
