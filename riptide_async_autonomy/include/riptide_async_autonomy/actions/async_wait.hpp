#include <riptide_async_autonomy/UWRTNodes.hpp>
#include <iostream>

using namespace BT;

class AsyncWait : public UWRTAsyncActionNode
{
public:
    AsyncWait(const std::string &name, const BT::NodeConfiguration &config)
        : UWRTAsyncActionNode(name, config) {}

    NodeStatus onStart()
    {
        double seconds = getInput<double>("seconds").value();

        start = rosnode->get_clock()->now();

        // std::cout << "started timer for " << seconds << " at " << start.seconds() << std::endl;

        return NodeStatus::RUNNING;
    }

    NodeStatus onRunning()
    {
        double seconds = getInput<double>("seconds").value();
        // std::cout << "checking difference " << (rosnode->get_clock()->now() - start).seconds() << std::endl;

        if ((rosnode->get_clock()->now() - start).seconds() < seconds)
        {
            return NodeStatus::RUNNING;
        }
        
        return NodeStatus::SUCCESS;
    }

    static PortsList providedPorts()
    {
        return {
            BT::InputPort<double>("seconds")};
    }

private:
    rclcpp::Time start;
};
