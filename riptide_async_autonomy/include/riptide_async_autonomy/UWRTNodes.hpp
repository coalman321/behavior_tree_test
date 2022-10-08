#pragma once

#include<string>
#include<stack>
#include <typeinfo>

#include <rclcpp/rclcpp.hpp>

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"

#include <riptide_msgs2/msg/tree_stack.hpp>

class UwrtBtNode {
    public:
    void init(rclcpp::Node::SharedPtr node) {
        this->rosnode = node;
    }

    protected:
    rclcpp::Node::SharedPtr rosnode;
};

/**
 * @brief UWRT superclass for integrating SyncActionNodes with ROS. 
 * Uses multiple-inheritance to create a class that has the properties of 
 * both the SyncActionNode and the UWRT general node superclass.
 */
class UWRTSyncActionNode : public BT::SyncActionNode, public UwrtBtNode {
    public:
    UWRTSyncActionNode(const std::string& name, const BT::NodeConfiguration& config)
     : SyncActionNode(name, config) { };

    static BT::PortsList providedPorts();
    virtual BT::NodeStatus tick() = 0;
};

/**
 * @brief UWRT superclass for integrating AsyncActionNodes with ROS. 
 * Uses multiple-inheritance to create a class that has the properties of 
 * both the SyncActionNode and the UWRT general node superclass.
 */
class UWRTAsyncActionNode : public BT::StatefulActionNode, public UwrtBtNode {
    public:
    UWRTAsyncActionNode(const std::string& name, const BT::NodeConfiguration& config)
     : StatefulActionNode(name, config) { };

    static BT::PortsList providedPorts(){ return {};}
    
    BT::NodeStatus onStart(){};
    BT::NodeStatus onRunning(){};
    void onHalted(){};
};

/**
 * @brief UWRT superclass for integrating ConditionNodes with ROS.
 * Similar to UWRTSyncActionNode, this class inherits both the BT 
 * ConditionNode and UwrtBtNode.
 */
class UWRTConditionNode : public BT::ConditionNode, UwrtBtNode {
    public:
    UWRTConditionNode(const std::string& name, const BT::NodeConfiguration& config)
     : ConditionNode(name, config) { };
    
    static BT::PortsList providedPorts();
    virtual BT::NodeStatus tick() = 0;
};

/**
 * @brief UWRT superclass for integrating DecoratorNodes with ROS.
 * Operates exactly the same as UWRTConditionNode and UWRTSyncActionNode.
 */
class UWRTDecoratorNode : public BT::DecoratorNode, UwrtBtNode {
    public:
    UWRTDecoratorNode(const std::string& name, const BT::NodeConfiguration& config)
     : DecoratorNode(name, config) { };
    
    static BT::PortsList providedPorts();
    virtual BT::NodeStatus tick() = 0;
};