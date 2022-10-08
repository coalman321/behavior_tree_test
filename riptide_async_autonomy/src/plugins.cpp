#include <riptide_async_autonomy/UWRTNodes.hpp>
#include <riptide_async_autonomy/actions/async_wait.hpp>

#include "behaviortree_cpp_v3/bt_factory.h"

BT_REGISTER_NODES(factory)
{
    factory.registerNodeType<AsyncWait>("AsyncWait");
}