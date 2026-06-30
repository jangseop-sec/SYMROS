#include <memory>
#include <cstring>
#include <cmath>

#include <sys/mman.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>

// SYMROS message headers
// SYMROS message headers


// SYMROS target include headers
// SYMROS target include headers

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rcl_action/rcl_action.h"
#include "rmw/types.h"

#include "symros/symros.hpp"
#include "symros_string.hpp"
#include "symros_math.hpp"


// linux/unix file io
#include <fcntl.h>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>

#define MAX_LEN 64
#define private public
#define protected public

using namespace symros;

// symros macro
#define SYMROS__INIT(is_target_) \
  SymROSManager::get_instance().init(is_target_);

#define SYMROS__DECLARE_SUBSCRIPTION(message_namespace_, subscription_name_, index_) \
  auto subscription_name_ = SymROSManager::get_instance().get_subscription<message_namespace_>(index_);

#define SYMROS__CALL_SUBSCRIPTION_CALLBACK(subscription_name_, message_name_, message_info_) \
  rclcpp::MessageInfo message_info_; \
  symcc_reset_constraints();   \
  subscription_name_->get_any_callback().dispatch(message_name_, message_info_);

#define SYMROS__DECLARE_SERVICE(service_namespace_, service_name_, index_) \
  auto service_name_ = SymROSManager::get_instance().get_service<service_namespace_>(index_);

#define SYMROS__CALL_SERVICE_CALLBACK(service_, request_ptr_)                             \
  do {                                                                                    \
    auto __req_info = std::make_shared<rmw_request_id_t>();                               \
    symcc_reset_constraints();                                                            \
    (service_)->get_any_callback().dispatch((service_), __req_info, (request_ptr_));      \
  } while (0)

#define SYMROS__DECLARE_ACTION_SERVER(service_namespace_, service_name_, index_) \
  auto service_name_ = SymROSManager::get_instance().get_action_server(index_);

template<typename ActionT>
void SYMROS_CallActionServerCallback(
    std::shared_ptr<rclcpp_action::ServerBase> action_server,
    std::shared_ptr<typename ActionT::Goal> goal_ptr)
{
    using RequestT = typename ActionT::Impl::SendGoalService::Request;

    rclcpp_action::GoalUUID uuid{};
    for (size_t i = 0; i < uuid.size(); i++) {
        uuid[i] = static_cast<uint8_t>(i + 1);
    }

    auto request = std::make_shared<RequestT>();
    request->goal = *goal_ptr;
    request->goal_id.uuid = uuid;

    std::shared_ptr<void> msg = request;

    rcl_action_goal_info_t goal_info{};
    for (size_t i = 0; i < uuid.size(); i++) {
        goal_info.goal_id.uuid[i] = uuid[i];
    }

    rmw_request_id_t header{};
    memset(&header, 0, sizeof(header));

    auto gdata = std::make_tuple(
        RCL_RET_OK,
        goal_info,
        header,
        msg
    );

    // 6. ServerBaseData wrapping
    auto data = std::make_shared<rclcpp_action::ServerBaseData>(
        std::move(gdata)
    );

    action_server->execute(reinterpret_cast<std::shared_ptr<void>&>(data));
}

extern "C" void symcc_make_symbolic_with_prefix(const void *start, size_t byte_length, const char * prefix);
extern "C" void symcc_reset_constraints();

bool STATE_MODE = false;

void symros_print_field(const std::string& name, const void* data, const std::string& type, size_t size) {}

void symros_make_symbolic_with_prefix(void * data, size_t size, int fd, std::string name, int is_target, std::string type) {

  int n = read(fd, data, size);
  if (n == -1) {
    perror("read failed");
    exit(1);
  }

  // STATE_MODE -> symbolize state + pre callback's message
  // !STATE_MODE -> symbolize target message
  if ((!STATE_MODE && is_target != 1) || (STATE_MODE && is_target != 2)) {
    void * tmp = malloc(size);
    symcc_make_symbolic_with_prefix(tmp, size, "tmp");
    free(tmp);
  } else {
    symcc_make_symbolic_with_prefix(data, size, name.c_str());
  }
  
  symros_print_field(name, data, type, size);
}

// SYMROS setup
// SYMROS setup

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);

  // SYMROS executor
  // SYMROS executor

  exit(0);
}