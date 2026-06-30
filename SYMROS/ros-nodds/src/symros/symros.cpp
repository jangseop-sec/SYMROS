#include <iostream>
#include <vector>

#include "symros/symros.hpp"

namespace symros
{
  void 
  SymROSManager::init(bool is_target)
  {
    this->sub_idx = 0;
    this->srv_idx = 0;
    this->act_srv_idx = 0;
    this->is_target = is_target;

    std::cout << "[symros_init] SymROSManager initialized" << std::endl;
  }

  void
  SymROSManager::add_subscription(std::shared_ptr<rclcpp::SubscriptionBase> sub_base)
  {
    if (sub_idx == -1) {
      throw std::runtime_error("SymROSManager not initialized");
    }
    sub_list.push_back(sub_base);
    sub_idx = sub_list.size()-1;
    
    if (is_target) {
      // TODO show result to python symros engine
      auto members = static_cast<const rosidl_typesupport_introspection_cpp::MessageMembers *>(sub_base->get_message_type_support_handle().data);
      std::string topic_namespace_ = members->message_namespace_;
      std::string topic_name_ = members->message_name_;

      std::string ret_prefix = "symros_result$$new_sub$$";
      std::string ret = ret_prefix + std::to_string(sub_idx) + "$$" + sub_base->get_topic_name() + "$$" + topic_namespace_ + "::" + topic_name_ + "\n";
      std::cout << ret;


      std::cout << "[symros_add_subscription] new subscription is created [" << sub_idx << "]" << members->message_namespace_ << "::" << members->message_name_ << std::endl;
    }
  }

  void
  SymROSManager::add_service(std::shared_ptr<rclcpp::ServiceBase> srv_base, std::string service_namespace_, std::string service_name_) {
    if (srv_idx == -1) {
      throw std::runtime_error("SymROSManager not initialized");
    }

    srv_list.push_back(srv_base);
    srv_idx = srv_list.size() - 1;

#ifndef SYMROS_MODE
    if (is_target) {
      std::string ret_prefix = "symros_result$$new_srv$$";
      std::string ret = ret_prefix + std::to_string(srv_idx) + "$$" + srv_base->get_service_name() + "$$" + service_namespace_ + "::" + service_name_  + "\n";
      std::cout << ret;
      std::cout << "[symros_add_service] new service is created [" << srv_idx << "]" << srv_base->get_service_name() << ":" << service_namespace_ << "::" << service_name_ << std::endl;
    }
#endif  // SYMROS_MODE
  }

  void
  SymROSManager::add_action(std::shared_ptr<rclcpp_action::ServerBase> action_server_base, std::string action_id_, std::string action_namespace_, std::string action_name_) {
    if (act_srv_idx == -1) {
      
      throw std::runtime_error("SymROSManager not initialized");
    }

    act_srv_list.push_back(action_server_base);
    act_srv_idx = act_srv_list.size() - 1;

#ifndef SYMROS_MODE
    if (is_target) {
      std::string ret_prefix = "symros_result$$new_action_srv$$";
      std::string ret = ret_prefix + std::to_string(act_srv_idx) + "$$" + action_id_ + "$$" + action_namespace_ + "::" + action_name_  + "\n";
      std::cout << ret;
      std::cout << "[symros_add_action_server] new action server is created [" << act_srv_idx << "]" << action_id_ << ":" << action_namespace_ << "::" << action_name_ << std::endl;
    }
#endif  // SYMROS_MODE
  }

  rclcpp::ParameterValue &
  SymROSManager::add_param(std::string name, std::string type, rclcpp::ParameterValue default_value) {
    #ifndef SYMROS_MODE
    if (is_target) {
      std::string ret_prefix = "symros_result$$new_param$$";
      std::string ret = ret_prefix + name + "$$" + type;
      
      if (default_value.get_type() != rclcpp::ParameterType::PARAMETER_NOT_SET) {
        ret += "$$";

        switch (default_value.get_type()) {
          case rclcpp::ParameterType::PARAMETER_BOOL:
            ret += default_value.get<bool>() ? "true" : "false";
            break;

          case rclcpp::ParameterType::PARAMETER_INTEGER:
            ret += std::to_string(default_value.get<int64_t>());
            break;

          case rclcpp::ParameterType::PARAMETER_DOUBLE:
            ret += std::to_string(default_value.get<double>());
            break;

          case rclcpp::ParameterType::PARAMETER_STRING:
            ret += default_value.get<std::string>();
            break;

          case rclcpp::ParameterType::PARAMETER_BYTE_ARRAY: {
            const auto & v = default_value.get<std::vector<uint8_t>>();
            ret += "[";
            for (size_t i = 0; i < v.size(); ++i) {
              if (i) ret += ", ";
              ret += std::to_string(static_cast<int>(v[i]));
            }
            ret += "]";
            break;
          }

          case rclcpp::ParameterType::PARAMETER_BOOL_ARRAY: {
            const auto & v = default_value.get<std::vector<bool>>();
            ret += "[";
            for (size_t i = 0; i < v.size(); ++i) {
              if (i) ret += ", ";
              ret += v[i] ? "true" : "false";
            }
            ret += "]";
            break;
          }

          case rclcpp::ParameterType::PARAMETER_INTEGER_ARRAY: {
            const auto & v = default_value.get<std::vector<int64_t>>();
            ret += "[";
            for (size_t i = 0; i < v.size(); ++i) {
              if (i) ret += ", ";
              ret += std::to_string(v[i]);
            }
            ret += "]";
            break;
          }

          case rclcpp::ParameterType::PARAMETER_DOUBLE_ARRAY: {
            const auto & v = default_value.get<std::vector<double>>();
            ret += "[";
            for (size_t i = 0; i < v.size(); ++i) {
              if (i) ret += ", ";
              ret += std::to_string(v[i]);
            }
            ret += "]";
            break;
          }

          case rclcpp::ParameterType::PARAMETER_STRING_ARRAY: {
            const auto & v = default_value.get<std::vector<std::string>>();
            ret += "[";
            for (size_t i = 0; i < v.size(); ++i) {
              if (i) ret += ", ";
              ret += "\"" + v[i] + "\"";
            }
            ret += "]";
            break;
          }

          default:
            // NOT_SET or unknown
            break;
        }

        ret += "\n";
      } else {
        ret += "$$NOT_SET\n";
      }
      ret += "\n";

      std::cout << ret;
    }
    #endif  // SYMROS_MODE

    if (param_list.find(name) == param_list.end()) {
      param_list.insert({name, default_value});
    }

    return param_list.at(name);
  }

  void SymROSManager::symros_add_param(std::string name, bool default_value) {
    param_list.insert({name, rclcpp::ParameterValue(default_value)});
  }
  void SymROSManager::symros_add_param(std::string name, int64_t default_value) {
    param_list.insert({name, rclcpp::ParameterValue(default_value)});
  }
  void SymROSManager::symros_add_param(std::string name, double default_value) {
    param_list.insert({name, rclcpp::ParameterValue(default_value)});
  }
  void SymROSManager::symros_add_param(std::string name, std::string default_value) {
    param_list.insert({name, rclcpp::ParameterValue(default_value)});
  }
  void SymROSManager::symros_add_param(std::string name, std::vector<uint8_t> default_value) {
    param_list.insert({name, rclcpp::ParameterValue(default_value)});
  }
  void SymROSManager::symros_add_param(std::string name, std::vector<int64_t> default_value) {
    param_list.insert({name, rclcpp::ParameterValue(default_value)});
  }
  void SymROSManager::symros_add_param(std::string name, std::vector<bool> default_value) {
    param_list.insert({name, rclcpp::ParameterValue(default_value)});
  }
  void SymROSManager::symros_add_param(std::string name, std::vector<double> default_value) {
    param_list.insert({name, rclcpp::ParameterValue(default_value)});
  }
  void SymROSManager::symros_add_param(std::string name, std::vector<std::string> default_value) {
    param_list.insert({name, rclcpp::ParameterValue(default_value)});
  }

  rclcpp::ParameterValue SymROSManager::symros_get_param(std::string name) {
    auto it = param_list.find(name);
    if (it != param_list.end()) {
      return it->second;
    } else {
      return rclcpp::ParameterValue();  // Return a default-constructed ParameterValue if not found
    }
  }

  bool SymROSManager::symros_has_param(std::string name) {
    return param_list.find(name) != param_list.end();
  }

  // Overwrite (or insert) the tracked value so the symros-managed map stays in
  // sync with values applied through the native parameter store (set / declare
  // with overrides). The read paths consult this map first, so a stale entry
  // here would shadow the real value.
  void SymROSManager::symros_set_param(std::string name, rclcpp::ParameterValue value) {
    param_list[name] = value;
  }

  // void
  // SymROSManager::add_service(std::shared_ptr<rclcpp::ServiceBase> srv_base)
  // {
    
  //   if (srv_idx == -1) {
  //     throw std::runtime_error("SymROSManager not initialized");
  //   }

  //   srv_list.push_back(srv_base);

  //   if (is_target) {

  //     auto members = static_cast<const rosidl_typesupport_introspection_cpp::ServiceMembers *>(srv_base->get_service_handle()->)

  //     std::string ret_prefix = "symros_result$$new_srv$$";
  //     std::string ret = ret_prefix + std::to_string(srv_idx) + "$$" + srv_base->get_service_name() + "$$" +  + "\n";
  //     std::cout << ret;
  //     std::cout << "[symros_add_service] new service is created " << srv_base->get_service_name() << std::endl;
  //   }
  
  //   srv_idx++;
  // }

}

