#pragma once

#include <Eigen/Dense>
#include <Eigen/StdVector>
#include <functional>
#include <robot_design/types.h>
#include <type_traits>
#include <vector>

namespace robot_design {

enum class LinkShape : Index { NONE, CAPSULE, CYLINDER };

enum class JointType : Index { NONE, FREE, HINGE, FIXED };

enum class JointControlMode : Index { POSITION, VELOCITY };

struct Link {
    Link(Index parent, Operations function, Scalar max_diameter, Scalar max_length,
        const Vector3& xyz_travel, Scalar max_feed_rate, Scalar max_spindle_speed,
        bool is_available, Scalar mach_num, Scalar programming_time, bool finishing,
        Scalar logistic_time, const std::string& label, const std::string& joint_label)
        : parent_(parent), function_(function), max_diameter_(max_diameter),
        max_length_(max_length), xyz_travel_(xyz_travel), max_feed_rate_(max_feed_rate),
        max_spindle_speed_(max_spindle_speed), is_available_(is_available),
        mach_num_(mach_num), programming_time_(programming_time), finishing_(finishing),
        logistic_time_(logistic_time), label_(label), joint_label_(joint_label) {}

    // Parent link index (-1 for base link)
    Index parent_;
    // Operations machine can do
    Operations function_;
    // Maximum diameter of job machine can take
    Scalar max_diameter_;
    // Maximum length of job machine can take
    Vector3 xyz_travel_;
    // Maximum feed rate of machine
    Scalar max_feed_rate_;
    // Maximum spindle rate of machine
    Scalar max_spindle_speed_;
    // Machine's availability
    bool is_available_;
    // Machine number
    Scalar mach_num_;
    // Time taken to program job (CNC Only)
    Scalar programming_time_;
    // Roughing vs Finishing job
    bool finishing_;
    // Time taken to transport between machines
    Scalar logistic_time_;
    // Machine label
    std::string label_;
    // Link label
    std::string joint_label_;

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
};

struct Robot {
  std::vector<Link, Eigen::aligned_allocator<Link>> links_;
};

} // namespace robot_design

namespace std {
/*
// Return hash values of enum types for those unordered containers mentioned below
template <> struct hash<robot_design::LinkShape> {
  std::size_t operator()(const robot_design::LinkShape &link_shape) const {
    using type = typename std::underlying_type<robot_design::LinkShape>::type;
    return std::hash<type>()(static_cast<type>(link_shape));
  }
};

template <> struct hash<robot_design::JointType> {
  std::size_t operator()(const robot_design::JointType &joint_type) const {
    using type = typename std::underlying_type<robot_design::JointType>::type;
    return std::hash<type>()(static_cast<type>(joint_type));
  }
};

template <> struct hash<robot_design::JointControlMode> {
  std::size_t operator()(const robot_design::JointControlMode &joint_control_mode) const {
    using type = typename std::underlying_type<robot_design::JointControlMode>::type;
    return std::hash<type>()(static_cast<type>(joint_control_mode));
  }
};
*/
} // namespace std
