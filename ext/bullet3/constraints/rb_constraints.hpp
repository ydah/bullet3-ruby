#pragma once

#include <memory>

#include <btBulletDynamicsCommon.h>
#include <rice/rice.hpp>

#include "../dynamics/rb_dynamics.hpp"

namespace bullet3 {
class TypedConstraint {
public:
  virtual ~TypedConstraint() = default;

  btTypedConstraint* get();
  const btTypedConstraint* get() const;

  Rice::Symbol constraint_type() const;
  bool enabled() const;
  bool set_enabled(bool enabled);
  btScalar breaking_impulse_threshold() const;
  btScalar set_breaking_impulse_threshold(btScalar threshold);
  btScalar debug_draw_size() const;
  btScalar set_debug_draw_size(btScalar size);
  void enable_feedback(bool enabled);
  bool needs_feedback() const;
  btScalar applied_impulse() const;

protected:
  explicit TypedConstraint(std::unique_ptr<btTypedConstraint> constraint);

private:
  std::unique_ptr<btTypedConstraint> constraint_;
};

class Point2PointConstraint : public TypedConstraint {
public:
  Point2PointConstraint(VALUE rigid_body_a, VALUE pivot_a);
  Point2PointConstraint(VALUE rigid_body_a, VALUE rigid_body_b, VALUE pivot_a, VALUE pivot_b);

  btVector3 pivot_in_a() const;
  btVector3 set_pivot_in_a(Rice::Object pivot);
  btVector3 pivot_in_b() const;
  btVector3 set_pivot_in_b(Rice::Object pivot);
};

class HingeConstraint : public TypedConstraint {
public:
  HingeConstraint(VALUE rigid_body_a, VALUE pivot_a, VALUE axis_a, bool use_reference_frame_a);
  HingeConstraint(VALUE rigid_body_a,
                  VALUE rigid_body_b,
                  VALUE pivot_a,
                  VALUE pivot_b,
                  VALUE axis_a,
                  VALUE axis_b,
                  bool use_reference_frame_a);

  btScalar angle();
  btScalar lower_limit() const;
  btScalar upper_limit() const;
  bool angular_motor_enabled();
  btScalar motor_target_velocity();
  btScalar max_motor_impulse();
  void enable_angular_motor(bool enabled, btScalar target_velocity, btScalar max_motor_impulse);
  void set_limit(btScalar low, btScalar high, btScalar softness, btScalar bias_factor, btScalar relaxation_factor);
  bullet3::Transform frame_a() const;
  bullet3::Transform frame_b() const;
};

class FixedConstraint : public TypedConstraint {
public:
  FixedConstraint(VALUE rigid_body_a, VALUE rigid_body_b, VALUE frame_a, VALUE frame_b);
};

class SliderConstraint : public TypedConstraint {
public:
  SliderConstraint(VALUE rigid_body_b, VALUE frame_b, bool use_linear_reference_frame_a);
  SliderConstraint(VALUE rigid_body_a, VALUE rigid_body_b, VALUE frame_a, VALUE frame_b, bool use_linear_reference_frame_a);

  bullet3::Transform frame_a() const;
  bullet3::Transform frame_b() const;
  void set_frames(VALUE frame_a, VALUE frame_b);
  btScalar lower_linear_limit();
  btScalar set_lower_linear_limit(btScalar limit);
  btScalar upper_linear_limit();
  btScalar set_upper_linear_limit(btScalar limit);
  btScalar lower_angular_limit();
  btScalar set_lower_angular_limit(btScalar limit);
  btScalar upper_angular_limit();
  btScalar set_upper_angular_limit(btScalar limit);
  btScalar linear_position() const;
  btScalar angular_position() const;
  bool powered_linear_motor();
  bool set_powered_linear_motor(bool enabled);
  btScalar target_linear_motor_velocity();
  btScalar set_target_linear_motor_velocity(btScalar velocity);
  btScalar max_linear_motor_force();
  btScalar set_max_linear_motor_force(btScalar force);
  bool powered_angular_motor();
  bool set_powered_angular_motor(bool enabled);
  btScalar target_angular_motor_velocity();
  btScalar set_target_angular_motor_velocity(btScalar velocity);
  btScalar max_angular_motor_force();
  btScalar set_max_angular_motor_force(btScalar force);
};

class ConeTwistConstraint : public TypedConstraint {
public:
  ConeTwistConstraint(VALUE rigid_body_a, VALUE frame_a);
  ConeTwistConstraint(VALUE rigid_body_a, VALUE rigid_body_b, VALUE frame_a, VALUE frame_b);

  void set_limit(btScalar swing_span1, btScalar swing_span2, btScalar twist_span, btScalar softness, btScalar bias_factor, btScalar relaxation_factor);
  btScalar swing_span1() const;
  btScalar swing_span2() const;
  btScalar twist_span() const;
  btScalar limit_softness() const;
  btScalar bias_factor() const;
  btScalar relaxation_factor() const;
  btScalar twist_angle() const;
  bool angular_only() const;
  bool set_angular_only(bool angular_only);
  btScalar damping() const;
  btScalar set_damping(btScalar damping);
  bool motor_enabled() const;
  void enable_motor(bool enabled);
  btScalar max_motor_impulse() const;
  btScalar set_max_motor_impulse(btScalar impulse);
  btQuaternion motor_target() const;
  btQuaternion set_motor_target(Rice::Object target);
  bullet3::Transform frame_a() const;
  bullet3::Transform frame_b() const;
};

class Generic6DofConstraint : public TypedConstraint {
public:
  Generic6DofConstraint(VALUE rigid_body_b, VALUE frame_b, bool use_linear_reference_frame_b);
  Generic6DofConstraint(VALUE rigid_body_a, VALUE rigid_body_b, VALUE frame_a, VALUE frame_b, bool use_linear_reference_frame_a);

  bullet3::Transform frame_a() const;
  bullet3::Transform frame_b() const;
  void set_frames(VALUE frame_a, VALUE frame_b);
  btVector3 linear_lower_limit() const;
  btVector3 set_linear_lower_limit(Rice::Object limit);
  btVector3 linear_upper_limit() const;
  btVector3 set_linear_upper_limit(Rice::Object limit);
  btVector3 angular_lower_limit() const;
  btVector3 set_angular_lower_limit(Rice::Object limit);
  btVector3 angular_upper_limit() const;
  btVector3 set_angular_upper_limit(Rice::Object limit);
  void set_limit(int axis, btScalar low, btScalar high);
  bool limited(int axis) const;
  btVector3 axis(int axis) const;
  btScalar angle(int axis) const;
  btScalar relative_pivot_position(int axis) const;
  bool use_frame_offset() const;
  bool set_use_frame_offset(bool enabled);
  bool use_linear_reference_frame_a() const;
  bool set_use_linear_reference_frame_a(bool enabled);
  void set_axis(Rice::Object axis1, Rice::Object axis2);
};

class Generic6DofSpring2Constraint : public TypedConstraint {
public:
  Generic6DofSpring2Constraint(VALUE rigid_body_b, VALUE frame_b, int rotate_order);
  Generic6DofSpring2Constraint(VALUE rigid_body_a, VALUE rigid_body_b, VALUE frame_a, VALUE frame_b, int rotate_order);

  bullet3::Transform frame_a() const;
  bullet3::Transform frame_b() const;
  void set_frames(VALUE frame_a, VALUE frame_b);
  btVector3 linear_lower_limit() const;
  btVector3 set_linear_lower_limit(Rice::Object limit);
  btVector3 linear_upper_limit() const;
  btVector3 set_linear_upper_limit(Rice::Object limit);
  btVector3 angular_lower_limit() const;
  btVector3 set_angular_lower_limit(Rice::Object limit);
  btVector3 angular_upper_limit() const;
  btVector3 set_angular_upper_limit(Rice::Object limit);
  void set_limit(int axis, btScalar low, btScalar high);
  bool limited(int axis);
  btVector3 axis(int axis) const;
  btScalar angle(int axis) const;
  btScalar relative_pivot_position(int axis) const;
  int rotation_order();
  int set_rotation_order(int rotate_order);
  void set_axis(Rice::Object axis1, Rice::Object axis2);
  void enable_motor(int axis, bool enabled);
  void set_servo(int axis, bool enabled);
  void set_target_velocity(int axis, btScalar velocity);
  void set_servo_target(int axis, btScalar target);
  void set_max_motor_force(int axis, btScalar force);
  void enable_spring(int axis, bool enabled);
  void set_stiffness(int axis, btScalar stiffness, bool limit_if_needed);
  void set_damping(int axis, btScalar damping, bool limit_if_needed);
  void set_equilibrium_point();
  void set_axis_equilibrium_point(int axis);
  void set_axis_equilibrium_point_value(int axis, btScalar value);
};

class GearConstraint : public TypedConstraint {
public:
  GearConstraint(VALUE rigid_body_a, VALUE rigid_body_b, VALUE axis_a, VALUE axis_b, btScalar ratio);

  btVector3 axis_a() const;
  btVector3 set_axis_a(Rice::Object axis);
  btVector3 axis_b() const;
  btVector3 set_axis_b(Rice::Object axis);
  btScalar ratio() const;
  btScalar set_ratio(btScalar ratio);
};

class Hinge2Constraint : public TypedConstraint {
public:
  Hinge2Constraint(VALUE rigid_body_a, VALUE rigid_body_b, VALUE anchor, VALUE axis1, VALUE axis2);

  btVector3 anchor();
  btVector3 anchor2();
  btVector3 axis1();
  btVector3 axis2();
  btScalar angle1();
  btScalar angle2();
  void set_upper_limit(btScalar limit);
  void set_lower_limit(btScalar limit);
};
} // namespace bullet3

void Init_Constraints(Rice::Module rb_mBullet);
