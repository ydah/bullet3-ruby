#pragma once

#include <memory>

#include <btBulletDynamicsCommon.h>
#include <rice/rice.hpp>

#include "../dynamics/rb_dynamics.hpp"

namespace bullet_ruby {
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
  bullet_ruby::Transform frame_a() const;
  bullet_ruby::Transform frame_b() const;
};

class FixedConstraint : public TypedConstraint {
public:
  FixedConstraint(VALUE rigid_body_a, VALUE rigid_body_b, VALUE frame_a, VALUE frame_b);
};
} // namespace bullet_ruby

void Init_Constraints(Rice::Module rb_mBullet);
