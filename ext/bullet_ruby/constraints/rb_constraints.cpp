#include "rb_constraints.hpp"

#include <stdexcept>

#include <BulletDynamics/ConstraintSolver/btFixedConstraint.h>
#include <BulletDynamics/ConstraintSolver/btHingeConstraint.h>
#include <BulletDynamics/ConstraintSolver/btPoint2PointConstraint.h>

#include "../linear_math/rb_transform.hpp"
#include "../util/type_conversions.hpp"

namespace {
bullet_ruby::RigidBody* rigid_body_from_value(VALUE value)
{
  bullet_ruby::RigidBody* body = Rice::detail::From_Ruby<bullet_ruby::RigidBody*>().convert(value);
  if (body == nullptr) {
    throw std::invalid_argument("expected Bullet::RigidBody");
  }
  return body;
}

btPoint2PointConstraint* point2point_from(bullet_ruby::TypedConstraint& constraint)
{
  return static_cast<btPoint2PointConstraint*>(constraint.get());
}

const btPoint2PointConstraint* point2point_from(const bullet_ruby::TypedConstraint& constraint)
{
  return static_cast<const btPoint2PointConstraint*>(constraint.get());
}

btHingeConstraint* hinge_from(bullet_ruby::TypedConstraint& constraint)
{
  return static_cast<btHingeConstraint*>(constraint.get());
}

const btHingeConstraint* hinge_from(const bullet_ruby::TypedConstraint& constraint)
{
  return static_cast<const btHingeConstraint*>(constraint.get());
}
} // namespace

namespace bullet_ruby {
TypedConstraint::TypedConstraint(std::unique_ptr<btTypedConstraint> constraint)
  : constraint_(std::move(constraint))
{
}

btTypedConstraint* TypedConstraint::get()
{
  return constraint_.get();
}

const btTypedConstraint* TypedConstraint::get() const
{
  return constraint_.get();
}

Rice::Symbol TypedConstraint::constraint_type() const
{
  if (dynamic_cast<const btFixedConstraint*>(constraint_.get()) != nullptr) {
    return Rice::Symbol("fixed");
  }

  switch (constraint_->getConstraintType()) {
    case POINT2POINT_CONSTRAINT_TYPE:
      return Rice::Symbol("point2point");
    case HINGE_CONSTRAINT_TYPE:
      return Rice::Symbol("hinge");
    case FIXED_CONSTRAINT_TYPE:
      return Rice::Symbol("fixed");
    default:
      return Rice::Symbol("unknown");
  }
}

bool TypedConstraint::enabled() const
{
  return constraint_->isEnabled();
}

bool TypedConstraint::set_enabled(bool enabled)
{
  constraint_->setEnabled(enabled);
  return enabled;
}

btScalar TypedConstraint::breaking_impulse_threshold() const
{
  return constraint_->getBreakingImpulseThreshold();
}

btScalar TypedConstraint::set_breaking_impulse_threshold(btScalar threshold)
{
  constraint_->setBreakingImpulseThreshold(threshold);
  return threshold;
}

btScalar TypedConstraint::debug_draw_size() const
{
  return constraint_->getDbgDrawSize();
}

btScalar TypedConstraint::set_debug_draw_size(btScalar size)
{
  constraint_->setDbgDrawSize(size);
  return size;
}

void TypedConstraint::enable_feedback(bool enabled)
{
  constraint_->enableFeedback(enabled);
}

bool TypedConstraint::needs_feedback() const
{
  return constraint_->needsFeedback();
}

btScalar TypedConstraint::applied_impulse() const
{
  return constraint_->getAppliedImpulse();
}

Point2PointConstraint::Point2PointConstraint(VALUE rigid_body_a, VALUE pivot_a)
  : TypedConstraint(std::make_unique<btPoint2PointConstraint>(
      *rigid_body_from_value(rigid_body_a)->get(),
      coerce_vector(Rice::Object(pivot_a))))
{
}

Point2PointConstraint::Point2PointConstraint(VALUE rigid_body_a, VALUE rigid_body_b, VALUE pivot_a, VALUE pivot_b)
  : TypedConstraint(std::make_unique<btPoint2PointConstraint>(
      *rigid_body_from_value(rigid_body_a)->get(),
      *rigid_body_from_value(rigid_body_b)->get(),
      coerce_vector(Rice::Object(pivot_a)),
      coerce_vector(Rice::Object(pivot_b))))
{
}

btVector3 Point2PointConstraint::pivot_in_a() const
{
  return point2point_from(*this)->getPivotInA();
}

btVector3 Point2PointConstraint::set_pivot_in_a(Rice::Object pivot)
{
  btVector3 vector = coerce_vector(pivot);
  point2point_from(*this)->setPivotA(vector);
  return vector;
}

btVector3 Point2PointConstraint::pivot_in_b() const
{
  return point2point_from(*this)->getPivotInB();
}

btVector3 Point2PointConstraint::set_pivot_in_b(Rice::Object pivot)
{
  btVector3 vector = coerce_vector(pivot);
  point2point_from(*this)->setPivotB(vector);
  return vector;
}

HingeConstraint::HingeConstraint(VALUE rigid_body_a, VALUE pivot_a, VALUE axis_a, bool use_reference_frame_a)
  : TypedConstraint(std::make_unique<btHingeConstraint>(
      *rigid_body_from_value(rigid_body_a)->get(),
      coerce_vector(Rice::Object(pivot_a)),
      coerce_vector(Rice::Object(axis_a)),
      use_reference_frame_a))
{
}

HingeConstraint::HingeConstraint(VALUE rigid_body_a,
                                 VALUE rigid_body_b,
                                 VALUE pivot_a,
                                 VALUE pivot_b,
                                 VALUE axis_a,
                                 VALUE axis_b,
                                 bool use_reference_frame_a)
  : TypedConstraint(std::make_unique<btHingeConstraint>(
      *rigid_body_from_value(rigid_body_a)->get(),
      *rigid_body_from_value(rigid_body_b)->get(),
      coerce_vector(Rice::Object(pivot_a)),
      coerce_vector(Rice::Object(pivot_b)),
      coerce_vector(Rice::Object(axis_a)),
      coerce_vector(Rice::Object(axis_b)),
      use_reference_frame_a))
{
}

btScalar HingeConstraint::angle()
{
  return hinge_from(*this)->getHingeAngle();
}

btScalar HingeConstraint::lower_limit() const
{
  return hinge_from(*this)->getLowerLimit();
}

btScalar HingeConstraint::upper_limit() const
{
  return hinge_from(*this)->getUpperLimit();
}

bool HingeConstraint::angular_motor_enabled()
{
  return hinge_from(*this)->getEnableAngularMotor();
}

btScalar HingeConstraint::motor_target_velocity()
{
  return hinge_from(*this)->getMotorTargetVelocity();
}

btScalar HingeConstraint::max_motor_impulse()
{
  return hinge_from(*this)->getMaxMotorImpulse();
}

void HingeConstraint::enable_angular_motor(bool enabled, btScalar target_velocity, btScalar max_motor_impulse)
{
  hinge_from(*this)->enableAngularMotor(enabled, target_velocity, max_motor_impulse);
}

void HingeConstraint::set_limit(btScalar low, btScalar high, btScalar softness, btScalar bias_factor, btScalar relaxation_factor)
{
  hinge_from(*this)->setLimit(low, high, softness, bias_factor, relaxation_factor);
}

Transform HingeConstraint::frame_a() const
{
  return Transform(hinge_from(*this)->getAFrame());
}

Transform HingeConstraint::frame_b() const
{
  return Transform(hinge_from(*this)->getBFrame());
}

FixedConstraint::FixedConstraint(VALUE rigid_body_a, VALUE rigid_body_b, VALUE frame_a, VALUE frame_b)
  : TypedConstraint(std::make_unique<btFixedConstraint>(
      *rigid_body_from_value(rigid_body_a)->get(),
      *rigid_body_from_value(rigid_body_b)->get(),
      coerce_transform(Rice::Object(frame_a)),
      coerce_transform(Rice::Object(frame_b))))
{
}
} // namespace bullet_ruby

void Init_Constraints(Rice::Module rb_mBullet)
{
  Rice::Module rb_mConstraints = Rice::define_module_under(rb_mBullet, "Constraints");

  Rice::define_class_under<bullet_ruby::TypedConstraint>(rb_mConstraints, "TypedConstraint")
    .define_method("constraint_type", &bullet_ruby::TypedConstraint::constraint_type)
    .define_method("enabled?", &bullet_ruby::TypedConstraint::enabled)
    .define_method("enabled=", &bullet_ruby::TypedConstraint::set_enabled)
    .define_method("breaking_impulse_threshold", &bullet_ruby::TypedConstraint::breaking_impulse_threshold)
    .define_method("breaking_impulse_threshold=", &bullet_ruby::TypedConstraint::set_breaking_impulse_threshold)
    .define_method("debug_draw_size", &bullet_ruby::TypedConstraint::debug_draw_size)
    .define_method("debug_draw_size=", &bullet_ruby::TypedConstraint::set_debug_draw_size)
    .define_method("enable_feedback", &bullet_ruby::TypedConstraint::enable_feedback)
    .define_method("needs_feedback?", &bullet_ruby::TypedConstraint::needs_feedback)
    .define_method("applied_impulse", &bullet_ruby::TypedConstraint::applied_impulse);

  Rice::define_class_under<bullet_ruby::Point2PointConstraint, bullet_ruby::TypedConstraint>(rb_mConstraints, "Point2PointConstraint")
    .define_constructor(Rice::Constructor<bullet_ruby::Point2PointConstraint, VALUE, VALUE>(),
      Rice::Arg("rigid_body_a").setValue().keepAlive(),
      Rice::Arg("pivot_a").setValue())
    .define_constructor(Rice::Constructor<bullet_ruby::Point2PointConstraint, VALUE, VALUE, VALUE, VALUE>(),
      Rice::Arg("rigid_body_a").setValue().keepAlive(),
      Rice::Arg("rigid_body_b").setValue().keepAlive(),
      Rice::Arg("pivot_a").setValue(),
      Rice::Arg("pivot_b").setValue())
    .define_method("pivot_in_a", &bullet_ruby::Point2PointConstraint::pivot_in_a)
    .define_method("pivot_in_a=", &bullet_ruby::Point2PointConstraint::set_pivot_in_a)
    .define_method("pivot_in_b", &bullet_ruby::Point2PointConstraint::pivot_in_b)
    .define_method("pivot_in_b=", &bullet_ruby::Point2PointConstraint::set_pivot_in_b);

  Rice::define_class_under<bullet_ruby::HingeConstraint, bullet_ruby::TypedConstraint>(rb_mConstraints, "HingeConstraint")
    .define_constructor(Rice::Constructor<bullet_ruby::HingeConstraint, VALUE, VALUE, VALUE, bool>(),
      Rice::Arg("rigid_body_a").setValue().keepAlive(),
      Rice::Arg("pivot_a").setValue(),
      Rice::Arg("axis_a").setValue(),
      Rice::Arg("use_reference_frame_a") = false)
    .define_constructor(Rice::Constructor<bullet_ruby::HingeConstraint, VALUE, VALUE, VALUE, VALUE, VALUE, VALUE, bool>(),
      Rice::Arg("rigid_body_a").setValue().keepAlive(),
      Rice::Arg("rigid_body_b").setValue().keepAlive(),
      Rice::Arg("pivot_a").setValue(),
      Rice::Arg("pivot_b").setValue(),
      Rice::Arg("axis_a").setValue(),
      Rice::Arg("axis_b").setValue(),
      Rice::Arg("use_reference_frame_a") = false)
    .define_method("angle", &bullet_ruby::HingeConstraint::angle)
    .define_method("lower_limit", &bullet_ruby::HingeConstraint::lower_limit)
    .define_method("upper_limit", &bullet_ruby::HingeConstraint::upper_limit)
    .define_method("angular_motor_enabled?", &bullet_ruby::HingeConstraint::angular_motor_enabled)
    .define_method("motor_target_velocity", &bullet_ruby::HingeConstraint::motor_target_velocity)
    .define_method("max_motor_impulse", &bullet_ruby::HingeConstraint::max_motor_impulse)
    .define_method("enable_angular_motor", &bullet_ruby::HingeConstraint::enable_angular_motor)
    .define_method("set_limit", &bullet_ruby::HingeConstraint::set_limit,
      Rice::Arg("low"),
      Rice::Arg("high"),
      Rice::Arg("softness") = btScalar(0.9),
      Rice::Arg("bias_factor") = btScalar(0.3),
      Rice::Arg("relaxation_factor") = btScalar(1.0))
    .define_method("frame_a", &bullet_ruby::HingeConstraint::frame_a)
    .define_method("frame_b", &bullet_ruby::HingeConstraint::frame_b);

  Rice::define_class_under<bullet_ruby::FixedConstraint, bullet_ruby::TypedConstraint>(rb_mConstraints, "FixedConstraint")
    .define_constructor(Rice::Constructor<bullet_ruby::FixedConstraint, VALUE, VALUE, VALUE, VALUE>(),
      Rice::Arg("rigid_body_a").setValue().keepAlive(),
      Rice::Arg("rigid_body_b").setValue().keepAlive(),
      Rice::Arg("frame_a").setValue(),
      Rice::Arg("frame_b").setValue());
}
