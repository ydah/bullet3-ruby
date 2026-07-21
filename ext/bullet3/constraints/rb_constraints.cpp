#include "rb_constraints.hpp"

#include <stdexcept>

#include <BulletDynamics/ConstraintSolver/btFixedConstraint.h>
#include <BulletDynamics/ConstraintSolver/btGearConstraint.h>
#include <BulletDynamics/ConstraintSolver/btGeneric6DofConstraint.h>
#include <BulletDynamics/ConstraintSolver/btGeneric6DofSpring2Constraint.h>
#include <BulletDynamics/ConstraintSolver/btHingeConstraint.h>
#include <BulletDynamics/ConstraintSolver/btHinge2Constraint.h>
#include <BulletDynamics/ConstraintSolver/btPoint2PointConstraint.h>
#include <BulletDynamics/ConstraintSolver/btSliderConstraint.h>
#include <BulletDynamics/ConstraintSolver/btConeTwistConstraint.h>

#include "../linear_math/rb_transform.hpp"
#include "../util/type_conversions.hpp"

namespace {
bullet3::RigidBody* rigid_body_from_value(VALUE value)
{
  bullet3::RigidBody* body = Rice::detail::From_Ruby<bullet3::RigidBody*>().convert(value);
  if (body == nullptr) {
    throw std::invalid_argument("expected Bullet3::RigidBody");
  }
  return body;
}

btPoint2PointConstraint* point2point_from(bullet3::TypedConstraint& constraint)
{
  return static_cast<btPoint2PointConstraint*>(constraint.get());
}

const btPoint2PointConstraint* point2point_from(const bullet3::TypedConstraint& constraint)
{
  return static_cast<const btPoint2PointConstraint*>(constraint.get());
}

btHingeConstraint* hinge_from(bullet3::TypedConstraint& constraint)
{
  return static_cast<btHingeConstraint*>(constraint.get());
}

const btHingeConstraint* hinge_from(const bullet3::TypedConstraint& constraint)
{
  return static_cast<const btHingeConstraint*>(constraint.get());
}

btSliderConstraint* slider_from(bullet3::TypedConstraint& constraint)
{
  return static_cast<btSliderConstraint*>(constraint.get());
}

const btSliderConstraint* slider_from(const bullet3::TypedConstraint& constraint)
{
  return static_cast<const btSliderConstraint*>(constraint.get());
}

btConeTwistConstraint* cone_twist_from(bullet3::TypedConstraint& constraint)
{
  return static_cast<btConeTwistConstraint*>(constraint.get());
}

const btConeTwistConstraint* cone_twist_from(const bullet3::TypedConstraint& constraint)
{
  return static_cast<const btConeTwistConstraint*>(constraint.get());
}

btGeneric6DofConstraint* generic_6dof_from(bullet3::TypedConstraint& constraint)
{
  return static_cast<btGeneric6DofConstraint*>(constraint.get());
}

const btGeneric6DofConstraint* generic_6dof_from(const bullet3::TypedConstraint& constraint)
{
  return static_cast<const btGeneric6DofConstraint*>(constraint.get());
}

btGeneric6DofSpring2Constraint* generic_6dof_spring2_from(bullet3::TypedConstraint& constraint)
{
  return static_cast<btGeneric6DofSpring2Constraint*>(constraint.get());
}

btGeneric6DofSpring2Constraint* generic_6dof_spring2_from(const bullet3::TypedConstraint& constraint)
{
  return const_cast<btGeneric6DofSpring2Constraint*>(
    static_cast<const btGeneric6DofSpring2Constraint*>(constraint.get()));
}

btGearConstraint* gear_from(bullet3::TypedConstraint& constraint)
{
  return static_cast<btGearConstraint*>(constraint.get());
}

const btGearConstraint* gear_from(const bullet3::TypedConstraint& constraint)
{
  return static_cast<const btGearConstraint*>(constraint.get());
}

btHinge2Constraint* hinge2_from(bullet3::TypedConstraint& constraint)
{
  return static_cast<btHinge2Constraint*>(constraint.get());
}

int validate_spatial_axis(int axis)
{
  if (axis < 0 || axis > 2) {
    throw std::out_of_range("axis must be between 0 and 2");
  }
  return axis;
}

int validate_dof_axis(int axis)
{
  if (axis < 0 || axis > 5) {
    throw std::out_of_range("axis must be between 0 and 5");
  }
  return axis;
}

RotateOrder rotate_order_from_int(int rotate_order)
{
  if (rotate_order < RO_XYZ || rotate_order > RO_ZYX) {
    throw std::out_of_range("rotate_order must be between 0 and 5");
  }
  return static_cast<RotateOrder>(rotate_order);
}

std::unique_ptr<btTypedConstraint> make_hinge2_constraint(VALUE rigid_body_a, VALUE rigid_body_b, VALUE anchor_value, VALUE axis1_value, VALUE axis2_value)
{
  btVector3 anchor = bullet3::coerce_vector(Rice::Object(anchor_value));
  btVector3 axis1 = bullet3::coerce_vector(Rice::Object(axis1_value));
  btVector3 axis2 = bullet3::coerce_vector(Rice::Object(axis2_value));
  return std::make_unique<btHinge2Constraint>(
    *rigid_body_from_value(rigid_body_a)->get(),
    *rigid_body_from_value(rigid_body_b)->get(),
    anchor,
    axis1,
    axis2);
}
} // namespace

namespace bullet3 {
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
    case CONETWIST_CONSTRAINT_TYPE:
      return Rice::Symbol("cone_twist");
    case D6_CONSTRAINT_TYPE:
      return Rice::Symbol("generic_6dof");
    case SLIDER_CONSTRAINT_TYPE:
      return Rice::Symbol("slider");
    case GEAR_CONSTRAINT_TYPE:
      return Rice::Symbol("gear");
    case FIXED_CONSTRAINT_TYPE:
      return Rice::Symbol("fixed");
    case D6_SPRING_2_CONSTRAINT_TYPE:
      if (dynamic_cast<const btHinge2Constraint*>(constraint_.get()) != nullptr) {
        return Rice::Symbol("hinge2");
      }
      return Rice::Symbol("generic_6dof_spring2");
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

SliderConstraint::SliderConstraint(VALUE rigid_body_b, VALUE frame_b, bool use_linear_reference_frame_a)
  : TypedConstraint(std::make_unique<btSliderConstraint>(
      *rigid_body_from_value(rigid_body_b)->get(),
      coerce_transform(Rice::Object(frame_b)),
      use_linear_reference_frame_a))
{
}

SliderConstraint::SliderConstraint(VALUE rigid_body_a, VALUE rigid_body_b, VALUE frame_a, VALUE frame_b, bool use_linear_reference_frame_a)
  : TypedConstraint(std::make_unique<btSliderConstraint>(
      *rigid_body_from_value(rigid_body_a)->get(),
      *rigid_body_from_value(rigid_body_b)->get(),
      coerce_transform(Rice::Object(frame_a)),
      coerce_transform(Rice::Object(frame_b)),
      use_linear_reference_frame_a))
{
}

Transform SliderConstraint::frame_a() const
{
  return Transform(slider_from(*this)->getFrameOffsetA());
}

Transform SliderConstraint::frame_b() const
{
  return Transform(slider_from(*this)->getFrameOffsetB());
}

void SliderConstraint::set_frames(VALUE frame_a, VALUE frame_b)
{
  slider_from(*this)->setFrames(coerce_transform(Rice::Object(frame_a)), coerce_transform(Rice::Object(frame_b)));
}

btScalar SliderConstraint::lower_linear_limit()
{
  return slider_from(*this)->getLowerLinLimit();
}

btScalar SliderConstraint::set_lower_linear_limit(btScalar limit)
{
  slider_from(*this)->setLowerLinLimit(limit);
  return limit;
}

btScalar SliderConstraint::upper_linear_limit()
{
  return slider_from(*this)->getUpperLinLimit();
}

btScalar SliderConstraint::set_upper_linear_limit(btScalar limit)
{
  slider_from(*this)->setUpperLinLimit(limit);
  return limit;
}

btScalar SliderConstraint::lower_angular_limit()
{
  return slider_from(*this)->getLowerAngLimit();
}

btScalar SliderConstraint::set_lower_angular_limit(btScalar limit)
{
  slider_from(*this)->setLowerAngLimit(limit);
  return limit;
}

btScalar SliderConstraint::upper_angular_limit()
{
  return slider_from(*this)->getUpperAngLimit();
}

btScalar SliderConstraint::set_upper_angular_limit(btScalar limit)
{
  slider_from(*this)->setUpperAngLimit(limit);
  return limit;
}

btScalar SliderConstraint::linear_position() const
{
  return slider_from(*this)->getLinearPos();
}

btScalar SliderConstraint::angular_position() const
{
  return slider_from(*this)->getAngularPos();
}

bool SliderConstraint::powered_linear_motor()
{
  return slider_from(*this)->getPoweredLinMotor();
}

bool SliderConstraint::set_powered_linear_motor(bool enabled)
{
  slider_from(*this)->setPoweredLinMotor(enabled);
  return enabled;
}

btScalar SliderConstraint::target_linear_motor_velocity()
{
  return slider_from(*this)->getTargetLinMotorVelocity();
}

btScalar SliderConstraint::set_target_linear_motor_velocity(btScalar velocity)
{
  slider_from(*this)->setTargetLinMotorVelocity(velocity);
  return velocity;
}

btScalar SliderConstraint::max_linear_motor_force()
{
  return slider_from(*this)->getMaxLinMotorForce();
}

btScalar SliderConstraint::set_max_linear_motor_force(btScalar force)
{
  slider_from(*this)->setMaxLinMotorForce(force);
  return force;
}

bool SliderConstraint::powered_angular_motor()
{
  return slider_from(*this)->getPoweredAngMotor();
}

bool SliderConstraint::set_powered_angular_motor(bool enabled)
{
  slider_from(*this)->setPoweredAngMotor(enabled);
  return enabled;
}

btScalar SliderConstraint::target_angular_motor_velocity()
{
  return slider_from(*this)->getTargetAngMotorVelocity();
}

btScalar SliderConstraint::set_target_angular_motor_velocity(btScalar velocity)
{
  slider_from(*this)->setTargetAngMotorVelocity(velocity);
  return velocity;
}

btScalar SliderConstraint::max_angular_motor_force()
{
  return slider_from(*this)->getMaxAngMotorForce();
}

btScalar SliderConstraint::set_max_angular_motor_force(btScalar force)
{
  slider_from(*this)->setMaxAngMotorForce(force);
  return force;
}

ConeTwistConstraint::ConeTwistConstraint(VALUE rigid_body_a, VALUE frame_a)
  : TypedConstraint(std::make_unique<btConeTwistConstraint>(
      *rigid_body_from_value(rigid_body_a)->get(),
      coerce_transform(Rice::Object(frame_a))))
{
}

ConeTwistConstraint::ConeTwistConstraint(VALUE rigid_body_a, VALUE rigid_body_b, VALUE frame_a, VALUE frame_b)
  : TypedConstraint(std::make_unique<btConeTwistConstraint>(
      *rigid_body_from_value(rigid_body_a)->get(),
      *rigid_body_from_value(rigid_body_b)->get(),
      coerce_transform(Rice::Object(frame_a)),
      coerce_transform(Rice::Object(frame_b))))
{
}

void ConeTwistConstraint::set_limit(btScalar swing_span1, btScalar swing_span2, btScalar twist_span, btScalar softness, btScalar bias_factor, btScalar relaxation_factor)
{
  cone_twist_from(*this)->setLimit(swing_span1, swing_span2, twist_span, softness, bias_factor, relaxation_factor);
}

btScalar ConeTwistConstraint::swing_span1() const
{
  return cone_twist_from(*this)->getSwingSpan1();
}

btScalar ConeTwistConstraint::swing_span2() const
{
  return cone_twist_from(*this)->getSwingSpan2();
}

btScalar ConeTwistConstraint::twist_span() const
{
  return cone_twist_from(*this)->getTwistSpan();
}

btScalar ConeTwistConstraint::limit_softness() const
{
  return cone_twist_from(*this)->getLimitSoftness();
}

btScalar ConeTwistConstraint::bias_factor() const
{
  return cone_twist_from(*this)->getBiasFactor();
}

btScalar ConeTwistConstraint::relaxation_factor() const
{
  return cone_twist_from(*this)->getRelaxationFactor();
}

btScalar ConeTwistConstraint::twist_angle() const
{
  return cone_twist_from(*this)->getTwistAngle();
}

bool ConeTwistConstraint::angular_only() const
{
  return cone_twist_from(*this)->getAngularOnly();
}

bool ConeTwistConstraint::set_angular_only(bool angular_only)
{
  cone_twist_from(*this)->setAngularOnly(angular_only);
  return angular_only;
}

btScalar ConeTwistConstraint::damping() const
{
  return cone_twist_from(*this)->getDamping();
}

btScalar ConeTwistConstraint::set_damping(btScalar damping)
{
  cone_twist_from(*this)->setDamping(damping);
  return damping;
}

bool ConeTwistConstraint::motor_enabled() const
{
  return cone_twist_from(*this)->isMotorEnabled();
}

void ConeTwistConstraint::enable_motor(bool enabled)
{
  cone_twist_from(*this)->enableMotor(enabled);
}

btScalar ConeTwistConstraint::max_motor_impulse() const
{
  return cone_twist_from(*this)->getMaxMotorImpulse();
}

btScalar ConeTwistConstraint::set_max_motor_impulse(btScalar impulse)
{
  cone_twist_from(*this)->setMaxMotorImpulse(impulse);
  return impulse;
}

btQuaternion ConeTwistConstraint::motor_target() const
{
  return cone_twist_from(*this)->getMotorTarget();
}

btQuaternion ConeTwistConstraint::set_motor_target(Rice::Object target)
{
  btQuaternion quaternion = coerce_quaternion(target);
  cone_twist_from(*this)->setMotorTarget(quaternion);
  return quaternion;
}

Transform ConeTwistConstraint::frame_a() const
{
  return Transform(cone_twist_from(*this)->getAFrame());
}

Transform ConeTwistConstraint::frame_b() const
{
  return Transform(cone_twist_from(*this)->getBFrame());
}

Generic6DofConstraint::Generic6DofConstraint(VALUE rigid_body_b, VALUE frame_b, bool use_linear_reference_frame_b)
  : TypedConstraint(std::make_unique<btGeneric6DofConstraint>(
      *rigid_body_from_value(rigid_body_b)->get(),
      coerce_transform(Rice::Object(frame_b)),
      use_linear_reference_frame_b))
{
}

Generic6DofConstraint::Generic6DofConstraint(VALUE rigid_body_a, VALUE rigid_body_b, VALUE frame_a, VALUE frame_b, bool use_linear_reference_frame_a)
  : TypedConstraint(std::make_unique<btGeneric6DofConstraint>(
      *rigid_body_from_value(rigid_body_a)->get(),
      *rigid_body_from_value(rigid_body_b)->get(),
      coerce_transform(Rice::Object(frame_a)),
      coerce_transform(Rice::Object(frame_b)),
      use_linear_reference_frame_a))
{
}

Transform Generic6DofConstraint::frame_a() const
{
  return Transform(generic_6dof_from(*this)->getFrameOffsetA());
}

Transform Generic6DofConstraint::frame_b() const
{
  return Transform(generic_6dof_from(*this)->getFrameOffsetB());
}

void Generic6DofConstraint::set_frames(VALUE frame_a, VALUE frame_b)
{
  generic_6dof_from(*this)->setFrames(coerce_transform(Rice::Object(frame_a)), coerce_transform(Rice::Object(frame_b)));
}

btVector3 Generic6DofConstraint::linear_lower_limit() const
{
  btVector3 limit;
  generic_6dof_from(*this)->getLinearLowerLimit(limit);
  return limit;
}

btVector3 Generic6DofConstraint::set_linear_lower_limit(Rice::Object limit)
{
  btVector3 vector = coerce_vector(limit);
  generic_6dof_from(*this)->setLinearLowerLimit(vector);
  return vector;
}

btVector3 Generic6DofConstraint::linear_upper_limit() const
{
  btVector3 limit;
  generic_6dof_from(*this)->getLinearUpperLimit(limit);
  return limit;
}

btVector3 Generic6DofConstraint::set_linear_upper_limit(Rice::Object limit)
{
  btVector3 vector = coerce_vector(limit);
  generic_6dof_from(*this)->setLinearUpperLimit(vector);
  return vector;
}

btVector3 Generic6DofConstraint::angular_lower_limit() const
{
  btVector3 limit;
  generic_6dof_from(*this)->getAngularLowerLimit(limit);
  return limit;
}

btVector3 Generic6DofConstraint::set_angular_lower_limit(Rice::Object limit)
{
  btVector3 vector = coerce_vector(limit);
  generic_6dof_from(*this)->setAngularLowerLimit(vector);
  return vector;
}

btVector3 Generic6DofConstraint::angular_upper_limit() const
{
  btVector3 limit;
  generic_6dof_from(*this)->getAngularUpperLimit(limit);
  return limit;
}

btVector3 Generic6DofConstraint::set_angular_upper_limit(Rice::Object limit)
{
  btVector3 vector = coerce_vector(limit);
  generic_6dof_from(*this)->setAngularUpperLimit(vector);
  return vector;
}

void Generic6DofConstraint::set_limit(int axis, btScalar low, btScalar high)
{
  generic_6dof_from(*this)->setLimit(validate_dof_axis(axis), low, high);
}

bool Generic6DofConstraint::limited(int axis) const
{
  return generic_6dof_from(*this)->isLimited(validate_dof_axis(axis));
}

btVector3 Generic6DofConstraint::axis(int axis) const
{
  return generic_6dof_from(*this)->getAxis(validate_spatial_axis(axis));
}

btScalar Generic6DofConstraint::angle(int axis) const
{
  return generic_6dof_from(*this)->getAngle(validate_spatial_axis(axis));
}

btScalar Generic6DofConstraint::relative_pivot_position(int axis) const
{
  return generic_6dof_from(*this)->getRelativePivotPosition(validate_spatial_axis(axis));
}

bool Generic6DofConstraint::use_frame_offset() const
{
  return generic_6dof_from(*this)->getUseFrameOffset();
}

bool Generic6DofConstraint::set_use_frame_offset(bool enabled)
{
  generic_6dof_from(*this)->setUseFrameOffset(enabled);
  return enabled;
}

bool Generic6DofConstraint::use_linear_reference_frame_a() const
{
  return generic_6dof_from(*this)->getUseLinearReferenceFrameA();
}

bool Generic6DofConstraint::set_use_linear_reference_frame_a(bool enabled)
{
  generic_6dof_from(*this)->setUseLinearReferenceFrameA(enabled);
  return enabled;
}

void Generic6DofConstraint::set_axis(Rice::Object axis1, Rice::Object axis2)
{
  generic_6dof_from(*this)->setAxis(coerce_vector(axis1), coerce_vector(axis2));
}

Generic6DofSpring2Constraint::Generic6DofSpring2Constraint(VALUE rigid_body_b, VALUE frame_b, int rotate_order)
  : TypedConstraint(std::make_unique<btGeneric6DofSpring2Constraint>(
      *rigid_body_from_value(rigid_body_b)->get(),
      coerce_transform(Rice::Object(frame_b)),
      rotate_order_from_int(rotate_order)))
{
}

Generic6DofSpring2Constraint::Generic6DofSpring2Constraint(VALUE rigid_body_a, VALUE rigid_body_b, VALUE frame_a, VALUE frame_b, int rotate_order)
  : TypedConstraint(std::make_unique<btGeneric6DofSpring2Constraint>(
      *rigid_body_from_value(rigid_body_a)->get(),
      *rigid_body_from_value(rigid_body_b)->get(),
      coerce_transform(Rice::Object(frame_a)),
      coerce_transform(Rice::Object(frame_b)),
      rotate_order_from_int(rotate_order)))
{
}

Transform Generic6DofSpring2Constraint::frame_a() const
{
  return Transform(generic_6dof_spring2_from(*this)->getFrameOffsetA());
}

Transform Generic6DofSpring2Constraint::frame_b() const
{
  return Transform(generic_6dof_spring2_from(*this)->getFrameOffsetB());
}

void Generic6DofSpring2Constraint::set_frames(VALUE frame_a, VALUE frame_b)
{
  generic_6dof_spring2_from(*this)->setFrames(coerce_transform(Rice::Object(frame_a)), coerce_transform(Rice::Object(frame_b)));
}

btVector3 Generic6DofSpring2Constraint::linear_lower_limit() const
{
  btVector3 limit;
  generic_6dof_spring2_from(*this)->getLinearLowerLimit(limit);
  return limit;
}

btVector3 Generic6DofSpring2Constraint::set_linear_lower_limit(Rice::Object limit)
{
  btVector3 vector = coerce_vector(limit);
  generic_6dof_spring2_from(*this)->setLinearLowerLimit(vector);
  return vector;
}

btVector3 Generic6DofSpring2Constraint::linear_upper_limit() const
{
  btVector3 limit;
  generic_6dof_spring2_from(*this)->getLinearUpperLimit(limit);
  return limit;
}

btVector3 Generic6DofSpring2Constraint::set_linear_upper_limit(Rice::Object limit)
{
  btVector3 vector = coerce_vector(limit);
  generic_6dof_spring2_from(*this)->setLinearUpperLimit(vector);
  return vector;
}

btVector3 Generic6DofSpring2Constraint::angular_lower_limit() const
{
  btVector3 limit;
  generic_6dof_spring2_from(*this)->getAngularLowerLimit(limit);
  return limit;
}

btVector3 Generic6DofSpring2Constraint::set_angular_lower_limit(Rice::Object limit)
{
  btVector3 vector = coerce_vector(limit);
  generic_6dof_spring2_from(*this)->setAngularLowerLimit(vector);
  return vector;
}

btVector3 Generic6DofSpring2Constraint::angular_upper_limit() const
{
  btVector3 limit;
  generic_6dof_spring2_from(*this)->getAngularUpperLimit(limit);
  return limit;
}

btVector3 Generic6DofSpring2Constraint::set_angular_upper_limit(Rice::Object limit)
{
  btVector3 vector = coerce_vector(limit);
  generic_6dof_spring2_from(*this)->setAngularUpperLimit(vector);
  return vector;
}

void Generic6DofSpring2Constraint::set_limit(int axis, btScalar low, btScalar high)
{
  generic_6dof_spring2_from(*this)->setLimit(validate_dof_axis(axis), low, high);
}

bool Generic6DofSpring2Constraint::limited(int axis)
{
  return generic_6dof_spring2_from(*this)->isLimited(validate_dof_axis(axis));
}

btVector3 Generic6DofSpring2Constraint::axis(int axis) const
{
  return generic_6dof_spring2_from(*this)->getAxis(validate_spatial_axis(axis));
}

btScalar Generic6DofSpring2Constraint::angle(int axis) const
{
  return generic_6dof_spring2_from(*this)->getAngle(validate_spatial_axis(axis));
}

btScalar Generic6DofSpring2Constraint::relative_pivot_position(int axis) const
{
  return generic_6dof_spring2_from(*this)->getRelativePivotPosition(validate_spatial_axis(axis));
}

int Generic6DofSpring2Constraint::rotation_order()
{
  return static_cast<int>(generic_6dof_spring2_from(*this)->getRotationOrder());
}

int Generic6DofSpring2Constraint::set_rotation_order(int rotate_order)
{
  generic_6dof_spring2_from(*this)->setRotationOrder(rotate_order_from_int(rotate_order));
  return rotate_order;
}

void Generic6DofSpring2Constraint::set_axis(Rice::Object axis1, Rice::Object axis2)
{
  generic_6dof_spring2_from(*this)->setAxis(coerce_vector(axis1), coerce_vector(axis2));
}

void Generic6DofSpring2Constraint::enable_motor(int axis, bool enabled)
{
  generic_6dof_spring2_from(*this)->enableMotor(validate_dof_axis(axis), enabled);
}

void Generic6DofSpring2Constraint::set_servo(int axis, bool enabled)
{
  generic_6dof_spring2_from(*this)->setServo(validate_dof_axis(axis), enabled);
}

void Generic6DofSpring2Constraint::set_target_velocity(int axis, btScalar velocity)
{
  generic_6dof_spring2_from(*this)->setTargetVelocity(validate_dof_axis(axis), velocity);
}

void Generic6DofSpring2Constraint::set_servo_target(int axis, btScalar target)
{
  generic_6dof_spring2_from(*this)->setServoTarget(validate_dof_axis(axis), target);
}

void Generic6DofSpring2Constraint::set_max_motor_force(int axis, btScalar force)
{
  generic_6dof_spring2_from(*this)->setMaxMotorForce(validate_dof_axis(axis), force);
}

void Generic6DofSpring2Constraint::enable_spring(int axis, bool enabled)
{
  generic_6dof_spring2_from(*this)->enableSpring(validate_dof_axis(axis), enabled);
}

void Generic6DofSpring2Constraint::set_stiffness(int axis, btScalar stiffness, bool limit_if_needed)
{
  generic_6dof_spring2_from(*this)->setStiffness(validate_dof_axis(axis), stiffness, limit_if_needed);
}

void Generic6DofSpring2Constraint::set_damping(int axis, btScalar damping, bool limit_if_needed)
{
  generic_6dof_spring2_from(*this)->setDamping(validate_dof_axis(axis), damping, limit_if_needed);
}

void Generic6DofSpring2Constraint::set_equilibrium_point()
{
  generic_6dof_spring2_from(*this)->setEquilibriumPoint();
}

void Generic6DofSpring2Constraint::set_axis_equilibrium_point(int axis)
{
  generic_6dof_spring2_from(*this)->setEquilibriumPoint(validate_dof_axis(axis));
}

void Generic6DofSpring2Constraint::set_axis_equilibrium_point_value(int axis, btScalar value)
{
  generic_6dof_spring2_from(*this)->setEquilibriumPoint(validate_dof_axis(axis), value);
}

GearConstraint::GearConstraint(VALUE rigid_body_a, VALUE rigid_body_b, VALUE axis_a, VALUE axis_b, btScalar ratio)
  : TypedConstraint(std::make_unique<btGearConstraint>(
      *rigid_body_from_value(rigid_body_a)->get(),
      *rigid_body_from_value(rigid_body_b)->get(),
      coerce_vector(Rice::Object(axis_a)),
      coerce_vector(Rice::Object(axis_b)),
      ratio))
{
}

btVector3 GearConstraint::axis_a() const
{
  return gear_from(*this)->getAxisA();
}

btVector3 GearConstraint::set_axis_a(Rice::Object axis)
{
  btVector3 vector = coerce_vector(axis);
  gear_from(*this)->setAxisA(vector);
  return vector;
}

btVector3 GearConstraint::axis_b() const
{
  return gear_from(*this)->getAxisB();
}

btVector3 GearConstraint::set_axis_b(Rice::Object axis)
{
  btVector3 vector = coerce_vector(axis);
  gear_from(*this)->setAxisB(vector);
  return vector;
}

btScalar GearConstraint::ratio() const
{
  return gear_from(*this)->getRatio();
}

btScalar GearConstraint::set_ratio(btScalar ratio)
{
  gear_from(*this)->setRatio(ratio);
  return ratio;
}

Hinge2Constraint::Hinge2Constraint(VALUE rigid_body_a, VALUE rigid_body_b, VALUE anchor, VALUE axis1, VALUE axis2)
  : TypedConstraint(make_hinge2_constraint(rigid_body_a, rigid_body_b, anchor, axis1, axis2))
{
}

btVector3 Hinge2Constraint::anchor()
{
  return hinge2_from(*this)->getAnchor();
}

btVector3 Hinge2Constraint::anchor2()
{
  return hinge2_from(*this)->getAnchor2();
}

btVector3 Hinge2Constraint::axis1()
{
  return hinge2_from(*this)->getAxis1();
}

btVector3 Hinge2Constraint::axis2()
{
  return hinge2_from(*this)->getAxis2();
}

btScalar Hinge2Constraint::angle1()
{
  return hinge2_from(*this)->getAngle1();
}

btScalar Hinge2Constraint::angle2()
{
  return hinge2_from(*this)->getAngle2();
}

void Hinge2Constraint::set_upper_limit(btScalar limit)
{
  hinge2_from(*this)->setUpperLimit(limit);
}

void Hinge2Constraint::set_lower_limit(btScalar limit)
{
  hinge2_from(*this)->setLowerLimit(limit);
}
} // namespace bullet3

void Init_Constraints(Rice::Module rb_mBullet)
{
  Rice::Module rb_mConstraints = Rice::define_module_under(rb_mBullet, "Constraints");

  Rice::define_class_under<bullet3::TypedConstraint>(rb_mConstraints, "TypedConstraint")
    .define_method("constraint_type", &bullet3::TypedConstraint::constraint_type)
    .define_method("enabled?", &bullet3::TypedConstraint::enabled)
    .define_method("enabled=", &bullet3::TypedConstraint::set_enabled)
    .define_method("breaking_impulse_threshold", &bullet3::TypedConstraint::breaking_impulse_threshold)
    .define_method("breaking_impulse_threshold=", &bullet3::TypedConstraint::set_breaking_impulse_threshold)
    .define_method("debug_draw_size", &bullet3::TypedConstraint::debug_draw_size)
    .define_method("debug_draw_size=", &bullet3::TypedConstraint::set_debug_draw_size)
    .define_method("enable_feedback", &bullet3::TypedConstraint::enable_feedback)
    .define_method("needs_feedback?", &bullet3::TypedConstraint::needs_feedback)
    .define_method("applied_impulse", &bullet3::TypedConstraint::applied_impulse);

  Rice::define_class_under<bullet3::Point2PointConstraint, bullet3::TypedConstraint>(rb_mConstraints, "Point2PointConstraint")
    .define_constructor(Rice::Constructor<bullet3::Point2PointConstraint, VALUE, VALUE>(),
      Rice::Arg("rigid_body_a").setValue().keepAlive(),
      Rice::Arg("pivot_a").setValue())
    .define_constructor(Rice::Constructor<bullet3::Point2PointConstraint, VALUE, VALUE, VALUE, VALUE>(),
      Rice::Arg("rigid_body_a").setValue().keepAlive(),
      Rice::Arg("rigid_body_b").setValue().keepAlive(),
      Rice::Arg("pivot_a").setValue(),
      Rice::Arg("pivot_b").setValue())
    .define_method("pivot_in_a", &bullet3::Point2PointConstraint::pivot_in_a)
    .define_method("pivot_in_a=", &bullet3::Point2PointConstraint::set_pivot_in_a)
    .define_method("pivot_in_b", &bullet3::Point2PointConstraint::pivot_in_b)
    .define_method("pivot_in_b=", &bullet3::Point2PointConstraint::set_pivot_in_b);

  Rice::define_class_under<bullet3::HingeConstraint, bullet3::TypedConstraint>(rb_mConstraints, "HingeConstraint")
    .define_constructor(Rice::Constructor<bullet3::HingeConstraint, VALUE, VALUE, VALUE, bool>(),
      Rice::Arg("rigid_body_a").setValue().keepAlive(),
      Rice::Arg("pivot_a").setValue(),
      Rice::Arg("axis_a").setValue(),
      Rice::Arg("use_reference_frame_a") = false)
    .define_constructor(Rice::Constructor<bullet3::HingeConstraint, VALUE, VALUE, VALUE, VALUE, VALUE, VALUE, bool>(),
      Rice::Arg("rigid_body_a").setValue().keepAlive(),
      Rice::Arg("rigid_body_b").setValue().keepAlive(),
      Rice::Arg("pivot_a").setValue(),
      Rice::Arg("pivot_b").setValue(),
      Rice::Arg("axis_a").setValue(),
      Rice::Arg("axis_b").setValue(),
      Rice::Arg("use_reference_frame_a") = false)
    .define_method("angle", &bullet3::HingeConstraint::angle)
    .define_method("lower_limit", &bullet3::HingeConstraint::lower_limit)
    .define_method("upper_limit", &bullet3::HingeConstraint::upper_limit)
    .define_method("angular_motor_enabled?", &bullet3::HingeConstraint::angular_motor_enabled)
    .define_method("motor_target_velocity", &bullet3::HingeConstraint::motor_target_velocity)
    .define_method("max_motor_impulse", &bullet3::HingeConstraint::max_motor_impulse)
    .define_method("enable_angular_motor", &bullet3::HingeConstraint::enable_angular_motor)
    .define_method("set_limit", &bullet3::HingeConstraint::set_limit,
      Rice::Arg("low"),
      Rice::Arg("high"),
      Rice::Arg("softness") = btScalar(0.9),
      Rice::Arg("bias_factor") = btScalar(0.3),
      Rice::Arg("relaxation_factor") = btScalar(1.0))
    .define_method("frame_a", &bullet3::HingeConstraint::frame_a)
    .define_method("frame_b", &bullet3::HingeConstraint::frame_b);

  Rice::define_class_under<bullet3::FixedConstraint, bullet3::TypedConstraint>(rb_mConstraints, "FixedConstraint")
    .define_constructor(Rice::Constructor<bullet3::FixedConstraint, VALUE, VALUE, VALUE, VALUE>(),
      Rice::Arg("rigid_body_a").setValue().keepAlive(),
      Rice::Arg("rigid_body_b").setValue().keepAlive(),
      Rice::Arg("frame_a").setValue(),
      Rice::Arg("frame_b").setValue());

  Rice::define_class_under<bullet3::SliderConstraint, bullet3::TypedConstraint>(rb_mConstraints, "SliderConstraint")
    .define_constructor(Rice::Constructor<bullet3::SliderConstraint, VALUE, VALUE, bool>(),
      Rice::Arg("rigid_body_b").setValue().keepAlive(),
      Rice::Arg("frame_b").setValue(),
      Rice::Arg("use_linear_reference_frame_a") = true)
    .define_constructor(Rice::Constructor<bullet3::SliderConstraint, VALUE, VALUE, VALUE, VALUE, bool>(),
      Rice::Arg("rigid_body_a").setValue().keepAlive(),
      Rice::Arg("rigid_body_b").setValue().keepAlive(),
      Rice::Arg("frame_a").setValue(),
      Rice::Arg("frame_b").setValue(),
      Rice::Arg("use_linear_reference_frame_a") = true)
    .define_method("frame_a", &bullet3::SliderConstraint::frame_a)
    .define_method("frame_b", &bullet3::SliderConstraint::frame_b)
    .define_method("set_frames", &bullet3::SliderConstraint::set_frames,
      Rice::Arg("frame_a").setValue(),
      Rice::Arg("frame_b").setValue())
    .define_method("lower_linear_limit", &bullet3::SliderConstraint::lower_linear_limit)
    .define_method("lower_linear_limit=", &bullet3::SliderConstraint::set_lower_linear_limit)
    .define_method("upper_linear_limit", &bullet3::SliderConstraint::upper_linear_limit)
    .define_method("upper_linear_limit=", &bullet3::SliderConstraint::set_upper_linear_limit)
    .define_method("lower_angular_limit", &bullet3::SliderConstraint::lower_angular_limit)
    .define_method("lower_angular_limit=", &bullet3::SliderConstraint::set_lower_angular_limit)
    .define_method("upper_angular_limit", &bullet3::SliderConstraint::upper_angular_limit)
    .define_method("upper_angular_limit=", &bullet3::SliderConstraint::set_upper_angular_limit)
    .define_method("linear_position", &bullet3::SliderConstraint::linear_position)
    .define_method("angular_position", &bullet3::SliderConstraint::angular_position)
    .define_method("powered_linear_motor?", &bullet3::SliderConstraint::powered_linear_motor)
    .define_method("powered_linear_motor=", &bullet3::SliderConstraint::set_powered_linear_motor)
    .define_method("target_linear_motor_velocity", &bullet3::SliderConstraint::target_linear_motor_velocity)
    .define_method("target_linear_motor_velocity=", &bullet3::SliderConstraint::set_target_linear_motor_velocity)
    .define_method("max_linear_motor_force", &bullet3::SliderConstraint::max_linear_motor_force)
    .define_method("max_linear_motor_force=", &bullet3::SliderConstraint::set_max_linear_motor_force)
    .define_method("powered_angular_motor?", &bullet3::SliderConstraint::powered_angular_motor)
    .define_method("powered_angular_motor=", &bullet3::SliderConstraint::set_powered_angular_motor)
    .define_method("target_angular_motor_velocity", &bullet3::SliderConstraint::target_angular_motor_velocity)
    .define_method("target_angular_motor_velocity=", &bullet3::SliderConstraint::set_target_angular_motor_velocity)
    .define_method("max_angular_motor_force", &bullet3::SliderConstraint::max_angular_motor_force)
    .define_method("max_angular_motor_force=", &bullet3::SliderConstraint::set_max_angular_motor_force);

  Rice::define_class_under<bullet3::ConeTwistConstraint, bullet3::TypedConstraint>(rb_mConstraints, "ConeTwistConstraint")
    .define_constructor(Rice::Constructor<bullet3::ConeTwistConstraint, VALUE, VALUE>(),
      Rice::Arg("rigid_body_a").setValue().keepAlive(),
      Rice::Arg("frame_a").setValue())
    .define_constructor(Rice::Constructor<bullet3::ConeTwistConstraint, VALUE, VALUE, VALUE, VALUE>(),
      Rice::Arg("rigid_body_a").setValue().keepAlive(),
      Rice::Arg("rigid_body_b").setValue().keepAlive(),
      Rice::Arg("frame_a").setValue(),
      Rice::Arg("frame_b").setValue())
    .define_method("set_limit", &bullet3::ConeTwistConstraint::set_limit,
      Rice::Arg("swing_span1"),
      Rice::Arg("swing_span2"),
      Rice::Arg("twist_span"),
      Rice::Arg("softness") = btScalar(1.0),
      Rice::Arg("bias_factor") = btScalar(0.3),
      Rice::Arg("relaxation_factor") = btScalar(1.0))
    .define_method("swing_span1", &bullet3::ConeTwistConstraint::swing_span1)
    .define_method("swing_span2", &bullet3::ConeTwistConstraint::swing_span2)
    .define_method("twist_span", &bullet3::ConeTwistConstraint::twist_span)
    .define_method("limit_softness", &bullet3::ConeTwistConstraint::limit_softness)
    .define_method("bias_factor", &bullet3::ConeTwistConstraint::bias_factor)
    .define_method("relaxation_factor", &bullet3::ConeTwistConstraint::relaxation_factor)
    .define_method("twist_angle", &bullet3::ConeTwistConstraint::twist_angle)
    .define_method("angular_only?", &bullet3::ConeTwistConstraint::angular_only)
    .define_method("angular_only=", &bullet3::ConeTwistConstraint::set_angular_only)
    .define_method("damping", &bullet3::ConeTwistConstraint::damping)
    .define_method("damping=", &bullet3::ConeTwistConstraint::set_damping)
    .define_method("motor_enabled?", &bullet3::ConeTwistConstraint::motor_enabled)
    .define_method("enable_motor", &bullet3::ConeTwistConstraint::enable_motor)
    .define_method("max_motor_impulse", &bullet3::ConeTwistConstraint::max_motor_impulse)
    .define_method("max_motor_impulse=", &bullet3::ConeTwistConstraint::set_max_motor_impulse)
    .define_method("motor_target", &bullet3::ConeTwistConstraint::motor_target)
    .define_method("motor_target=", &bullet3::ConeTwistConstraint::set_motor_target)
    .define_method("frame_a", &bullet3::ConeTwistConstraint::frame_a)
    .define_method("frame_b", &bullet3::ConeTwistConstraint::frame_b);

  Rice::define_class_under<bullet3::Generic6DofConstraint, bullet3::TypedConstraint>(rb_mConstraints, "Generic6DofConstraint")
    .define_constructor(Rice::Constructor<bullet3::Generic6DofConstraint, VALUE, VALUE, bool>(),
      Rice::Arg("rigid_body_b").setValue().keepAlive(),
      Rice::Arg("frame_b").setValue(),
      Rice::Arg("use_linear_reference_frame_b") = true)
    .define_constructor(Rice::Constructor<bullet3::Generic6DofConstraint, VALUE, VALUE, VALUE, VALUE, bool>(),
      Rice::Arg("rigid_body_a").setValue().keepAlive(),
      Rice::Arg("rigid_body_b").setValue().keepAlive(),
      Rice::Arg("frame_a").setValue(),
      Rice::Arg("frame_b").setValue(),
      Rice::Arg("use_linear_reference_frame_a") = true)
    .define_method("frame_a", &bullet3::Generic6DofConstraint::frame_a)
    .define_method("frame_b", &bullet3::Generic6DofConstraint::frame_b)
    .define_method("set_frames", &bullet3::Generic6DofConstraint::set_frames,
      Rice::Arg("frame_a").setValue(),
      Rice::Arg("frame_b").setValue())
    .define_method("linear_lower_limit", &bullet3::Generic6DofConstraint::linear_lower_limit)
    .define_method("linear_lower_limit=", &bullet3::Generic6DofConstraint::set_linear_lower_limit)
    .define_method("linear_upper_limit", &bullet3::Generic6DofConstraint::linear_upper_limit)
    .define_method("linear_upper_limit=", &bullet3::Generic6DofConstraint::set_linear_upper_limit)
    .define_method("angular_lower_limit", &bullet3::Generic6DofConstraint::angular_lower_limit)
    .define_method("angular_lower_limit=", &bullet3::Generic6DofConstraint::set_angular_lower_limit)
    .define_method("angular_upper_limit", &bullet3::Generic6DofConstraint::angular_upper_limit)
    .define_method("angular_upper_limit=", &bullet3::Generic6DofConstraint::set_angular_upper_limit)
    .define_method("set_limit", &bullet3::Generic6DofConstraint::set_limit)
    .define_method("limited?", &bullet3::Generic6DofConstraint::limited)
    .define_method("axis", &bullet3::Generic6DofConstraint::axis)
    .define_method("angle", &bullet3::Generic6DofConstraint::angle)
    .define_method("relative_pivot_position", &bullet3::Generic6DofConstraint::relative_pivot_position)
    .define_method("use_frame_offset?", &bullet3::Generic6DofConstraint::use_frame_offset)
    .define_method("use_frame_offset=", &bullet3::Generic6DofConstraint::set_use_frame_offset)
    .define_method("use_linear_reference_frame_a?", &bullet3::Generic6DofConstraint::use_linear_reference_frame_a)
    .define_method("use_linear_reference_frame_a=", &bullet3::Generic6DofConstraint::set_use_linear_reference_frame_a)
    .define_method("set_axis", &bullet3::Generic6DofConstraint::set_axis);

  Rice::define_class_under<bullet3::Generic6DofSpring2Constraint, bullet3::TypedConstraint>(rb_mConstraints, "Generic6DofSpring2Constraint")
    .define_constructor(Rice::Constructor<bullet3::Generic6DofSpring2Constraint, VALUE, VALUE, int>(),
      Rice::Arg("rigid_body_b").setValue().keepAlive(),
      Rice::Arg("frame_b").setValue(),
      Rice::Arg("rotate_order") = 0)
    .define_constructor(Rice::Constructor<bullet3::Generic6DofSpring2Constraint, VALUE, VALUE, VALUE, VALUE, int>(),
      Rice::Arg("rigid_body_a").setValue().keepAlive(),
      Rice::Arg("rigid_body_b").setValue().keepAlive(),
      Rice::Arg("frame_a").setValue(),
      Rice::Arg("frame_b").setValue(),
      Rice::Arg("rotate_order") = 0)
    .define_method("frame_a", &bullet3::Generic6DofSpring2Constraint::frame_a)
    .define_method("frame_b", &bullet3::Generic6DofSpring2Constraint::frame_b)
    .define_method("set_frames", &bullet3::Generic6DofSpring2Constraint::set_frames,
      Rice::Arg("frame_a").setValue(),
      Rice::Arg("frame_b").setValue())
    .define_method("linear_lower_limit", &bullet3::Generic6DofSpring2Constraint::linear_lower_limit)
    .define_method("linear_lower_limit=", &bullet3::Generic6DofSpring2Constraint::set_linear_lower_limit)
    .define_method("linear_upper_limit", &bullet3::Generic6DofSpring2Constraint::linear_upper_limit)
    .define_method("linear_upper_limit=", &bullet3::Generic6DofSpring2Constraint::set_linear_upper_limit)
    .define_method("angular_lower_limit", &bullet3::Generic6DofSpring2Constraint::angular_lower_limit)
    .define_method("angular_lower_limit=", &bullet3::Generic6DofSpring2Constraint::set_angular_lower_limit)
    .define_method("angular_upper_limit", &bullet3::Generic6DofSpring2Constraint::angular_upper_limit)
    .define_method("angular_upper_limit=", &bullet3::Generic6DofSpring2Constraint::set_angular_upper_limit)
    .define_method("set_limit", &bullet3::Generic6DofSpring2Constraint::set_limit)
    .define_method("limited?", &bullet3::Generic6DofSpring2Constraint::limited)
    .define_method("axis", &bullet3::Generic6DofSpring2Constraint::axis)
    .define_method("angle", &bullet3::Generic6DofSpring2Constraint::angle)
    .define_method("relative_pivot_position", &bullet3::Generic6DofSpring2Constraint::relative_pivot_position)
    .define_method("rotation_order", &bullet3::Generic6DofSpring2Constraint::rotation_order)
    .define_method("rotation_order=", &bullet3::Generic6DofSpring2Constraint::set_rotation_order)
    .define_method("set_axis", &bullet3::Generic6DofSpring2Constraint::set_axis)
    .define_method("enable_motor", &bullet3::Generic6DofSpring2Constraint::enable_motor)
    .define_method("set_servo", &bullet3::Generic6DofSpring2Constraint::set_servo)
    .define_method("set_target_velocity", &bullet3::Generic6DofSpring2Constraint::set_target_velocity)
    .define_method("set_servo_target", &bullet3::Generic6DofSpring2Constraint::set_servo_target)
    .define_method("set_max_motor_force", &bullet3::Generic6DofSpring2Constraint::set_max_motor_force)
    .define_method("enable_spring", &bullet3::Generic6DofSpring2Constraint::enable_spring)
    .define_method("set_stiffness", &bullet3::Generic6DofSpring2Constraint::set_stiffness,
      Rice::Arg("axis"),
      Rice::Arg("stiffness"),
      Rice::Arg("limit_if_needed") = true)
    .define_method("set_damping", &bullet3::Generic6DofSpring2Constraint::set_damping,
      Rice::Arg("axis"),
      Rice::Arg("damping"),
      Rice::Arg("limit_if_needed") = true)
    .define_method("set_equilibrium_point", &bullet3::Generic6DofSpring2Constraint::set_equilibrium_point)
    .define_method("set_axis_equilibrium_point", &bullet3::Generic6DofSpring2Constraint::set_axis_equilibrium_point)
    .define_method("set_axis_equilibrium_point_value", &bullet3::Generic6DofSpring2Constraint::set_axis_equilibrium_point_value);

  Rice::define_class_under<bullet3::GearConstraint, bullet3::TypedConstraint>(rb_mConstraints, "GearConstraint")
    .define_constructor(Rice::Constructor<bullet3::GearConstraint, VALUE, VALUE, VALUE, VALUE, btScalar>(),
      Rice::Arg("rigid_body_a").setValue().keepAlive(),
      Rice::Arg("rigid_body_b").setValue().keepAlive(),
      Rice::Arg("axis_a").setValue(),
      Rice::Arg("axis_b").setValue(),
      Rice::Arg("ratio") = btScalar(1.0))
    .define_method("axis_a", &bullet3::GearConstraint::axis_a)
    .define_method("axis_a=", &bullet3::GearConstraint::set_axis_a)
    .define_method("axis_b", &bullet3::GearConstraint::axis_b)
    .define_method("axis_b=", &bullet3::GearConstraint::set_axis_b)
    .define_method("ratio", &bullet3::GearConstraint::ratio)
    .define_method("ratio=", &bullet3::GearConstraint::set_ratio);

  Rice::define_class_under<bullet3::Hinge2Constraint, bullet3::TypedConstraint>(rb_mConstraints, "Hinge2Constraint")
    .define_constructor(Rice::Constructor<bullet3::Hinge2Constraint, VALUE, VALUE, VALUE, VALUE, VALUE>(),
      Rice::Arg("rigid_body_a").setValue().keepAlive(),
      Rice::Arg("rigid_body_b").setValue().keepAlive(),
      Rice::Arg("anchor").setValue(),
      Rice::Arg("axis1").setValue(),
      Rice::Arg("axis2").setValue())
    .define_method("anchor", &bullet3::Hinge2Constraint::anchor)
    .define_method("anchor2", &bullet3::Hinge2Constraint::anchor2)
    .define_method("axis1", &bullet3::Hinge2Constraint::axis1)
    .define_method("axis2", &bullet3::Hinge2Constraint::axis2)
    .define_method("angle1", &bullet3::Hinge2Constraint::angle1)
    .define_method("angle2", &bullet3::Hinge2Constraint::angle2)
    .define_method("set_upper_limit", &bullet3::Hinge2Constraint::set_upper_limit)
    .define_method("set_lower_limit", &bullet3::Hinge2Constraint::set_lower_limit);
}
