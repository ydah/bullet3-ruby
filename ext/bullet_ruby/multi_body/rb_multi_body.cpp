#include "rb_multi_body.hpp"

#include <stdexcept>

#include <ruby/thread.h>

#include "../util/type_conversions.hpp"

namespace {
bullet_ruby::MultiBody* multi_body_from_value(VALUE value)
{
  bullet_ruby::MultiBody* body = Rice::detail::From_Ruby<bullet_ruby::MultiBody*>().convert(value);
  if (body == nullptr) {
    throw std::invalid_argument("expected Bullet::MultiBody::MultiBody");
  }
  return body;
}

bullet_ruby::MultiBodyConstraint* multi_body_constraint_from_value(VALUE value)
{
  bullet_ruby::MultiBodyConstraint* constraint = Rice::detail::From_Ruby<bullet_ruby::MultiBodyConstraint*>().convert(value);
  if (constraint == nullptr) {
    throw std::invalid_argument("expected Bullet::MultiBody::MultiBodyConstraint");
  }
  return constraint;
}

bullet_ruby::MultiBodyLinkCollider* multi_body_collider_from_value(VALUE value)
{
  bullet_ruby::MultiBodyLinkCollider* collider = Rice::detail::From_Ruby<bullet_ruby::MultiBodyLinkCollider*>().convert(value);
  if (collider == nullptr) {
    throw std::invalid_argument("expected Bullet::MultiBody::MultiBodyLinkCollider");
  }
  return collider;
}

bullet_ruby::RigidBody* rigid_body_from_value(VALUE value)
{
  bullet_ruby::RigidBody* body = Rice::detail::From_Ruby<bullet_ruby::RigidBody*>().convert(value);
  if (body == nullptr) {
    throw std::invalid_argument("expected Bullet::RigidBody");
  }
  return body;
}

btCollisionShape* collision_shape_from_value(VALUE value)
{
  btCollisionShape* shape = Rice::detail::From_Ruby<btCollisionShape*>().convert(value);
  if (shape == nullptr) {
    throw std::invalid_argument("expected Bullet::Shapes::CollisionShape");
  }
  return shape;
}

btMultiBodyJointMotor* joint_motor_from(bullet_ruby::MultiBodyConstraint& constraint)
{
  return static_cast<btMultiBodyJointMotor*>(constraint.get());
}

const btMultiBodyJointMotor* joint_motor_from(const bullet_ruby::MultiBodyConstraint& constraint)
{
  return static_cast<const btMultiBodyJointMotor*>(constraint.get());
}

btMultiBodyJointLimitConstraint* joint_limit_from(bullet_ruby::MultiBodyConstraint& constraint)
{
  return static_cast<btMultiBodyJointLimitConstraint*>(constraint.get());
}

const btMultiBodyJointLimitConstraint* joint_limit_from(const bullet_ruby::MultiBodyConstraint& constraint)
{
  return static_cast<const btMultiBodyJointLimitConstraint*>(constraint.get());
}

btMultiBodyPoint2Point* point2point_from(bullet_ruby::MultiBodyConstraint& constraint)
{
  return static_cast<btMultiBodyPoint2Point*>(constraint.get());
}

const btMultiBodyPoint2Point* point2point_from(const bullet_ruby::MultiBodyConstraint& constraint)
{
  return static_cast<const btMultiBodyPoint2Point*>(constraint.get());
}

struct StepSimulationArgs {
  btMultiBodyDynamicsWorld* world;
  btScalar time_step;
  int max_sub_steps;
  btScalar fixed_time_step;
  int result;
};

void* step_simulation_without_gvl(void* pointer)
{
  auto* args = static_cast<StepSimulationArgs*>(pointer);
  args->result = args->world->stepSimulation(args->time_step, args->max_sub_steps, args->fixed_time_step);
  return nullptr;
}
} // namespace

namespace bullet_ruby {
MultiBody::MultiBody(int link_count, btScalar base_mass, Rice::Object base_inertia, bool fixed_base, bool can_sleep, bool deprecated_multi_dof)
  : multi_body_(std::make_unique<btMultiBody>(
      link_count,
      base_mass,
      coerce_vector(base_inertia),
      fixed_base,
      can_sleep,
      deprecated_multi_dof))
{
}

btMultiBody* MultiBody::get()
{
  return multi_body_.get();
}

const btMultiBody* MultiBody::get() const
{
  return multi_body_.get();
}

int MultiBody::validate_link_index(int link) const
{
  if (link < 0 || link >= multi_body_->getNumLinks()) {
    throw std::out_of_range("link index is out of range");
  }
  return link;
}

int MultiBody::num_links() const
{
  return multi_body_->getNumLinks();
}

int MultiBody::num_dofs() const
{
  return multi_body_->getNumDofs();
}

int MultiBody::num_pos_vars() const
{
  return multi_body_->getNumPosVars();
}

btScalar MultiBody::base_mass() const
{
  return multi_body_->getBaseMass();
}

btScalar MultiBody::set_base_mass(btScalar mass)
{
  multi_body_->setBaseMass(mass);
  return mass;
}

btVector3 MultiBody::base_inertia() const
{
  return multi_body_->getBaseInertia();
}

btVector3 MultiBody::set_base_inertia(Rice::Object inertia)
{
  btVector3 vector = coerce_vector(inertia);
  multi_body_->setBaseInertia(vector);
  return vector;
}

btVector3 MultiBody::base_position() const
{
  return multi_body_->getBasePos();
}

btVector3 MultiBody::set_base_position(Rice::Object position)
{
  btVector3 vector = coerce_vector(position);
  multi_body_->setBasePos(vector);
  return vector;
}

btVector3 MultiBody::base_velocity() const
{
  return multi_body_->getBaseVel();
}

btVector3 MultiBody::set_base_velocity(Rice::Object velocity)
{
  btVector3 vector = coerce_vector(velocity);
  multi_body_->setBaseVel(vector);
  return vector;
}

btVector3 MultiBody::base_omega() const
{
  return multi_body_->getBaseOmega();
}

btVector3 MultiBody::set_base_omega(Rice::Object omega)
{
  btVector3 vector = coerce_vector(omega);
  multi_body_->setBaseOmega(vector);
  return vector;
}

btQuaternion MultiBody::world_to_base_rotation() const
{
  return multi_body_->getWorldToBaseRot();
}

btQuaternion MultiBody::set_world_to_base_rotation(Rice::Object rotation)
{
  btQuaternion quaternion = coerce_quaternion(rotation);
  multi_body_->setWorldToBaseRot(quaternion);
  return quaternion;
}

Transform MultiBody::base_world_transform() const
{
  return Transform(multi_body_->getBaseWorldTransform());
}

Transform MultiBody::set_base_world_transform(Rice::Object transform)
{
  btTransform value = coerce_transform(transform);
  multi_body_->setBaseWorldTransform(value);
  return Transform(value);
}

bool MultiBody::fixed_base() const
{
  return multi_body_->hasFixedBase();
}

bool MultiBody::can_sleep() const
{
  return multi_body_->getCanSleep();
}

bool MultiBody::set_can_sleep(bool can_sleep)
{
  multi_body_->setCanSleep(can_sleep);
  return can_sleep;
}

bool MultiBody::awake() const
{
  return multi_body_->isAwake();
}

void MultiBody::wake_up()
{
  multi_body_->wakeUp();
}

void MultiBody::go_to_sleep()
{
  multi_body_->goToSleep();
}

btScalar MultiBody::linear_damping() const
{
  return multi_body_->getLinearDamping();
}

btScalar MultiBody::set_linear_damping(btScalar damping)
{
  multi_body_->setLinearDamping(damping);
  return damping;
}

btScalar MultiBody::angular_damping() const
{
  return multi_body_->getAngularDamping();
}

btScalar MultiBody::set_angular_damping(btScalar damping)
{
  multi_body_->setAngularDamping(damping);
  return damping;
}

bool MultiBody::self_collision() const
{
  return multi_body_->hasSelfCollision();
}

bool MultiBody::set_self_collision(bool enabled)
{
  multi_body_->setHasSelfCollision(enabled);
  return enabled;
}

btScalar MultiBody::joint_position(int link) const
{
  return multi_body_->getJointPos(validate_link_index(link));
}

btScalar MultiBody::set_joint_position(int link, btScalar position)
{
  multi_body_->setJointPos(validate_link_index(link), position);
  return position;
}

btScalar MultiBody::joint_velocity(int link) const
{
  return multi_body_->getJointVel(validate_link_index(link));
}

btScalar MultiBody::set_joint_velocity(int link, btScalar velocity)
{
  multi_body_->setJointVel(validate_link_index(link), velocity);
  return velocity;
}

void MultiBody::setup_fixed(int link, btScalar mass, Rice::Object inertia, int parent, Rice::Object parent_to_this_rotation, Rice::Object parent_pivot, Rice::Object child_pivot, bool disable_parent_collision)
{
  multi_body_->setupFixed(
    validate_link_index(link),
    mass,
    coerce_vector(inertia),
    parent,
    coerce_quaternion(parent_to_this_rotation),
    coerce_vector(parent_pivot),
    coerce_vector(child_pivot),
    disable_parent_collision);
}

void MultiBody::setup_revolute(int link, btScalar mass, Rice::Object inertia, int parent, Rice::Object parent_to_this_rotation, Rice::Object joint_axis, Rice::Object parent_pivot, Rice::Object child_pivot, bool disable_parent_collision)
{
  multi_body_->setupRevolute(
    validate_link_index(link),
    mass,
    coerce_vector(inertia),
    parent,
    coerce_quaternion(parent_to_this_rotation),
    coerce_vector(joint_axis),
    coerce_vector(parent_pivot),
    coerce_vector(child_pivot),
    disable_parent_collision);
}

void MultiBody::setup_prismatic(int link, btScalar mass, Rice::Object inertia, int parent, Rice::Object parent_to_this_rotation, Rice::Object joint_axis, Rice::Object parent_pivot, Rice::Object child_pivot, bool disable_parent_collision)
{
  multi_body_->setupPrismatic(
    validate_link_index(link),
    mass,
    coerce_vector(inertia),
    parent,
    coerce_quaternion(parent_to_this_rotation),
    coerce_vector(joint_axis),
    coerce_vector(parent_pivot),
    coerce_vector(child_pivot),
    disable_parent_collision);
}

void MultiBody::finalize_multi_dof()
{
  multi_body_->finalizeMultiDof();
}

void MultiBody::clear_forces_and_torques()
{
  multi_body_->clearForcesAndTorques();
}

void MultiBody::clear_velocities()
{
  multi_body_->clearVelocities();
}

void MultiBody::add_base_force(Rice::Object force)
{
  multi_body_->addBaseForce(coerce_vector(force));
}

void MultiBody::add_base_torque(Rice::Object torque)
{
  multi_body_->addBaseTorque(coerce_vector(torque));
}

void MultiBody::add_link_force(int link, Rice::Object force)
{
  multi_body_->addLinkForce(validate_link_index(link), coerce_vector(force));
}

void MultiBody::add_link_torque(int link, Rice::Object torque)
{
  multi_body_->addLinkTorque(validate_link_index(link), coerce_vector(torque));
}

void MultiBody::add_joint_torque(int link, btScalar torque)
{
  multi_body_->addJointTorque(validate_link_index(link), torque);
}

int MultiBody::parent(int link) const
{
  return multi_body_->getParent(validate_link_index(link));
}

void MultiBody::set_base_collider(MultiBodyLinkCollider& collider)
{
  multi_body_->setBaseCollider(collider.get());
}

void MultiBody::set_link_collider(int link, MultiBodyLinkCollider& collider)
{
  multi_body_->getLink(validate_link_index(link)).m_collider = collider.get();
}

MultiBodyLinkCollider::MultiBodyLinkCollider(MultiBody& multi_body, int link)
  : collider_(std::make_unique<btMultiBodyLinkCollider>(multi_body.get(), link))
{
}

btMultiBodyLinkCollider* MultiBodyLinkCollider::get()
{
  return collider_.get();
}

const btMultiBodyLinkCollider* MultiBodyLinkCollider::get() const
{
  return collider_.get();
}

int MultiBodyLinkCollider::link() const
{
  return collider_->m_link;
}

Transform MultiBodyLinkCollider::world_transform() const
{
  return Transform(collider_->getWorldTransform());
}

Transform MultiBodyLinkCollider::set_world_transform(Rice::Object transform)
{
  btTransform value = coerce_transform(transform);
  collider_->setWorldTransform(value);
  return Transform(value);
}

void MultiBodyLinkCollider::set_collision_shape(VALUE collision_shape)
{
  collider_->setCollisionShape(collision_shape_from_value(collision_shape));
}

btCollisionShape* MultiBodyLinkCollider::collision_shape() const
{
  return collider_->getCollisionShape();
}

int MultiBodyLinkCollider::collision_flags() const
{
  return collider_->getCollisionFlags();
}

int MultiBodyLinkCollider::set_collision_flags(int flags)
{
  collider_->setCollisionFlags(flags);
  return flags;
}

bool MultiBodyLinkCollider::kinematic() const
{
  return collider_->isKinematic();
}

bool MultiBodyLinkCollider::static_or_kinematic() const
{
  return collider_->isStaticOrKinematic();
}

MultiBodyConstraint::MultiBodyConstraint(std::unique_ptr<btMultiBodyConstraint> constraint)
  : constraint_(std::move(constraint))
{
}

btMultiBodyConstraint* MultiBodyConstraint::get()
{
  return constraint_.get();
}

const btMultiBodyConstraint* MultiBodyConstraint::get() const
{
  return constraint_.get();
}

Rice::Symbol MultiBodyConstraint::constraint_type() const
{
  switch (constraint_->getConstraintType()) {
    case MULTIBODY_CONSTRAINT_LIMIT:
      return Rice::Symbol("joint_limit");
    case MULTIBODY_CONSTRAINT_1DOF_JOINT_MOTOR:
      return Rice::Symbol("joint_motor");
    case MULTIBODY_CONSTRAINT_POINT_TO_POINT:
      return Rice::Symbol("point2point");
    default:
      return Rice::Symbol("unknown");
  }
}

int MultiBodyConstraint::num_rows() const
{
  return constraint_->getNumRows();
}

btScalar MultiBodyConstraint::max_applied_impulse() const
{
  return constraint_->getMaxAppliedImpulse();
}

btScalar MultiBodyConstraint::set_max_applied_impulse(btScalar impulse)
{
  constraint_->setMaxAppliedImpulse(impulse);
  return impulse;
}

btScalar MultiBodyConstraint::applied_impulse(int row)
{
  return constraint_->getAppliedImpulse(row);
}

btScalar MultiBodyConstraint::position(int row) const
{
  return constraint_->getPosition(row);
}

MultiBodyJointMotor::MultiBodyJointMotor(VALUE multi_body, int link, btScalar desired_velocity, btScalar max_motor_impulse)
  : MultiBodyConstraint(std::make_unique<btMultiBodyJointMotor>(
      multi_body_from_value(multi_body)->get(),
      link,
      desired_velocity,
      max_motor_impulse))
{
}

MultiBodyJointMotor::MultiBodyJointMotor(VALUE multi_body, int link, int link_dof, btScalar desired_velocity, btScalar max_motor_impulse)
  : MultiBodyConstraint(std::make_unique<btMultiBodyJointMotor>(
      multi_body_from_value(multi_body)->get(),
      link,
      link_dof,
      desired_velocity,
      max_motor_impulse))
{
}

void MultiBodyJointMotor::set_velocity_target(btScalar velocity, btScalar kd)
{
  joint_motor_from(*this)->setVelocityTarget(velocity, kd);
}

void MultiBodyJointMotor::set_position_target(btScalar position, btScalar kp)
{
  joint_motor_from(*this)->setPositionTarget(position, kp);
}

btScalar MultiBodyJointMotor::erp() const
{
  return joint_motor_from(*this)->getErp();
}

btScalar MultiBodyJointMotor::set_erp(btScalar erp)
{
  joint_motor_from(*this)->setErp(erp);
  return erp;
}

void MultiBodyJointMotor::set_rhs_clamp(btScalar clamp)
{
  joint_motor_from(*this)->setRhsClamp(clamp);
}

MultiBodyJointLimitConstraint::MultiBodyJointLimitConstraint(VALUE multi_body, int link, btScalar lower, btScalar upper)
  : MultiBodyConstraint(std::make_unique<btMultiBodyJointLimitConstraint>(
      multi_body_from_value(multi_body)->get(),
      link,
      lower,
      upper))
{
}

btScalar MultiBodyJointLimitConstraint::lower_bound() const
{
  return joint_limit_from(*this)->getLowerBound();
}

btScalar MultiBodyJointLimitConstraint::set_lower_bound(btScalar lower)
{
  joint_limit_from(*this)->setLowerBound(lower);
  return lower;
}

btScalar MultiBodyJointLimitConstraint::upper_bound() const
{
  return joint_limit_from(*this)->getUpperBound();
}

btScalar MultiBodyJointLimitConstraint::set_upper_bound(btScalar upper)
{
  joint_limit_from(*this)->setUpperBound(upper);
  return upper;
}

MultiBodyPoint2Point::MultiBodyPoint2Point(VALUE multi_body, int link, VALUE rigid_body, VALUE pivot_in_a, VALUE pivot_in_b)
  : MultiBodyConstraint(std::make_unique<btMultiBodyPoint2Point>(
      multi_body_from_value(multi_body)->get(),
      link,
      rigid_body_from_value(rigid_body)->get(),
      coerce_vector(Rice::Object(pivot_in_a)),
      coerce_vector(Rice::Object(pivot_in_b))))
{
}

MultiBodyPoint2Point::MultiBodyPoint2Point(VALUE multi_body_a, int link_a, VALUE multi_body_b, int link_b, VALUE pivot_in_a, VALUE pivot_in_b)
  : MultiBodyConstraint(std::make_unique<btMultiBodyPoint2Point>(
      multi_body_from_value(multi_body_a)->get(),
      link_a,
      multi_body_from_value(multi_body_b)->get(),
      link_b,
      coerce_vector(Rice::Object(pivot_in_a)),
      coerce_vector(Rice::Object(pivot_in_b))))
{
}

btVector3 MultiBodyPoint2Point::pivot_in_b() const
{
  return point2point_from(*this)->getPivotInB();
}

btVector3 MultiBodyPoint2Point::set_pivot_in_b(Rice::Object pivot)
{
  btVector3 vector = coerce_vector(pivot);
  point2point_from(*this)->setPivotInB(vector);
  return vector;
}

MultiBodyDynamicsWorld::MultiBodyDynamicsWorld()
  : configuration_(std::make_unique<btDefaultCollisionConfiguration>()),
    dispatcher_(std::make_unique<btCollisionDispatcher>(configuration_.get())),
    broadphase_(std::make_unique<btDbvtBroadphase>()),
    solver_(std::make_unique<btMultiBodyConstraintSolver>()),
    world_(std::make_unique<btMultiBodyDynamicsWorld>(
      dispatcher_.get(),
      broadphase_.get(),
      solver_.get(),
      configuration_.get()))
{
}

MultiBodyDynamicsWorld::~MultiBodyDynamicsWorld()
{
  constraints_.clear();
  colliders_.clear();
  multi_bodies_.clear();
  constraint_values_.clear();
  collider_values_.clear();
  multi_body_values_.clear();
}

btMultiBodyDynamicsWorld* MultiBodyDynamicsWorld::get()
{
  return world_.get();
}

btVector3 MultiBodyDynamicsWorld::gravity() const
{
  return world_->getGravity();
}

btVector3 MultiBodyDynamicsWorld::set_gravity(Rice::Object gravity)
{
  btVector3 vector = coerce_vector(gravity);
  world_->setGravity(vector);
  return vector;
}

void MultiBodyDynamicsWorld::add_multi_body_object(VALUE multi_body, int group, int mask)
{
  MultiBody* body = multi_body_from_value(multi_body);
  btMultiBody* bullet_body = body->get();
  if (multi_bodies_.insert(bullet_body).second) {
    world_->addMultiBody(bullet_body, group, mask);
    multi_body_values_[bullet_body] = multi_body;
  }
}

void MultiBodyDynamicsWorld::remove_multi_body_object(VALUE multi_body)
{
  MultiBody* body = multi_body_from_value(multi_body);
  btMultiBody* bullet_body = body->get();
  auto iterator = multi_bodies_.find(bullet_body);
  if (iterator != multi_bodies_.end()) {
    world_->removeMultiBody(bullet_body);
    multi_bodies_.erase(iterator);
  }
  multi_body_values_.erase(bullet_body);
}

void MultiBodyDynamicsWorld::add_constraint_object(VALUE constraint)
{
  MultiBodyConstraint* typed_constraint = multi_body_constraint_from_value(constraint);
  btMultiBodyConstraint* bullet_constraint = typed_constraint->get();
  if (constraints_.insert(bullet_constraint).second) {
    world_->addMultiBodyConstraint(bullet_constraint);
    constraint_values_[bullet_constraint] = constraint;
  }
}

void MultiBodyDynamicsWorld::remove_constraint_object(VALUE constraint)
{
  MultiBodyConstraint* typed_constraint = multi_body_constraint_from_value(constraint);
  btMultiBodyConstraint* bullet_constraint = typed_constraint->get();
  auto iterator = constraints_.find(bullet_constraint);
  if (iterator != constraints_.end()) {
    world_->removeMultiBodyConstraint(bullet_constraint);
    constraints_.erase(iterator);
  }
  constraint_values_.erase(bullet_constraint);
}

void MultiBodyDynamicsWorld::add_link_collider_object(VALUE collider, int group, int mask)
{
  MultiBodyLinkCollider* link_collider = multi_body_collider_from_value(collider);
  btMultiBodyLinkCollider* bullet_collider = link_collider->get();
  if (colliders_.insert(bullet_collider).second) {
    world_->addCollisionObject(bullet_collider, group, mask);
    collider_values_[bullet_collider] = collider;
  }
}

void MultiBodyDynamicsWorld::remove_link_collider_object(VALUE collider)
{
  MultiBodyLinkCollider* link_collider = multi_body_collider_from_value(collider);
  btMultiBodyLinkCollider* bullet_collider = link_collider->get();
  auto iterator = colliders_.find(bullet_collider);
  if (iterator != colliders_.end()) {
    world_->removeCollisionObject(bullet_collider);
    colliders_.erase(iterator);
  }
  collider_values_.erase(bullet_collider);
}

int MultiBodyDynamicsWorld::step_simulation(btScalar time_step, int max_sub_steps, btScalar fixed_time_step)
{
  StepSimulationArgs args{world_.get(), time_step, max_sub_steps, fixed_time_step, 0};
  rb_thread_call_without_gvl(step_simulation_without_gvl, &args, nullptr, nullptr);
  return args.result;
}

int MultiBodyDynamicsWorld::num_multi_bodies() const
{
  return world_->getNumMultibodies();
}

int MultiBodyDynamicsWorld::num_multi_body_constraints() const
{
  return world_->getNumMultiBodyConstraints();
}

int MultiBodyDynamicsWorld::num_collision_objects() const
{
  return world_->getNumCollisionObjects();
}

void MultiBodyDynamicsWorld::forward_kinematics()
{
  world_->forwardKinematics();
}

void MultiBodyDynamicsWorld::clear_forces()
{
  world_->clearForces();
}

void MultiBodyDynamicsWorld::clear_multi_body_forces()
{
  world_->clearMultiBodyForces();
}

void MultiBodyDynamicsWorld::mark() const
{
  for (const auto& entry : multi_body_values_) {
    rb_gc_mark(entry.second);
  }
  for (const auto& entry : constraint_values_) {
    rb_gc_mark(entry.second);
  }
  for (const auto& entry : collider_values_) {
    rb_gc_mark(entry.second);
  }
}
} // namespace bullet_ruby

namespace Rice {
template <>
void ruby_mark<bullet_ruby::MultiBodyDynamicsWorld>(bullet_ruby::MultiBodyDynamicsWorld* world)
{
  if (world != nullptr) {
    world->mark();
  }
}
} // namespace Rice

void Init_MultiBody(Rice::Module rb_mBullet)
{
  Rice::Module rb_mMultiBody = Rice::define_module_under(rb_mBullet, "MultiBody");

  Rice::define_class_under<bullet_ruby::MultiBody>(rb_mMultiBody, "MultiBody")
    .define_constructor(Rice::Constructor<bullet_ruby::MultiBody, int, btScalar, Rice::Object, bool, bool, bool>(),
      Rice::Arg("link_count"),
      Rice::Arg("base_mass"),
      Rice::Arg("base_inertia"),
      Rice::Arg("fixed_base"),
      Rice::Arg("can_sleep"),
      Rice::Arg("deprecated_multi_dof") = true)
    .define_method("num_links", &bullet_ruby::MultiBody::num_links)
    .define_method("num_dofs", &bullet_ruby::MultiBody::num_dofs)
    .define_method("num_pos_vars", &bullet_ruby::MultiBody::num_pos_vars)
    .define_method("base_mass", &bullet_ruby::MultiBody::base_mass)
    .define_method("base_mass=", &bullet_ruby::MultiBody::set_base_mass)
    .define_method("base_inertia", &bullet_ruby::MultiBody::base_inertia)
    .define_method("base_inertia=", &bullet_ruby::MultiBody::set_base_inertia)
    .define_method("base_position", &bullet_ruby::MultiBody::base_position)
    .define_method("base_position=", &bullet_ruby::MultiBody::set_base_position)
    .define_method("base_velocity", &bullet_ruby::MultiBody::base_velocity)
    .define_method("base_velocity=", &bullet_ruby::MultiBody::set_base_velocity)
    .define_method("base_omega", &bullet_ruby::MultiBody::base_omega)
    .define_method("base_omega=", &bullet_ruby::MultiBody::set_base_omega)
    .define_method("world_to_base_rotation", &bullet_ruby::MultiBody::world_to_base_rotation)
    .define_method("world_to_base_rotation=", &bullet_ruby::MultiBody::set_world_to_base_rotation)
    .define_method("base_world_transform", &bullet_ruby::MultiBody::base_world_transform)
    .define_method("base_world_transform=", &bullet_ruby::MultiBody::set_base_world_transform)
    .define_method("fixed_base?", &bullet_ruby::MultiBody::fixed_base)
    .define_method("can_sleep?", &bullet_ruby::MultiBody::can_sleep)
    .define_method("can_sleep=", &bullet_ruby::MultiBody::set_can_sleep)
    .define_method("awake?", &bullet_ruby::MultiBody::awake)
    .define_method("wake_up", &bullet_ruby::MultiBody::wake_up)
    .define_method("go_to_sleep", &bullet_ruby::MultiBody::go_to_sleep)
    .define_method("linear_damping", &bullet_ruby::MultiBody::linear_damping)
    .define_method("linear_damping=", &bullet_ruby::MultiBody::set_linear_damping)
    .define_method("angular_damping", &bullet_ruby::MultiBody::angular_damping)
    .define_method("angular_damping=", &bullet_ruby::MultiBody::set_angular_damping)
    .define_method("self_collision?", &bullet_ruby::MultiBody::self_collision)
    .define_method("self_collision=", &bullet_ruby::MultiBody::set_self_collision)
    .define_method("joint_position", &bullet_ruby::MultiBody::joint_position)
    .define_method("set_joint_position", &bullet_ruby::MultiBody::set_joint_position)
    .define_method("joint_velocity", &bullet_ruby::MultiBody::joint_velocity)
    .define_method("set_joint_velocity", &bullet_ruby::MultiBody::set_joint_velocity)
    .define_method("setup_fixed", &bullet_ruby::MultiBody::setup_fixed,
      Rice::Arg("link"),
      Rice::Arg("mass"),
      Rice::Arg("inertia"),
      Rice::Arg("parent"),
      Rice::Arg("parent_to_this_rotation"),
      Rice::Arg("parent_pivot"),
      Rice::Arg("child_pivot"),
      Rice::Arg("disable_parent_collision") = true)
    .define_method("setup_revolute", &bullet_ruby::MultiBody::setup_revolute,
      Rice::Arg("link"),
      Rice::Arg("mass"),
      Rice::Arg("inertia"),
      Rice::Arg("parent"),
      Rice::Arg("parent_to_this_rotation"),
      Rice::Arg("joint_axis"),
      Rice::Arg("parent_pivot"),
      Rice::Arg("child_pivot"),
      Rice::Arg("disable_parent_collision") = false)
    .define_method("setup_prismatic", &bullet_ruby::MultiBody::setup_prismatic,
      Rice::Arg("link"),
      Rice::Arg("mass"),
      Rice::Arg("inertia"),
      Rice::Arg("parent"),
      Rice::Arg("parent_to_this_rotation"),
      Rice::Arg("joint_axis"),
      Rice::Arg("parent_pivot"),
      Rice::Arg("child_pivot"),
      Rice::Arg("disable_parent_collision") = false)
    .define_method("finalize_multi_dof", &bullet_ruby::MultiBody::finalize_multi_dof)
    .define_method("clear_forces_and_torques", &bullet_ruby::MultiBody::clear_forces_and_torques)
    .define_method("clear_velocities", &bullet_ruby::MultiBody::clear_velocities)
    .define_method("add_base_force", &bullet_ruby::MultiBody::add_base_force)
    .define_method("add_base_torque", &bullet_ruby::MultiBody::add_base_torque)
    .define_method("add_link_force", &bullet_ruby::MultiBody::add_link_force)
    .define_method("add_link_torque", &bullet_ruby::MultiBody::add_link_torque)
    .define_method("add_joint_torque", &bullet_ruby::MultiBody::add_joint_torque)
    .define_method("parent", &bullet_ruby::MultiBody::parent)
    .define_method("set_base_collider", &bullet_ruby::MultiBody::set_base_collider,
      Rice::Arg("collider").keepAlive())
    .define_method("set_link_collider", &bullet_ruby::MultiBody::set_link_collider,
      Rice::Arg("link"),
      Rice::Arg("collider").keepAlive());

  Rice::define_class_under<bullet_ruby::MultiBodyLinkCollider>(rb_mMultiBody, "MultiBodyLinkCollider")
    .define_constructor(Rice::Constructor<bullet_ruby::MultiBodyLinkCollider, bullet_ruby::MultiBody&, int>(),
      Rice::Arg("multi_body").keepAlive(),
      Rice::Arg("link"))
    .define_method("link", &bullet_ruby::MultiBodyLinkCollider::link)
    .define_method("world_transform", &bullet_ruby::MultiBodyLinkCollider::world_transform)
    .define_method("world_transform=", &bullet_ruby::MultiBodyLinkCollider::set_world_transform)
    .define_method("collision_shape=", &bullet_ruby::MultiBodyLinkCollider::set_collision_shape,
      Rice::Arg("collision_shape").setValue().keepAlive())
    .define_method("collision_shape", &bullet_ruby::MultiBodyLinkCollider::collision_shape)
    .define_method("collision_flags", &bullet_ruby::MultiBodyLinkCollider::collision_flags)
    .define_method("collision_flags=", &bullet_ruby::MultiBodyLinkCollider::set_collision_flags)
    .define_method("kinematic?", &bullet_ruby::MultiBodyLinkCollider::kinematic)
    .define_method("static_or_kinematic?", &bullet_ruby::MultiBodyLinkCollider::static_or_kinematic);

  Rice::define_class_under<bullet_ruby::MultiBodyConstraint>(rb_mMultiBody, "MultiBodyConstraint")
    .define_method("constraint_type", &bullet_ruby::MultiBodyConstraint::constraint_type)
    .define_method("num_rows", &bullet_ruby::MultiBodyConstraint::num_rows)
    .define_method("max_applied_impulse", &bullet_ruby::MultiBodyConstraint::max_applied_impulse)
    .define_method("max_applied_impulse=", &bullet_ruby::MultiBodyConstraint::set_max_applied_impulse)
    .define_method("applied_impulse", &bullet_ruby::MultiBodyConstraint::applied_impulse)
    .define_method("position", &bullet_ruby::MultiBodyConstraint::position);

  Rice::define_class_under<bullet_ruby::MultiBodyJointMotor, bullet_ruby::MultiBodyConstraint>(rb_mMultiBody, "MultiBodyJointMotor")
    .define_constructor(Rice::Constructor<bullet_ruby::MultiBodyJointMotor, VALUE, int, btScalar, btScalar>(),
      Rice::Arg("multi_body").setValue().keepAlive(),
      Rice::Arg("link"),
      Rice::Arg("desired_velocity"),
      Rice::Arg("max_motor_impulse"))
    .define_constructor(Rice::Constructor<bullet_ruby::MultiBodyJointMotor, VALUE, int, int, btScalar, btScalar>(),
      Rice::Arg("multi_body").setValue().keepAlive(),
      Rice::Arg("link"),
      Rice::Arg("link_dof"),
      Rice::Arg("desired_velocity"),
      Rice::Arg("max_motor_impulse"))
    .define_method("set_velocity_target", &bullet_ruby::MultiBodyJointMotor::set_velocity_target,
      Rice::Arg("velocity"),
      Rice::Arg("kd") = btScalar(1.0))
    .define_method("set_position_target", &bullet_ruby::MultiBodyJointMotor::set_position_target,
      Rice::Arg("position"),
      Rice::Arg("kp") = btScalar(1.0))
    .define_method("erp", &bullet_ruby::MultiBodyJointMotor::erp)
    .define_method("erp=", &bullet_ruby::MultiBodyJointMotor::set_erp)
    .define_method("rhs_clamp=", &bullet_ruby::MultiBodyJointMotor::set_rhs_clamp);

  Rice::define_class_under<bullet_ruby::MultiBodyJointLimitConstraint, bullet_ruby::MultiBodyConstraint>(rb_mMultiBody, "MultiBodyJointLimitConstraint")
    .define_constructor(Rice::Constructor<bullet_ruby::MultiBodyJointLimitConstraint, VALUE, int, btScalar, btScalar>(),
      Rice::Arg("multi_body").setValue().keepAlive(),
      Rice::Arg("link"),
      Rice::Arg("lower"),
      Rice::Arg("upper"))
    .define_method("lower_bound", &bullet_ruby::MultiBodyJointLimitConstraint::lower_bound)
    .define_method("lower_bound=", &bullet_ruby::MultiBodyJointLimitConstraint::set_lower_bound)
    .define_method("upper_bound", &bullet_ruby::MultiBodyJointLimitConstraint::upper_bound)
    .define_method("upper_bound=", &bullet_ruby::MultiBodyJointLimitConstraint::set_upper_bound);

  Rice::define_class_under<bullet_ruby::MultiBodyPoint2Point, bullet_ruby::MultiBodyConstraint>(rb_mMultiBody, "MultiBodyPoint2Point")
    .define_constructor(Rice::Constructor<bullet_ruby::MultiBodyPoint2Point, VALUE, int, VALUE, VALUE, VALUE>(),
      Rice::Arg("multi_body").setValue().keepAlive(),
      Rice::Arg("link"),
      Rice::Arg("rigid_body").setValue().keepAlive(),
      Rice::Arg("pivot_in_a").setValue(),
      Rice::Arg("pivot_in_b").setValue())
    .define_constructor(Rice::Constructor<bullet_ruby::MultiBodyPoint2Point, VALUE, int, VALUE, int, VALUE, VALUE>(),
      Rice::Arg("multi_body_a").setValue().keepAlive(),
      Rice::Arg("link_a"),
      Rice::Arg("multi_body_b").setValue().keepAlive(),
      Rice::Arg("link_b"),
      Rice::Arg("pivot_in_a").setValue(),
      Rice::Arg("pivot_in_b").setValue())
    .define_method("pivot_in_b", &bullet_ruby::MultiBodyPoint2Point::pivot_in_b)
    .define_method("pivot_in_b=", &bullet_ruby::MultiBodyPoint2Point::set_pivot_in_b);

  Rice::define_class_under<bullet_ruby::MultiBodyDynamicsWorld>(rb_mMultiBody, "MultiBodyDynamicsWorld")
    .define_constructor(Rice::Constructor<bullet_ruby::MultiBodyDynamicsWorld>())
    .define_singleton_method("create", [](Rice::Object) -> bullet_ruby::MultiBodyDynamicsWorld* {
      return new bullet_ruby::MultiBodyDynamicsWorld();
    }, Rice::Return().takeOwnership())
    .define_method("gravity", &bullet_ruby::MultiBodyDynamicsWorld::gravity)
    .define_method("gravity=", &bullet_ruby::MultiBodyDynamicsWorld::set_gravity)
    .define_method("add_multi_body", &bullet_ruby::MultiBodyDynamicsWorld::add_multi_body_object,
      Rice::Arg("multi_body").setValue().keepAlive(),
      Rice::Arg("group") = int(btBroadphaseProxy::DefaultFilter),
      Rice::Arg("mask") = int(btBroadphaseProxy::AllFilter))
    .define_method("remove_multi_body", &bullet_ruby::MultiBodyDynamicsWorld::remove_multi_body_object,
      Rice::Arg("multi_body").setValue())
    .define_method("add_constraint", &bullet_ruby::MultiBodyDynamicsWorld::add_constraint_object,
      Rice::Arg("constraint").setValue().keepAlive())
    .define_method("remove_constraint", &bullet_ruby::MultiBodyDynamicsWorld::remove_constraint_object,
      Rice::Arg("constraint").setValue())
    .define_method("add_link_collider", &bullet_ruby::MultiBodyDynamicsWorld::add_link_collider_object,
      Rice::Arg("collider").setValue().keepAlive(),
      Rice::Arg("group") = int(btBroadphaseProxy::DefaultFilter),
      Rice::Arg("mask") = int(btBroadphaseProxy::AllFilter))
    .define_method("remove_link_collider", &bullet_ruby::MultiBodyDynamicsWorld::remove_link_collider_object,
      Rice::Arg("collider").setValue())
    .define_method("step_simulation", &bullet_ruby::MultiBodyDynamicsWorld::step_simulation,
      Rice::Arg("time_step"),
      Rice::Arg("max_sub_steps") = 1,
      Rice::Arg("fixed_time_step") = btScalar(1.0 / 60.0))
    .define_method("num_multi_bodies", &bullet_ruby::MultiBodyDynamicsWorld::num_multi_bodies)
    .define_method("num_multi_body_constraints", &bullet_ruby::MultiBodyDynamicsWorld::num_multi_body_constraints)
    .define_method("num_collision_objects", &bullet_ruby::MultiBodyDynamicsWorld::num_collision_objects)
    .define_method("forward_kinematics", &bullet_ruby::MultiBodyDynamicsWorld::forward_kinematics)
    .define_method("clear_forces", &bullet_ruby::MultiBodyDynamicsWorld::clear_forces)
    .define_method("clear_multi_body_forces", &bullet_ruby::MultiBodyDynamicsWorld::clear_multi_body_forces);
}
