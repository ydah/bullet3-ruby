#include "rb_dynamics.hpp"

#include <stdexcept>

#include <ruby/thread.h>

#include "../util/type_conversions.hpp"

namespace {
btCollisionShape* coerce_collision_shape(Rice::Object object)
{
  btCollisionShape* shape = Rice::detail::From_Ruby<btCollisionShape*>().convert(object);
  if (shape == nullptr) {
    throw std::invalid_argument("expected Bullet::Shapes::CollisionShape");
  }

  return shape;
}

btVector3 calculate_local_inertia(btCollisionShape& shape, btScalar mass)
{
  btVector3 inertia(0, 0, 0);
  if (mass != btScalar(0)) {
    shape.calculateLocalInertia(mass, inertia);
  }
  return inertia;
}

struct StepSimulationArgs {
  btDiscreteDynamicsWorld* world;
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
CollisionConfiguration::CollisionConfiguration()
  : configuration_(std::make_unique<btDefaultCollisionConfiguration>())
{
}

btDefaultCollisionConfiguration* CollisionConfiguration::get()
{
  return configuration_.get();
}

CollisionDispatcher::CollisionDispatcher(CollisionConfiguration& configuration)
  : dispatcher_(std::make_unique<btCollisionDispatcher>(configuration.get()))
{
}

btCollisionDispatcher* CollisionDispatcher::get()
{
  return dispatcher_.get();
}

DbvtBroadphase::DbvtBroadphase()
  : broadphase_(std::make_unique<btDbvtBroadphase>())
{
}

btDbvtBroadphase* DbvtBroadphase::get()
{
  return broadphase_.get();
}

SequentialImpulseConstraintSolver::SequentialImpulseConstraintSolver()
  : solver_(std::make_unique<btSequentialImpulseConstraintSolver>())
{
}

btSequentialImpulseConstraintSolver* SequentialImpulseConstraintSolver::get()
{
  return solver_.get();
}

MotionState::MotionState()
  : motion_state_(std::make_unique<btDefaultMotionState>(btTransform::getIdentity()))
{
}

MotionState::MotionState(VALUE start_transform)
  : motion_state_(std::make_unique<btDefaultMotionState>(coerce_transform(Rice::Object(start_transform))))
{
}

btMotionState* MotionState::get()
{
  return motion_state_.get();
}

Transform MotionState::world_transform() const
{
  btTransform transform;
  motion_state_->getWorldTransform(transform);
  return Transform(transform);
}

Transform MotionState::set_world_transform(Rice::Object transform)
{
  btTransform value = coerce_transform(transform);
  motion_state_->setWorldTransform(value);
  return Transform(value);
}

RigidBodyConstructionInfo::RigidBodyConstructionInfo(btScalar mass,
                                                     MotionState& motion_state,
                                                     VALUE collision_shape)
  : local_inertia_(calculate_local_inertia(*coerce_collision_shape(Rice::Object(collision_shape)), mass)),
    info_(std::make_unique<btRigidBody::btRigidBodyConstructionInfo>(
      mass,
      motion_state.get(),
      coerce_collision_shape(Rice::Object(collision_shape)),
      local_inertia_))
{
}

RigidBodyConstructionInfo::RigidBodyConstructionInfo(btScalar mass,
                                                     MotionState& motion_state,
                                                     VALUE collision_shape,
                                                     VALUE local_inertia)
  : local_inertia_(coerce_vector(Rice::Object(local_inertia))),
    info_(std::make_unique<btRigidBody::btRigidBodyConstructionInfo>(
      mass,
      motion_state.get(),
      coerce_collision_shape(Rice::Object(collision_shape)),
      local_inertia_))
{
}

btRigidBody::btRigidBodyConstructionInfo& RigidBodyConstructionInfo::get()
{
  return *info_;
}

btScalar RigidBodyConstructionInfo::mass() const
{
  return info_->m_mass;
}

btVector3 RigidBodyConstructionInfo::local_inertia() const
{
  return info_->m_localInertia;
}

btScalar RigidBodyConstructionInfo::friction() const
{
  return info_->m_friction;
}

btScalar RigidBodyConstructionInfo::set_friction(btScalar friction)
{
  info_->m_friction = friction;
  return friction;
}

btScalar RigidBodyConstructionInfo::restitution() const
{
  return info_->m_restitution;
}

btScalar RigidBodyConstructionInfo::set_restitution(btScalar restitution)
{
  info_->m_restitution = restitution;
  return restitution;
}

RigidBody::RigidBody(RigidBodyConstructionInfo& construction_info)
  : rigid_body_(std::make_unique<btRigidBody>(construction_info.get()))
{
}

btRigidBody* RigidBody::get()
{
  return rigid_body_.get();
}

const btRigidBody* RigidBody::get() const
{
  return rigid_body_.get();
}

btScalar RigidBody::mass() const
{
  btScalar inverse_mass = rigid_body_->getInvMass();
  return inverse_mass == btScalar(0) ? btScalar(0) : btScalar(1) / inverse_mass;
}

btCollisionShape* RigidBody::collision_shape() const
{
  return rigid_body_->getCollisionShape();
}

Transform RigidBody::world_transform() const
{
  btTransform transform;
  if (rigid_body_->getMotionState() != nullptr) {
    rigid_body_->getMotionState()->getWorldTransform(transform);
  } else {
    transform = rigid_body_->getWorldTransform();
  }
  return Transform(transform);
}

Transform RigidBody::set_world_transform(Rice::Object transform)
{
  btTransform value = coerce_transform(transform);
  rigid_body_->setWorldTransform(value);
  rigid_body_->setCenterOfMassTransform(value);
  if (rigid_body_->getMotionState() != nullptr) {
    rigid_body_->getMotionState()->setWorldTransform(value);
  }
  return Transform(value);
}

btVector3 RigidBody::linear_velocity() const
{
  return rigid_body_->getLinearVelocity();
}

btVector3 RigidBody::set_linear_velocity(Rice::Object velocity)
{
  btVector3 vector = coerce_vector(velocity);
  rigid_body_->setLinearVelocity(vector);
  return vector;
}

btVector3 RigidBody::angular_velocity() const
{
  return rigid_body_->getAngularVelocity();
}

btVector3 RigidBody::set_angular_velocity(Rice::Object velocity)
{
  btVector3 vector = coerce_vector(velocity);
  rigid_body_->setAngularVelocity(vector);
  return vector;
}

btVector3 RigidBody::total_force() const
{
  return rigid_body_->getTotalForce();
}

btVector3 RigidBody::total_torque() const
{
  return rigid_body_->getTotalTorque();
}

btScalar RigidBody::friction() const
{
  return rigid_body_->getFriction();
}

btScalar RigidBody::set_friction(btScalar friction)
{
  rigid_body_->setFriction(friction);
  return friction;
}

btScalar RigidBody::restitution() const
{
  return rigid_body_->getRestitution();
}

btScalar RigidBody::set_restitution(btScalar restitution)
{
  rigid_body_->setRestitution(restitution);
  return restitution;
}

Rice::Array RigidBody::damping() const
{
  Rice::Array array;
  array.push(rigid_body_->getLinearDamping());
  array.push(rigid_body_->getAngularDamping());
  return array;
}

Rice::Array RigidBody::set_damping(btScalar linear_damping, btScalar angular_damping)
{
  rigid_body_->setDamping(linear_damping, angular_damping);
  return damping();
}

void RigidBody::apply_central_force(Rice::Object force)
{
  rigid_body_->applyCentralForce(coerce_vector(force));
}

void RigidBody::apply_force(Rice::Object force, Rice::Object relative_position)
{
  rigid_body_->applyForce(coerce_vector(force), coerce_vector(relative_position));
}

void RigidBody::apply_torque(Rice::Object torque)
{
  rigid_body_->applyTorque(coerce_vector(torque));
}

void RigidBody::apply_central_impulse(Rice::Object impulse)
{
  rigid_body_->applyCentralImpulse(coerce_vector(impulse));
}

void RigidBody::apply_impulse(Rice::Object impulse, Rice::Object relative_position)
{
  rigid_body_->applyImpulse(coerce_vector(impulse), coerce_vector(relative_position));
}

void RigidBody::clear_forces()
{
  rigid_body_->clearForces();
}

void RigidBody::activate()
{
  rigid_body_->activate(true);
}

bool RigidBody::active() const
{
  return rigid_body_->isActive();
}

bool RigidBody::static_object() const
{
  return rigid_body_->isStaticObject();
}

bool RigidBody::kinematic_object() const
{
  return rigid_body_->isKinematicObject();
}

DiscreteDynamicsWorld::DiscreteDynamicsWorld()
{
  build_owned_world();
}

DiscreteDynamicsWorld::DiscreteDynamicsWorld(CollisionDispatcher& dispatcher,
                                             DbvtBroadphase& broadphase,
                                             SequentialImpulseConstraintSolver& solver,
                                             CollisionConfiguration& configuration)
  : world_(std::make_unique<btDiscreteDynamicsWorld>(
      dispatcher.get(),
      broadphase.get(),
      solver.get(),
      configuration.get()))
{
}

DiscreteDynamicsWorld::~DiscreteDynamicsWorld()
{
  if (world_ == nullptr) {
    return;
  }

  for (btRigidBody* rigid_body : rigid_bodies_) {
    world_->removeRigidBody(rigid_body);
  }
}

void DiscreteDynamicsWorld::build_owned_world()
{
  owned_configuration_ = std::make_unique<btDefaultCollisionConfiguration>();
  owned_dispatcher_ = std::make_unique<btCollisionDispatcher>(owned_configuration_.get());
  owned_broadphase_ = std::make_unique<btDbvtBroadphase>();
  owned_solver_ = std::make_unique<btSequentialImpulseConstraintSolver>();
  world_ = std::make_unique<btDiscreteDynamicsWorld>(
    owned_dispatcher_.get(),
    owned_broadphase_.get(),
    owned_solver_.get(),
    owned_configuration_.get());
}

btDiscreteDynamicsWorld* DiscreteDynamicsWorld::get()
{
  return world_.get();
}

btVector3 DiscreteDynamicsWorld::gravity() const
{
  return world_->getGravity();
}

btVector3 DiscreteDynamicsWorld::set_gravity(Rice::Object gravity)
{
  btVector3 vector = coerce_vector(gravity);
  world_->setGravity(vector);
  return vector;
}

void DiscreteDynamicsWorld::add_rigid_body(RigidBody& rigid_body)
{
  btRigidBody* body = rigid_body.get();
  if (rigid_bodies_.insert(body).second) {
    world_->addRigidBody(body);
  }
}

void DiscreteDynamicsWorld::remove_rigid_body(RigidBody& rigid_body)
{
  btRigidBody* body = rigid_body.get();
  auto iterator = rigid_bodies_.find(body);
  if (iterator != rigid_bodies_.end()) {
    world_->removeRigidBody(body);
    rigid_bodies_.erase(iterator);
  }
}

int DiscreteDynamicsWorld::step_simulation(btScalar time_step, int max_sub_steps, btScalar fixed_time_step)
{
  StepSimulationArgs args{world_.get(), time_step, max_sub_steps, fixed_time_step, 0};
  rb_thread_call_without_gvl(step_simulation_without_gvl, &args, nullptr, nullptr);
  return args.result;
}

int DiscreteDynamicsWorld::num_collision_objects() const
{
  return world_->getNumCollisionObjects();
}

void DiscreteDynamicsWorld::clear_forces()
{
  world_->clearForces();
}

void DiscreteDynamicsWorld::synchronize_motion_states()
{
  world_->synchronizeMotionStates();
}
} // namespace bullet_ruby

void Init_Dynamics(Rice::Module rb_mBullet)
{
  Rice::define_class_under<bullet_ruby::CollisionConfiguration>(rb_mBullet, "CollisionConfiguration")
    .define_constructor(Rice::Constructor<bullet_ruby::CollisionConfiguration>());

  Rice::define_class_under<bullet_ruby::CollisionDispatcher>(rb_mBullet, "CollisionDispatcher")
    .define_constructor(Rice::Constructor<bullet_ruby::CollisionDispatcher, bullet_ruby::CollisionConfiguration&>(),
      Rice::Arg("configuration").keepAlive());

  Rice::define_class_under<bullet_ruby::DbvtBroadphase>(rb_mBullet, "DbvtBroadphase")
    .define_constructor(Rice::Constructor<bullet_ruby::DbvtBroadphase>());

  Rice::define_class_under<bullet_ruby::SequentialImpulseConstraintSolver>(rb_mBullet, "SequentialImpulseConstraintSolver")
    .define_constructor(Rice::Constructor<bullet_ruby::SequentialImpulseConstraintSolver>());

  Rice::define_class_under<bullet_ruby::MotionState>(rb_mBullet, "MotionState")
    .define_constructor(Rice::Constructor<bullet_ruby::MotionState>())
    .define_constructor(Rice::Constructor<bullet_ruby::MotionState, VALUE>(),
      Rice::Arg("start_transform").setValue())
    .define_method("world_transform", &bullet_ruby::MotionState::world_transform)
    .define_method("world_transform=", &bullet_ruby::MotionState::set_world_transform);

  Rice::define_class_under<bullet_ruby::RigidBodyConstructionInfo>(rb_mBullet, "RigidBodyConstructionInfo")
    .define_constructor(Rice::Constructor<bullet_ruby::RigidBodyConstructionInfo,
        btScalar,
        bullet_ruby::MotionState&,
        VALUE>(),
      Rice::Arg("mass"),
      Rice::Arg("motion_state").keepAlive(),
      Rice::Arg("collision_shape").setValue().keepAlive())
    .define_constructor(Rice::Constructor<bullet_ruby::RigidBodyConstructionInfo,
        btScalar,
        bullet_ruby::MotionState&,
        VALUE,
        VALUE>(),
      Rice::Arg("mass"),
      Rice::Arg("motion_state").keepAlive(),
      Rice::Arg("collision_shape").setValue().keepAlive(),
      Rice::Arg("local_inertia").setValue())
    .define_method("mass", &bullet_ruby::RigidBodyConstructionInfo::mass)
    .define_method("local_inertia", &bullet_ruby::RigidBodyConstructionInfo::local_inertia)
    .define_method("friction", &bullet_ruby::RigidBodyConstructionInfo::friction)
    .define_method("friction=", &bullet_ruby::RigidBodyConstructionInfo::set_friction)
    .define_method("restitution", &bullet_ruby::RigidBodyConstructionInfo::restitution)
    .define_method("restitution=", &bullet_ruby::RigidBodyConstructionInfo::set_restitution);

  Rice::define_class_under<bullet_ruby::RigidBody>(rb_mBullet, "RigidBody")
    .define_constructor(Rice::Constructor<bullet_ruby::RigidBody, bullet_ruby::RigidBodyConstructionInfo&>(),
      Rice::Arg("construction_info").keepAlive())
    .define_method("mass", &bullet_ruby::RigidBody::mass)
    .define_method("collision_shape", &bullet_ruby::RigidBody::collision_shape)
    .define_method("world_transform", &bullet_ruby::RigidBody::world_transform)
    .define_method("world_transform=", &bullet_ruby::RigidBody::set_world_transform)
    .define_method("linear_velocity", &bullet_ruby::RigidBody::linear_velocity)
    .define_method("linear_velocity=", &bullet_ruby::RigidBody::set_linear_velocity)
    .define_method("angular_velocity", &bullet_ruby::RigidBody::angular_velocity)
    .define_method("angular_velocity=", &bullet_ruby::RigidBody::set_angular_velocity)
    .define_method("total_force", &bullet_ruby::RigidBody::total_force)
    .define_method("total_torque", &bullet_ruby::RigidBody::total_torque)
    .define_method("friction", &bullet_ruby::RigidBody::friction)
    .define_method("friction=", &bullet_ruby::RigidBody::set_friction)
    .define_method("restitution", &bullet_ruby::RigidBody::restitution)
    .define_method("restitution=", &bullet_ruby::RigidBody::set_restitution)
    .define_method("damping", &bullet_ruby::RigidBody::damping)
    .define_method("set_damping", &bullet_ruby::RigidBody::set_damping)
    .define_method("apply_central_force", &bullet_ruby::RigidBody::apply_central_force)
    .define_method("apply_force", &bullet_ruby::RigidBody::apply_force)
    .define_method("apply_torque", &bullet_ruby::RigidBody::apply_torque)
    .define_method("apply_central_impulse", &bullet_ruby::RigidBody::apply_central_impulse)
    .define_method("apply_impulse", &bullet_ruby::RigidBody::apply_impulse)
    .define_method("clear_forces", &bullet_ruby::RigidBody::clear_forces)
    .define_method("activate", &bullet_ruby::RigidBody::activate)
    .define_method("active?", &bullet_ruby::RigidBody::active)
    .define_method("static?", &bullet_ruby::RigidBody::static_object)
    .define_method("kinematic?", &bullet_ruby::RigidBody::kinematic_object);

  Rice::define_class_under<bullet_ruby::DiscreteDynamicsWorld>(rb_mBullet, "DiscreteDynamicsWorld")
    .define_constructor(Rice::Constructor<bullet_ruby::DiscreteDynamicsWorld>())
    .define_constructor(Rice::Constructor<bullet_ruby::DiscreteDynamicsWorld,
        bullet_ruby::CollisionDispatcher&,
        bullet_ruby::DbvtBroadphase&,
        bullet_ruby::SequentialImpulseConstraintSolver&,
        bullet_ruby::CollisionConfiguration&>(),
      Rice::Arg("dispatcher").keepAlive(),
      Rice::Arg("broadphase").keepAlive(),
      Rice::Arg("solver").keepAlive(),
      Rice::Arg("configuration").keepAlive())
    .define_singleton_method("create", [](Rice::Object) -> bullet_ruby::DiscreteDynamicsWorld* {
      return new bullet_ruby::DiscreteDynamicsWorld();
    }, Rice::Return().takeOwnership())
    .define_method("gravity", &bullet_ruby::DiscreteDynamicsWorld::gravity)
    .define_method("gravity=", &bullet_ruby::DiscreteDynamicsWorld::set_gravity)
    .define_method("add_rigid_body", &bullet_ruby::DiscreteDynamicsWorld::add_rigid_body,
      Rice::Arg("rigid_body").keepAlive())
    .define_method("remove_rigid_body", &bullet_ruby::DiscreteDynamicsWorld::remove_rigid_body)
    .define_method("step_simulation", &bullet_ruby::DiscreteDynamicsWorld::step_simulation,
      Rice::Arg("time_step"),
      Rice::Arg("max_sub_steps") = 1,
      Rice::Arg("fixed_time_step") = btScalar(1.0 / 60.0))
    .define_method("num_collision_objects", &bullet_ruby::DiscreteDynamicsWorld::num_collision_objects)
    .define_method("clear_forces", &bullet_ruby::DiscreteDynamicsWorld::clear_forces)
    .define_method("synchronize_motion_states", &bullet_ruby::DiscreteDynamicsWorld::synchronize_motion_states);
}
