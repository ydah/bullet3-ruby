#include "rb_dynamics.hpp"

#include <stdexcept>

#include <ruby/thread.h>

#include "../constraints/rb_constraints.hpp"
#include "../util/type_conversions.hpp"
#include "../vehicle/rb_vehicle.hpp"

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

void set_vector_hash_value(Rice::Hash& hash, const char* key, const btVector3& value)
{
  hash[Rice::Symbol(key)] = Rice::Object(Rice::detail::To_Ruby<btVector3>().convert(btVector3(value)));
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
  actions_.clear();
  constraints_.clear();
  rigid_bodies_.clear();
  action_values_.clear();
  constraint_values_.clear();
  collision_object_values_.clear();
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

void DiscreteDynamicsWorld::add_rigid_body_object(VALUE rigid_body)
{
  RigidBody* body = Rice::detail::From_Ruby<RigidBody*>().convert(rigid_body);
  add_rigid_body(*body);
  collision_object_values_[body->get()] = rigid_body;
}

void DiscreteDynamicsWorld::remove_rigid_body(RigidBody& rigid_body)
{
  btRigidBody* body = rigid_body.get();
  auto iterator = rigid_bodies_.find(body);
  if (iterator != rigid_bodies_.end()) {
    world_->removeRigidBody(body);
    rigid_bodies_.erase(iterator);
  }
  collision_object_values_.erase(body);
}

void DiscreteDynamicsWorld::remove_rigid_body_object(VALUE rigid_body)
{
  RigidBody* body = Rice::detail::From_Ruby<RigidBody*>().convert(rigid_body);
  remove_rigid_body(*body);
}

void DiscreteDynamicsWorld::add_constraint_object(VALUE constraint, bool disable_collisions_between_linked_bodies)
{
  TypedConstraint* typed_constraint = Rice::detail::From_Ruby<TypedConstraint*>().convert(constraint);
  if (typed_constraint == nullptr) {
    throw std::invalid_argument("expected Bullet::Constraints::TypedConstraint");
  }

  btTypedConstraint* bullet_constraint = typed_constraint->get();
  if (constraints_.insert(bullet_constraint).second) {
    world_->addConstraint(bullet_constraint, disable_collisions_between_linked_bodies);
    constraint_values_[bullet_constraint] = constraint;
  }
}

void DiscreteDynamicsWorld::remove_constraint_object(VALUE constraint)
{
  TypedConstraint* typed_constraint = Rice::detail::From_Ruby<TypedConstraint*>().convert(constraint);
  if (typed_constraint == nullptr) {
    throw std::invalid_argument("expected Bullet::Constraints::TypedConstraint");
  }

  btTypedConstraint* bullet_constraint = typed_constraint->get();
  auto iterator = constraints_.find(bullet_constraint);
  if (iterator != constraints_.end()) {
    world_->removeConstraint(bullet_constraint);
    constraints_.erase(iterator);
  }
  constraint_values_.erase(bullet_constraint);
}

void DiscreteDynamicsWorld::add_vehicle_object(VALUE vehicle)
{
  RaycastVehicle* raycast_vehicle = Rice::detail::From_Ruby<RaycastVehicle*>().convert(vehicle);
  if (raycast_vehicle == nullptr) {
    throw std::invalid_argument("expected Bullet::RaycastVehicle");
  }

  btRaycastVehicle* bullet_vehicle = raycast_vehicle->get();
  if (actions_.insert(bullet_vehicle).second) {
    world_->addAction(bullet_vehicle);
    action_values_[bullet_vehicle] = vehicle;
  }
}

void DiscreteDynamicsWorld::remove_vehicle_object(VALUE vehicle)
{
  RaycastVehicle* raycast_vehicle = Rice::detail::From_Ruby<RaycastVehicle*>().convert(vehicle);
  if (raycast_vehicle == nullptr) {
    throw std::invalid_argument("expected Bullet::RaycastVehicle");
  }

  btRaycastVehicle* bullet_vehicle = raycast_vehicle->get();
  auto iterator = actions_.find(bullet_vehicle);
  if (iterator != actions_.end()) {
    world_->removeAction(bullet_vehicle);
    actions_.erase(iterator);
  }
  action_values_.erase(bullet_vehicle);
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

int DiscreteDynamicsWorld::num_constraints() const
{
  return world_->getNumConstraints();
}

void DiscreteDynamicsWorld::clear_forces()
{
  world_->clearForces();
}

void DiscreteDynamicsWorld::synchronize_motion_states()
{
  world_->synchronizeMotionStates();
}

VALUE DiscreteDynamicsWorld::ruby_object_for(const btCollisionObject* collision_object) const
{
  auto iterator = collision_object_values_.find(collision_object);
  if (iterator == collision_object_values_.end()) {
    return Qnil;
  }
  return iterator->second;
}

Rice::Object DiscreteDynamicsWorld::ray_test_closest(Rice::Object from, Rice::Object to) const
{
  btVector3 ray_from = coerce_vector(from);
  btVector3 ray_to = coerce_vector(to);
  btCollisionWorld::ClosestRayResultCallback callback(ray_from, ray_to);
  world_->rayTest(ray_from, ray_to, callback);

  if (!callback.hasHit()) {
    return Rice::Object(Qnil);
  }

  Rice::Hash hit;
  hit[Rice::Symbol("body")] = Rice::Object(ruby_object_for(callback.m_collisionObject));
  hit[Rice::Symbol("fraction")] = callback.m_closestHitFraction;
  set_vector_hash_value(hit, "point", callback.m_hitPointWorld);
  set_vector_hash_value(hit, "normal", callback.m_hitNormalWorld);
  return hit;
}

Rice::Array DiscreteDynamicsWorld::ray_test_all(Rice::Object from, Rice::Object to) const
{
  btVector3 ray_from = coerce_vector(from);
  btVector3 ray_to = coerce_vector(to);
  btCollisionWorld::AllHitsRayResultCallback callback(ray_from, ray_to);
  world_->rayTest(ray_from, ray_to, callback);

  Rice::Array hits;
  for (int index = 0; index < callback.m_collisionObjects.size(); ++index) {
    Rice::Hash hit;
    hit[Rice::Symbol("body")] = Rice::Object(ruby_object_for(callback.m_collisionObjects[index]));
    hit[Rice::Symbol("fraction")] = callback.m_hitFractions[index];
    set_vector_hash_value(hit, "point", callback.m_hitPointWorld[index]);
    set_vector_hash_value(hit, "normal", callback.m_hitNormalWorld[index]);
    hits.push(hit);
  }
  return hits;
}

Rice::Array DiscreteDynamicsWorld::contact_manifolds() const
{
  Rice::Array manifolds;
  btDispatcher* dispatcher = world_->getDispatcher();
  int manifold_count = dispatcher->getNumManifolds();

  for (int manifold_index = 0; manifold_index < manifold_count; ++manifold_index) {
    btPersistentManifold* manifold = dispatcher->getManifoldByIndexInternal(manifold_index);
    Rice::Hash manifold_hash;
    manifold_hash[Rice::Symbol("body0")] = Rice::Object(ruby_object_for(static_cast<const btCollisionObject*>(manifold->getBody0())));
    manifold_hash[Rice::Symbol("body1")] = Rice::Object(ruby_object_for(static_cast<const btCollisionObject*>(manifold->getBody1())));

    Rice::Array points;
    for (int point_index = 0; point_index < manifold->getNumContacts(); ++point_index) {
      const btManifoldPoint& point = manifold->getContactPoint(point_index);
      Rice::Hash point_hash;
      set_vector_hash_value(point_hash, "position_world_on_a", point.getPositionWorldOnA());
      set_vector_hash_value(point_hash, "position_world_on_b", point.getPositionWorldOnB());
      set_vector_hash_value(point_hash, "normal_world_on_b", point.m_normalWorldOnB);
      point_hash[Rice::Symbol("distance")] = point.getDistance();
      point_hash[Rice::Symbol("life_time")] = point.getLifeTime();
      points.push(point_hash);
    }

    manifold_hash[Rice::Symbol("points")] = points;
    manifolds.push(manifold_hash);
  }

  return manifolds;
}

void DiscreteDynamicsWorld::mark() const
{
  for (const auto& entry : collision_object_values_) {
    rb_gc_mark(entry.second);
  }
  for (const auto& entry : constraint_values_) {
    rb_gc_mark(entry.second);
  }
  for (const auto& entry : action_values_) {
    rb_gc_mark(entry.second);
  }
}
} // namespace bullet_ruby

namespace Rice {
template <>
void ruby_mark<bullet_ruby::DiscreteDynamicsWorld>(bullet_ruby::DiscreteDynamicsWorld* world)
{
  if (world != nullptr) {
    world->mark();
  }
}
} // namespace Rice

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
    .define_method("add_rigid_body", &bullet_ruby::DiscreteDynamicsWorld::add_rigid_body_object,
      Rice::Arg("rigid_body").setValue().keepAlive())
    .define_method("remove_rigid_body", &bullet_ruby::DiscreteDynamicsWorld::remove_rigid_body_object,
      Rice::Arg("rigid_body").setValue())
    .define_method("add_constraint", &bullet_ruby::DiscreteDynamicsWorld::add_constraint_object,
      Rice::Arg("constraint").setValue().keepAlive(),
      Rice::Arg("disable_collisions_between_linked_bodies") = false)
    .define_method("remove_constraint", &bullet_ruby::DiscreteDynamicsWorld::remove_constraint_object,
      Rice::Arg("constraint").setValue())
    .define_method("add_vehicle", &bullet_ruby::DiscreteDynamicsWorld::add_vehicle_object,
      Rice::Arg("vehicle").setValue().keepAlive())
    .define_method("remove_vehicle", &bullet_ruby::DiscreteDynamicsWorld::remove_vehicle_object,
      Rice::Arg("vehicle").setValue())
    .define_method("step_simulation", &bullet_ruby::DiscreteDynamicsWorld::step_simulation,
      Rice::Arg("time_step"),
      Rice::Arg("max_sub_steps") = 1,
      Rice::Arg("fixed_time_step") = btScalar(1.0 / 60.0))
    .define_method("num_collision_objects", &bullet_ruby::DiscreteDynamicsWorld::num_collision_objects)
    .define_method("num_constraints", &bullet_ruby::DiscreteDynamicsWorld::num_constraints)
    .define_method("clear_forces", &bullet_ruby::DiscreteDynamicsWorld::clear_forces)
    .define_method("synchronize_motion_states", &bullet_ruby::DiscreteDynamicsWorld::synchronize_motion_states)
    .define_method("ray_test_closest", &bullet_ruby::DiscreteDynamicsWorld::ray_test_closest)
    .define_method("ray_test", &bullet_ruby::DiscreteDynamicsWorld::ray_test_closest)
    .define_method("ray_test_all", &bullet_ruby::DiscreteDynamicsWorld::ray_test_all)
    .define_method("contact_manifolds", &bullet_ruby::DiscreteDynamicsWorld::contact_manifolds);
}
