#include "rb_soft_body.hpp"

#include <stdexcept>
#include <vector>

#include <ruby/thread.h>

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

bullet_ruby::SoftBody* soft_body_from_value(VALUE value)
{
  bullet_ruby::SoftBody* body = Rice::detail::From_Ruby<bullet_ruby::SoftBody*>().convert(value);
  if (body == nullptr) {
    throw std::invalid_argument("expected Bullet::SoftBody::SoftBody");
  }
  return body;
}

Rice::Object vector_object(const btVector3& vector)
{
  return Rice::Object(Rice::detail::To_Ruby<btVector3>().convert(btVector3(vector)));
}

Rice::Symbol soft_body_feature_symbol(btSoftBody::eFeature::_ feature)
{
  switch (feature) {
    case btSoftBody::eFeature::Node:
      return Rice::Symbol("node");
    case btSoftBody::eFeature::Link:
      return Rice::Symbol("link");
    case btSoftBody::eFeature::Face:
      return Rice::Symbol("face");
    case btSoftBody::eFeature::Tetra:
      return Rice::Symbol("tetra");
    case btSoftBody::eFeature::None:
    default:
      return Rice::Symbol("none");
  }
}

std::vector<btVector3> vector_array_from_object(Rice::Object object)
{
  Rice::Array array = bullet_ruby::array_from_object(object);
  std::vector<btVector3> vectors;
  vectors.reserve(array.size());
  for (long index = 0; index < array.size(); ++index) {
    vectors.push_back(bullet_ruby::coerce_vector(Rice::Object(array[index])));
  }
  return vectors;
}

std::vector<btScalar> flat_vertices_from_object(Rice::Object object)
{
  std::vector<btVector3> vectors = vector_array_from_object(object);
  std::vector<btScalar> flat_vertices;
  flat_vertices.reserve(vectors.size() * 3);
  for (const btVector3& vector : vectors) {
    flat_vertices.push_back(vector.getX());
    flat_vertices.push_back(vector.getY());
    flat_vertices.push_back(vector.getZ());
  }
  return flat_vertices;
}

std::vector<int> triangle_indices_from_object(Rice::Object object)
{
  Rice::Array array = bullet_ruby::array_from_object(object);
  std::vector<int> indices;
  indices.reserve(array.size() * 3);
  for (long index = 0; index < array.size(); ++index) {
    Rice::Object item(array[index]);
    if (rb_obj_is_kind_of(item.value(), rb_cArray) || bullet_ruby::object_responds_to(item, "to_a")) {
      Rice::Array triangle = bullet_ruby::array_from_object(item);
      if (triangle.size() != 3) {
        throw std::invalid_argument("triangle indices must have 3 elements");
      }
      indices.push_back(Rice::detail::From_Ruby<int>().convert(triangle[0]));
      indices.push_back(Rice::detail::From_Ruby<int>().convert(triangle[1]));
      indices.push_back(Rice::detail::From_Ruby<int>().convert(triangle[2]));
    } else {
      indices.push_back(Rice::detail::From_Ruby<int>().convert(item));
    }
  }

  if (indices.size() % 3 != 0) {
    throw std::invalid_argument("triangle index count must be a multiple of 3");
  }
  return indices;
}

struct StepSimulationArgs {
  btSoftRigidDynamicsWorld* world;
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
SoftBodyWorldInfo::SoftBodyWorldInfo()
  : owned_world_info_(std::make_unique<btSoftBodyWorldInfo>()),
    world_info_(owned_world_info_.get())
{
  world_info_->m_sparsesdf.Initialize();
}

SoftBodyWorldInfo::SoftBodyWorldInfo(btSoftBodyWorldInfo* world_info)
  : world_info_(world_info)
{
  if (world_info_ == nullptr) {
    throw std::invalid_argument("expected btSoftBodyWorldInfo");
  }
}

btSoftBodyWorldInfo& SoftBodyWorldInfo::get()
{
  return *world_info_;
}

const btSoftBodyWorldInfo& SoftBodyWorldInfo::get() const
{
  return *world_info_;
}

btVector3 SoftBodyWorldInfo::gravity() const
{
  return world_info_->m_gravity;
}

btVector3 SoftBodyWorldInfo::set_gravity(Rice::Object gravity)
{
  btVector3 vector = coerce_vector(gravity);
  world_info_->m_gravity = vector;
  return vector;
}

btScalar SoftBodyWorldInfo::air_density() const
{
  return world_info_->air_density;
}

btScalar SoftBodyWorldInfo::set_air_density(btScalar value)
{
  world_info_->air_density = value;
  return value;
}

btScalar SoftBodyWorldInfo::water_density() const
{
  return world_info_->water_density;
}

btScalar SoftBodyWorldInfo::set_water_density(btScalar value)
{
  world_info_->water_density = value;
  return value;
}

btScalar SoftBodyWorldInfo::water_offset() const
{
  return world_info_->water_offset;
}

btScalar SoftBodyWorldInfo::set_water_offset(btScalar value)
{
  world_info_->water_offset = value;
  return value;
}

btVector3 SoftBodyWorldInfo::water_normal() const
{
  return world_info_->water_normal;
}

btVector3 SoftBodyWorldInfo::set_water_normal(Rice::Object normal)
{
  btVector3 vector = coerce_vector(normal);
  world_info_->water_normal = vector;
  return vector;
}

btScalar SoftBodyWorldInfo::max_displacement() const
{
  return world_info_->m_maxDisplacement;
}

btScalar SoftBodyWorldInfo::set_max_displacement(btScalar value)
{
  world_info_->m_maxDisplacement = value;
  return value;
}

void SoftBodyWorldInfo::reset_sparse_sdf()
{
  world_info_->m_sparsesdf.Reset();
}

SoftBody::SoftBody(btSoftBody* soft_body)
  : soft_body_(soft_body)
{
  if (soft_body_ == nullptr) {
    throw std::invalid_argument("expected btSoftBody");
  }
}

btSoftBody* SoftBody::get()
{
  return soft_body_.get();
}

const btSoftBody* SoftBody::get() const
{
  return soft_body_.get();
}

int SoftBody::validate_node_index(int index) const
{
  if (index < 0 || index >= soft_body_->m_nodes.size()) {
    throw std::out_of_range("node index is out of range");
  }
  return index;
}

int SoftBody::node_count() const
{
  return soft_body_->m_nodes.size();
}

int SoftBody::link_count() const
{
  return soft_body_->m_links.size();
}

int SoftBody::face_count() const
{
  return soft_body_->m_faces.size();
}

int SoftBody::tetra_count() const
{
  return soft_body_->m_tetras.size();
}

int SoftBody::cluster_count() const
{
  return soft_body_->clusterCount();
}

btScalar SoftBody::total_mass() const
{
  return soft_body_->getTotalMass();
}

void SoftBody::set_total_mass(btScalar mass, bool from_faces)
{
  soft_body_->setTotalMass(mass, from_faces);
}

btScalar SoftBody::node_mass(int index) const
{
  return soft_body_->getMass(validate_node_index(index));
}

btScalar SoftBody::set_node_mass(int index, btScalar mass)
{
  soft_body_->setMass(validate_node_index(index), mass);
  return mass;
}

btVector3 SoftBody::node_position(int index) const
{
  return soft_body_->m_nodes[validate_node_index(index)].m_x;
}

btVector3 SoftBody::node_velocity(int index) const
{
  return soft_body_->m_nodes[validate_node_index(index)].m_v;
}

btVector3 SoftBody::center_of_mass() const
{
  return soft_body_->getCenterOfMass();
}

btVector3 SoftBody::linear_velocity()
{
  return soft_body_->getLinearVelocity();
}

void SoftBody::set_linear_velocity(Rice::Object velocity)
{
  soft_body_->setLinearVelocity(coerce_vector(velocity));
}

void SoftBody::set_velocity(Rice::Object velocity)
{
  soft_body_->setVelocity(coerce_vector(velocity));
}

void SoftBody::set_angular_velocity(Rice::Object velocity)
{
  soft_body_->setAngularVelocity(coerce_vector(velocity));
}

void SoftBody::add_force(Rice::Object force)
{
  soft_body_->addForce(coerce_vector(force));
}

void SoftBody::add_velocity(Rice::Object velocity)
{
  soft_body_->addVelocity(coerce_vector(velocity));
}

void SoftBody::translate(Rice::Object translation)
{
  soft_body_->translate(coerce_vector(translation));
}

void SoftBody::rotate(Rice::Object rotation)
{
  soft_body_->rotate(coerce_quaternion(rotation));
}

void SoftBody::scale(Rice::Object scaling)
{
  soft_body_->scale(coerce_vector(scaling));
}

void SoftBody::transform(Rice::Object transform)
{
  soft_body_->transform(coerce_transform(transform));
}

void SoftBody::set_pose(bool volume, bool frame)
{
  soft_body_->setPose(volume, frame);
}

int SoftBody::generate_bending_constraints(int distance)
{
  return soft_body_->generateBendingConstraints(distance);
}

int SoftBody::generate_clusters(int count)
{
  return soft_body_->generateClusters(count);
}

void SoftBody::randomize_constraints()
{
  soft_body_->randomizeConstraints();
}

btVector3 SoftBody::wind_velocity()
{
  return soft_body_->getWindVelocity();
}

btVector3 SoftBody::set_wind_velocity(Rice::Object velocity)
{
  btVector3 vector = coerce_vector(velocity);
  soft_body_->setWindVelocity(vector);
  return vector;
}

void SoftBody::set_zero_velocity()
{
  soft_body_->setZeroVelocity();
}

bool SoftBody::self_collision()
{
  return soft_body_->useSelfCollision();
}

bool SoftBody::set_self_collision(bool enabled)
{
  soft_body_->setSelfCollision(enabled);
  return enabled;
}

Rice::Array SoftBody::aabb() const
{
  btVector3 aabb_min;
  btVector3 aabb_max;
  soft_body_->getAabb(aabb_min, aabb_max);
  Rice::Array array;
  array.push(vector_object(aabb_min));
  array.push(vector_object(aabb_max));
  return array;
}

Rice::Object SoftBody::ray_test(Rice::Object from, Rice::Object to)
{
  btSoftBody::sRayCast result;
  if (!soft_body_->rayTest(coerce_vector(from), coerce_vector(to), result)) {
    return Rice::Object(Qnil);
  }

  Rice::Hash hash;
  hash[Rice::Symbol("feature")] = soft_body_feature_symbol(result.feature);
  hash[Rice::Symbol("index")] = result.index;
  hash[Rice::Symbol("fraction")] = result.fraction;
  return hash;
}

void SoftBody::append_anchor(int node, RigidBody& rigid_body, bool disable_collision_between_linked_bodies, btScalar influence)
{
  soft_body_->appendAnchor(validate_node_index(node), rigid_body.get(), disable_collision_between_linked_bodies, influence);
}

SoftRigidDynamicsWorld::SoftRigidDynamicsWorld()
  : configuration_(std::make_unique<btSoftBodyRigidBodyCollisionConfiguration>()),
    dispatcher_(std::make_unique<btCollisionDispatcher>(configuration_.get())),
    broadphase_(std::make_unique<btDbvtBroadphase>()),
    solver_(std::make_unique<btSequentialImpulseConstraintSolver>()),
    world_(std::make_unique<btSoftRigidDynamicsWorld>(
      dispatcher_.get(),
      broadphase_.get(),
      solver_.get(),
      configuration_.get()))
{
}

SoftRigidDynamicsWorld::~SoftRigidDynamicsWorld()
{
  soft_bodies_.clear();
  rigid_bodies_.clear();
  soft_body_values_.clear();
  collision_object_values_.clear();
}

btSoftRigidDynamicsWorld* SoftRigidDynamicsWorld::get()
{
  return world_.get();
}

btVector3 SoftRigidDynamicsWorld::gravity() const
{
  return world_->getGravity();
}

btVector3 SoftRigidDynamicsWorld::set_gravity(Rice::Object gravity)
{
  btVector3 vector = coerce_vector(gravity);
  world_->setGravity(vector);
  world_->getWorldInfo().m_gravity = vector;
  return vector;
}

SoftBodyWorldInfo* SoftRigidDynamicsWorld::world_info()
{
  return new SoftBodyWorldInfo(&world_->getWorldInfo());
}

void SoftRigidDynamicsWorld::add_rigid_body_object(VALUE rigid_body)
{
  RigidBody* body = rigid_body_from_value(rigid_body);
  btRigidBody* bullet_body = body->get();
  if (rigid_bodies_.insert(bullet_body).second) {
    world_->addRigidBody(bullet_body);
    collision_object_values_[bullet_body] = rigid_body;
  }
}

void SoftRigidDynamicsWorld::remove_rigid_body_object(VALUE rigid_body)
{
  RigidBody* body = rigid_body_from_value(rigid_body);
  btRigidBody* bullet_body = body->get();
  auto iterator = rigid_bodies_.find(bullet_body);
  if (iterator != rigid_bodies_.end()) {
    world_->removeRigidBody(bullet_body);
    rigid_bodies_.erase(iterator);
  }
  collision_object_values_.erase(bullet_body);
}

void SoftRigidDynamicsWorld::add_soft_body_object(VALUE soft_body, int collision_filter_group, int collision_filter_mask)
{
  SoftBody* body = soft_body_from_value(soft_body);
  btSoftBody* bullet_body = body->get();
  if (soft_bodies_.insert(bullet_body).second) {
    world_->addSoftBody(bullet_body, collision_filter_group, collision_filter_mask);
    soft_body_values_[bullet_body] = soft_body;
  }
}

void SoftRigidDynamicsWorld::remove_soft_body_object(VALUE soft_body)
{
  SoftBody* body = soft_body_from_value(soft_body);
  btSoftBody* bullet_body = body->get();
  auto iterator = soft_bodies_.find(bullet_body);
  if (iterator != soft_bodies_.end()) {
    world_->removeSoftBody(bullet_body);
    soft_bodies_.erase(iterator);
  }
  soft_body_values_.erase(bullet_body);
}

int SoftRigidDynamicsWorld::step_simulation(btScalar time_step, int max_sub_steps, btScalar fixed_time_step)
{
  StepSimulationArgs args{world_.get(), time_step, max_sub_steps, fixed_time_step, 0};
  rb_thread_call_without_gvl(step_simulation_without_gvl, &args, nullptr, nullptr);
  return args.result;
}

int SoftRigidDynamicsWorld::num_collision_objects() const
{
  return world_->getNumCollisionObjects();
}

int SoftRigidDynamicsWorld::num_soft_bodies() const
{
  return world_->getSoftBodyArray().size();
}

int SoftRigidDynamicsWorld::draw_flags() const
{
  return world_->getDrawFlags();
}

int SoftRigidDynamicsWorld::set_draw_flags(int flags)
{
  world_->setDrawFlags(flags);
  return flags;
}

void SoftRigidDynamicsWorld::clear_forces()
{
  world_->clearForces();
}

void SoftRigidDynamicsWorld::mark() const
{
  for (const auto& entry : collision_object_values_) {
    rb_gc_mark(entry.second);
  }
  for (const auto& entry : soft_body_values_) {
    rb_gc_mark(entry.second);
  }
}
} // namespace bullet_ruby

namespace Rice {
template <>
void ruby_mark<bullet_ruby::SoftRigidDynamicsWorld>(bullet_ruby::SoftRigidDynamicsWorld* world)
{
  if (world != nullptr) {
    world->mark();
  }
}
} // namespace Rice

void Init_SoftBody(Rice::Module rb_mBullet)
{
  Rice::Module rb_mSoftBody = Rice::define_module_under(rb_mBullet, "SoftBody");

  Rice::define_class_under<bullet_ruby::SoftBodyWorldInfo>(rb_mSoftBody, "SoftBodyWorldInfo")
    .define_constructor(Rice::Constructor<bullet_ruby::SoftBodyWorldInfo>())
    .define_method("gravity", &bullet_ruby::SoftBodyWorldInfo::gravity)
    .define_method("gravity=", &bullet_ruby::SoftBodyWorldInfo::set_gravity)
    .define_method("air_density", &bullet_ruby::SoftBodyWorldInfo::air_density)
    .define_method("air_density=", &bullet_ruby::SoftBodyWorldInfo::set_air_density)
    .define_method("water_density", &bullet_ruby::SoftBodyWorldInfo::water_density)
    .define_method("water_density=", &bullet_ruby::SoftBodyWorldInfo::set_water_density)
    .define_method("water_offset", &bullet_ruby::SoftBodyWorldInfo::water_offset)
    .define_method("water_offset=", &bullet_ruby::SoftBodyWorldInfo::set_water_offset)
    .define_method("water_normal", &bullet_ruby::SoftBodyWorldInfo::water_normal)
    .define_method("water_normal=", &bullet_ruby::SoftBodyWorldInfo::set_water_normal)
    .define_method("max_displacement", &bullet_ruby::SoftBodyWorldInfo::max_displacement)
    .define_method("max_displacement=", &bullet_ruby::SoftBodyWorldInfo::set_max_displacement)
    .define_method("reset_sparse_sdf", &bullet_ruby::SoftBodyWorldInfo::reset_sparse_sdf);

  Rice::define_class_under<bullet_ruby::SoftBody>(rb_mSoftBody, "SoftBody")
    .define_method("node_count", &bullet_ruby::SoftBody::node_count)
    .define_method("link_count", &bullet_ruby::SoftBody::link_count)
    .define_method("face_count", &bullet_ruby::SoftBody::face_count)
    .define_method("tetra_count", &bullet_ruby::SoftBody::tetra_count)
    .define_method("cluster_count", &bullet_ruby::SoftBody::cluster_count)
    .define_method("total_mass", &bullet_ruby::SoftBody::total_mass)
    .define_method("set_total_mass", &bullet_ruby::SoftBody::set_total_mass,
      Rice::Arg("mass"),
      Rice::Arg("from_faces") = false)
    .define_method("node_mass", &bullet_ruby::SoftBody::node_mass)
    .define_method("set_node_mass", &bullet_ruby::SoftBody::set_node_mass)
    .define_method("node_position", &bullet_ruby::SoftBody::node_position)
    .define_method("node_velocity", &bullet_ruby::SoftBody::node_velocity)
    .define_method("center_of_mass", &bullet_ruby::SoftBody::center_of_mass)
    .define_method("linear_velocity", &bullet_ruby::SoftBody::linear_velocity)
    .define_method("linear_velocity=", &bullet_ruby::SoftBody::set_linear_velocity)
    .define_method("set_velocity", &bullet_ruby::SoftBody::set_velocity)
    .define_method("set_angular_velocity", &bullet_ruby::SoftBody::set_angular_velocity)
    .define_method("add_force", &bullet_ruby::SoftBody::add_force)
    .define_method("add_velocity", &bullet_ruby::SoftBody::add_velocity)
    .define_method("translate", &bullet_ruby::SoftBody::translate)
    .define_method("rotate", &bullet_ruby::SoftBody::rotate)
    .define_method("scale", &bullet_ruby::SoftBody::scale)
    .define_method("transform", &bullet_ruby::SoftBody::transform)
    .define_method("set_pose", &bullet_ruby::SoftBody::set_pose)
    .define_method("generate_bending_constraints", &bullet_ruby::SoftBody::generate_bending_constraints)
    .define_method("generate_clusters", &bullet_ruby::SoftBody::generate_clusters)
    .define_method("randomize_constraints", &bullet_ruby::SoftBody::randomize_constraints)
    .define_method("wind_velocity", &bullet_ruby::SoftBody::wind_velocity)
    .define_method("wind_velocity=", &bullet_ruby::SoftBody::set_wind_velocity)
    .define_method("set_zero_velocity", &bullet_ruby::SoftBody::set_zero_velocity)
    .define_method("self_collision?", &bullet_ruby::SoftBody::self_collision)
    .define_method("self_collision=", &bullet_ruby::SoftBody::set_self_collision)
    .define_method("aabb", &bullet_ruby::SoftBody::aabb)
    .define_method("ray_test", &bullet_ruby::SoftBody::ray_test)
    .define_method("append_anchor", &bullet_ruby::SoftBody::append_anchor,
      Rice::Arg("node"),
      Rice::Arg("rigid_body").keepAlive(),
      Rice::Arg("disable_collision_between_linked_bodies") = false,
      Rice::Arg("influence") = btScalar(1.0));

  Rice::define_class_under<bullet_ruby::SoftRigidDynamicsWorld>(rb_mSoftBody, "SoftRigidDynamicsWorld")
    .define_constructor(Rice::Constructor<bullet_ruby::SoftRigidDynamicsWorld>())
    .define_singleton_method("create", [](Rice::Object) -> bullet_ruby::SoftRigidDynamicsWorld* {
      return new bullet_ruby::SoftRigidDynamicsWorld();
    }, Rice::Return().takeOwnership())
    .define_method("gravity", &bullet_ruby::SoftRigidDynamicsWorld::gravity)
    .define_method("gravity=", &bullet_ruby::SoftRigidDynamicsWorld::set_gravity)
    .define_method("world_info", &bullet_ruby::SoftRigidDynamicsWorld::world_info, Rice::Return().takeOwnership())
    .define_method("add_rigid_body", &bullet_ruby::SoftRigidDynamicsWorld::add_rigid_body_object,
      Rice::Arg("rigid_body").setValue().keepAlive())
    .define_method("remove_rigid_body", &bullet_ruby::SoftRigidDynamicsWorld::remove_rigid_body_object,
      Rice::Arg("rigid_body").setValue())
    .define_method("add_soft_body", &bullet_ruby::SoftRigidDynamicsWorld::add_soft_body_object,
      Rice::Arg("soft_body").setValue().keepAlive(),
      Rice::Arg("collision_filter_group") = int(btBroadphaseProxy::DefaultFilter),
      Rice::Arg("collision_filter_mask") = int(btBroadphaseProxy::AllFilter))
    .define_method("remove_soft_body", &bullet_ruby::SoftRigidDynamicsWorld::remove_soft_body_object,
      Rice::Arg("soft_body").setValue())
    .define_method("step_simulation", &bullet_ruby::SoftRigidDynamicsWorld::step_simulation,
      Rice::Arg("time_step"),
      Rice::Arg("max_sub_steps") = 1,
      Rice::Arg("fixed_time_step") = btScalar(1.0 / 60.0))
    .define_method("num_collision_objects", &bullet_ruby::SoftRigidDynamicsWorld::num_collision_objects)
    .define_method("num_soft_bodies", &bullet_ruby::SoftRigidDynamicsWorld::num_soft_bodies)
    .define_method("draw_flags", &bullet_ruby::SoftRigidDynamicsWorld::draw_flags)
    .define_method("draw_flags=", &bullet_ruby::SoftRigidDynamicsWorld::set_draw_flags)
    .define_method("clear_forces", &bullet_ruby::SoftRigidDynamicsWorld::clear_forces);

  Rice::Module rb_mHelpers = Rice::define_module_under(rb_mSoftBody, "Helpers");
  rb_mHelpers.define_singleton_method("create_rope", [](Rice::Object, bullet_ruby::SoftBodyWorldInfo& world_info, Rice::Object from, Rice::Object to, int resolution, int fixed_nodes) {
    return new bullet_ruby::SoftBody(btSoftBodyHelpers::CreateRope(
      world_info.get(),
      bullet_ruby::coerce_vector(from),
      bullet_ruby::coerce_vector(to),
      resolution,
      fixed_nodes));
  }, Rice::Return().takeOwnership());
  rb_mHelpers.define_singleton_method("create_patch", [](Rice::Object,
                                                         bullet_ruby::SoftBodyWorldInfo& world_info,
                                                         Rice::Object corner00,
                                                         Rice::Object corner10,
                                                         Rice::Object corner01,
                                                         Rice::Object corner11,
                                                         int resolution_x,
                                                         int resolution_y,
                                                         int fixed_nodes,
                                                         bool generate_diagonals,
                                                         btScalar perturbation) {
    return new bullet_ruby::SoftBody(btSoftBodyHelpers::CreatePatch(
      world_info.get(),
      bullet_ruby::coerce_vector(corner00),
      bullet_ruby::coerce_vector(corner10),
      bullet_ruby::coerce_vector(corner01),
      bullet_ruby::coerce_vector(corner11),
      resolution_x,
      resolution_y,
      fixed_nodes,
      generate_diagonals,
      perturbation));
  }, Rice::Return().takeOwnership(), Rice::Arg("world_info"), Rice::Arg("corner00"), Rice::Arg("corner10"), Rice::Arg("corner01"), Rice::Arg("corner11"), Rice::Arg("resolution_x"), Rice::Arg("resolution_y"), Rice::Arg("fixed_nodes"), Rice::Arg("generate_diagonals"), Rice::Arg("perturbation") = btScalar(0.0));
  rb_mHelpers.define_singleton_method("create_ellipsoid", [](Rice::Object, bullet_ruby::SoftBodyWorldInfo& world_info, Rice::Object center, Rice::Object radius, int resolution) {
    return new bullet_ruby::SoftBody(btSoftBodyHelpers::CreateEllipsoid(
      world_info.get(),
      bullet_ruby::coerce_vector(center),
      bullet_ruby::coerce_vector(radius),
      resolution));
  }, Rice::Return().takeOwnership());
  rb_mHelpers.define_singleton_method("create_from_tri_mesh", [](Rice::Object, bullet_ruby::SoftBodyWorldInfo& world_info, Rice::Object vertices, Rice::Object triangles, bool randomize_constraints) {
    std::vector<btScalar> flat_vertices = flat_vertices_from_object(vertices);
    std::vector<int> indices = triangle_indices_from_object(triangles);
    return new bullet_ruby::SoftBody(btSoftBodyHelpers::CreateFromTriMesh(
      world_info.get(),
      flat_vertices.data(),
      indices.data(),
      static_cast<int>(indices.size() / 3),
      randomize_constraints));
  }, Rice::Return().takeOwnership(), Rice::Arg("world_info"), Rice::Arg("vertices"), Rice::Arg("triangles"), Rice::Arg("randomize_constraints") = true);
  rb_mHelpers.define_singleton_method("create_from_convex_hull", [](Rice::Object, bullet_ruby::SoftBodyWorldInfo& world_info, Rice::Object points, bool randomize_constraints) {
    std::vector<btVector3> vectors = vector_array_from_object(points);
    return new bullet_ruby::SoftBody(btSoftBodyHelpers::CreateFromConvexHull(
      world_info.get(),
      vectors.data(),
      static_cast<int>(vectors.size()),
      randomize_constraints));
  }, Rice::Return().takeOwnership(), Rice::Arg("world_info"), Rice::Arg("points"), Rice::Arg("randomize_constraints") = true);
}
