#pragma once

#include <memory>
#include <unordered_map>
#include <unordered_set>

#include <BulletSoftBody/btSoftBodyHelpers.h>
#include <BulletSoftBody/btSoftBodyRigidBodyCollisionConfiguration.h>
#include <BulletSoftBody/btSoftRigidDynamicsWorld.h>
#include <rice/rice.hpp>

#include "../dynamics/rb_dynamics.hpp"

namespace bullet3 {
class SoftBodyWorldInfo {
public:
  SoftBodyWorldInfo();
  explicit SoftBodyWorldInfo(btSoftBodyWorldInfo* world_info);

  btSoftBodyWorldInfo& get();
  const btSoftBodyWorldInfo& get() const;
  btVector3 gravity() const;
  btVector3 set_gravity(Rice::Object gravity);
  btScalar air_density() const;
  btScalar set_air_density(btScalar value);
  btScalar water_density() const;
  btScalar set_water_density(btScalar value);
  btScalar water_offset() const;
  btScalar set_water_offset(btScalar value);
  btVector3 water_normal() const;
  btVector3 set_water_normal(Rice::Object normal);
  btScalar max_displacement() const;
  btScalar set_max_displacement(btScalar value);
  void reset_sparse_sdf();

private:
  std::unique_ptr<btSoftBodyWorldInfo> owned_world_info_;
  btSoftBodyWorldInfo* world_info_;
};

class SoftBody {
public:
  explicit SoftBody(btSoftBody* soft_body);

  btSoftBody* get();
  const btSoftBody* get() const;
  int node_count() const;
  int link_count() const;
  int face_count() const;
  int tetra_count() const;
  int cluster_count() const;
  btScalar total_mass() const;
  void set_total_mass(btScalar mass, bool from_faces);
  btScalar node_mass(int index) const;
  btScalar set_node_mass(int index, btScalar mass);
  btVector3 node_position(int index) const;
  btVector3 node_velocity(int index) const;
  btVector3 center_of_mass() const;
  btVector3 linear_velocity();
  void set_linear_velocity(Rice::Object velocity);
  void set_velocity(Rice::Object velocity);
  void set_angular_velocity(Rice::Object velocity);
  void add_force(Rice::Object force);
  void add_velocity(Rice::Object velocity);
  void translate(Rice::Object translation);
  void rotate(Rice::Object rotation);
  void scale(Rice::Object scaling);
  void transform(Rice::Object transform);
  void set_pose(bool volume, bool frame);
  int generate_bending_constraints(int distance);
  int generate_clusters(int count);
  void randomize_constraints();
  btVector3 wind_velocity();
  btVector3 set_wind_velocity(Rice::Object velocity);
  void set_zero_velocity();
  bool self_collision();
  bool set_self_collision(bool enabled);
  Rice::Array aabb() const;
  Rice::Object ray_test(Rice::Object from, Rice::Object to);
  void append_anchor(int node, RigidBody& rigid_body, bool disable_collision_between_linked_bodies, btScalar influence);

private:
  int validate_node_index(int index) const;

  std::unique_ptr<btSoftBody> soft_body_;
};

class SoftRigidDynamicsWorld {
public:
  SoftRigidDynamicsWorld();
  ~SoftRigidDynamicsWorld();

  btSoftRigidDynamicsWorld* get();
  btVector3 gravity() const;
  btVector3 set_gravity(Rice::Object gravity);
  SoftBodyWorldInfo* world_info();
  void add_rigid_body_object(VALUE rigid_body);
  void remove_rigid_body_object(VALUE rigid_body);
  void add_soft_body_object(VALUE soft_body, int collision_filter_group, int collision_filter_mask);
  void remove_soft_body_object(VALUE soft_body);
  int step_simulation(btScalar time_step, int max_sub_steps, btScalar fixed_time_step);
  int num_collision_objects() const;
  int num_soft_bodies() const;
  int draw_flags() const;
  int set_draw_flags(int flags);
  void clear_forces();
  void mark() const;

private:
  std::unique_ptr<btSoftBodyRigidBodyCollisionConfiguration> configuration_;
  std::unique_ptr<btCollisionDispatcher> dispatcher_;
  std::unique_ptr<btDbvtBroadphase> broadphase_;
  std::unique_ptr<btSequentialImpulseConstraintSolver> solver_;
  std::unique_ptr<btSoftRigidDynamicsWorld> world_;
  std::unordered_set<btRigidBody*> rigid_bodies_;
  std::unordered_set<btSoftBody*> soft_bodies_;
  std::unordered_map<const btCollisionObject*, VALUE> collision_object_values_;
  std::unordered_map<const btSoftBody*, VALUE> soft_body_values_;
};
} // namespace bullet3

void Init_SoftBody(Rice::Module rb_mBullet);
