#include <rice/rice.hpp>

void Init_Shapes(Rice::Module rb_mBullet);
void Init_Collision(Rice::Module rb_mBullet);
void Init_SoftBody(Rice::Module rb_mBullet);
void Init_Constraints(Rice::Module rb_mBullet);
void Init_Dynamics(Rice::Module rb_mBullet);
void Init_DebugDraw(Rice::Module rb_mBullet);
void Init_IO(Rice::Module rb_mBullet);
void Init_Matrix3x3(Rice::Module rb_mBullet);
void Init_MultiBody(Rice::Module rb_mBullet);
void Init_Quaternion(Rice::Module rb_mBullet);
void Init_Transform(Rice::Module rb_mBullet);
void Init_Vector3(Rice::Module rb_mBullet);
void Init_Vehicle(Rice::Module rb_mBullet);

extern "C"
void Init_bullet_ruby()
{
  Rice::Module rb_mBullet = Rice::define_module("Bullet");
  Init_Vector3(rb_mBullet);
  Init_Quaternion(rb_mBullet);
  Init_Matrix3x3(rb_mBullet);
  Init_Transform(rb_mBullet);
  Init_Shapes(rb_mBullet);
  Init_Dynamics(rb_mBullet);
  Init_Collision(rb_mBullet);
  Init_Constraints(rb_mBullet);
  Init_DebugDraw(rb_mBullet);
  Init_IO(rb_mBullet);
  Init_Vehicle(rb_mBullet);
  Init_SoftBody(rb_mBullet);
  Init_MultiBody(rb_mBullet);
}
