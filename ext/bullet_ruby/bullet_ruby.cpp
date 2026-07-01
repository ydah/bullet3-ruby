#include <rice/rice.hpp>

void Init_Shapes(Rice::Module rb_mBullet);
void Init_Matrix3x3(Rice::Module rb_mBullet);
void Init_Quaternion(Rice::Module rb_mBullet);
void Init_Vector3(Rice::Module rb_mBullet);

extern "C"
void Init_bullet_ruby()
{
  Rice::Module rb_mBullet = Rice::define_module("Bullet");
  Init_Vector3(rb_mBullet);
  Init_Quaternion(rb_mBullet);
  Init_Matrix3x3(rb_mBullet);
  Init_Shapes(rb_mBullet);
}
