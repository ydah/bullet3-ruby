#include <rice/rice.hpp>

void Init_Vector3(Rice::Module rb_mBullet);

extern "C"
void Init_bullet_ruby()
{
  Rice::Module rb_mBullet = Rice::define_module("Bullet");
  Init_Vector3(rb_mBullet);
}
