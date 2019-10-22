
typedef struct gyro_accel_val
{
  float yaw;
  float pitch;
  float roll;
} gyro_accel_ypr;


void gyro_accel_init(bool calibrate);
gyro_accel_ypr gyra_accel_get_current();