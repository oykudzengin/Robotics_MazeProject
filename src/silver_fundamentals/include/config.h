#ifndef CONFIG_H
#define CONFIG_H

#define LIDAR_POINTS 682
#define PI 3.14159265
#define WHEEL_RADIUS 3.2 //cm
#define WHEEL_BASE 26.5 //cm
#define ANGLE_SPAN (4.178563637658954*180/PI)
#define ANGLE_MIN (-2.086213869974017*180/PI)
#define ANGLE_MAX (ANGLE_MIN + ANGLE_SPAN)
#define ANGLE_STEP (0.006135923322290182*180/PI)


enum direction {
    none,
    left,
    right,
  };

  enum globDirection {
    RIGHT = 0,
    UP = 1,
    LEFT = 2,
    DOWN = 3,
  };

#endif //CONFIG_H
