#ifndef CONFIG_H
#define CONFIG_H


#define DOTASK2

#define LIDAR_POINTS 682
#define PI 3.14159265
#define WHEEL_RADIUS 3.25 //cm
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
    g_RIGHT = 0,
    g_UP = 1,
    g_LEFT = 2,
    g_DOWN = 3,
  };

    enum locDirection {
    l_RIGHT = 1,
    l_UP = 0,
    l_LEFT = 3,
    l_DOWN = 2,
  };

#endif //CONFIG_H
