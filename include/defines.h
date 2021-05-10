#ifndef __DEFINES_
#define __DEFINES_

#define DELTA_T 0.1 //seconds
#define THETA1 PI/12 // 15º
#define THETA2 PI/6 // 30º
#define THETA3 PI/4 // 45º
#define DIST1 4
#define DIST2 10
#define THRUSTX 30000
#define THRUSTY 30000
#define THRUSTW 195000
#define N_ITER 100
#define WIDTH 3
#define LENGTH 10
#define MASS 5000
#define J 23000 
#define XP 5
#define S_AIR 15
#define S_WATER 2.5
#define V_MAX 2.7
#define A_MAX 0.3857
#define M_ARM 1
#define TRUE_WIND_DIR PI
#define TRUE_WATER_DIR PI
#define WIND_SPEED 0 //20
#define WATER_SPEED 0//2
#define CRS_WATER 2.545759
#define CRS_AIR 0.904313
#define RO_WATER 1000
#define RO_AIR 1.225

constexpr auto POSE_TOL = 0.2;
constexpr auto VEL_TOL = 1.0;

enum class ZoneType { central, left, right };
enum class Quadrant { first, second, third, fourth };

#endif