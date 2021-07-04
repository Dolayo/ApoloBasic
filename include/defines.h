#ifndef __DEFINES_
#define __DEFINES_

#define DELTA_T 0.1 //seconds
#define THETA1 1*PI/60 // 3º
#define THETA2 1*PI/6 // 30º
#define THETA3 1*PI/4 // 45º  estos son los valores con los que llega, mira a ver si los puedes apurar un poco

#define DIST1 4//4
#define DIST2 12//12
#define THRUSTX 28000
#define THRUSTY 28000
#define THRUSTW 192000
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

constexpr double POSE_TOL = 0.5;
constexpr double VEL_TOL = 0.5;
constexpr double YAW_TOL = PI / 36;//5º

constexpr double X_START = 1.0;
constexpr double Y_START = 8.0;

constexpr double X_GOAL = 9.0;//9.0
constexpr double Y_GOAL = 8.0;//8.0

constexpr double REENTRY_ANGLE_K = 2;

constexpr double radius = 8;

constexpr unsigned int NUM_ITERS = 6;

enum class ZoneType { central, left, right };
enum class Quadrant { first, second, third, fourth };

#endif