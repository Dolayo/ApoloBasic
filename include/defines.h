#ifndef __DEFINES_
#define __DEFINES_

#define DELTA_T 0.1 //seconds
#define THETA1 3*PI/180 // 1� para 20�,30� y 40�
#define THETA2 10*PI/180 // 5� para 20�(10� vale tambien), 10� para 45�, 10� para 30�
#define THETA3 1*PI/6 // 30� para 20�, 30� y 40�

#define TEST_YAW -PI/360//-PI / 6

#define DIST1 8//8 valor seguro 20�, 30� y 40�
#define DIST2 18//16 valor seguro 20�, 30� y 40�
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

constexpr double POSE_TOL = 1;
constexpr double VEL_TOL = 0.5;
constexpr double YAW_TOL = PI / 36;//5�

constexpr double CIRC_TOL = 0.5;

constexpr double CIRC_TOL_2 = 2;

constexpr double X_START = 0.0;
constexpr double Y_START = 30.0;

constexpr double X_GOAL = 30.0;
constexpr double Y_GOAL = 0.0;

constexpr double REENTRY_ANGLE_K = 1.6;//1.5 para 20�, 1.6 para 45�, 1.6 para 30� un poco justo
//Para el caso sin miniajustes: 1.6 para 10 y 20 grados

constexpr double DIST_ADJ1 = 8;
constexpr double DIST_ADJ2 = 18;

constexpr double radius = 8;

constexpr unsigned int NUM_ITERS = 6;

enum class ZoneType { central, left, right };
enum class Quadrant { first, second, third, fourth };

#endif