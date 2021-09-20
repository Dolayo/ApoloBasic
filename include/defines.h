#ifndef __DEFINES_
#define __DEFINES_

#define DELTA_T 0.1 //seconds
#define THETA1 3*PI/180 // 1� para 20�,30� y 40�
#define THETA2 10*PI/180 // 5� para 20�(10� vale tambien), 10� para 45�, 10� para 30�
#define THETA3 1*PI/6 // 30� para 20�, 30� y 40�

//! Orientacion inicial, usado para testeo
#define TEST_YAW -PI/2//-PI / 6

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
#define V_MAX 2.7//2.7
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

//! Numero de iteraciones a efectuar al crear un nuevo path
constexpr int NUM_ITER_PATH = 500;//500

//! Tolerancia de posicion
constexpr double POSE_TOL = 1;

//! Tolerancia de velocidad
constexpr double VEL_TOL = 0.5;

//! Tolerancia de orientacion con respecto a la recta tangente a la curva
constexpr double YAW_TOL = PI / 36;//5�(/36)

//! Pertenencia a la circunferencia
constexpr double CIRC_TOL_INNER = 1;//0.5

//! Entrada en la circunferencia
constexpr double CIRC_TOL_OUTER = 3;

//!????
constexpr double CIRC_TOL_2 = 2;

//! Coordenadas absolutas X e Y del punto de inicio
constexpr double X_START = 0.0;
constexpr double Y_START = 30.0;

//! Orientacion relativa del robot en el punto de incio
constexpr double YAW_START = 0;


//! Coordenadas absolutas X e Y del punto destino
constexpr double X_GOAL = 30.0;//30
constexpr double Y_GOAL = 0.0;//0

//! Velocidad incial de avance
constexpr double VX_INIT = 0.0;//V_MAX

//! Radio de reconexion para el RRT*
constexpr double radius = 8;

//!??????
constexpr unsigned int NUM_ITERS = 6;

//! Distancias usadas para calcular la distancia de los puntos de control respecto a los puntos inicial y final
constexpr double FACTOR_INIT = 8;
constexpr double FACTOR_GOAL = 8;

//! Zona en la que se encuentra el punto goal con respecto al robot
enum class ZoneType { central, left, right };

//! Cuadrante en el que se encuentra el punto goal con respecto al robot
enum class Quadrant { first, second, third, fourth };

//! Zona en la que se encuentra el robot respecto a la curva
enum class CurveZone {Inner, Medium, Outer};



#endif