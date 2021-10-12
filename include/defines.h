#ifndef __DEFINES_
#define __DEFINES_

#define DELTA_T 0.1 //seconds
#define THETA1 2*PI/180 // 5º normal, 2º spline
#define THETA2 5*PI/180 // 10º normal, 5º spline
#define THETA3 20*PI/180 // 30º normal, 20º spline

//! Orientacion inicial, usado para testeo
#define TEST_YAW -PI/2//-PI / 6

#define DIST1 8//8 valor seguro 20º, 30º y 40º
#define DIST2 18//16 valor seguro 20º, 30º y 40º
#define THRUSTX 28000
#define THRUSTX_spline THRUSTX*0.95
#define THRUSTY 28000
#define THRUSTY_spline THRUSTY*0.95
#define THRUSTW 192000
#define THRUSTW_spline THRUSTW/0.95
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
constexpr int NUM_ITER_PATH = 500;//500,  339 fin para el banco de pruebas de distancia30 

//! Tolerancia de posicion
constexpr double POSE_TOL = 1;

//! Tolerancia de velocidad
constexpr double VEL_TOL = 0.5;

//! Tolerancia de orientacion con respecto a la recta tangente a la curva
constexpr double YAW_TOL = PI / 90;//5º(/36)

//! Pertenencia a la circunferencia
constexpr double CIRC_TOL_INNER = 0.5;//0.5

//! Entrada en la circunferencia
constexpr double CIRC_TOL_OUTER = 10;

//!????
constexpr double CIRC_TOL_2 = 2;

//! Coordenadas absolutas X e Y del punto de inicio
constexpr double X_START = 0.0;//0
constexpr double Y_START = 10.0;//30

//! Orientacion relativa del robot en el punto de incio
constexpr double YAW_START = 0;


//! Coordenadas absolutas X e Y del punto destino
constexpr double X_GOAL = 10.0;//30
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