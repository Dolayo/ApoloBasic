#ifndef __DEFINES_
#define __DEFINES_

#define DELTA_T 0.1 //seconds
#define THETA1 3*PI/180 // 3º normal, 
#define THETA2 10*PI/180 // 10º normal, 
#define THETA3 15*PI/180 // 15º normal, 



#define DIST1 5//5
#define DIST2 10//10
#define THRUSTX 28000
#define THRUSTX_spline 0 //27500 esta va bien para v_init = 0, 
#define THRUSTY 28000
#define THRUSTY_spline THRUSTY
#define THRUSTW 192000
#define THRUSTW_spline THRUSTW*1.0//La idea del giro no es muy buena, valio para 10 de chiripa, mejor bajar la velocidad en X
#define N_ITER 100
#define WIDTH 3
#define LENGTH 10
#define MASS 5000
#define J 23000 
#define XP 5
#define S_AIR 15
#define S_WATER 2.5
#define V_MAX 2.0//2.7
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
constexpr int NUM_ITER_PATH = 3000;//2000,  339 fin para el banco de pruebas de distancia30 

//! Tolerancia de posicion
constexpr double POSE_TOL = 1.0;

//! Tolerancia de posicion que aun estamos probando
constexpr double POSE_TOL_SHARP = 0.1;

//! Tolerancia de velocidad
constexpr double VEL_TOL = 0.5;

//! Tolerancia de orientacion con respecto a la recta tangente a la curva
constexpr double YAW_TOL = 2*PI / 180;//2º

//! Pertenencia a la circunferencia
constexpr double CIRC_TOL_INNER_OUTSIDE = 0.5;//0.5
constexpr double CIRC_TOL_INNER_INSIDE = 0.5;//0.5
// He cambiado a 0.5 porque le he puesto que vuelva a la curva en medium si esta dentro, si no vuelve a ponerlo en 1.5

//! Entrada en la circunferencia
constexpr double CIRC_TOL_OUTER = 10;

//!????
constexpr double CIRC_TOL_2 = 2;

//! Coordenadas absolutas X e Y del punto de inicio
constexpr double X_START = 0.0 + 1.0;//0
constexpr double Y_START = 0;//30

//! Orientacion relativa del robot en el punto de incio
constexpr double YAW_START = 0.0;


//! Coordenadas absolutas X e Y del punto destino
constexpr double X_GOAL = 0 + 1.0;//30
constexpr double Y_GOAL = -30.0;//0

//! Orientacion goal, usado para testeo
constexpr double TEST_YAW = -160.0 * PI / 180.0;

//! Velocidad incial de avance
constexpr double VX_INIT = V_MAX;//V_MAX

//! Radio de reconexion para el RRT*
constexpr double radius = 25;//25

//! Iteraciones del planificador en fase de puntos controlados
constexpr unsigned int NUM_ITERS = 4;


constexpr double T_TOP = 0.99;//0.9
constexpr double T_LOW = 0.01;//0.01

//! Velocidad de frenado para tomar la curva
constexpr double VX_SLOW= 0.1;//1.0

//! K spline center
constexpr double K_CENTER = 2.0;

//! K spline side
constexpr double K_SIDE = 3.0;

//! Maximo angulo de factibilidad para seguir la curva
constexpr double MAX_ANG = 25.0;

//! Maximo angulo de tolerancia para llegada al nodo goal
constexpr double MAX_ANG_TOL = 8.0;

//! Zona en la que se encuentra el punto goal con respecto al robot
enum class ZoneType { central, left, right };

//! Cuadrante en el que se encuentra el punto goal con respecto al robot
enum class Quadrant { first, second, third, fourth };

//! Zona en la que se encuentra el robot respecto a la curva
enum class CurveZone {Inner, Medium, Outer};



#endif