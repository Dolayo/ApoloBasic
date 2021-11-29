#include "mrcore.h"
#include <wx/wx.h>
#include "canvas.h"
#include <wx/sizer.h>
#include "defines.h"

enum id{ID_RDT = 1, ID_RDTstar = 2, ID_Plan = 3, ID_Ship = 4, ID_Sim = 5, ID_Stop = 6, ID_EGK = 7
};



class MainWindow : public wxFrame
{
public:
    MainWindow(const wxString& title);
	
	
private:

    int the_planner;

	canvas* MyGLCanvas;
    //wxPanel* panel;
    wxButton* button;
    wxStatusBar* sb;

    wxMenu *menuPlanners;
    wxMenu *menuHelp;
    wxMenuBar *menuBar;

    wxBoxSizer *box;
    wxPanel* pan;

    wxButton* _button_sim;
    wxButton* _button_stop;

    // Cajas de texto

    wxTextCtrl* _thrustXinp;
    wxTextCtrl* _thrustYinp;
    wxTextCtrl* _thrustWinp;
    wxTextCtrl* _timeinp;

    wxTextCtrl* _Wind_Force_Drag;
    wxTextCtrl* _Water_Force_Drag;

    wxTextCtrl* _Wind_Force_Side;
    wxTextCtrl* _Water_Force_Side;

    wxTextCtrl* _Wind_Moment;
    wxTextCtrl* _Water_Moment;
    wxTextCtrl* _Rotational_Moment;

    // Etiquetas

    wxStaticText* _label_Wind_Force_Drag;
    wxStaticText* _label_Water_Force_Drag;
    wxStaticText* _label_Wind_Force_Side;
    wxStaticText* _label_Water_Force_Side;
    wxStaticText* _label_Wind_Moment;
    wxStaticText* _label_Water_Moment;
    wxStaticText* _label_Rotational_Moment;

    wxStaticText* _label_thrustXinp;
    wxStaticText* _label_thrustYinp;
    wxStaticText* _label_thrustWinp;
    wxStaticText* _label_timeinp;

    wxTimer* _mytimer;

    int height;
    int width;

    World _world;
    World sea;
	WheeledBaseSim *myrobot;
    Ship* _p_myship;
	Sampler *_p_sampler;
	PathPlanner *_p_planner;
    RobotPath _sol;
	EGKRobotPath* _p_solution;

    double _x_start{ X_START };
    double _y_start{ Y_START };
    double _yaw_start{ YAW_START };

    double _x_goal{X_GOAL};
    double _y_goal{Y_GOAL};
    
    int _n_iter{ NUM_ITERS };

    void createEnvironment();
    void createShipEnvironment();
    
    void Resize(wxSizeEvent& event);
	void OnPlan(wxCommandEvent& event);
    void OnRDT(wxCommandEvent& event);
    void OnRDTstar(wxCommandEvent& event);
    void OnShip(wxCommandEvent& event);
    void OnTimer(wxTimerEvent& event);
    void OnStop(wxCommandEvent& WXUNUSED);
    void OnSimulate(wxCommandEvent& WXUNUSED);
    void OnEGK(wxCommandEvent& event);
    void OnExit(wxCloseEvent& event);
    void OnAbout(wxCommandEvent& event);
	DECLARE_EVENT_TABLE()
};
