#include "mrcore.h"
#include <wx/wx.h>
#include <canvas.h>
#include <wx/sizer.h>

enum id{ID_RDT = 1, ID_RDTstar = 2, ID_Plan = 3, ID_Ship = 4};

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

    int height;
    int width;

    World world;
    World sea;
	WheeledBaseSim *myrobot;
    Ship* myship;
	Sampler *sampler;
	PathPlanner *planner;
	RobotPath solution;

    void createEnvironment();
    void createShipEnvironment();
    
    void Resize(wxSizeEvent& event);
	void OnPlan(wxCommandEvent& event);
    void OnRDT(wxCommandEvent& event);
    void OnRDTstar(wxCommandEvent& event);
    void OnShip(wxCommandEvent& event);
    void OnExit(wxCloseEvent& event);
    void OnAbout(wxCommandEvent& event);
	DECLARE_EVENT_TABLE()
};
