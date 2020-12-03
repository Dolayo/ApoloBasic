//#include <wx/wx.h>
#include <canvas.h>

enum{ID_Hello = 1, ID_Plan = 2};

class MainWindow : public wxFrame
{
public:
    MainWindow(const wxString& title);
	
	
private:

	canvas* MyGLCanvas;
    wxPanel* panel;
    wxButton* button;
    wxStatusBar* sb;
    int height;
    int width;

    World world;
	WheeledBaseSim *myrobot;
	Sampler *sampler;
	PathPlanner *planner;
	RobotPath solution;

    void createEnvironment();
    

	void OnPlan(wxCommandEvent& event);
    void OnExample(wxCommandEvent& event);
    void OnExit(wxCloseEvent& event);
    void OnAbout(wxCommandEvent& event);
	DECLARE_EVENT_TABLE()
};
