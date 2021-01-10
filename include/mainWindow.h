//#include <wx/wx.h>
#include <canvas.h>
#include <wx/sizer.h>
enum{ID_Example = 1, ID_Plan = 2};

class MainWindow : public wxFrame
{
public:
    MainWindow(const wxString& title);
	
	
private:

	canvas* MyGLCanvas;
    wxPanel* panel;
    wxButton* button;
    wxStatusBar* sb;

    wxMenu *menuExample;
    wxMenu *menuHelp;
    wxMenuBar *menuBar;

    wxBoxSizer *box;
    wxPanel* pan;

    int height;
    int width;

    World world;
	WheeledBaseSim *myrobot;
	Sampler *sampler;
	PathPlanner *planner;
	RobotPath solution;

    void createEnvironment();
    
    void Resize(wxSizeEvent& event);
	void OnPlan(wxCommandEvent& event);
    void OnExample(wxCommandEvent& event);
    void OnExit(wxCloseEvent& event);
    void OnAbout(wxCommandEvent& event);
	DECLARE_EVENT_TABLE()
};
