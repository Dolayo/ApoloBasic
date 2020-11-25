#include <wx/wx.h>
#include <wx/glcanvas.h>
#include <mrcore.h>

using namespace mr;
using namespace std;

class canvas: public wxGLCanvas {

private:

	wxGLContext* m_context;

	void Render();
	

public:

    GLScene scene;
	World world;
	WheeledBaseSim *myrobot;
	Sampler *sampler;
	PathPlanner *planner;
	RobotPath solution;

	canvas(wxFrame* parent);

	virtual ~canvas()
	{
		world.destroyContent();
		Close(true);
	}

	void OnMouseMove(wxMouseEvent& event);
	void OnMouseClick(wxMouseEvent& event);
	void Resized(wxSizeEvent& event);
	void OnKey(wxKeyEvent& event);
	void Paintit(wxPaintEvent& event);

protected:
    
	DECLARE_EVENT_TABLE()
};