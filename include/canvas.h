#include "mrcore.h"
#include <wx/wx.h>
#include <wx/glcanvas.h>
#include "rdtstar.h"
#include "EGKRRT.h"
#include "Ship.h"


using namespace mr;
using namespace std;



class canvas: public wxGLCanvas {

private:

	wxGLContext* m_context;	
	GLScene scene;
	World w;
	WheeledBaseSim *r;
	Ship* sh;
	Sampler *s;
	PathPlanner *p;
	EGKRobotPath sol;

	bool flag;

	friend class MainWindow;

public:

	canvas(wxFrame* parent, wxPoint pos, wxSize size);

	virtual ~canvas()
	{
		w.destroyContent();
		delete m_context;
		Close(true);
	}

	//void show_world(bool f){show_world_flag = f;}

	void create_world();

	void OnMouseMove(wxMouseEvent& event);
	void OnMouseClick(wxMouseEvent& event);
	void Resized(wxSizeEvent& event);
	void OnKey(wxKeyEvent& event);
	void OnPaint(wxPaintEvent& event);


protected:
    
	DECLARE_EVENT_TABLE()
};
