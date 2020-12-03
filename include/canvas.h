#include <wx/wx.h>
#include <wx/glcanvas.h>
#include <mrcore.h>

/////////////// DEBUG
#include <iostream>
#include <wx/wfstream.h>
#include <wx/txtstrm.h>
/////////////// DEBUG

using namespace mr;
using namespace std;

class canvas: public wxGLCanvas {

private:

	wxGLContext* m_context;	
	GLScene scene;
	World w;
	WheeledBaseSim *r;
	Sampler *s;
	PathPlanner *p;
	RobotPath sol;

	bool flag;

	friend class MainWindow;

public:

	canvas(wxWindow* parent, wxPoint pos, wxSize size);

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