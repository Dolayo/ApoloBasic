#include <wx/wx.h>
#include <canvas.h>

enum{ID_Hello = 1, ID_Plan = 2};

class MyFrame : public wxFrame
{
public:
    MyFrame(const wxString& title);
	
	
private:

	canvas * MyGLCanvas;
	wxFrame *frameGL;

	void OnPlan(wxCommandEvent& event);
    void OnHello(wxCommandEvent& event);
    void OnExit(wxCloseEvent& event);
    void OnAbout(wxCommandEvent& event);
	DECLARE_EVENT_TABLE()
};
