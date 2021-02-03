
#include "canvas.h"
//#include <wx/wfstream.h>
//#include <wx/txtstrm.h>
BEGIN_EVENT_TABLE(canvas, wxGLCanvas)
    EVT_PAINT(canvas::OnPaint)
	EVT_SIZE(canvas::Resized)
	EVT_MOTION(canvas::OnMouseMove)
	EVT_RIGHT_DOWN(canvas::OnMouseClick)
	EVT_RIGHT_UP(canvas::OnMouseClick)
	EVT_LEFT_DOWN(canvas::OnMouseClick)
	EVT_LEFT_UP(canvas::OnMouseClick)
	EVT_CHAR(canvas::OnKey)
END_EVENT_TABLE()

canvas::canvas(wxFrame *parent, wxPoint pos, wxSize size): 
		wxGLCanvas(parent, wxID_ANY,NULL, pos, size, wxFULL_REPAINT_ON_RESIZE, wxT("Canvas")){

	
    int argc = 1;
    char* argv[1] = { wxString((wxTheApp->argv)[0]).char_str() };


	m_context = new wxGLContext(this);
	p = nullptr;
	r = nullptr;
	s = nullptr;
	flag=false;
}


void canvas::create_world()
{
		scene.init();

		scene.addObject(&w);

		flag = true;

		Refresh();
}



void canvas::OnPaint(wxPaintEvent& WXUNUSED(event)){

    wxGLCanvas::SetCurrent(*m_context);

    wxPaintDC(this);

if(flag)
{
	scene.Draw();
	p->drawGL();

	sol.drawGL();
}
	
	
    glFlush();
    SwapBuffers();
}

void canvas::Resized(wxSizeEvent& event)
{
	Refresh(false);
	//OnSize(event);
	//Update();
	//event.Skip();
}

void canvas::OnMouseMove(wxMouseEvent& event)
{
	wxPoint pt = event.GetPosition();//return the physical mouse position

	if(event.RightIsDown()||event.LeftIsDown())
	{
		scene.MouseMove(pt.x,pt.y); 
		Refresh(false);
	}
	event.Skip();
}

void canvas::OnMouseClick(wxMouseEvent& event)
{
	wxPoint pt = event.GetPosition();
	
	scene.MouseButton(pt.x,pt.y,2,event.RightIsDown(),event.ShiftDown(),event.ControlDown()); 
	scene.MouseButton(pt.x,pt.y,0,event.LeftIsDown(),event.ShiftDown(),event.ControlDown());
	
	SetFocus();
	event.Skip();
}

void canvas::OnKey(wxKeyEvent& event)
{

	wxChar u_keycode = event.GetUnicodeKey();

	if(u_keycode == WXK_NONE)
	{
		int i_keycode = event.GetKeyCode();

		scene.KeyDown(i_keycode);
		scene.SpecialKeyDown(i_keycode);
	}

	else
	{
		scene.KeyDown(u_keycode);
		scene.SpecialKeyDown(u_keycode);
	}
	
	SetFocus();
	Refresh();
}