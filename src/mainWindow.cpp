#include <mainWindow.h>


BEGIN_EVENT_TABLE(MyFrame, wxFrame)
    EVT_BUTTON(ID_Plan,  MyFrame::OnPlan)
END_EVENT_TABLE()

MyFrame::MyFrame(const wxString& title): wxFrame(NULL, wxID_ANY, title, wxDefaultPosition, wxSize(400, 500))
{
    wxMenu *menuFile = new wxMenu;
    menuFile->Append(ID_Hello, "&Hello...\tCtrl-J", "Help string shown in status bar for this menu item");
    menuFile->AppendSeparator();
    menuFile->Append(wxID_EXIT);
    wxMenu *menuHelp = new wxMenu;
    menuHelp->Append(wxID_ABOUT);
    wxMenuBar *menuBar = new wxMenuBar;
    menuBar->Append(menuFile, "&File");
    menuBar->Append(menuHelp, "&Help");
    SetMenuBar( menuBar );
    CreateStatusBar();
    SetStatusText("Welcome to wxWidgets!");
    Bind(wxEVT_MENU, &MyFrame::OnHello, this, ID_Hello);
    Bind(wxEVT_MENU, &MyFrame::OnAbout, this, wxID_ABOUT);
    //Bind(wxEVT_MENU, &MyFrame::OnExit, this, wxID_EXIT);
	Connect(wxEVT_CLOSE_WINDOW, wxCloseEventHandler(MyFrame::OnExit));
	Centre();

	frameGL = new wxFrame((wxFrame *)NULL, -1,  wxT("Hello GL World"), wxPoint(600,100), wxSize(800,800));
    MyGLCanvas = new canvas(frameGL);

	wxPanel *panel = new wxPanel(this, wxID_ANY);
	wxButton *button = new wxButton(panel, ID_Plan,wxT("Plan"), wxPoint(20, 20));

  	Centre();
}

void MyFrame::OnPlan(wxCommandEvent& WXUNUSED(event))
{
    WBState gen(MyGLCanvas->myrobot,&(MyGLCanvas->world));
	WBState *start=gen.createStateFromPoint3D(2.0,-8,0);
	WBState *goal=gen.createStateFromPoint3D(8.0,1.5,2);
	MyGLCanvas->solution.path.clear();
	MyGLCanvas->planner->setStartAndGoalStates(start, goal); //generico a cualquier planificador
	delete start;
	delete goal;
	if(MyGLCanvas->planner->computePlan(3000))MyGLCanvas->solution.path=(MyGLCanvas->planner->getPlan())->path;
	Refresh();
}



void MyFrame::OnExit(wxCloseEvent& event)
{
	wxMessageDialog *dial = new wxMessageDialog(NULL,
      wxT("Are you sure to quit?"), wxT("Question"),
      wxYES_NO | wxNO_DEFAULT | wxICON_QUESTION);

	int ret = dial->ShowModal();
	dial->Destroy();

	if (ret == wxID_YES) {
		MyGLCanvas->~canvas();
		frameGL->Destroy();
    	this->Destroy();
	} else {
		event.Veto();
	}
}

void MyFrame::OnAbout(wxCommandEvent& event)
{
    wxMessageBox("This is a wxWidgets Hello World example",
                 "About Hello World", wxOK | wxICON_INFORMATION);
}

void MyFrame::OnHello(wxCommandEvent& event)
{
    //wxLogMessage("Hello world from wxWidgets!");

	//wxFrame *frameGL = new wxFrame((wxFrame *)NULL, -1,  wxT("Hello GL World"), wxPoint(600,100), wxSize(800,800));
    //new wxGLCanvasSubClass(frameGL);
	frameGL->Show(TRUE);

}