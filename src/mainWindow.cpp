#include <mainWindow.h>
#include <iostream>


BEGIN_EVENT_TABLE(MainWindow, wxFrame)
    EVT_BUTTON(ID_Plan,  MainWindow::OnPlan)
END_EVENT_TABLE()

MainWindow::MainWindow(const wxString& title): wxFrame(NULL, wxID_ANY, title, wxDefaultPosition, wxSize(1200, 800))
{
	////////////DEBUG
	wxFFileOutputStream output( stderr );
	wxTextOutputStream cout( output );
	////////////DEBUG
    wxMenu *menuFile = new wxMenu;
    menuFile->Append(ID_Hello, wxT("&Hello...\tCtrl-J"), wxT("Help string shown in status bar for this menu item"));
    menuFile->AppendSeparator();
    menuFile->Append(wxID_EXIT);

    wxMenu *menuHelp = new wxMenu;
    menuHelp->Append(wxID_ABOUT);

    wxMenuBar *menuBar = new wxMenuBar;
    menuBar->Append(menuFile, wxT("&Example"));
    menuBar->Append(menuHelp, wxT("&Help"));

    SetMenuBar( menuBar );

    sb = CreateStatusBar();
    sb->SetStatusText(wxT("ApoloLite Ready!"));	

    Bind(wxEVT_MENU, &MainWindow::OnExample, this, ID_Hello);
    Bind(wxEVT_MENU, &MainWindow::OnAbout, this, wxID_ABOUT);
    //Bind(wxEVT_MENU, &MyFrame::OnExit, this, wxID_EXIT);

	Connect(wxEVT_CLOSE_WINDOW, wxCloseEventHandler(MainWindow::OnExit));
	//Centre();

	//frameGL = new wxFrame((wxFrame *)NULL, -1,  wxT("Visualization Window"), wxPoint(600,100), wxSize(800,800));
    
	int width=GetClientSize().GetWidth();
	int heigth=GetClientSize().GetHeight();

	MyGLCanvas = new canvas(this, wxPoint(width-800, 0), wxSize(800, heigth));

	panel = new wxPanel(this, wxID_ANY, wxPoint(0, 0), wxSize(200, heigth));
	button = new wxButton(panel, ID_Plan, wxT("Plan"), wxPoint(0, 0),wxSize(100, 50));

  	//Centre();
}

void MainWindow::OnPlan(wxCommandEvent& WXUNUSED(event))
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



void MainWindow::OnExit(wxCloseEvent& event)
{
	wxMessageDialog *dial = new wxMessageDialog(NULL,
      wxT("Are you sure to quit?"), wxT("Question"),
      wxYES_NO | wxNO_DEFAULT | wxICON_QUESTION);

	int ret = dial->ShowModal();
	dial->Destroy();

	if (ret == wxID_YES) {
		delete MyGLCanvas;
    	this->Destroy();
	} else {
		event.Veto();
	}
}

void MainWindow::OnAbout(wxCommandEvent& event)
{
    wxMessageBox("This is a wxWidgets Hello World example",
                 "About Hello World", wxOK | wxICON_INFORMATION);
}

void MainWindow::OnExample(wxCommandEvent& event)
{
    //wxLogMessage("Hello world from wxWidgets!");

	//wxFrame *frameGL = new wxFrame((wxFrame *)NULL, -1,  wxT("Hello GL World"), wxPoint(600,100), wxSize(800,800));
    //new wxGLCanvasSubClass(frameGL);
	//frameGL->Show(TRUE);

}