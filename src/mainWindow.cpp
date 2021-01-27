#include <mainWindow.h>
#include <iostream>


BEGIN_EVENT_TABLE(MainWindow, wxFrame)
    EVT_BUTTON(ID_Plan,  MainWindow::OnPlan)
	EVT_SIZE(MainWindow::Resize)
END_EVENT_TABLE()

MainWindow::MainWindow(const wxString& title): wxFrame(NULL, wxID_ANY, title, wxDefaultPosition, wxSize(1200, 800), wxDEFAULT_FRAME_STYLE | wxHSCROLL | wxVSCROLL)
{
	////////////DEBUG
	//wxFFileOutputStream output( stderr );
	//wxTextOutputStream cout( output );
	////////////DEBUG

    menuExample = new wxMenu;
    menuExample->Append(ID_Example, wxT("&RDT...\tCtrl-J"), wxT("Example of an RDT planner"));
    //menuFile->AppendSeparator();
    //menuFile->Append(wxID_EXIT);

    menuHelp = new wxMenu;
    menuHelp->Append(wxID_ABOUT);

	//Juntar todos los desplegables en un objeto global que se inserte en la ventana principal
    menuBar = new wxMenuBar;
    menuBar->Append(menuExample, wxT("&Example"));
    menuBar->Append(menuHelp, wxT("&Help"));

    SetMenuBar( menuBar );

    sb = CreateStatusBar();
    sb->SetStatusText(wxT("ApoloLite Ready!"));	

    Bind(wxEVT_MENU, &MainWindow::OnExample, this, ID_Example);
    Bind(wxEVT_MENU, &MainWindow::OnAbout, this, wxID_ABOUT);
    //Bind(wxEVT_MENU, &MyFrame::OnExit, this, wxID_EXIT);
	

	Connect(wxEVT_CLOSE_WINDOW, wxCloseEventHandler(MainWindow::OnExit));
	//Centre();

	//frameGL = new wxFrame((wxFrame *)NULL, -1,  wxT("Visualization Window"), wxPoint(600,100), wxSize(800,800));
    
	width=GetClientSize().GetWidth();
	height=GetClientSize().GetHeight();

	wxColour col1;
	col1.Set(255,0,0);
	pan = new wxPanel(this, wxID_ANY);
	//pan->SetBackgroundColour(col1);
	MyGLCanvas = new canvas(this, wxPoint((int)(width)/3, 0), wxSize((int)(width*2)/3, height));
	button = new wxButton(this, ID_Plan, wxT("Plan"), wxPoint(20, 20),wxSize(100, 50));

	
	box=new wxBoxSizer(wxVERTICAL);

	//box->Add(button, wxSizerFlags(1).Expand().Border(wxALL, 10));
	//SetAutoLayout(true);
	//box->Add(pan, wxSizerFlags(1).Expand().Border(wxALL, 10));
	
	//SetSizer(box);
	//box->Fit(MyGLCanvas);
	

  	//Centre();
}
void MainWindow::Resize(wxSizeEvent& event)
{
	width=GetClientSize().GetWidth();
	height=GetClientSize().GetHeight();

	//MyGLCanvas->SetSize((int)(width)/3, 0,(int)(width*2)/3, height);
}
void MainWindow::createEnvironment()
{
	//Intializing test environment Faces included in a FacePart
	Face suelo(Transformation3D(0,0,0),0,-10,10,10);
	Face tablon_fino1(Transformation3D(8,3,2,X_AXIS,-0.53),0,0,0.2,3.95);
	Face tablon_fino2(Transformation3D(8.5,3,2,X_AXIS,-0.53),0,0,0.2,3.95);
	Face tablon_grueso(Transformation3D(2,3,2,X_AXIS,-0.53),0,0,1.4,3.95);
	Face plataforma(Transformation3D(2,0,2),0,0,8,3);
	Face paredfondo1(Transformation3D(0,0,0,Y_AXIS,PI/2),-4,-10,0,10);
	Face paredfondo2;

	paredfondo2.setBase(Transformation3D(0,0,0,X_AXIS,-PI/2));
	paredfondo2.addVertex(0,-4);
	paredfondo2.addVertex(10,-4);
	paredfondo2.addVertex(10,0);
	paredfondo2.addVertex(6,0);
	paredfondo2.addVertex(6,-1.5);
	paredfondo2.addVertex(4,-1.5);
	paredfondo2.addVertex(4,0);
	paredfondo2.addVertex(0,0);

	FaceSetPart *building=new FaceSetPart; 
	building->addFace(suelo);
	building->addFace(tablon_fino1);
	building->addFace(tablon_fino2);
	building->addFace(tablon_grueso);
	building->addFace(plataforma);
	building->addFace(paredfondo1);
	building->addFace(paredfondo2);
	
	world+=building;
}

void MainWindow::OnPlan(wxCommandEvent& WXUNUSED(event))
{
	sb->SetStatusText(wxT("Planning!"));

    WBStar gen(myrobot,&world, 0);
	WBStar *start=gen.createStateFromPoint3D(2.0,-8,0);
	WBStar *goal=gen.createStateFromPoint3D(8.0,1.5,2);
	solution.path.clear();
	planner->setStartAndGoalStates(start, goal); //generico a cualquier planificador
	delete start;
	delete goal;
	if(planner->computePlan(3000))solution.path=(planner->getPlan())->path;//3000
	MyGLCanvas->p = this->planner;
	MyGLCanvas->sol = this->solution;
	MyGLCanvas->Refresh(false);

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
		world.destroyContent();
    	this->Destroy();
	} else {
		event.Veto();
	}
}

void MainWindow::OnAbout(wxCommandEvent& event)
{
    wxMessageBox("Click on example and then on plan to see the algorithm's magic ",
                 "Some hints", wxOK | wxICON_INFORMATION);
}

void MainWindow::OnExample(wxCommandEvent& event)
{
    //wxLogMessage("Hello world from wxWidgets!");

	//wxFrame *frameGL = new wxFrame((wxFrame *)NULL, -1,  wxT("Hello GL World"), wxPoint(600,100), wxSize(800,800));
    //new wxGLCanvasSubClass(frameGL);
	//frameGL->Show(TRUE);

	sb->SetStatusText(wxT("Example Ready!"));
	
	createEnvironment();

	//creo el robot
	myrobot=new Pioneer3ATSim;
	myrobot->setRelativePosition(Vector3D(2.0,8,0));
	world+=myrobot;

	//creo un planificador y su sistema de muestreo
	sampler=new RandomSampler(&world);
	planner=new RDTstar;
	(dynamic_cast<SBPathPlanner *>(planner))->setSampler(sampler); //solo especifico de los basados en muestreo

	MyGLCanvas->w = this->world;
	MyGLCanvas->r = this->myrobot;
	MyGLCanvas->s = this->sampler;
	MyGLCanvas->p = this->planner;

	MyGLCanvas->create_world();
	
}