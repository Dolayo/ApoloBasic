#include <mainWindow.h>


#include <iostream>
#include <string>
#include <wx/timer.h>
#include <wx/txtstrm.h>



BEGIN_EVENT_TABLE(MainWindow, wxFrame)
EVT_BUTTON(ID_Plan, MainWindow::OnPlan)
EVT_BUTTON(ID_Sim, MainWindow::OnSimulate)
EVT_BUTTON(ID_Stop, MainWindow::OnStop)
EVT_SIZE(MainWindow::Resize)
END_EVENT_TABLE()


MainWindow::MainWindow(const wxString& title) : wxFrame(NULL, wxID_ANY, title, wxDefaultPosition, wxSize(1200, 800), wxDEFAULT_FRAME_STYLE | wxHSCROLL | wxVSCROLL),
myrobot(nullptr),
_myship(nullptr),
planner(nullptr),
sampler(nullptr),
_thrustXinp(nullptr),
_thrustYinp(nullptr),
_button_sim(nullptr),
_timeinp(nullptr),
_label_thrustXinp(nullptr),
_label_thrustYinp(nullptr),
_label_timeinp(nullptr),
the_planner(0)
{
	////////////DEBUG
	//wxFFileOutputStream output( stderr );
	//wxTextOutputStream cout( output );
	////////////DEBUG

    menuPlanners = new wxMenu;
	menuPlanners->Append(ID_RDT, wxT("&RDT...\tCtrl-J"), wxT("Example of an RDT planner"));
	menuPlanners->Append(ID_RDTstar, wxT("&RDT*...\tCtrl-K"), wxT("Example of an RDT star planner"));
	menuPlanners->Append(ID_Ship, wxT("&Ship...\tCtrl-L"), wxT("Da Ship"));
    //menuFile->AppendSeparator();
    //menuFile->Append(wxID_EXIT);

    menuHelp = new wxMenu;
    menuHelp->Append(wxID_ABOUT);

	//Juntar todos los desplegables en un objeto global que se inserte en la ventana principal
    menuBar = new wxMenuBar;
    menuBar->Append(menuPlanners, wxT("&Planners"));
    menuBar->Append(menuHelp, wxT("&Help"));

    SetMenuBar( menuBar );

    sb = CreateStatusBar();
    sb->SetStatusText(wxT("ApoloLite Ready!"));	

	Bind(wxEVT_MENU, &MainWindow::OnRDT, this, ID_RDT);
    Bind(wxEVT_MENU, &MainWindow::OnRDTstar, this, ID_RDTstar);
    Bind(wxEVT_MENU, &MainWindow::OnAbout, this, wxID_ABOUT);
	Bind(wxEVT_MENU, &MainWindow::OnShip, this, ID_Ship);
	_mytimer = new wxTimer;
	_mytimer->Bind(wxEVT_TIMER, &MainWindow::OnTimer, this);
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

void MainWindow::createShipEnvironment()
{
	//Intializing test environment Faces included in a FacePart
	Face deep(Transformation3D(0, 0, 0), 0, -10, 10, 10);
	deep.setColor(0.1451, 0.1569, 0.3137,1);
	Face land1;
	Face land2;
	Face land3;
	Face land4;

	land1.setBase(Transformation3D(0, 0, 0, X_AXIS, -PI / 2));
	land2.setBase(Transformation3D(0, -2, 0, X_AXIS, -PI / 2));
	land3.setBase(Transformation3D(5, -2, 0, Y_AXIS, -PI / 2));
	land4.setBase(Transformation3D(0, 0, 2, Z_AXIS, -PI / 2));

	land1.addVertex(5, 0);
	land1.addVertex(5, -2);
	land1.addVertex(0, -2);
	land1.addVertex(0, 0);
	land1.setColor(0.502, 0.251, 0.1, 1);

	land2.addVertex(5, 0);
	land2.addVertex(5, -2);
	land2.addVertex(0, -2);
	land2.addVertex(0, 0);
	land2.setColor(0.502, 0.251, 0.1, 1);

	land3.addVertex(2, 0);
	land3.addVertex(2, 2);
	land3.addVertex(0, 2);
	land3.addVertex(0, 0);
	land3.setColor(0.502, 0.251, 0.1, 1);

	land4.addVertex(2, 0);
	land4.addVertex(2, 5);
	land4.addVertex(0, 5);
	land4.addVertex(0, 0);
	land4.setColor(0.502, 0.251, 0.1, 1);

	FaceSetPart* shore = new FaceSetPart;
	shore->addFace(deep);
	shore->addFace(land1);
	shore->addFace(land2);
	shore->addFace(land3);
	shore->addFace(land4);

	world += shore;
}

void MainWindow::OnPlan(wxCommandEvent& WXUNUSED(event))
{
	sb->SetStatusText(wxT("Planning!"));


	switch (this->the_planner)
	{
		case 0:
		{
			WBState gen(myrobot, &world);
			WBState* start = gen.createStateFromPoint3D(2.0, -8, 0);
			WBState* goal = gen.createStateFromPoint3D(8.0, 1.5, 2);
			solution.path.clear();
			planner->setStartAndGoalStates(start, goal); //generico a cualquier planificador
			delete start;
			delete goal;
			if (planner->computePlan(3000))solution.path = (planner->getPlan())->path;//3000
			MyGLCanvas->p = this->planner;
			MyGLCanvas->sol = this->solution;
			MyGLCanvas->Refresh(false);
			break; 
		}
		case 1:
		{
			WBStar gen(myrobot, &world, 0);
			WBStar* start = gen.createStateFromPoint3D(2.0, -8, 0);
			WBStar* goal = gen.createStateFromPoint3D(8.0, 1.5, 2);
			solution.path.clear();
			planner->setStartAndGoalStates(start, goal); //generico a cualquier planificador
			delete start;
			delete goal;
			if (planner->computePlan(3000))solution.path = (planner->getPlan())->path;//3000
			MyGLCanvas->p = this->planner;
			MyGLCanvas->sol = this->solution;
			MyGLCanvas->Refresh(false);
			break;
		}
		
		default:
			sb->SetStatusText(wxT("Something went wrong!"));
			break;
	}
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

void MainWindow::OnRDT(wxCommandEvent& event)
{
	//wxLogMessage("Hello world from wxWidgets!");

	//wxFrame *frameGL = new wxFrame((wxFrame *)NULL, -1,  wxT("Hello GL World"), wxPoint(600,100), wxSize(800,800));
	//new wxGLCanvasSubClass(frameGL);
	//frameGL->Show(TRUE);


	this->the_planner = 0;

	sb->SetStatusText(wxT("RDT Ready!"));

	createEnvironment();

	//creo el robot
	myrobot = new Pioneer3ATSim;
	myrobot->setRelativePosition(Vector3D(2.0, 8, 0));
	world += myrobot;

	//creo un planificador y su sistema de muestreo
	sampler = new RandomSampler(&world);
	planner = new RDTplanner;
	(dynamic_cast<SBPathPlanner*>(planner))->setSampler(sampler); //solo especifico de los basados en muestreo

	MyGLCanvas->w = this->world;
	MyGLCanvas->r = this->myrobot;
	MyGLCanvas->s = this->sampler;
	MyGLCanvas->p = this->planner;

	MyGLCanvas->create_world();

}

void MainWindow::OnRDTstar(wxCommandEvent& event)
{
    //wxLogMessage("Hello world from wxWidgets!");

	//wxFrame *frameGL = new wxFrame((wxFrame *)NULL, -1,  wxT("Hello GL World"), wxPoint(600,100), wxSize(800,800));
    //new wxGLCanvasSubClass(frameGL);
	//frameGL->Show(TRUE);

	this->the_planner = 1;

	sb->SetStatusText(wxT("RDT* Ready!"));
	
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

void MainWindow::OnShip(wxCommandEvent& event)
{
	this->the_planner = 2;

	sb->SetStatusText(wxT("Ship Ready!"));

	_label_thrustXinp = new wxStaticText(this, wxID_ANY, "Thrust x", wxPoint(20, 80), wxDefaultSize);
	_thrustXinp = new wxTextCtrl(this, wxID_ANY, "0.0", wxPoint(20, 100), wxDefaultSize,
		wxTE_LEFT, wxDefaultValidator, wxTextCtrlNameStr);

	_label_thrustYinp = new wxStaticText(this, wxID_ANY, "Thrust y", wxPoint(20, 140), wxDefaultSize);
	_thrustYinp = new wxTextCtrl(this, wxID_ANY, "0.0", wxPoint(20, 160), wxDefaultSize,
		wxTE_LEFT, wxDefaultValidator, wxTextCtrlNameStr);

	/*
		_label_timeinp = new wxStaticText(this, wxID_ANY, "Time", wxPoint(20, 200), wxDefaultSize);
	_timeinp = new wxTextCtrl(this, wxID_ANY, "0.0", wxPoint(20, 220), wxDefaultSize,
		wxTE_LEFT, wxDefaultValidator, wxTextCtrlNameStr);
	*/

	_label_Wind_Force_Drag = new wxStaticText(this, wxID_ANY, "Wind Force drag", wxPoint(140, 80), wxDefaultSize);
	_Wind_Force_Drag = new wxTextCtrl(this, wxID_ANY, "0.0", wxPoint(140, 100), wxDefaultSize,
		wxTE_RIGHT | wxTE_READONLY, wxDefaultValidator, wxTextCtrlNameStr);



	_button_sim = new wxButton(this, ID_Sim, wxT("Start"), wxPoint(140, 20), wxSize(60, 30));

	_button_stop = new wxButton(this, ID_Stop, wxT("Stop"), wxPoint(140, 50), wxSize(60, 30));

	createShipEnvironment();

	//creo el robot
	_myship = new Ship();
	_myship->setRelativePosition(Vector3D(2.0, 8, 0));
	_myship->setState(2, 8, 0, 0, 0, 0);
	world += _myship;
	/*
		//creo un planificador y su sistema de muestreo
	sampler = new RandomSampler(&world);
	planner = new RDTstar;
	(dynamic_cast<SBPathPlanner*>(planner))->setSampler(sampler); //solo especifico de los basados en muestreo
	*/


	MyGLCanvas->w = this->world;
	//MyGLCanvas->r = this->myship;
	MyGLCanvas->sh = this->_myship;
	//MyGLCanvas->s = this->sampler;
	//MyGLCanvas->p = this->planner;

	MyGLCanvas->create_world();
}
void MainWindow::OnTimer(wxTimerEvent& event)
{

	_myship->simulate(0.1);

	(*_Wind_Force_Drag).WriteText(to_string(round(_myship->getWind_Force_Drag())));
	
	MyGLCanvas->sh = _myship;
	MyGLCanvas->Refresh(true);
}

void MainWindow::OnStop(wxCommandEvent& WXUNUSED(event))
{
	_myship->move(0, 0);
}

void MainWindow::OnSimulate(wxCommandEvent& WXUNUSED(event))
{
	/*myship->setRelativePosition(Vector3D(2.0, 8, 0));
	myship->setU(0);
	myship->setV(0);
	myship->setW(0);*/

	float thrustX_f = std::stof(_thrustXinp->GetLineText(0).ToStdString());
	float thrustY_f = std::stof(_thrustYinp->GetLineText(0).ToStdString());
	//float time_f = std::stof(_timeinp->GetLineText(0).ToStdString());

	_myship->move(thrustX_f, thrustY_f);
	//myship->simulate(0.1);

	_mytimer->Start(100);	

	/*while (((std::chrono::steady_clock::now() - start)) < sim_t)
	{
		if ((std::chrono::steady_clock::now() - now)>step)
		{
			
			now = std::chrono::steady_clock::now();
		}

		if ((std::chrono::steady_clock::now() - start) > sim_t/2)
		{
			myship->move(0, 0);
		}

		MyGLCanvas->sh = myship;
		MyGLCanvas->Refresh(true);
	}*/
}