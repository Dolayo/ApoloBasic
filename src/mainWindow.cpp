#include <mainWindow.h>


#include <iostream>
#include <string>
#include <wx/timer.h>
#include <wx/txtstrm.h>
#include <ShipState.h>
#include "EGKRRT.h"


BEGIN_EVENT_TABLE(MainWindow, wxFrame)
EVT_BUTTON(ID_Plan, MainWindow::OnPlan)
EVT_BUTTON(ID_Sim, MainWindow::OnSimulate)
EVT_BUTTON(ID_Stop, MainWindow::OnStop)
EVT_SIZE(MainWindow::Resize)
END_EVENT_TABLE()


MainWindow::MainWindow(const wxString& title) : wxFrame(NULL, wxID_ANY, title, wxDefaultPosition, wxSize(1200, 800), wxDEFAULT_FRAME_STYLE | wxHSCROLL | wxVSCROLL),
myrobot(nullptr),
_p_myship(nullptr),
_p_planner(nullptr),
_p_sampler(nullptr),
_p_solution(nullptr),
_thrustXinp(nullptr),
//_thrustYinp(nullptr),
_button_sim(nullptr),
_timeinp(nullptr),
_label_thrustXinp(nullptr),
//_label_thrustYinp(nullptr),
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
	menuPlanners->Append(ID_EGK, wxT("&EGK...\tCtrl-L"), wxT("EGK-RRT"));
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
	Bind(wxEVT_MENU, &MainWindow::OnEGK, this, ID_EGK);
	
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
	
	_world+=building;
}

void MainWindow::createShipEnvironment_Blank()
{
	//Intializing test environment Faces included in a FacePart
	Face deep(Transformation3D(0, 0, 0), 0, -60, 60, 60);
	deep.setColor(0.18, 0.67, 0.80, 1);//0.1451, 0.1569, 0.3137,1

	Face land1;
	Face land2;
	Face land3;
	Face land4;
	Face land5;

	land1.setBase(Transformation3D(0, 20, 0, X_AXIS, -PI / 2));
	land2.setBase(Transformation3D(0, -20, 0, X_AXIS, -PI / 2));
	land3.setBase(Transformation3D(50, -20, 0, Y_AXIS, -PI / 2));
	land4.setBase(Transformation3D(0, 0, 10, Z_AXIS, 0));//-PI / 2
	land5.setBase(Transformation3D(10, -20, 0, Y_AXIS, -PI / 2));

	land1.addVertex(50, 0);
	land1.addVertex(50, -10);
	land1.addVertex(10, -10);
	land1.addVertex(10, 0);
	land1.setColor(0.502, 0.251, 0.1, 1);//0.502, 0.251, 0.1, 1

	land2.addVertex(50, 0);
	land2.addVertex(50, -10);
	land2.addVertex(10, -10);
	land2.addVertex(10, 0);
	land2.setColor(0.502, 0.251, 0.1, 1);//0.502, 0.251, 0.1, 1

	land3.addVertex(10, 0);
	land3.addVertex(10, 40);
	land3.addVertex(0, 40);
	land3.addVertex(0, 0);
	land3.setColor(0.502, 0.251, 0.1, 1);//0.502, 0.251, 0.1, 1

	land4.addVertex(10, -20);
	land4.addVertex(50, -20);
	land4.addVertex(50, 20);
	land4.addVertex(10, 20);
	land4.setColor(0.502, 0.251, 0.1, 1);

	land5.addVertex(10, 0);
	land5.addVertex(10, 40);
	land5.addVertex(0, 40);
	land5.addVertex(0, 0);
	land5.setColor(0.502, 0.251, 0.1, 1);//0.502, 0.251, 0.1, 1

	FaceSetPart* shore = new FaceSetPart;
	shore->addFace(deep);
	/*shore->addFace(land1);
	shore->addFace(land2);
	shore->addFace(land3);
	shore->addFace(land4);
	shore->addFace(land5);*/

	_world += shore;
}

void MainWindow::createShipEnvironment_Obstacle1()
{
	//Intializing test environment Faces included in a FacePart
	Face deep(Transformation3D(0, 0, 0), 0, -60, 60, 60);
	deep.setColor(0.18, 0.67, 0.80,1);//0.1451, 0.1569, 0.3137,1

	Face land1;
	Face land2;
	Face land3;
	Face land4;
	Face land5;

	land1.setBase(Transformation3D(0, 20, 0, X_AXIS, -PI / 2));
	land2.setBase(Transformation3D(0, -20, 0, X_AXIS, -PI / 2));
	land3.setBase(Transformation3D(50, -20, 0, Y_AXIS, -PI / 2));
	land4.setBase(Transformation3D(0, 0, 10, Z_AXIS, 0));//-PI / 2
	land5.setBase(Transformation3D(10, -20, 0, Y_AXIS, -PI / 2));

	land1.addVertex(50, 0);
	land1.addVertex(50, -10);
	land1.addVertex(10, -10);
	land1.addVertex(10, 0);
	land1.setColor(0.502, 0.251, 0.1, 1);//0.502, 0.251, 0.1, 1

	land2.addVertex(50, 0);
	land2.addVertex(50, -10);
	land2.addVertex(10, -10);
	land2.addVertex(10, 0);
	land2.setColor(0.502, 0.251, 0.1, 1);//0.502, 0.251, 0.1, 1

	land3.addVertex(10, 0);
	land3.addVertex(10, 40);
	land3.addVertex(0, 40);
	land3.addVertex(0, 0);
	land3.setColor(0.502, 0.251, 0.1, 1);//0.502, 0.251, 0.1, 1

	land4.addVertex(10, -20);
	land4.addVertex(50, -20);
	land4.addVertex(50, 20);
	land4.addVertex(10, 20);
	land4.setColor(0.502, 0.251, 0.1, 1);

	land5.addVertex(10, 0);
	land5.addVertex(10, 40);
	land5.addVertex(0, 40);
	land5.addVertex(0, 0);
	land5.setColor(0.502, 0.251, 0.1, 1);//0.502, 0.251, 0.1, 1

	FaceSetPart* shore = new FaceSetPart;
	shore->addFace(deep);
	shore->addFace(land1);
	shore->addFace(land2);
	shore->addFace(land3);
	shore->addFace(land4);
	shore->addFace(land5);

	_world += shore;
}

void MainWindow::createShipEnvironment_Obstacle2()
{
	//Intializing test environment Faces included in a FacePart
	Face deep(Transformation3D(0, 0, 0), 0, -60, 60, 60);
	deep.setColor(0.18, 0.67, 0.80, 1);//0.1451, 0.1569, 0.3137,1

	Face land1;
	Face land2;
	Face land3;
	Face land4;

	Face land5;
	Face land6;
	Face land7;
	Face land8;

	land1.setBase(Transformation3D(0, 0, 0, X_AXIS, -PI / 2));
	land2.setBase(Transformation3D(0, -2, 0, X_AXIS, -PI / 2));
	land3.setBase(Transformation3D(25, -2, 0, Y_AXIS, -PI / 2));
	land4.setBase(Transformation3D(0, 0, 2, Z_AXIS, -PI / 2));//-PI / 2

	land5.setBase(Transformation3D(35, 0, 0, X_AXIS, -PI / 2));
	land6.setBase(Transformation3D(35, -2, 0, X_AXIS, -PI / 2));
	land7.setBase(Transformation3D(35, -2, 0, Y_AXIS, -PI / 2));
	land8.setBase(Transformation3D(0, 0, 2, Z_AXIS, 0));//-PI / 2


	//! First obstacle
	land1.addVertex(25, 0);
	land1.addVertex(25, -2);
	land1.addVertex(-25, -2);
	land1.addVertex(-25, 0);
	land1.setColor(0.502, 0.251, 0.1, 1);//0.502, 0.251, 0.1, 1

	land2.addVertex(25, 0);
	land2.addVertex(25, -2);
	land2.addVertex(-25, -2);
	land2.addVertex(-25, 0);
	land2.setColor(0.502, 0.251, 0.1, 1);//0.502, 0.251, 0.1, 1

	land3.addVertex(2, 0);
	land3.addVertex(2, 2);
	land3.addVertex(0, 2);
	land3.addVertex(0, 0);
	land3.setColor(0.502, 0.251, 0.1, 1);//0.502, 0.251, 0.1, 1

	land4.addVertex(2, -25);
	land4.addVertex(2, 25);
	land4.addVertex(0, 25);
	land4.addVertex(0, -25);
	land4.setColor(0.502, 0.251, 0.1, 1);

	//! Second obstacle
	land5.addVertex(45, 0);
	land5.addVertex(45, -2);
	land5.addVertex(0, -2);
	land5.addVertex(0, 0);
	land5.setColor(0.502, 0.251, 0.1, 1);//0.502, 0.251, 0.1, 1

	land6.addVertex(45, 0);
	land6.addVertex(45, -2);
	land6.addVertex(0, -2);
	land6.addVertex(0, 0);
	land6.setColor(0.502, 0.251, 0.1, 1);//0.502, 0.251, 0.1, 1

	land7.addVertex(2, 0);
	land7.addVertex(2, 2);
	land7.addVertex(0, 2);
	land7.addVertex(0, 0);
	land7.setColor(0.502, 0.251, 0.1, 1);//0.502, 0.251, 0.1, 1

	land8.addVertex(35, 0);
	land8.addVertex(35, -2);
	land8.addVertex(80, -2);
	land8.addVertex(80, 0);
	land8.setColor(0.502, 0.251, 0.1, 1);

	FaceSetPart* shore = new FaceSetPart;
	shore->addFace(deep);
	shore->addFace(land1);
	shore->addFace(land2);
	shore->addFace(land3);
	shore->addFace(land4);
	shore->addFace(land5);
	shore->addFace(land6);
	shore->addFace(land7);
	shore->addFace(land8);

	_world += shore;
}

void MainWindow::createShipEnvironment_Obstacle3()
{
	//Intializing test environment Faces included in a FacePart
	Face deep(Transformation3D(0, 0, 0), 0, -60, 60, 60);
	deep.setColor(0.18, 0.67, 0.80, 1);//0.1451, 0.1569, 0.3137,1

	Face land1;
	Face land2;
	Face land3;
	Face land4;
	//Face transversal;
	Face room1;
	Face room2;
	Face room3;

	vector<Vector2D> dock1_list_bod;
	PrismaticPart* dock1 = new PrismaticPart;
	double dock1_width = 2.0;
	double dock1_length = 60.0;
	dock1_list_bod.push_back(Vector2D(dock1_width / 2, dock1_width / 2));
	dock1_list_bod.push_back(Vector2D(dock1_width / 2, -dock1_width / 2));
	dock1_list_bod.push_back(Vector2D(-dock1_width / 2, -dock1_width / 2));
	dock1_list_bod.push_back(Vector2D(-dock1_width / 2, dock1_width / 2));
	dock1->setPolygonalBase(dock1_list_bod);
	dock1->setHeight(dock1_length);
	dock1->setRelativePosition(Vector3D(35, -60, dock1_width / 2));
	dock1->setRelativeOrientation(0, PI / 2, PI / 2);
	dock1->setColor(0.502, 0.251, 0.1);

	vector<Vector2D> dock2_list_bod;
	PrismaticPart* dock2 = new PrismaticPart;
	double dock2_width = 2.0;
	double dock2_length = 20.0;
	dock2_list_bod.push_back(Vector2D(dock2_width / 2, dock2_width / 2));
	dock2_list_bod.push_back(Vector2D(dock2_width / 2, -dock2_width / 2));
	dock2_list_bod.push_back(Vector2D(-dock2_width / 2, -dock2_width / 2));
	dock2_list_bod.push_back(Vector2D(-dock2_width / 2, dock2_width / 2));
	dock2->setPolygonalBase(dock2_list_bod);
	dock2->setHeight(dock2_length);
	dock2->setRelativePosition(Vector3D(0, -40, dock2_width / 2));
	dock2->setRelativeOrientation(0, PI / 2, 0);
	dock2->setColor(0.502, 0.251, 0.1);

	vector<Vector2D> dock3_list_bod;
	PrismaticPart* dock3 = new PrismaticPart;
	double dock3_width = 2.0;
	double dock3_length = 20.0;
	dock3_list_bod.push_back(Vector2D(dock3_width / 2, dock3_width / 2));
	dock3_list_bod.push_back(Vector2D(dock3_width / 2, -dock3_width / 2));
	dock3_list_bod.push_back(Vector2D(-dock3_width / 2, -dock3_width / 2));
	dock3_list_bod.push_back(Vector2D(-dock3_width / 2, dock3_width / 2));
	dock3->setPolygonalBase(dock3_list_bod);
	dock3->setHeight(dock3_length);
	dock3->setRelativePosition(Vector3D(0, -20, dock3_width / 2));
	dock3->setRelativeOrientation(0, PI / 2, 0);
	dock3->setColor(0.502, 0.251, 0.1);

	vector<Vector2D> dock4_list_bod;
	PrismaticPart* dock4 = new PrismaticPart;
	double dock4_width = 2.0;
	double dock4_length = 40.0;
	dock4_list_bod.push_back(Vector2D(dock4_width / 2, dock4_width / 2));
	dock4_list_bod.push_back(Vector2D(dock4_width / 2, -dock4_width / 2));
	dock4_list_bod.push_back(Vector2D(-dock4_width / 2, -dock4_width / 2));
	dock4_list_bod.push_back(Vector2D(-dock4_width / 2, dock4_width / 2));
	dock4->setPolygonalBase(dock4_list_bod);
	dock4->setHeight(dock4_length);
	dock4->setRelativePosition(Vector3D(45, 20, dock4_width / 2));
	dock4->setRelativeOrientation(0, PI / 2, PI / 2);
	dock4->setColor(0.502, 0.251, 0.1);




	//! Paredes exteriores
	land1.setBase(Transformation3D(0, 60, 0, X_AXIS, -PI / 2));
	land2.setBase(Transformation3D(0, -60, 0, X_AXIS, -PI / 2));
	land3.setBase(Transformation3D(60, -60, 0, Y_AXIS, -PI / 2));
	land4.setBase(Transformation3D(0, -60, 0, Y_AXIS, -PI / 2));
	//transversal.setBase(Transformation3D(30, -60, 0, Y_AXIS, -PI / 2));
	//room1.setBase(Transformation3D(0, -40, 0, X_AXIS, -PI / 2));
	//room2.setBase(Transformation3D(0, -20, 0, X_AXIS, -PI / 2));
	//room3.setBase(Transformation3D(45, 20, 0, Y_AXIS, -PI / 2));

	land1.addVertex(60, 0);
	land1.addVertex(60, -10);
	land1.addVertex(0, -10);
	land1.addVertex(0, 0);
	land1.setColor(0.502, 0.251, 0.1, 1);//0.502, 0.251, 0.1, 1
	
	land2.addVertex(60, 0);
	land2.addVertex(60, -10);
	land2.addVertex(0, -10);
	land2.addVertex(0, 0);
	land2.setColor(0.502, 0.251, 0.1, 1);//0.502, 0.251, 0.1, 1

	land3.addVertex(0, 0);
	land3.addVertex(0, 120);
	land3.addVertex(10, 120);
	land3.addVertex(10, 0);
	land3.setColor(0.502, 0.251, 0.1, 1);//0.502, 0.251, 0.1, 1

	land4.addVertex(0, 0);
	land4.addVertex(0, 120);
	land4.addVertex(10, 120);
	land4.addVertex(10, 0);
	land4.setColor(0.502, 0.251, 0.1, 1);

	/*transversal.addVertex(0, 0);
	transversal.addVertex(0, 80);
	transversal.addVertex(10, 80);
	transversal.addVertex(10, 0);
	transversal.setColor(0.502, 0.251, 0.1, 1);*/

	/*room1.addVertex(20, 0);
	room1.addVertex(20, -10);
	room1.addVertex(0, -10);
	room1.addVertex(0, 0);
	room1.setColor(0.502, 0.251, 0.1, 1);*/

	/*room2.addVertex(20, 0);
	room2.addVertex(20, -10);
	room2.addVertex(0, -10);
	room2.addVertex(0, 0);
	room2.setColor(0.502, 0.251, 0.1, 1);*/

	/*room3.addVertex(0, 0);
	room3.addVertex(0, 40);
	room3.addVertex(10, 40);
	room3.addVertex(10, 0);
	room3.setColor(0.502, 0.251, 0.1, 1);*/


	

	FaceSetPart* shore = new FaceSetPart;
	shore->addFace(deep);
	shore->addFace(land1);
	shore->addFace(land2);
	shore->addFace(land3);
	shore->addFace(land4);
	//shore->addFace(transversal);
	//shore->addFace(room1);
	/*shore->addFace(room2);
	shore->addFace(room3);*/

	_world += dock1;
	_world += dock2;
	_world += dock3;
	_world += dock4;
	_world += shore;
}

void MainWindow::OnPlan(wxCommandEvent& WXUNUSED(event))
{
	sb->SetStatusText(wxT("Planning!"));

	switch (this->the_planner)
	{
		case 0:// RRT
		{
			WBState gen(myrobot, &_world);
			WBState* start = gen.createStateFromPoint3D(2.0, 8.0, 0);
			WBState* goal = gen.createStateFromPoint3D(2.0, -8.0, 0);

			_p_planner->setStartAndGoalStates(start, goal); //generico a cualquier planificador
			delete start;
			delete goal;
			if (_p_planner->computePlan(3000))_sol.path = (_p_planner->getPlan())->path;//3000
			MyGLCanvas->p = this->_p_planner;
			MyGLCanvas->sol = _sol;
			MyGLCanvas->Refresh(false);
			break; 
		}
		case 1:// RRT*
		{
			WBStar gen(myrobot, &_world, 0);
			WBStar* start = gen.createStateFromPoint3D(2.0, -8, 0);
			WBStar* goal = gen.createStateFromPoint3D(8.0, 1.5, 0);

			_p_planner->setStartAndGoalStates(start, goal); //generico a cualquier planificador
			delete start;
			delete goal;
			if (_p_planner->computePlan(3))_sol.path = (_p_planner->getPlan())->path;//3000
			MyGLCanvas->p = this->_p_planner;
			MyGLCanvas->sol = _sol;
			MyGLCanvas->Refresh(false);
			break;
		}
		case 2: //EGK
		{
			ShipState* start;
			ShipState* goal;

			if (!(_p_planner->isSolved()))
			{
				ShipState gen(_p_myship, &_world);
				start = dynamic_cast<ShipState*>(gen.createStateFromPoint3D(_x_start, _y_start, _yaw_start));
				goal = dynamic_cast<ShipState*>(gen.createStateFromPoint3D(_x_goal, _y_goal, 0));
				_p_myship->setRelativePosition(Vector3D(_x_start, _y_start, 0));
				//_p_myship->setRelativeOrientation(0,0,0);
				_p_myship->setVels(Vector3D(VX_INIT,0,0));
				_p_myship->setState(_x_start, _y_start, _yaw_start);

				ShipState* start = dynamic_cast<ShipState*>(gen.createStateFromPoint3D(_x_start, _y_start, _yaw_start));
				ShipState* goal = dynamic_cast<ShipState*>(gen.createStateFromPoint3D(_x_goal, _y_goal, 0));

				start->setVels(Vector3D(VX_INIT, 0, 0));

				//solution.path.clear();
				_p_planner->setStartAndGoalStates(start, goal); //generico a cualquier planificador
				delete start;
				delete goal;
			}
				

			

			EGKRRT* p_EGKplanner = dynamic_cast<EGKRRT*>(_p_planner);

			if (!p_EGKplanner)
				break;

			//p_EGKplanner->testingPlan();
			bool success = p_EGKplanner->computePlan(_n_iter);

			const RobotPath* solution_plan = nullptr;

			solution_plan = _p_planner->getPlan();

			if (solution_plan && !_p_solution)
				_p_solution = new EGKRobotPath;

			if (solution_plan)
				_p_solution->path.assign(solution_plan->path.begin(), solution_plan->path.end());

			if(_p_planner)
				MyGLCanvas->p = this->_p_planner;

			if (_p_solution)
				MyGLCanvas->EGKsol = this->_p_solution;

			MyGLCanvas->Refresh(false);
			sb->SetStatusText(wxT("EGK done"));
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
		_world.destroyContent();
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
	_world += myrobot;

	//creo un planificador y su sistema de muestreo
	_p_sampler = new RandomSampler(&_world);
	_p_planner = new RDTplanner;
	(dynamic_cast<SBPathPlanner*>(_p_planner))->setSampler(_p_sampler); //solo especifico de los basados en muestreo

	MyGLCanvas->w = this->_world;
	MyGLCanvas->r = this->myrobot;
	MyGLCanvas->s = this->_p_sampler;
	MyGLCanvas->p = this->_p_planner;

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
	_world+=myrobot;

	//creo un planificador y su sistema de muestreo
	_p_sampler=new RandomSampler(&_world);
	_p_planner=new RDTstar;
	(dynamic_cast<SBPathPlanner *>(_p_planner))->setSampler(_p_sampler); //solo especifico de los basados en muestreo

	MyGLCanvas->w = this->_world;
	MyGLCanvas->r = this->myrobot;
	MyGLCanvas->s = this->_p_sampler;
	MyGLCanvas->p = this->_p_planner;

	MyGLCanvas->create_world();
	
}

void MainWindow::OnShip(wxCommandEvent& event)
{
	this->the_planner = 2;

	sb->SetStatusText(wxT("Ship Ready!"));

	_label_thrustXinp = new wxStaticText(this, wxID_ANY, "Thrust x", wxPoint(20, 80), wxDefaultSize);
	_thrustXinp = new wxTextCtrl(this, wxID_ANY, "0.0", wxPoint(20, 100), wxDefaultSize,
		wxTE_LEFT, wxDefaultValidator, wxTextCtrlNameStr);

	/*_label_thrustYinp = new wxStaticText(this, wxID_ANY, "Thrust y", wxPoint(20, 140), wxDefaultSize);
	_thrustYinp = new wxTextCtrl(this, wxID_ANY, "0.0", wxPoint(20, 160), wxDefaultSize,
		wxTE_LEFT, wxDefaultValidator, wxTextCtrlNameStr);*/

	_label_thrustWinp = new wxStaticText(this, wxID_ANY, "Thrust Rot", wxPoint(20, 140), wxDefaultSize);
	_thrustWinp = new wxTextCtrl(this, wxID_ANY, "0.0", wxPoint(20, 160), wxDefaultSize,
		wxTE_LEFT, wxDefaultValidator, wxTextCtrlNameStr);

	
		_label_timeinp = new wxStaticText(this, wxID_ANY, "Time", wxPoint(20, 200), wxDefaultSize);
	_timeinp = new wxTextCtrl(this, wxID_ANY, "0.0", wxPoint(20, 220), wxDefaultSize,
		wxTE_LEFT, wxDefaultValidator, wxTextCtrlNameStr);
	

	/*_label_Wind_Force_Drag = new wxStaticText(this, wxID_ANY, "Wind Force drag", wxPoint(140, 80), wxDefaultSize);
	_Wind_Force_Drag = new wxTextCtrl(this, wxID_ANY, "0.0", wxPoint(140, 100), wxDefaultSize,
		wxTE_RIGHT | wxTE_READONLY, wxDefaultValidator, wxTextCtrlNameStr);*/



	_button_sim = new wxButton(this, ID_Sim, wxT("Start"), wxPoint(140, 20), wxSize(60, 30));

	_button_stop = new wxButton(this, ID_Stop, wxT("Stop"), wxPoint(140, 50), wxSize(60, 30));

	createShipEnvironment_Obstacle2();

	//creo el robot
	_p_myship = new Ship();
	_p_myship->setRelativePosition(Vector3D(2.0, 8, 0));
	_p_myship->setState(2, 8, 0, 0, 0, 0);
	_world += _p_myship;
	/*
		//creo un planificador y su sistema de muestreo
	sampler = new RandomSampler(&world);
	planner = new RDTstar;
	(dynamic_cast<SBPathPlanner*>(planner))->setSampler(sampler); //solo especifico de los basados en muestreo
	*/


	MyGLCanvas->w = this->_world;
	//MyGLCanvas->r = this->myship;
	MyGLCanvas->sh = this->_p_myship;
	//MyGLCanvas->s = this->sampler;
	//MyGLCanvas->p = this->planner;

	MyGLCanvas->create_world();
}

void MainWindow::OnTimer(wxTimerEvent& event)
{

	//_myship->simpleSim(0.1);
	_p_myship->simpleDynamicsSim(_time_f);
	//(*_Wind_Force_Drag).WriteText(to_string(round(_myship->getWind_Force_Drag())));
	
	MyGLCanvas->sh = _p_myship;
	MyGLCanvas->Refresh(true);
}

void MainWindow::OnStop(wxCommandEvent& WXUNUSED(event))
{
	_p_myship->setThrusts(0, 0);
}

void MainWindow::OnSimulate(wxCommandEvent& WXUNUSED(event))
{
	/*myship->setRelativePosition(Vector3D(2.0, 8, 0));
	myship->setU(0);
	myship->setV(0);
	myship->setW(0);*/

	float thrustX_f = std::stof(_thrustXinp->GetLineText(0).ToStdString());
	//float thrustY_f = std::stof(_thrustYinp->GetLineText(0).ToStdString());
	float thrustW_M = std::stof(_thrustWinp->GetLineText(0).ToStdString());
	_time_f = std::stof(_timeinp->GetLineText(0).ToStdString());

	_p_myship->setThrusts(thrustX_f, 0.0, thrustW_M);
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

void MainWindow::OnEGK(wxCommandEvent& event)
{
	this->the_planner = 2;

	sb->SetStatusText(wxT("EGK-RRT Ready!"));

	_world.destroyContent();
	createShipEnvironment_Obstacle3();

	if (_p_myship)
		delete _p_myship;
	_p_myship = new Ship();

	_p_myship->setRelativePosition(Vector3D(_x_start, _y_start, 0));
	_p_myship->setRelativeOrientation(0,0, _yaw_start);
	_p_myship->setVels(Vector3D(V_MAX, 0, 0));

	_p_myship->setState(_x_start, _y_start, _yaw_start/*-PI / 2*/);
	_world += _p_myship;
	
	//creo un planificador y su sistema de muestreo
	if (!_p_sampler)
		_p_sampler = new RandomSampler(&_world);

	if (_p_planner)
		delete _p_planner;
	_p_planner = new EGKRRT;

	(dynamic_cast<SBPathPlanner*>(_p_planner))->setSampler(_p_sampler); //solo especifico de los basados en muestreo
	
	MyGLCanvas->w = this->_world;
	MyGLCanvas->sh = this->_p_myship;
	MyGLCanvas->s = this->_p_sampler;
	MyGLCanvas->p = this->_p_planner;

	MyGLCanvas->create_world();
}