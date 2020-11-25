// wxWidgets "Hello World" Program
// For compilers that support precompilation, includes "wx/wx.h".

#include <wx/wx.h>
#include <wx/glcanvas.h>
#include <mrcore.h>

using namespace mr;
using namespace std;

//////////////////////////////////////////////////////////////// Canvas
class wxGLCanvasSubClass: public wxGLCanvas {

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

	wxGLCanvasSubClass(wxFrame* parent);

	virtual ~wxGLCanvasSubClass()
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

BEGIN_EVENT_TABLE(wxGLCanvasSubClass, wxGLCanvas)
    EVT_PAINT    (wxGLCanvasSubClass::Paintit)
	EVT_SIZE(wxGLCanvasSubClass::Resized)
	EVT_MOTION(wxGLCanvasSubClass::OnMouseMove)
	EVT_RIGHT_DOWN(wxGLCanvasSubClass::OnMouseClick)
	EVT_RIGHT_UP(wxGLCanvasSubClass::OnMouseClick)
	EVT_LEFT_DOWN(wxGLCanvasSubClass::OnMouseClick)
	EVT_LEFT_UP(wxGLCanvasSubClass::OnMouseClick)
	EVT_CHAR(wxGLCanvasSubClass::OnKey)
END_EVENT_TABLE()

wxGLCanvasSubClass::wxGLCanvasSubClass(wxFrame *parent): wxGLCanvas(parent, wxID_ANY,  wxDefaultPosition, wxDefaultSize, 0, wxT("GLCanvas")){
    int argc = 1;
    char* argv[1] = { wxString((wxTheApp->argv)[0]).char_str() };
	m_context = new wxGLContext(this);
}


void wxGLCanvasSubClass::Paintit(wxPaintEvent& WXUNUSED(event)){
    Render();
}

void wxGLCanvasSubClass::Render()
{
    SetCurrent();

    wxPaintDC(this);
	/*
	glClearColor(0.0, 0.0, 0.0, 0.0);
    glClear(GL_COLOR_BUFFER_BIT);
    glViewport(0, 0, (GLint)GetSize().x, (GLint)GetSize().y);

    glBegin(GL_POLYGON);
        glColor3f(1.0, 1.0, 1.0);
        glVertex2f(-0.5, -0.5);
        glVertex2f(-0.5, 0.5);
        glVertex2f(0.5, 0.5);
        glVertex2f(0.5, -0.5);
        glVertex2f(0.0, -0.8);
    glEnd();

    glBegin(GL_POLYGON);
        glColor3f(1.0, 0.0, 0.0);
        glVertex2f(0.1, 0.1);
        glVertex2f(-0.1, 0.1);
        glVertex2f(-0.1, -0.1);
        glVertex2f(0.1, -0.1);
    glEnd();*/
	
	scene.init();

	///////////////////////////////////////// Create environment
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
	///////////////////////////////////////// Create environment

	scene.addObject(&world);

    //creo el robot
	myrobot=new Pioneer3ATSim;
	myrobot->setRelativePosition(Vector3D(2.0,8,0));
	world+=myrobot;
	
	//creo un planificador y su sistema de muestreo
	sampler=new RandomSampler(&world);
	planner=new RDTplanner;
	(dynamic_cast<SBPathPlanner *>(planner))->setSampler(sampler); //solo especifico de los basados en muestreo
	

	scene.Draw();
	planner->drawGL();

	solution.drawGL();
	
    glFlush();
    SwapBuffers();
}

void wxGLCanvasSubClass::Resized(wxSizeEvent& event)
{
	Refresh(false);
	event.Skip();
}

void wxGLCanvasSubClass::OnMouseMove(wxMouseEvent& event)
{
	wxPoint pt = event.GetPosition();//return the physical mouse position

	if(event.RightIsDown()||event.LeftIsDown())
	{
		scene.MouseMove(pt.x,pt.y); 
		Refresh(false);
	}
	event.Skip();
}

void wxGLCanvasSubClass::OnMouseClick(wxMouseEvent& event)
{
	wxPoint pt = event.GetPosition();
	
	scene.MouseButton(pt.x,pt.y,2,event.RightIsDown(),event.ShiftDown(),event.ControlDown()); 
	scene.MouseButton(pt.x,pt.y,0,event.LeftIsDown(),event.ShiftDown(),event.ControlDown());
	
	SetFocus();
	event.Skip();
}

void wxGLCanvasSubClass::OnKey(wxKeyEvent& event)
{

	
	scene.KeyDown(event.GetUnicodeKey());
	scene.SpecialKeyDown(event.GetUnicodeKey());

	SetFocus();
	Refresh();
}

/////////////////////////////////////////////////////////////// ~Canvas

/////////////////////////////////////////////////////////////// MYFRAME

enum{ID_Hello = 1, ID_Plan = 2};

class MyFrame : public wxFrame
{
public:
    MyFrame(const wxString& title);
	
	
private:

	wxGLCanvasSubClass * MyGLCanvas;
	wxFrame *frameGL;

	void OnPlan(wxCommandEvent& event);
    void OnHello(wxCommandEvent& event);
    void OnExit(wxCloseEvent& event);
    void OnAbout(wxCommandEvent& event);
	DECLARE_EVENT_TABLE()
};

BEGIN_EVENT_TABLE(MyFrame, wxFrame)
    EVT_BUTTON(ID_Plan,  MyFrame::OnPlan)
END_EVENT_TABLE()

MyFrame::MyFrame(const wxString& title): wxFrame(NULL, wxID_ANY, title, wxDefaultPosition, wxSize(1000, 700))
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
    MyGLCanvas = new wxGLCanvasSubClass(frameGL);

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
}



void MyFrame::OnExit(wxCloseEvent& event)
{
	wxMessageDialog *dial = new wxMessageDialog(NULL,
      wxT("Are you sure to quit?"), wxT("Question"),
      wxYES_NO | wxNO_DEFAULT | wxICON_QUESTION);

	int ret = dial->ShowModal();
	dial->Destroy();

	if (ret == wxID_YES) {
		MyGLCanvas->~wxGLCanvasSubClass();
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
/////////////////////////////////////////////////////////////// ~MYFRAME

/////////////////////////////////////////////////////////////// 	MYAPP
class MyApp : public wxApp
{
public:
    virtual bool OnInit();
	MyFrame * frame;
};

wxIMPLEMENT_APP(MyApp);

bool MyApp::OnInit()
{
    frame = new MyFrame("Button"); 
    
    frame->Show(true);
    return true;
}
/////////////////////////////////////////////////////////////// 	~MYAPP