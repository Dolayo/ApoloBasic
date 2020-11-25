#include <canvas.h>

BEGIN_EVENT_TABLE(canvas, wxGLCanvas)
    EVT_PAINT    (canvas::Paintit)
	EVT_SIZE(canvas::Resized)
	EVT_MOTION(canvas::OnMouseMove)
	EVT_RIGHT_DOWN(canvas::OnMouseClick)
	EVT_RIGHT_UP(canvas::OnMouseClick)
	EVT_LEFT_DOWN(canvas::OnMouseClick)
	EVT_LEFT_UP(canvas::OnMouseClick)
	EVT_CHAR(canvas::OnKey)
END_EVENT_TABLE()

canvas::canvas(wxFrame *parent): wxGLCanvas(parent, wxID_ANY,  wxDefaultPosition, wxDefaultSize, 0, wxT("GLCanvas")){
    int argc = 1;
    char* argv[1] = { wxString((wxTheApp->argv)[0]).char_str() };
	m_context = new wxGLContext(this);
}


void canvas::Paintit(wxPaintEvent& WXUNUSED(event)){
    Render();
}

void canvas::Render()
{
    SetCurrent();

    wxPaintDC(this);

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

void canvas::Resized(wxSizeEvent& event)
{
	Refresh(false);
	event.Skip();
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

	
	scene.KeyDown(event.GetUnicodeKey());
	scene.SpecialKeyDown(event.GetUnicodeKey());

	SetFocus();
	Refresh();
}