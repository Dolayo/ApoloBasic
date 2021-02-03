#include "mrcore.h"
#include <wx/wx.h>
#include "mainWindow.h"



class ApoloLite : public wxApp
{
public:
    ApoloLite():frame(nullptr){}
	virtual bool OnInit();
	MainWindow* frame;
};

wxIMPLEMENT_APP(ApoloLite);



bool ApoloLite::OnInit()
{
    frame = new MainWindow(wxT("ApoloLite")); 

    frame->Centre();
    frame->Show(true);
    SetTopWindow(frame);
    
    return true;
}