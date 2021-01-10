#include "main.h"

wxIMPLEMENT_APP(ApoloLite);

bool ApoloLite::OnInit()
{
    frame = new MainWindow(wxT("ApoloLite")); 

    frame->Centre();
    frame->Show(true);
    SetTopWindow(frame);
    
    return true;
}