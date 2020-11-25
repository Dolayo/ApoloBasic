#include "main.h"

wxIMPLEMENT_APP(MyApp);

bool MyApp::OnInit()
{
    frame = new MyFrame("Button"); 
    
    frame->Show(true);
    return true;
}