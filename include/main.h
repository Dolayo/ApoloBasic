//#include <wx/wx.h>

#include<mainWindow.h>


using namespace mr;
using namespace std;

class ApoloLite : public wxApp
{
public:
    virtual bool OnInit();
	MainWindow* frame;
};
