#include <wx/wx.h>
#include<mainWindow.h>


using namespace mr;
using namespace std;

class MyApp : public wxApp
{
public:
    virtual bool OnInit();
	MyFrame * frame;
};
