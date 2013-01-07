#include "cinder/app/AppBasic.h"
#include "cinder/gl/gl.h"

#include "VKinect.h"

using namespace ci;
using namespace ci::app;
using namespace std;
using namespace V;

class ProjectionMappingApp : public AppBasic {
public:
    void prepareSettings( Settings *settings );
    void setup();
    void mouseDown( MouseEvent event );
    void update();
    void draw();
    
private:
    Kinect kinect;
};

void ProjectionMappingApp::prepareSettings( Settings *settings )
{
    Vec2i size(min(1920, Display::getMainDisplay()->getWidth()),
               min(1080, Display::getMainDisplay()->getHeight()));
    settings->setWindowSize(size.x, size.y);
    settings->setFrameRate(60.0f);
}

void ProjectionMappingApp::setup()
{
    
    try {
        kinect.setup();
    } catch ( int e ) {
        console() << "No kinect. Exit sadface." << endl;
        quit();
    }

}

void ProjectionMappingApp::mouseDown( MouseEvent event )
{
}

void ProjectionMappingApp::update()
{
    kinect.update();
}

void ProjectionMappingApp::draw()
{
	// clear out the window with black
	gl::clear( Color( 0, 0, 0 ) );
    kinect.drawSkeletons(Rectf(0, 0, getWindowWidth(), getWindowHeight()));

}


CINDER_APP_BASIC( ProjectionMappingApp, RendererGl )
