#include "cinder/app/AppBasic.h"
#include "cinder/gl/gl.h"
#include "cinder/gl/Vbo.h"
#include "cinder/Camera.h"

#include "VKinect.h"

using namespace ci;
using namespace ci::app;
using namespace ci::gl;
using namespace std;
using namespace V;

class ProjectionMappingApp : public AppBasic {
public:
    void prepareSettings( Settings *settings );
    void setup();
    void keyDown(KeyEvent ev);
    void mouseDown( MouseEvent event );
    void mouseDrag(MouseEvent ev);
    void mouseUp(MouseEvent ev);
    void update();
    void draw();
    
private:
    Kinect kinect;
    
    ci::gl::VboMesh vbo;
    
    ci::CameraPersp camera;
    Vec3f cameraPosition;
    static const Vec3f DEFAULT_CAMERA_POSITION;
    Vec2i dragStart;
};
const Vec3f ProjectionMappingApp::DEFAULT_CAMERA_POSITION = Vec3f(0.0f, 0.0f, 750.f);

void ProjectionMappingApp::prepareSettings( Settings *settings )
{
    Vec2i size(min(1280, Display::getMainDisplay()->getWidth()),
               min(960, Display::getMainDisplay()->getHeight()));
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
    
    camera = CameraPersp(getWindowWidth(), getWindowHeight(), 60.f);
    cameraPosition = DEFAULT_CAMERA_POSITION;
    dragStart = Vec2i::zero();

    
    VboMesh::Layout layout;
    layout.setStaticIndices();
    layout.setDynamicPositions();
    layout.setStaticNormals();
    layout.setStaticColorsRGB();
    
    Vec2i kSize = kinect.getDepthSize();
    vector< uint32_t > indices;
    indices.reserve(kSize.x * kSize.y);
    vector< Vec3f > normals;
    normals.reserve(kSize.x * kSize.y);
    vector< Color > colors;
    colors.reserve(kSize.x * kSize.y);
    
    vbo = VboMesh(kSize.x * kSize.y, indices.capacity(), layout, GL_POINTS);
    
    for ( int y = 0; y < kSize.y; ++y ) {
        for ( int x = 0; x < kSize.x; ++x ) {
            indices.push_back(x + y * kSize.y);
            Color color(1, 1, 1);
            colors.push_back(color);
            normals.push_back(Vec3f(0, 0, 1));
        }
    }
    
    vbo.bufferIndices(indices);
    vbo.bufferColorsRGB(colors);
    vbo.bufferNormals(normals);
}

void ProjectionMappingApp::mouseDown( MouseEvent event )
{
}

void ProjectionMappingApp::update()
{
    kinect.update();
    camera.lookAt(cameraPosition, Vec3f::zero(), Vec3f::yAxis());

    
    Vec2i kSize = kinect.getDepthSize();
    vector< Vec3f > positions;
    positions.reserve(kSize.x * kSize.y);
    
    Surface8u surface(kinect.getDepthImage());

    
    gl::VboMesh::VertexIter it = vbo.mapVertexBuffer();
	for( int x = 0; x < kSize.x; ++x ) {
		for( int y = 0; y < kSize.y; ++y ) {
            ColorAT< uint8_t > color = surface.getPixel(Vec2i(x, y));
            it.setPosition(Vec3f(x - kSize.x / 2.f, -y + kSize.y / 2.f, -color.r));
            ++it;
        }
    }
}

void ProjectionMappingApp::draw()
{
	// clear out the window with black
    gl::setMatrices(camera);
	gl::clear( Color( 0, 0, 0 ) );
    gl::color(1, 1, 1);
    glPointSize(3.f);
    gl::draw(vbo);

}

void ProjectionMappingApp::keyDown(KeyEvent ev)
{
    if ( ev.getCode() == KeyEvent::KEY_SPACE ) {
        cameraPosition = DEFAULT_CAMERA_POSITION;
    }
}

void ProjectionMappingApp::mouseDrag(MouseEvent ev)
{
    if ( dragStart == Vec2i::zero() ) dragStart = ev.getPos() - cameraPosition.xy();
    cameraPosition.x = ev.getX() - dragStart.x;
    cameraPosition.y = ev.getY() - dragStart.y;
}

void ProjectionMappingApp::mouseUp(MouseEvent ev)
{
    if ( dragStart != Vec2i::zero() ) dragStart = Vec2i::zero();
}


CINDER_APP_BASIC( ProjectionMappingApp, RendererGl )
