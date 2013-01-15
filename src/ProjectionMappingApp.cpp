#include "cinder/app/AppBasic.h"
#include "cinder/gl/gl.h"
#include "cinder/gl/Vbo.h"
#include "cinder/gl/GlslProg.h"
#include "cinder/Camera.h"

#include "VKinect.h"
#include "Resources.h"

using namespace ci;
using namespace ci::app;
using namespace ci::gl;
using namespace std;
using namespace V;

class ProjectionMappingApp : public AppBasic {
public:
    ProjectionMappingApp();
    
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
    ci::gl::GlslProg shader;
    
    ci::CameraPersp camera;
    ci::Vec3f cameraPosition;
    static const Vec3f DEFAULT_CAMERA_POSITION;
    ci::Vec2i dragStart;
};
const Vec3f ProjectionMappingApp::DEFAULT_CAMERA_POSITION = Vec3f(0.0f, 0.0f, 100.f);


ProjectionMappingApp::ProjectionMappingApp ()
{
    
}

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
        kinect.setup(Vec2i(640, 480), NODE_TYPE_IMAGE | NODE_TYPE_DEPTH);
    } catch ( int e ) {
        console() << "No kinect. Exit sadface." << endl;
        quit();
    }
    kinect.getDevice()->setAlignWithDepthGenerator();
    
    camera = CameraPersp(getWindowWidth(), getWindowHeight(), 60.f);
    cameraPosition = DEFAULT_CAMERA_POSITION;
    dragStart = Vec2i::zero();

    
    VboMesh::Layout layout;
    layout.setStaticIndices();
    layout.setDynamicPositions();
    layout.setStaticNormals();
    layout.setStaticColorsRGB();
    layout.setStaticTexCoords2d();
    
    Vec2i kSize = kinect.getDepthSize();
    vector< uint32_t > indices;
    indices.reserve(kSize.x * kSize.y);
    vector< Vec3f > normals;
    normals.reserve(indices.capacity());
    vector< Color > colors;
    colors.reserve(indices.capacity());
    vector< Vec2f > texCoords;
    texCoords.reserve(indices.capacity());
    
    vbo = VboMesh(kSize.x * kSize.y, indices.capacity(), layout, GL_POINTS);
    
    for ( int y = 0; y < kSize.y; ++y ) {
        for ( int x = 0; x < kSize.x; ++x ) {
            indices.push_back(x + y * kSize.y);
            Color color(1, 1, 1);
            colors.push_back(color);
            normals.push_back(Vec3f(0, 0, 1));
            texCoords.push_back(Vec2f((float)x / (float)kSize.x, (float)y / (float)kSize.y));
        }
    }
    
    vbo.bufferIndices(indices);
    vbo.bufferColorsRGB(colors);
    vbo.bufferNormals(normals);
    vbo.bufferTexCoords2d(0, texCoords);
    
    
    try {
		shader = gl::GlslProg(loadResource(RES_SHADER_VERT), loadResource(RES_SHADER_FRAG));
	}
	catch( gl::GlslProgCompileExc &exc ) {
		std::cout << "shader compile error: " << RES_SHADER_VERT << " / " << RES_SHADER_FRAG << std::endl;
		std::cout << exc.what();
        quit();
	}
	catch( ... ) {
		std::cout << "Unable to load shader: " << RES_SHADER_VERT << " / " << RES_SHADER_FRAG << std::endl;
        quit();
	}

}

void ProjectionMappingApp::mouseDown( MouseEvent event )
{
}

void ProjectionMappingApp::update()
{
    kinect.update();
    camera.lookAt(cameraPosition, Vec3f::zero(), Vec3f::yAxis());
    
    if ( kinect.getDevice()->isDepthDataNew() ) {
        Vec2i kSize = kinect.getDepthSize();
        vector< Vec3f > positions;
        positions.reserve(kSize.x * kSize.y);
        
        double start = getElapsedSeconds();
        XnPoint3D *pointCloud = kinect.getDepthMapRealWorld();
        
        start = getElapsedSeconds();
        gl::VboMesh::VertexIter it = vbo.mapVertexBuffer();
        for( int x = 0; x < kSize.x; ++x ) {
            for( int y = 0; y < kSize.y; ++y ) {
                XnPoint3D *point = pointCloud + (y + x * kSize.x);
                it.setPosition(point->X / 5.f, point->Y / 5.f, -(point->Z / 5.f));
                
                ++it;
            }
        }
    }    
}

void ProjectionMappingApp::draw()
{
    gl::pushMatrices();
    gl::setMatrices(camera);
    {
        shader.bind();
        kinect.getColorTexture()->bind();
        {
            gl::clear( Color( 0, 0, 0 ) );
            gl::color(1, 1, 1);
            glPointSize(3.f);
            gl::draw(vbo);
        }
        kinect.getColorTexture()->unbind();
        shader.unbind();
    }
    gl::popMatrices();
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
