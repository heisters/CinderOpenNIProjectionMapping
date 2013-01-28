#include "cinder/app/AppBasic.h"
#include "cinder/gl/gl.h"
#include "cinder/gl/Vbo.h"
#include "cinder/gl/GlslProg.h"
#include "cinder/Camera.h"
#include "cinder/params/Params.h"
#include "cinder/CinderMath.h"

#define _USE_MATH_DEFINES
#include <math.h>

#include "VKinect.h"
#include "VFakeKinect.h"
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
    void mouseDown( MouseEvent ev );
    void mouseDrag(MouseEvent ev);
    void mouseUp(MouseEvent ev);
    void update();
    void draw();
    
private:
    FakeKinect kinect;
    
    ci::gl::VboMesh vbo;
    ci::gl::GlslProg shader;
    
    ci::CameraPersp camera;
    
    
    class UI {
    public:
        UI() :
        cameraRotation(ci::Vec2f::zero()),
        cameraDistance(100.f)
        { }
        
        void setup ()
        {
            ui = params::InterfaceGl("APP", Vec2i(400, 200));
            ui.addParam("camera rotation X", &cameraRotation.x);
            ui.addParam("camera rotation Y", &cameraRotation.y);
            ui.addParam("camera distance", &cameraDistance);
        }
        
        params::InterfaceGl ui;
        
        ci::Vec2f cameraRotation, cameraRotationDragStart;
        float cameraDistance, cameraDistanceDragStart;
    };
    UI ui;
};


ProjectionMappingApp::ProjectionMappingApp ()
{
    
}

void ProjectionMappingApp::prepareSettings( Settings *settings )
{
    Vec2i size(min(1280, Display::getMainDisplay()->getWidth()),
               min(960, Display::getMainDisplay()->getHeight() - 40)); // allow room for chrome
    settings->setWindowSize(size.x, size.y);
    settings->setFrameRate(60.0f);
}

void ProjectionMappingApp::setup()
{
    ui.setup();
    
    
    try {
        kinect.setup(Vec2i(640, 480), NODE_TYPE_IMAGE | NODE_TYPE_DEPTH);
    } catch ( int e ) {
        console() << "No kinect. Exit sadface." << endl;
        quit();
    }
    kinect.getDevice()->setAlignWithDepthGenerator();
    
    camera = CameraPersp(getWindowWidth(), getWindowHeight(), 60.f);

    
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
            colors.push_back(Color(1, 1, 1));
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

void ProjectionMappingApp::update()
{
    Vec2f rotation = ui.cameraRotation * (M_PI / 180.f);
    Vec3f cameraPosition = Vec3f(ui.cameraDistance * cos(rotation.x) * sin(rotation.y),
                                 ui.cameraDistance * sin(rotation.x) * sin(rotation.y),
                                 ui.cameraDistance * cos(rotation.y));
    camera.lookAt(cameraPosition, Vec3f::zero(), Vec3f::yAxis());

    
    kinect.update();
    if ( kinect.getDevice()->isDepthDataNew() ) {
        Vec2i kSize = kinect.getDepthSize();
        vector< Vec3f > positions;
        positions.reserve(kSize.x * kSize.y);
        
        double start = getElapsedSeconds();
        XnPoint3D *pointCloud = kinect.getDepthMapRealWorld();
        
        start = getElapsedSeconds();
        for ( gl::VboMesh::VertexIter it = vbo.mapVertexBuffer(); it.getIndex() < vbo.getNumIndices(); ++it ) {
            XnPoint3D *point = pointCloud + it.getIndex();
            it.setPosition(point->X * 0.5f, point->Y * 0.5f, -(point->Z * 0.5));
        }
    }
}

void ProjectionMappingApp::draw()
{
    gl::clear( Color( 0, 0, 0 ) );
    
    ui.ui.draw();
    
    gl::pushMatrices();
    gl::setMatrices(camera);
    {
        shader.bind();
        kinect.getColorTexture()->bind();
        {
            gl::color(1, 1, 1);
            glPointSize(1.f);
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
        ui.cameraRotation = Vec2f::zero();
        ui.cameraDistance = 100.f;
    }
}

void ProjectionMappingApp::mouseDrag(MouseEvent ev)
{
    if ( ev.isLeft() ) {
        ui.cameraRotation = ev.getPos().yx() - ui.cameraRotationDragStart;
    } else if ( ev.isRight() ) {
        ui.cameraDistance = ev.getPos().y - ui.cameraDistanceDragStart;
    }
}

void ProjectionMappingApp::mouseDown(MouseEvent ev)
{
    ui.cameraRotationDragStart = ev.getPos().yx() - ui.cameraRotation;
    ui.cameraDistanceDragStart = ev.getPos().y - ui.cameraDistance;
}

void ProjectionMappingApp::mouseUp(MouseEvent ev)
{
}


CINDER_APP_BASIC( ProjectionMappingApp, RendererGl )
