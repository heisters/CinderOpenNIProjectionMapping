#include "cinder/app/AppBasic.h"
#include "cinder/gl/gl.h"
#include "cinder/gl/Vbo.h"
#include "cinder/gl/GlslProg.h"
#include "cinder/Camera.h"
#include "cinder/params/Params.h"

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
        cameraRotation(Quatf()),
        cameraDistance(100.f)
        { }
        
        void setup ()
        {
            ui = params::InterfaceGl("APP", Vec2i(400, 200));
            ui.addParam("camera rotation", &cameraRotation);
            ui.addParam("camera distance", &cameraDistance);
        }
        
        Quatf cameraRotation;
        float cameraDistance;
        params::InterfaceGl ui;
        ci::Vec3f mouseDragStart, cameraDragStart;
        float cameraDistanceDragStart;
    };
    UI ui;
};


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
    float camDist = ui.cameraDistance;
    Quatf quat = ui.cameraRotation;
    quat.w *= -1.0f; // reverse rotation
    Vec3f camTarget = Vec3f::zero();
    Vec3f camOffset = quat * Vec3f(0, 0, camDist);
    Vec3f camEye    = camTarget + camOffset;
    Vec3f camUp     = quat * Vec3f::yAxis();
    camera.lookAt(camEye, camTarget, camUp);

    
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
    gl::pushMatrices();
    gl::setMatrices(camera);
    {
        shader.bind();
        kinect.getColorTexture()->bind();
        {
            gl::clear( Color( 0, 0, 0 ) );
            gl::color(1, 1, 1);
            glPointSize(1.f);
            gl::draw(vbo);
        }
        kinect.getColorTexture()->unbind();
        shader.unbind();
    }
    gl::popMatrices();
    
    ui.ui.draw();
}

void ProjectionMappingApp::keyDown(KeyEvent ev)
{
    if ( ev.getCode() == KeyEvent::KEY_SPACE ) {
        ui.cameraRotation = Quatf();
        ui.cameraDistance = 100.f;
    }
}

void ProjectionMappingApp::mouseDrag(MouseEvent ev)
{
    if ( ev.isLeft() ) {
        ui.cameraRotation.v = Vec3f((ev.getY() - ui.mouseDragStart.y) / getWindowHeight(),
                                    (ev.getX() - ui.mouseDragStart.x) / getWindowWidth(),
                                    0) + ui.cameraDragStart;
        ui.cameraRotation.normalize();
    } else if ( ev.isRight() ) {
        ui.cameraDistance = ev.getY() - ui.mouseDragStart.z + ui.cameraDistanceDragStart;
//        cameraPosition.x = ev.getX() - mouseDragStart.x;
//        cameraPosition.z = ev.getY() - mouseDragStart.z;
    }
}

void ProjectionMappingApp::mouseDown(MouseEvent ev)
{
    ui.mouseDragStart = (Vec3f)ev.getPos().xyy();
    ui.cameraDragStart = ui.cameraRotation.v;
    ui.cameraDistanceDragStart = ui.cameraDistance;
}

void ProjectionMappingApp::mouseUp(MouseEvent ev)
{
}


CINDER_APP_BASIC( ProjectionMappingApp, RendererGl )
