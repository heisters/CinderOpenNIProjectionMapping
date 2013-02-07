/*
 
 TODO:
 - separate window for projector
 - refactor
 - only run calibration on demand
 - record calibration observation when the grid is held still
 */

#include "cinder/app/AppBasic.h"
#include "cinder/gl/gl.h"
#include "cinder/gl/Vbo.h"
#include "cinder/gl/GlslProg.h"
#include "cinder/Camera.h"
#include "cinder/params/Params.h"
#include "cinder/CinderMath.h"
#include "CinderOpenCv.h"

#define _USE_MATH_DEFINES
#include <math.h>

#include <boost/foreach.hpp>

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
    void drawDebug();
    void updateCalibration();
    
    void captureChessboardObservation();
    void calibrateCamera();
    
    Kinect kinect;
    
    ci::gl::VboMesh vbo;
    ci::gl::GlslProg shader;
    
    ci::CameraPersp camera;
    
    vector< Vec3f > cornerPoints;
    vector< vector < cv::Point3f > > chessboardObservations;
    vector< vector < cv::Point2f > > chessboardStates;
    
    
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
            ui.addParam("calibrate", &doCalibration);
            ui.addParam("capture", &doCapture);
            ui.addParam("run camera calibration", &doCalibrateCamera);
            
            ui.hide();
        }
        
        void draw ()
        {
            // avoid drawing anything at all
            if ( ui.isVisible() ) ui.draw();
        }
        
        params::InterfaceGl & operator() () { return ui; }
        
        
        ci::Vec2f cameraRotation, cameraRotationDragStart;
        float cameraDistance, cameraDistanceDragStart;
        bool doCalibration, doCapture, doCalibrateCamera;
    private:
        params::InterfaceGl ui;
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
            indices.push_back(x + y * kSize.x);
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
    Vec3f cameraPosition = Vec3f(ui.cameraDistance * cos(rotation.y) * sin(rotation.x),
                                 ui.cameraDistance * cos(rotation.x) * sin(rotation.y),
                                 ui.cameraDistance * cos(rotation.x));
    camera.lookAt(cameraPosition, Vec3f::zero(), Vec3f::yAxis());

    
    kinect.update();
    if ( kinect.getDevice()->isDepthDataNew() ) {
        Vec2i kSize = kinect.getDepthSize();
        vector< Vec3f > positions;
        positions.reserve(kSize.x * kSize.y);
        
        XnPoint3D *pointCloud = kinect.getDepthMapRealWorld();

        XnPoint3D *point;

        for ( gl::VboMesh::VertexIter it = vbo.mapVertexBuffer(); it.getIndex() < vbo.getNumIndices(); ++it ) {
            point = pointCloud + it.getIndex();
            // TODO: remove this scale, adjust camera
            it.setPosition(point->X * 0.5f, point->Y * 0.5f, -(point->Z * 0.5));
        }
        
        updateCalibration();
    }
}

void ProjectionMappingApp::updateCalibration()
{
    cornerPoints.clear();
    if ( !ui.doCalibration ) return;
    
    Surface8u surface = Surface8u(kinect.getColorImage());
    cv::Mat currentFrame = toOcvRef(surface);
    vector< cv::Point2f > corners;
    corners.reserve(9 * 6);
    cv::findChessboardCorners(currentFrame, cv::Size(9, 6), corners);
    
    XnPoint3D *pointCloud = kinect.getDepthMapRealWorld();
    XnPoint3D *point;
    for (vector< cv::Point2f >::iterator it = corners.begin() ; it < corners.end(); it++ ) {
        int index = kinect.getColorSize().x * (int)it->y + (int)it->x;
        point = pointCloud + index;
        // TODO: remove this scale, adjust camera
        cornerPoints.push_back(Vec3f(point->X * 0.5f, point->Y * 0.5f, -(point->Z * 0.5)));
    }
    
    captureChessboardObservation();
    calibrateCamera();
}

void ProjectionMappingApp::captureChessboardObservation() {
    if ( !ui.doCalibration || !ui.doCapture ) return;
    ui.doCapture = false;
    
    if ( cornerPoints.size() != 9 * 6 ) {
        console() << "NO CAPTCHA!!" << endl;
        return;
    }
    
    console() << "YUMMY!" << endl;
    
    vector< cv::Point3f > observation;
    vector< cv::Point2f > state;
    int i = 0;
    BOOST_FOREACH(Vec3f p1, cornerPoints)
    {
        observation.push_back(cv::Point3f(p1.x, p1.y, p1.z));
        state.push_back(cv::Point2f(i % 9, i / 9));
        i++;
    }
    chessboardObservations.push_back(observation);
    chessboardStates.push_back(state);
}

void ProjectionMappingApp::calibrateCamera()
{
    if ( !ui.doCalibration || !ui.doCalibrateCamera ) return;
    ui.doCalibrateCamera = false;
    
    cv::Mat cameraMatrix = cv::initCameraMatrix2D(chessboardObservations, chessboardStates, cv::Size(9, 6));
    
    cv::Mat distCoeffs;
    vector< cv::Mat > rvecs, tvecs;
    
    cv::calibrateCamera(chessboardObservations, chessboardStates, cv::Size(9, 6), cameraMatrix, distCoeffs, rvecs, tvecs, CV_CALIB_USE_INTRINSIC_GUESS);
    
    console() << cameraMatrix << endl;
    BOOST_FOREACH(cv::Mat v, rvecs)
    {
        console() << v << endl;
    }
    BOOST_FOREACH(cv::Mat v, tvecs)
    {
        console() << v << endl;
    }
    
    
    chessboardObservations.clear();
    chessboardStates.clear();
}

void ProjectionMappingApp::draw()
{
    gl::clear( Color( 0, 0, 0 ) );
    
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
    
    drawDebug();
}

void ProjectionMappingApp::drawDebug()
{
    ui.draw();

    if ( ui().isVisible() ) {
        gl::pushMatrices();
        gl::setMatrices(camera);
        {
            drawCoordinateFrame(250.f, 3.f, 2.f);
            
            BOOST_FOREACH(Vec3f p, cornerPoints)
            {
                gl::color(1, 0, 0);
                gl::drawSphere(p, 2);
            }
        }
        gl::popMatrices();
    }
}

void ProjectionMappingApp::keyDown(KeyEvent ev)
{
    if ( ev.getCode() == KeyEvent::KEY_SPACE ) {
        ui.cameraRotation = Vec2f::zero();
        ui.cameraDistance = 100.f;
    } else if ( ev.getCode() == KeyEvent::KEY_BACKSLASH ) {
        ui().isVisible() ? ui().hide() : ui().show();
    }
}

void ProjectionMappingApp::mouseDrag(MouseEvent ev)
{
    if ( ui().isVisible() ) return;
    
    if ( ev.isLeft() ) {
        ui.cameraRotation = ev.getPos().xy() - ui.cameraRotationDragStart;
    } else if ( ev.isRight() ) {
        ui.cameraDistance = ev.getPos().y - ui.cameraDistanceDragStart;
    }
}

void ProjectionMappingApp::mouseDown(MouseEvent ev)
{
    if ( ui().isVisible() ) return;
    
    ui.cameraRotationDragStart = ev.getPos().xy() - ui.cameraRotation;
    ui.cameraDistanceDragStart = ev.getPos().y - ui.cameraDistance;
}

void ProjectionMappingApp::mouseUp(MouseEvent ev)
{
}


CINDER_APP_BASIC( ProjectionMappingApp, RendererGl )
