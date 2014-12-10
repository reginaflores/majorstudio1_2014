#pragma once

#include "ofMain.h"
#include "ofxOpenCv.h"
#include "ofxCv.h"
#include "ofxKinect.h"
#include "ofxSecondWindow.h"
#include "ofxKinectProjectorToolkit.h"
#include "ofxUI.h"
#include "ofxBox2d.h"

// this must match the display resolution of your projector
#define PROJECTOR_RESOLUTION_X 1024
#define PROJECTOR_RESOLUTION_Y 768


//This is a C++ thing that has to do with different libraries using the same names - this is to resolve those conflicts
using namespace ofxCv;
using namespace cv;


class ofApp : public ofBaseApp{

	public:
		void setup();
		void update();
		void draw();

		void keyPressed(int key);
		void keyReleased(int key);
		void mouseMoved(int x, int y );
		void mouseDragged(int x, int y, int button);
		void mousePressed(int x, int y, int button);
		void mouseReleased(int x, int y, int button);
		void windowResized(int w, int h);
		void dragEvent(ofDragInfo dragInfo);
		void gotMessage(ofMessage msg);
    
    void guiEvent(ofxUIEventArgs &e);

		
    
    ofxCv::ContourFinder        contourFinder;
    ofxKinectProjectorToolkit   kpt;
    ofxKinect                   kinect;
    
    ofxCvGrayscaleImage         bgImage;
    ofxCvGrayscaleImage         grayImage;
    ofxCvGrayscaleImage         grayThreshNear;
    ofxCvGrayscaleImage         grayThreshFar;
    
    ofxUICanvas                 *gui;
    ofxSecondWindow             secondWindow;

    ofColor                     blobColors[12];
    
    float                       nearThreshold;
    float                       farThreshold;
    float                       minArea;
    float                       maxArea;
    float                       threshold;
    float                       persistence;
    float                       maxDistance;

    int                         num;
    vector <ofPolyline>                 lines;
    ofxBox2d                            box2d;
    vector <ofPtr<ofxBox2dCircle> >		circles;
    vector <ofPtr<ofxBox2dEdge> >       edges;
    
    ofFbo fbo;
    ofShader shader;
    
    //fullscreen
    bool toggleOn;
    
    ofImage image;

};
