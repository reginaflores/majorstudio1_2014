#include "ofApp.h"

//--------------------------------------------------------------
void ofApp::setup(){
    
    // Box2d sestup
    box2d.init();
    //box2d.setGravity(0, 30);
    box2d.createGround();
    box2d.setFPS(30.0);
    
//setting up default variables
    nearThreshold = 230; //depth for filtering the image
    farThreshold = 10; //depth for filtering the image
    minArea = 1000; //min area for contours
    maxArea = 70000; //max area for contours
    threshold = 15;
    persistence = 15;
    maxDistance = 32;
    radius = 4;
    vel = 10;
    
    //Filling the array of colors with color
    blobColors[0] = ofColor(255, 0, 0);
    blobColors[1] = ofColor(0, 255, 0);
    blobColors[2] = ofColor(0, 0, 255);
    blobColors[3] = ofColor(255, 255, 0);
    blobColors[4] = ofColor(255, 0, 255);
    blobColors[5] = ofColor(0, 255, 255);
    blobColors[6] = ofColor(255, 127, 0);
    blobColors[7] = ofColor(127, 0, 255);
    blobColors[8] = ofColor(0, 255, 127);
    blobColors[9] = ofColor(127, 255, 0);
    blobColors[10]= ofColor(255, 0, 127);
    blobColors[11]= ofColor(0, 127, 255);

    //Setting up Kinect
    kinect.setRegistration(true); //always have to do this - takes the depth image and alligns it with rgb image
    kinect.init(); //initialize
    kinect.open(); //makes kinect start
    grayImage.allocate(kinect.width, kinect.height); //textures that we are allocating to be the same size as the kinects image
    grayThreshNear.allocate(kinect.width, kinect.height); //we proces the kinects image and send this to ofxCV
    grayThreshFar.allocate(kinect.width, kinect.height); //for now just allocating
    
    //calibration file in the data folder
    kpt.loadCalibration("calibration.xml");
    
    //initializing 2nd window
    secondWindow.setup("main", ofGetScreenWidth(), 0, PROJECTOR_RESOLUTION_X, PROJECTOR_RESOLUTION_Y, true);


    // setup gui
    gui = new ofxUICanvas();
    gui->setHeight(800);
    gui->setName("parameters");
    gui->addLabel("kinect");
    gui->addSpacer();
    gui->addSlider("nearThresh", 0, 255, &nearThreshold);
    gui->addSlider("farThresh", 0, 255, &farThreshold);
    gui->addLabel("contours");
    gui->addSpacer();
    gui->addSlider("minArea", 0, 5000, &minArea);
    gui->addSlider("maxArea", 15000, 150000, &maxArea);
    gui->addSlider("threshold", 1, 100, &threshold);
    gui->addSlider("persistence", 1, 100, &persistence);
    gui->addSlider("maxDistance", 1, 100, &maxDistance);
    gui->addIntSlider("radius", 0, 50, &radius);
    gui->addIntSlider("velocity", 0, 20, &vel);
    
    gui->addToggle("FXA Pass", &fxaPassOn);
    gui->addToggle("Bloom", &bloomOn);
    gui->addToggle("dofPassOn", &dofPassOn);
    gui->addToggle("Kalaidascope", &kalOn);
    gui->addToggle("Noise Warp", &noiseWarpOn);
    gui->addToggle("Pixel", &pixelPassOn);
    gui->addToggle("Edge", &edgePassOn);
    gui->addToggle("Vertical", &vertTiltOn);
    gui->addToggle("God Ray", &godRayOn);
    
    gui-> autoSizeToFitWidgets();
    
    /////////////////////////////////////
    // Setup post-processing chain
    post.init(ofGetWidth(), ofGetHeight());
    
    fxaPass = post.createPass<FxaaPass>();
    bloomPass = post.createPass<BloomPass>();
    dofPass = post.createPass<DofPass>();
    kalPass = post.createPass<KaleidoscopePass>();
    noiseWarpPass = post.createPass<NoiseWarpPass>();
    pixelPass = post.createPass<PixelatePass>();
    edgePass = post.createPass<EdgePass>();
    verticalTilt = post.createPass<VerticalTiltShifPass>();
    godRaysPass = post.createPass<GodRaysPass>();

/////////////////////////////////////////////////////
    //creating blobs

  
    int numBlobs = 5;
    
    for(int i = 0; i < numBlobs; i++){
        
  
        vector <ofPoint> pts;
        
        for(int j = 0; j< 6; j++){
            //ofPoint p = ofPoint(ofRandom(0,secondWindow.getWidth()), ofRandom(0,secondWindow.getHeight()));
            //ofPoint p = ofPoint(ofRandom(0,100), ofRandom(1,10));
            ofPoint p = ofPoint(ofRandom(0,100), ofRandom(1,10));
            
            
            
            pts.push_back(p);
            
        }
        
        ofPtr<ofxBox2dPolygon> poly = ofPtr<ofxBox2dPolygon>(new ofxBox2dPolygon);
        poly.get()->addVertices(pts);
        poly.get()->setPhysics(1.0, 0.3, 0.3);
        poly.get()->create(box2d.getWorld());
        polyShapes.push_back(poly);
    
    
    
    }
    
    




}

//--------------------------------------------------------------
void ofApp::update(){
    
    //kinect updates
    kinect.update();
    
    //framerate of OF might be faster than kinect
    if(kinect.isFrameNew()) {
        // process kinect depth image - turns people into Blobs
        //turn this into a black and white image
        //black out anything that is past a certain depth
        grayImage.setFromPixels(kinect.getDepthPixels(), kinect.width, kinect.height);
        grayThreshNear = grayImage;
        grayThreshFar = grayImage;
        grayThreshNear.threshold(nearThreshold, true);
        grayThreshFar.threshold(farThreshold);
        //takes kinect depth image such that white is past a certain depth
        cvAnd(grayThreshNear.getCvImage(), grayThreshFar.getCvImage(), grayImage.getCvImage(), NULL);
        grayImage.flagImageChanged(); //finally becomes a black and white image
        

        // set contour tracker parameters
        //Blob finder - takes white blobs and gives you all the points on its perimiter
        contourFinder.setMinArea(minArea);
        contourFinder.setMaxArea(maxArea);
        contourFinder.setThreshold(threshold);
        contourFinder.getTracker().setPersistence(persistence);
        contourFinder.getTracker().setMaximumDistance(maxDistance);
        
        // determine found contours
        contourFinder.findContours(grayImage);
    }
    
    lines.clear();
    edges.clear();
    
    //counterFinder.size = number of people in field of view
    for(int i = 0; i < contourFinder.size(); i++) {
        //making points from the countour of the blob
        vector<cv::Point> points = contourFinder.getContour(i);
        
        lines.push_back(ofPolyline());
        
        for (int j=0; j<points.size(); j++){
            
            ofVec3f wp = kinect.getWorldCoordinateAt(points[j].x, points[j].y);
            ofVec2f pp = kpt.getProjectedPoint(wp);

            lines.back().addVertex(
                                   ofMap(pp.x, 1, 0, 0, secondWindow.getWidth()),//flipped 0,1 to 1,0 as hack to fix direction issue

                                   ofMap(pp.y, 0, 1, 0, secondWindow.getHeight())
            );
        
        }
        
        ofPtr <ofxBox2dEdge> edge = ofPtr<ofxBox2dEdge>(new ofxBox2dEdge);
        lines.back().simplify();//simplify helps to reduce the number of points because it can be computationally expensive
        
        for (int i=0; i<lines.back().size(); i++) {
            edge.get()->addVertex(lines.back()[i]);
        }
        
        //poly.setPhysics(1, .2, 1);  // uncomment this to see it fall!
        edge.get()->create(box2d.getWorld());
        edges.push_back(edge);
        
    }
    
    
    box2d.update();

    
    //ofxPostProcessing
    fxaPass->setEnabled(fxaPassOn);
    bloomPass->setEnabled(bloomOn);
    dofPass->setEnabled(dofPassOn);
    kalPass->setEnabled(kalOn);
    noiseWarpPass->setEnabled(noiseWarpOn);
    pixelPass->setEnabled(pixelPassOn);
    edgePass->setEnabled(edgePassOn);
    verticalTilt->setEnabled(vertTiltOn);
    godRaysPass->setEnabled(godRayOn);


}

//--------------------------------------------------------------
void ofApp::draw(){
    
    
    // GUI
    ofBackground(0);
    ofSetColor(255);
    ofPushMatrix();
    kinect.draw(0, 0);
    ofTranslate(640, 0);
    grayImage.draw(0, 0);
    ofTranslate(-640, 480);
    contourFinder.draw();
    ofTranslate(640, 0);
    ofPopMatrix();

    // MAIN WINDOW
    secondWindow.begin();
    post.begin();
    
    ofPushMatrix();
    ofTranslate(secondWindow.getWidth(), secondWindow.getHeight());
    ofRotate(180);
    
    ofBackground(0);

    
    //countor finder has a thing called the tracker - gives you data associated with the countour
    RectTracker& tracker = contourFinder.getTracker();
    
    for(int i = 0; i < contourFinder.size(); i++) {
        // get contour, label, center point, and age of contour
        vector<cv::Point> points = contourFinder.getContour(i);//getting the counttour points (green line around blob)
        int label = contourFinder.getLabel(i);//the label is keeping track of blobs over frames. trying to identify a single object from frame to frame
        ofPoint center = toOf(contourFinder.getCenter(i));//center point of the point array
        int age = tracker.getAge(label);//not really using this so much
        
        // map contour using calibration and draw to main window
        //drawing the countours as they are mapped
        ofBeginShape();
        ofFill();
        ofSetColor(blobColors[label % 12]); //mod operator gives a persistant color
        for (int j=0; j<points.size(); j++) { //loop through all points
            //world cordinate is a 3D cordinate inside the kinect depth cloud
            //this is a standard way of getting kinect info
            //pp = pixesl point, coresponding pixel between 0 and 1 for both x and y. Scale it to whatever the screen size is. ofMap scales it.
            ofVec3f wp = kinect.getWorldCoordinateAt(points[j].x, points[j].y);
            ofVec2f pp = kpt.getProjectedPoint(wp);
            ofVertex(
                     ofMap(pp.x, 1, 0, 0, secondWindow.getWidth()),//flipped 0,1 to 1,0 as hack to fix direction issue
                     ofMap(pp.y, 0, 1, 0, secondWindow.getHeight())
                     );
        }
        ofEndShape();
        
    }
    
    
    for(int i = 0; i < polyShapes.size(); i++){
        ofSetColor(0, 255, 0);
        polyShapes[i]->draw();
        ofSetColor(255);
    }
  
    ofPopMatrix();
    post.end();
    secondWindow.end();
    
    

}

//--------------------------------------------------------------
void ofApp::keyPressed(int key){

}

//--------------------------------------------------------------
void ofApp::keyReleased(int key){

}

//--------------------------------------------------------------
void ofApp::mouseMoved(int x, int y ){

}

//--------------------------------------------------------------
void ofApp::mouseDragged(int x, int y, int button){

}

//--------------------------------------------------------------
void ofApp::mousePressed(int x, int y, int button){

}

//--------------------------------------------------------------
void ofApp::mouseReleased(int x, int y, int button){

}

//--------------------------------------------------------------
void ofApp::windowResized(int w, int h){

}

//--------------------------------------------------------------
void ofApp::gotMessage(ofMessage msg){

}

//--------------------------------------------------------------
void ofApp::dragEvent(ofDragInfo dragInfo){ 

}
