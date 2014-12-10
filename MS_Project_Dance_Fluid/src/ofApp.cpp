#include "ofApp.h"

//--------------------------------------------------------------
void ofApp::setup(){
    
    //Used for Screen Effect code
   // ofToggleFullscreen();
  //
    
    
    
    // Box2d sestup
    box2d.init();
    box2d.setGravity(0, 30);
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
    maxSize = 10;
    rendorScale = 10.0;
    flameNum = 4;
    velDis = 0.99;
    dis = 0.99;
    
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
    gui->addIntSlider("Max Size", 3, 50, &maxSize);
    gui->addSlider("Rendor Scale", 0.0, 100.0, &rendorScale);
    gui->addIntSlider("# of Flames", 1, 6, &flameNum);
    gui->addSlider("Velocity Dis", 0.0, 5.0, &velDis);
    gui->addSlider("Dissipation", 0.0, 5.0, &dis);
    
    fbo.allocate(ofGetWidth(), ofGetHeight());
    
    shader.load("noise.vert","noise.frag");
    
    
    
    ////////////////////////////
    //Fluid
    //
    ofEnableAlphaBlending();
    ofSetCircleResolution(100);
    
    width = secondWindow.getWidth();
    height = secondWindow.getHeight();
    
    // Initial Allocation
    //
    fluid.allocate(width, height, 0.5);
    
    // Seting the gravity set up & injecting the background image
    //
    fluid.dissipation = dis;
    fluid.velocityDissipation = velDis;
    
    fluid.setGravity(ofVec2f(0.0,0.0));
    //    fluid.setGravity(ofVec2f(0.0,0.0098));
    
    //  Set obstacle
    //
    fluid.begin();
    ofSetColor(0,0);
    ofSetColor(255);
    
    //ofCircle(ofMap(i, 0, flameNum, 0, width), height*0.35, 40);
    ofCircle(width*0.5, height*0.35, 40);

    fluid.end();
    fluid.setUseObstacles(false);
    
    // Adding constant forces
    //
    for(int i=0; i < flameNum; i++){
        fluid.addConstantForce(ofPoint(ofMap(i+0.5, 0, flameNum, 0, width),height*0.85), ofPoint(0,-2), ofFloatColor(0.5,0.1,0.0), 10.f);
    }
    ofSetWindowShape(width, height);
    
    
    //setting the number of people in the screen to 10
    for(int i = 0; i < 10; i++){
        vector<ofVec2f> newVector;
        newVector.resize(50);
        fluidPoints.push_back(newVector);
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
    
    //box2d update
    
    lines.clear();
    edges.clear();
    
    
    // add some circles every so often
   // if((int)ofRandom(0, 5) == 0) {
        ofPtr<ofxBox2dCircle> c = ofPtr<ofxBox2dCircle>(new ofxBox2dCircle);
        c.get()->setPhysics(0.2, 0.2, 0.002);
        
        //here we can change the look of the balls falling
        //c.get()->setup(box2d.getWorld(), ofRandom(0, secondWindow.getWidth()), -20, ofRandom(3, 10));
        c.get()->setup(box2d.getWorld(), ofRandom(0, secondWindow.getWidth()), -20, 4);
        c.get()->setVelocity(0, 10
                             ); // shoot them down!
        circles.push_back(c);
   // }
   
    //counterFinder.size = number of people in field of view
    for(int i = 0; i < contourFinder.size(); i++) {
        //making points from the countour of the blob
        vector<cv::Point> points = contourFinder.getContour(i);
        
        lines.push_back(ofPolyline());
        
        
        //keep track of fluid points
        //number of points is maxSize
        //loop through points array but skip them so we get maxSize evenly spaced points
        for (int j=0; j<maxSize; j++){
        
            int index = ofMap(j, 0, maxSize, 0, points.size()-1); //want evenly spaced points around contour
            
            ofVec3f wp = kinect.getWorldCoordinateAt(points[index].x, points[index].y);
            ofVec2f pp = kpt.getProjectedPoint(wp);
            
            pp.x = pp.x*secondWindow.getWidth();
            pp.y = pp.y*secondWindow.getHeight();
            
            
            ofPoint m = pp;
            
            ofPoint d = (m - fluidPoints[i][j])*rendorScale;
            
            ofPoint cc = ofPoint(640*0.5, 480*0.5) - m;
            cc.normalize();
            fluid.addTemporalForce(m, d, ofFloatColor(cc.x,cc.y,0.5)*sin(ofGetElapsedTimef()),3.0f);

            fluidPoints[i][j] = pp;
            
        }
        
        for (int j=0; j<points.size(); j++){
            
            ofVec3f wp = kinect.getWorldCoordinateAt(points[j].x, points[j].y);
            ofVec2f pp = kpt.getProjectedPoint(wp);
            
            
            
            lines.back().addVertex(
                                   ofMap(pp.x, 0, 1, 0, secondWindow.getWidth()),
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
    
    fluid.dissipation = dis;
    fluid.velocityDissipation = velDis;
    
    box2d.update();
    fluid.update();
    

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

    
    //Alpha masking code
    fbo.begin();
    
    ofClear(0, 0);
    ofSetColor(0, 255);
    ofRect(0, 0, fbo.getWidth(), fbo.getHeight());
    ofEnableBlendMode(OF_BLENDMODE_SUBTRACT);
    ofSetColor(255);
    
    
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
                     ofMap(pp.x, 0, 1, 0, secondWindow.getWidth()),
                     ofMap(pp.y, 0, 1, 0, secondWindow.getHeight())
                     );
        }
        ofEndShape();

////testing only
//        for(int j=0; j<maxSize; j++){
//            ofSetColor(0,255,0);
//            ofCircle(fluidPoints[i][j].x, fluidPoints[i][j].y, 30);
//            
//        }
//        ofSetColor(255);
    }

    ofDisableBlendMode();
    
    fbo.end();
    
    // MAIN WINDOW
    secondWindow.begin();
    
    ofEnableBlendMode(OF_BLENDMODE_ALPHA);
    
    
        
        shader.begin();
        shader.setUniform2f("resolution", ofGetWidth(), ofGetHeight());
        ofRect(0, 0, ofGetWidth(), ofGetHeight());
        
        shader.end();
    
    //testing code
    //int num = 100;
    //for(int i = 0; i < num; i++){
        //ofSetColor(ofRandom(255), ofRandom(255), ofRandom(255));
        //ofCircle(ofRandom(ofGetWidth()), ofRandom(ofGetHeight()), 10);
    //}
    
    ofSetColor(255);
    fbo.draw(0,0);
    ofDisableBlendMode();
    
    //box2d draw
    // some circles :)
    for (int i=0; i<circles.size(); i++) {
        ofFill();
        ofSetColor(255);
        //ofSetHexColor(0xc0dd3b);
        circles[i].get()->draw();
    }

   //ofBackgroundGradient(ofColor::gray, ofColor::black, OF_GRADIENT_LINEAR);
    
    
    
   fluid.draw();
    
    secondWindow.end();
  
}

//--------------------------------------------------------------
void ofApp::keyPressed(int key){
    
    if( key == 'g'){
    
        gui->toggleVisible();
    }

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
