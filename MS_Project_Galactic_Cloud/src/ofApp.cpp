#include "ofApp.h"

//--------------------------------------------------------------
void ofApp::setup(){
    
    ofToggleFullscreen();
    
    ofBackground(0);
    
    //initialize variables
    num = 500;
    boxWidth = 600;
    boxHeight = 400;
    boxDepth = 500;
    noiseFactor = 0.001;
    minVel = -2;
    maxVel = 2;
    opacity = 40;
    length = 80;
    speedRotate = 0.16;
    
    //initialize noise array
    for(int i = 0; i < 9; i++){
        noiseArray.push_back(ofRandom(0,noiseFactor));
    }
    
    //initialize particles array
    for(int i =0; i < num; i++){
        Particle p;
        particles.push_back(p);
        particles[i].init(boxWidth, boxHeight, boxDepth, noiseArray, minVel, maxVel, opacity, length);
        }
    
    
    //initializing 2nd window
    secondWindow.setup("main", ofGetScreenWidth(), 0, PROJECTOR_RESOLUTION_X, PROJECTOR_RESOLUTION_Y, true);
    
    
    //////////////////////////
    //Gui Setup
    gui = new ofxUICanvas();
    
    gui->addLabel("Adjust the fields");
    gui->addLabel("and press spacebar");
    gui->addLabel("to update");

    
    gui-> addIntSlider("Number of Particles", 10, 2000, &num);
    gui-> addSlider("Noise Factor", 0, 0.05, &noiseFactor);
    gui-> addSlider("Min Velocity", -20, 0, &minVel);
    gui-> addSlider("Max Velocity", 0, 20, &maxVel);
    gui-> addIntSlider("Opacity", 0, 255, &opacity);
    gui-> addIntSlider("String Length", 3, 100, &length);
    gui->addSlider("Rotation", 0, 0.5, &speedRotate);
    
    gui->addToggle("Fullscreen", &toggleOn);
    
    gui-> autoSizeToFitWidgets();
    /////////////////////////

}

//--------------------------------------------------------------
void ofApp::update(){
    
    //check if gui num is bigger than size
    while(particles.size()<num){
        Particle p;
        p.init(boxWidth, boxHeight, boxDepth, noiseArray, minVel, maxVel, opacity, length);
        particles.push_back(p);
        //particles[particles.size()-1].init(boxWidth, boxHeight, boxDepth, noiseArray);
        
        }
    
    for(int i =0; i < num; i++){
        particles[i].update(particles);
        particles[i].bounding();

    }


}

//--------------------------------------------------------------
void ofApp::refresh(){
    
    //clear noise array and put in a new set of numbers from the gui
    
    noiseArray.clear();
    
    for(int i = 0; i < 9; i++){
        noiseArray.push_back(ofRandom(0,noiseFactor));
        
    }
    
    for(int i = 0; i < num; i++){
        particles[i].setNoiseArray(noiseArray);
        particles[i].setVelocity(minVel, maxVel);
        particles[i].setOpacity(opacity);
        particles[i].setLength(length);
    }
    

}
//--------------------------------------------------------------
void ofApp::draw(){
    
    // MAIN WINDOW
    secondWindow.begin();

    ofBackground(0);
    cam.begin();
    
    
    //rotate the cam
    ofRotate(ofGetFrameNum()*speedRotate, 0, 1, 0);
    
    
//    float x = ofSignedNoise(speedRotate*ofGetFrameNum(), 5);
//    float y = ofSignedNoise(speedRotate*ofGetFrameNum(), 15);
//    float z = ofSignedNoise(speedRotate*ofGetFrameNum(), 25);
//    ofRotate(ofGetFrameNum(), x, y, z);

    ofNoFill();
    //ofDrawBox(0, 0, 0, 200, 200, 200);
    //ofSetColor(255,255,255,100);
    //ofDrawBox(0, 0, 0, boxWidth, boxHeight, boxDepth);//3D box centered in screen

    for(int i =0; i < num; i++){
        //ofSetColor(0);
        particles[i].draw();
    }
    
    cam.end();
    secondWindow.end();
}

//--------------------------------------------------------------
void ofApp::keyPressed(int key){
    
    if(key == ' '){
        refresh();
    }
    
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

//--------------------------------------------------------------
void ofApp::guiEvent(ofxUIEventArgs &e){
    
    if(e.getName()=="Fullscreen"){
        ofToggleFullscreen();
    }
}
