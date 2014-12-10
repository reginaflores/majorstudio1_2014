//
//  Particle.cpp
//  12_Final_Project_2
//
//  Created by Regina Flores on 11/28/14.
//
//

#include "Particle.h"


Particle::Particle(){
    
    
}
//--------------------------------------------------------------

void Particle::init( int _boxWidth, int _boxHeight, int _boxDepth, vector<float> _noiseArray, float _minVel, float _maxVel, int _opacity, int _length){

    boxWidth = _boxWidth;
    boxHeight = _boxHeight;
    boxDepth = _boxDepth;
    
    noiseArray = _noiseArray;
    
    minVel = _minVel;
    maxVel = _maxVel;
    opacity = _opacity;
    
    pos.x = ofRandom(-boxWidth/2, boxWidth/2);
    pos.y = ofRandom(-boxHeight/2, boxHeight/2);
    pos.z = ofRandom(-boxDepth/2, boxDepth/2);
    
    vel.x = ofRandom(-2,2);
    vel.y = ofRandom(-2,2);
    vel.z = ofRandom(-2,2);
    
}
//--------------------------------------------------------------

void Particle::draw(){

    ofBeginShape();
    
    /////////////////////////
    //this gives the strings color or B/W
    //ofSetColor(ofRandom(0,255),ofRandom(0,255),ofRandom(0,255), opacity);
    ofSetColor(255,255,255, opacity);
    /////////////////////////

    //Build the strings
    for(int i =0; i < posArray.size(); i++){

        ofCurveVertex(posArray[i].x, posArray[i].y, posArray[i].z);
         //ofCircle(posArray[i].x, posArray[i].y, posArray[i].z, 2);
    }
    
    ofEndShape();
}

//--------------------------------------------------------------

void Particle::update(vector<Particle> &particlesA){
    
    vel.x = ofMap(ofNoise(noiseArray[0]*pos.x, noiseArray[1]*pos.y, noiseArray[2]*pos.z, 30), 0, 1, minVel, maxVel);
    vel.y = ofMap(ofNoise(noiseArray[3]*pos.x, noiseArray[4]*pos.y, noiseArray[5]*pos.z, 100), 0, 1, minVel, maxVel);
    vel.z = ofMap(ofNoise(noiseArray[6]*pos.x, noiseArray[7]*pos.y, noiseArray[8]*pos.z, 3), 0, 1, minVel, maxVel);

    /////////
    ofPoint v1 = cohesion(particlesA);
    ofPoint v2 = separation(particlesA);
    ofPoint v3 = allignment(particlesA);
    
    vel = v1+v2+v3;
    pos = pos + vel;
    
    
    posArray.push_back(pos);
    
    if(posArray.size()>length){
        posArray.erase(posArray.begin()); //erases the oldest element of the array such that size is always 8
    }
    
}
//--------------------------------------------------------------
//--------------------------------------------------------------
///Using my flocking algorithm as a test///

ofPoint Particle::cohesion(vector<Particle> &particlesA){
    
    //RULE 1
    //finding the center of mass of the system
    ofPoint centerMass;
    for(int i = 0; i < particlesA.size(); i++){
        //determining if the particle in the loop is itself
        //to avoid itself
        if (&particlesA[i] == this) {
            continue;
        }
        
        centerMass += particlesA[i].pos;
        
    }
    
    centerMass = centerMass/(particlesA.size()-1);
    
    ofPoint cohesion_velocity;
    cohesion_velocity = (centerMass-pos)/100.0f;
    return cohesion_velocity;
}

ofPoint Particle::separation(vector<Particle> &particlesA){
    //RULE 2
    //particles keep a small distance from each other
    
    ofPoint displace;
    for(int i = 0; i < particlesA.size(); i++){
        //determining if the particle in the loop is itself
        //to avoid itself
        if (&particlesA[i] == this) {
            continue;
        }
        
        // distance between me and particles[i]
        float dist = pow(particlesA[i].pos.x - pos.x, 2) + pow(particlesA[i].pos.y - pos.y, 2) + pow(particlesA[i].pos.z - pos.z, 2);
        dist = sqrt(dist);
        
        if (dist < 20.0) {
            displace  = displace - (particlesA[i].pos - pos);
        }
        
    }
    
    ofPoint separation_velocity;
    separation_velocity = displace;
    return separation_velocity;
}

ofPoint Particle::allignment(vector<Particle> &particlesA){
    
    //RULE 3
    //match velocity
    
    ofPoint aveVel;
    for(int i = 0; i < particlesA.size(); i++){
        //determining if the particle in the loop is itself
        //to avoid itself
        if (&particlesA[i] == this) {
            continue;
        }
        
        aveVel += particlesA[i].vel;
        
    }
    
    aveVel = aveVel/(particlesA.size()-1);
    
    ofPoint allignment_velocity;
    allignment_velocity = (aveVel - vel)/8.0f;
    return allignment_velocity;
    
}

//--------------------------------------------------------------
//--------------------------------------------------------------

void Particle::randomPos(){
//this function is used to simulate continuous movement of the particles
//without this function the particle system has disjoint behavior as
//it moves through the bounding box
    
    pos.x = ofRandom(-boxWidth/2, boxWidth/2);
    pos.y = ofRandom(-boxHeight/2, boxHeight/2);
    pos.z = ofRandom(-boxDepth/2, boxDepth/2);
    
}

//--------------------------------------------------------------
void Particle::setNoiseArray( vector<float> _noiseArray ){
    //using the GUI and passing the noise parameters that user modifies
    noiseArray = _noiseArray;
    
}
//--------------------------------------------------------------

void Particle::setVelocity(float _minVel, float _maxVel){
    //using the GUI and passing the velocity parameters that user modifies
    minVel = _minVel;
    maxVel = _maxVel;

}
//--------------------------------------------------------------
void Particle::setOpacity(int _opacity){
    //using the GUI and passing the opacity parameters that user modifies
    opacity = _opacity;
}
//--------------------------------------------------------------
void Particle::setLength(int _length){
    //using the GUI and passing the string parameters that user modifies
    length = _length;
}
//--------------------------------------------------------------

void Particle::bounding(){
    
    //bounding the particles to be inside a 3D box

    if(pos.y > boxHeight/2){
        pos.y = -boxHeight/2;
        posArray.clear(); //clearing the array in order to allow for more seamless trasition as particles move through the boundaries
        randomPos();
    
    }
    if(pos.y < -boxHeight/2){
        pos.y = boxHeight/2;
        posArray.clear();
        randomPos();

    }
    
    if(pos.x > boxWidth/2){
        pos.x = -boxWidth/2;
        posArray.clear();
        randomPos();

    }
    if(pos.x < -boxWidth/2){
        pos.x = boxWidth/2;
        posArray.clear();
        randomPos();

    }
    if(pos.z < -boxDepth/2){
        pos.z = boxDepth/2;
        posArray.clear();
        randomPos();

    }
    if(pos.z > boxDepth/2){
        pos.z = -boxDepth/2;
        posArray.clear();
        randomPos();

    }


}