
//
//  Particle.h
//  12_Final_Project_2
//
//  Created by Regina Flores on 11/28/14.
//
//

#pragma once
#include "ofMain.h"

class Particle{
public:
    
    Particle();
    
    void init(int _boxWidth,
              int _boxHeight,
              int _boxDepth,
              vector<float> _noiseArray,
              float _minVel,
              float _maxVel,
              int _opacity,
              int _length);
    void draw();
    void update(vector<Particle> &particlesA);
    void bounding();
    void randomPos();
    void setNoiseArray(vector<float> _noiseArray);
    void setVelocity(float _minVel, float _maxVel);
    void setOpacity(int _opacity);
    void setLength(int _length);
    
    //testing flocking
    ofPoint cohesion(vector<Particle> &particlesA);
    ofPoint separation(vector<Particle> &particlesA);
    ofPoint allignment(vector<Particle> &particlesA);
    
    //position and velocity variables
    ofPoint pos;
    vector<ofPoint> posArray;
    ofPoint vel;
    
    
    //passed variables
    int boxWidth;
    int boxHeight;
    int boxDepth;
    float minVel;
    float maxVel;
    int opacity;
    int length;

    //random array to varry noise function
    vector<float> noiseArray;

    
    
};
