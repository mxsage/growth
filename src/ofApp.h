#pragma once

#include "ofMain.h"
#include "ofxGui.h"

#include "simulation.h"

class ofApp : public ofBaseApp {
public:
  void setup();
  void update();
  void draw();

  void keyPressed(int key);
  void keyReleased(int key);
  void mouseMoved(int x, int y);
  void mouseDragged(int x, int y, int button);
  void mousePressed(int x, int y, int button);
  void mouseReleased(int x, int y, int button);
  void mouseEntered(int x, int y);
  void mouseExited(int x, int y);
  void windowResized(int w, int h);
  void dragEvent(ofDragInfo dragInfo);
  void gotMessage(ofMessage msg);

  Simulation sim;

  ofEasyCam cam;

  ofLight light;
  ofColor lightColor;
  float colorHue;
  ofColor materialColor;

  ofMaterial material;

  bool updateSimulation{true};

  ofVboMesh m;

  ofParameter<int> maxNeighbors;
  ofParameter<float> radius;
  ofParameter<float> collisionFactor;
  ofParameter<float> springLength;
  ofParameter<float> bulgeFactor;
  ofParameter<float> springFactor;
  ofParameter<float> planarFactor;
  ofParameter<float> dampening;
  ofParameter<float> threshold;

  ofParameter<bool> longestAxis;

  ofxPanel gui;
};
