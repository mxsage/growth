#include "ofApp.h"

void ofApp::setup() {
    ofBackground(0);
    sim.initialize();

    ofSetSmoothLighting(true);

    gui.setup();
    gui.add(maxNeighbors.set("maxNeighbors", 12, 4, 30));
    gui.add(radius.set("radius", 3, .01, 10));
    gui.add(springLength.set("springLength", 0, 0, 10));
    gui.add(collisionFactor.set("collisionFactor", 1, 0, 1));
    gui.add(bulgeFactor.set("bulgeFactor", 0.1, 0, 1));
    gui.add(springFactor.set("springFactor", 0.1, 0, 1));
    gui.add(planarFactor.set("planarFactor", 1, 0, 1));
    gui.add(dampening.set("dampening", 0.1, 0, 1));
    gui.add(threshold.set("threshold", 10, 0, 300));
    gui.add(longestAxis.set("longestAxis", true));
}

void ofApp::update() {
    Parameters p;
    p.maxNeighbors = maxNeighbors;
    p.radius = radius;
    p.collisionFactor = collisionFactor;
    p.bulgeFactor = bulgeFactor;
    p.springFactor = springFactor;
    p.springLength = springLength;
    p.planarFactor = planarFactor;
    p.dampening = dampening;
    p.longestAxis = longestAxis;
    p.threshold = threshold;

    sim.setParameters(p);
    if (updateSimulation) {
        sim.tick();
        m = sim.getMesh();
    }
    ofSetWindowTitle(ofToString(ofGetFrameRate()));
}

void ofApp::draw() {
    ofEnableLighting();
    cam.begin();
    ofEnableDepthTest();
    m.draw();
    ofDisableDepthTest();
    cam.end();
    gui.draw();
}

void ofApp::keyPressed(int key) {
    if (key == 'r') {
        sim.initialize();
    }
    if (key == ' ') {
        updateSimulation = !updateSimulation;
    }
    if (key == 's') {
        m.save(ofToDataPath("mesh.ply"));
    }
}

void ofApp::keyReleased(int key) {}
void ofApp::mouseMoved(int x, int y) {}
void ofApp::mouseDragged(int x, int y, int button) {}
void ofApp::mousePressed(int x, int y, int button) {}
void ofApp::mouseReleased(int x, int y, int button) {}
void ofApp::mouseEntered(int x, int y) {}
void ofApp::mouseExited(int x, int y) {}
void ofApp::windowResized(int w, int h) {}
void ofApp::gotMessage(ofMessage msg) {}
void ofApp::dragEvent(ofDragInfo dragInfo) {}
