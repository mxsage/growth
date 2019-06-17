#include "ofApp.h"
#include "ofMain.h"

int main() {
  ofGLFWWindowSettings settings;
  settings.setGLVersion(4, 1);
  settings.setSize(800, 800);
  settings.numSamples = 4;
  settings.windowMode = ofWindowMode::OF_WINDOW;
  ofCreateWindow(settings);
  ofRunApp(new ofApp());
}
