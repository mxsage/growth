#pragma once

#include "ofMain.h"
#include <unordered_map>
#include <vector>

struct Parameters
{
    size_t maxNeighbors{12};

    double radius{3};
    double collisionFactor{1};

    double springLength{0};
    double bulgeFactor{0.1};
    double springFactor{0.1};
    double planarFactor{1};

    double dampening{0.1};

    double threshold{10};

    double foodExponent{1.0};

    bool longestAxis{true};

    int numThreads{7};
};

class Particle
{
  public:
    Particle() = default;
    ~Particle() = default;

    bool connectedTo(int i) const;
    void addLink(int i);
    void removeLink(int i);

    std::vector<int> links;

    int age{0};
    double food{0};
    double curvature{0};

    glm::vec3 position, delta, normal;

    int collisions{0};

    int index;

    bool frozen{false};
};

class Simulation
{
  public:
    Simulation() = default;
    ~Simulation() = default;

    void initialize();

    void setParameters(const Parameters &p);

    void tick();

    ofVboMesh getMesh();

  private:
    Parameters parameters;
    void addCollisionForce();
    void addBulgeForce(Particle &p);
    void addSpringForce(Particle &p);
    void addPlanarForce(Particle &p);

    void integrateParticles();

    void calculateNormals();
    int getNext(const Particle &p, int i);
    int getNext(const Particle &p, int i, int j);
    void orderNeighbors(Particle &p);
    void split(Particle &p);
    void setLinks(Particle &parent, Particle &baby);
    int findShortestAxis(const Particle &p);
    int findLongestAxis(const Particle &p);
    void setPositions(Particle &parent, Particle &baby);
    void calculateNormal(Particle &p);

    unsigned simFrame{0};
    std::unordered_map<int, Particle> cells;
};
