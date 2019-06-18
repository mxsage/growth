#include "simulation.h"

#include "kdtree.h"

bool Particle::connectedTo(int i) const
{
    return std::find(links.begin(), links.end(), i) != links.end();
}

void Particle::addLink(int i)
{
    if (!connectedTo(i))
    {
        links.push_back(i);
    }
}

void Particle::removeLink(int i)
{
    auto e = std::find(links.begin(), links.end(), i);

    if (e == links.end())
    {
        std::cout << "Error! Removing link that isn't there!" << std::endl;
        return;
    }

    links.erase(e);
}

int Simulation::findShortestAxis(const Particle &p)
{
    const std::vector<int> &links = p.links;

    double minLength{0};
    int shortestIndex{-1};
    for (size_t i = 0; i < links.size(); i++)
    {
        double distance = glm::length((p.position - cells[links[i]].position));
        size_t opposite = (i + (links.size() / 2)) % links.size();
        distance += glm::length((p.position - cells[links[opposite]].position));
        if (distance < minLength || i == 0)
        {
            minLength = distance;
            shortestIndex = (int)i;
        }
    }
    return shortestIndex;
}

int Simulation::findLongestAxis(const Particle &p)
{
    const std::vector<int> &links = p.links;

    double maxLength = std::numeric_limits<double>::max();
    int longestIndex{-1};
    for (size_t i = 0; i < links.size(); i++)
    {
        double distance = glm::length((p.position - cells[links[i]].position));
        size_t opposite = (i + (links.size() / 2)) % links.size();
        distance += glm::length((p.position - cells[links[opposite]].position));
        if (distance > maxLength || i == 0)
        {
            maxLength = distance;
            longestIndex = (int)i;
        }
    }
    return longestIndex;
}

void Simulation::setLinks(Particle &parent, Particle &baby)
{
    orderNeighbors(parent);

    int firstIndex;
    if (parameters.longestAxis)
    {
        firstIndex = findShortestAxis(parent);
    }
    else
    {
        firstIndex = findLongestAxis(parent);
    }

    std::vector<int> &links = parent.links;
    std::rotate(links.begin(), links.begin() + firstIndex, links.end());

    size_t originalNumLinks = links.size();

    std::vector<int> removable;
    removable.reserve(links.size() / 2);
    // baby.links.clear();

    std::copy(links.begin(), links.begin() + ((originalNumLinks / 2) + 1),
              std::back_inserter(removable));

    for (int i = 0; i < removable.size(); ++i)
    {
        Particle &r = cells[removable[i]];
        baby.addLink(r.index);
        r.addLink(baby.index);

        if ((r.index != removable[0]) &&
            (r.index != removable[originalNumLinks / 2]))
        {
            parent.removeLink(r.index);
            r.removeLink(parent.index);
        }
    }

    parent.addLink(baby.index);
    baby.addLink(parent.index);
}

void Simulation::setPositions(Particle &parent, Particle &baby)
{
    glm::vec3 babyAverage = parent.position;
    for (size_t i = 0; i < baby.links.size(); ++i)
    {
        babyAverage += cells[baby.links[i]].position;
    }

    babyAverage /= baby.links.size() + 1;

    glm::vec3 parentAverage = parent.position;
    for (size_t i = 0; i < parent.links.size(); ++i)
    {
        parentAverage += cells[parent.links[i]].position;
    }
    parentAverage /= parent.links.size() + 1;

    // set positions
    baby.position = babyAverage;
    parent.position = parentAverage;
}

void Simulation::split(Particle &p)
{
    int index = (int)cells.size();
    cells[index] = Particle();
    Particle &baby = cells[index];

    baby.normal = p.normal;
    baby.position = p.position;
    baby.index = index;
    baby.food = 1;
    setLinks(p, baby);

    setPositions(p, baby);

    calculateNormal(p);
    calculateNormal(baby);

    baby.food = 0;
    p.food = 0;
}

std::unordered_map<int, Particle> generateCells(const ofMesh &m)
{
    std::unordered_map<int, Particle> cells;

    auto connect = [&](const int i, const int j) {
        if (!cells[i].connectedTo(j))
        {
            cells[i].addLink(j);
        }
        if (!cells[j].connectedTo(i))
        {
            cells[j].addLink(i);
        }
    };

    for (int i = 0; i < m.getNumVertices(); ++i)
    {
        Particle p;
        p.position = m.getVertex(i);
        p.normal = glm::normalize(m.getNormal(i));
        p.index = i;
        cells[i] = p;
    }

    for (int i = 0; i < m.getNumIndices(); i += 3)
    {
        connect(m.getIndex(i), m.getIndex(i + 1));
        connect(m.getIndex(i), m.getIndex(i + 2));
        connect(m.getIndex(i + 1), m.getIndex(i + 2));
    }

    return cells;
}

int Simulation::getNext(const Particle &p, int i) { return getNext(p, i, i); }

int Simulation::getNext(const Particle &p, int previousIndex, int currentIndex)
{
    const Particle &current = cells[currentIndex];
    for (int i = 0; i < p.links.size(); i++)
    {
        for (int j = 0; j < current.links.size(); j++)
        {
            if ((p.links[i] == current.links[j]) &&
                (p.links[i] != previousIndex) && (p.links[i] != currentIndex))
            {
                return p.links[i];
            }
        }
    }

    return -1;
}

void Simulation::orderNeighbors(Particle &p)
{
    if (p.links.size() < 3)
    {
        return;
    }

    std::vector<int> orderedLinks;
    orderedLinks.reserve(p.links.size());

    orderedLinks.push_back(p.links[0]);
    orderedLinks.push_back(getNext(p, p.links[0]));

    for (size_t i = 2; i < p.links.size(); i++)
    {
        orderedLinks.push_back(
            getNext(p, orderedLinks[i - 2], orderedLinks[i - 1]));
    }

    p.links = orderedLinks;
}

void Simulation::calculateNormal(Particle &p)
{
    orderNeighbors(p);

    glm::vec3 newNormal;

    for (size_t i = 0; i < p.links.size(); i++)
    {
        glm::vec3 c = cells[p.links[i]].position;
        glm::vec3 d = cells[p.links[(i + 1) % p.links.size()]].position;
        newNormal += glm::cross(d, c);
    }

    // area = newNormal.squaredNorm();
    newNormal = glm::normalize(newNormal);

    if (glm::dot(newNormal, p.normal) >= 0)
    {
        p.normal = newNormal;
    }
    else
    {
        p.normal = -newNormal;
        std::reverse(p.links.begin(), p.links.end());
    }
}

void Simulation::initialize()
{
    simFrame = 0;

    ofMesh m;
    m.load(ofToDataPath("models/icosahedron.ply"));

    cells = generateCells(m);
}

void Simulation::integrateParticles()
{
    for (auto &iter : cells)
    {
        Particle &p = iter.second;
        p.position += p.delta * parameters.dampening;
        p.delta = glm::vec3(0, 0, 0);
    }
}

void Simulation::addCollisionForce()
{
    // build tree
    using Point = std::array<double, 3>;
    using Tree = jk::tree::KDTree<int, 3>;

    Tree tree;
    for (const auto &iter : cells)
    {
        const Particle &p = iter.second;
        tree.addPoint(Point{{p.position.x, p.position.y, p.position.z}},
                      iter.first, false);
    }
    tree.splitOutstanding();

    const double radiusSquared = std::pow(parameters.radius, 2);

    auto collisionDetection = [&](std::vector<int> indexes) {
        for (int i : indexes)
        {
            Particle &p = cells[i];
            p.collisions = 0;

            for (auto neighbor : tree.searchCapacityLimitedBall(
                     Point{{p.position.x, p.position.y, p.position.z}},
                     radiusSquared, parameters.maxNeighbors))
            {
                const Particle &q = cells[neighbor.payload];
                glm::vec3 displacement = p.position - q.position;

                float distanceSquared = displacement.x * displacement.x +
                                        displacement.y * displacement.y +
                                        displacement.z * displacement.z;

                if ((i != neighbor.payload) &&
                    (!p.connectedTo(neighbor.payload)))
                {
                    displacement = glm::normalize(displacement);
                    displacement *= (radiusSquared - distanceSquared) /
                                    radiusSquared; // TODO sqrt here?
                    p.delta += displacement * parameters.collisionFactor;
                    p.collisions++;
                }
            }
        }
    };

    std::vector<std::thread> threads;

    int numCellsPerThread = (int)cells.size() / parameters.numThreads;

    std::vector<int> indexes;
    int counter{0};
    for (const auto &iter : cells)
    {
        indexes.push_back(iter.first);
        if (counter != 0 && counter % numCellsPerThread == 0 &&
            counter < numCellsPerThread * parameters.numThreads)
        {
            threads.emplace_back(std::thread(collisionDetection, indexes));
            indexes.clear();
        }
        counter++;
    }

    // add last thread with remainders
    threads.emplace_back(std::thread(collisionDetection, indexes));

    for (auto &thread : threads)
    {
        if (thread.joinable())
        {
            thread.join();
        }
    }
}

void Simulation::addBulgeForce(Particle &p)
{
    double bulgeDistance{0};
    double thetaL, thetaD, thetaC, radicand;
    for (size_t i = 0; i < p.links.size(); i++)
    {
        glm::vec3 d = cells[p.links[i]].position - p.position;
        thetaL = acos(glm::dot(d, p.normal) / d.length());
        thetaD = asin(d.length() * sin(thetaL) / parameters.springLength);
        thetaC = M_PI - thetaD - thetaL;

        if (!std::isfinite(thetaC))
        {
            continue;
        }

        radicand = std::pow(parameters.springLength, 2) + glm::length2(d) -
                   2.0 * d.length() * parameters.springLength * cos(thetaC);

        if (radicand < 0.0)
        {
            radicand = 0;
        }

        bulgeDistance += std::sqrt(radicand);
    }

    bulgeDistance /= p.links.size();

    p.delta += p.normal * bulgeDistance * parameters.bulgeFactor;
}

void Simulation::addSpringForce(Particle &p)
{
    glm::vec3 target;
    for (auto l : p.links)
    {
        glm::vec3 d = cells[l].position - p.position;
        d = glm::normalize(d);
        d *= parameters.springLength;
        target += d;
    }

    target /= p.links.size();
    target *= parameters.springLength;
    p.delta += target * parameters.springFactor;
}

void Simulation::addPlanarForce(Particle &p)
{
    glm::vec3 planarTarget{0, 0, 0};

    for (size_t i = 0; i < p.links.size(); ++i)
    {
        planarTarget += cells[p.links[i]].position;
    }
    planarTarget /= (double)p.links.size();
    planarTarget = planarTarget - p.position;

    p.curvature = -1.0 * planarTarget.length() *
                  glm::dot(p.normal, glm::normalize(planarTarget));

    p.delta += planarTarget * parameters.planarFactor;
}

void Simulation::tick()
{
    addCollisionForce();

    for (auto &iter : cells)
    {
        Particle &p = iter.second;
        calculateNormal(p);
        addBulgeForce(p);
        addSpringForce(p);
        addPlanarForce(p);

        p.food += std::pow(std::max(p.curvature / 20.0, 0.00001),
                           parameters.foodExponent);
    }

    for (auto &iter : cells)
    {
        Particle &p = iter.second;
        if (p.food > parameters.threshold)
        {
            split(p);
        }
    }

    integrateParticles();
}

ofVboMesh Simulation::getMesh()
{
    ofVboMesh m;
    m.setMode(ofPrimitiveMode::OF_PRIMITIVE_TRIANGLES);
    for (int i = 0; i < cells.size(); ++i)
    {
        m.addVertex(cells[i].position);
        m.addColor(ofColor((cells[i].curvature + 5.0) * 20.0));
    }

    for (const auto &iter : cells)
    {
        const Particle &p = iter.second;
        size_t numLinks = p.links.size();
        for (int i = 0; i < numLinks; ++i)
        {
            m.addIndex(p.index);
            m.addIndex(p.links[i]);
            m.addIndex(p.links[(i + 1) % numLinks]);
        }
    }

    return m;
}

void Simulation::setParameters(const Parameters &p) { parameters = p; }
