#ifndef PATELLASIMULATION_H
#define PATELLASIMULATION_H

#include "linalg.h"
#include "stl.h"
#include "spring.h"
#include <vector>

class PatellaSimulation {
public:
    // Constructor
    PatellaSimulation(STL* patellaObj, double mass);

    // Method to simulate patella movement
    void simulate(const seq<vec3>& separatorPoints, const seq<vec3>& separatorNormals, const float& correctionAmount, const vec3& rotationAxis);

    // Method to calculate total force from skeletal points
    vec3 getSkeletalForce(const seq<vec3>& separatorPoints, const seq<vec3>& separatorNormals, const float& correctionAmount);

    // Method to add a spring to the simulation
    void addSpring(Spring* spring);

    // Method to get the list of springs
    const std::vector<Spring*>& getSprings() const;

    // Method to simulate one time step
    void step(double deltaTime, const seq<vec3>& separatorPoints, const seq<vec3>& separatorNormals, const float& correctionAmount);

    // Method to get the new position of the patella object
    mat4 getNewPosition() const;

private:
    STL* patellaObj; // Pointer to the patella object
    vec3 currentPosition; // Current position of the patella
    vec3 velocity; // Velocity of the patella
    vec3 acceleration; // Acceleration of the patella
    vec3 totalForce; // Total force acting on the patella
    double mass; // Mass of the patella
    std::vector<Spring*> springs; // List of springs associated with the simulation
    vec3 rotationAxis;
    float rotationAngle;

};

#endif // PATELLASIMULATION_H