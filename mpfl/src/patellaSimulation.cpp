#include "patellaSimulation.h"
#include <iostream>

// Constructor for PatellaSimulation
PatellaSimulation::PatellaSimulation(STL* patellaObj, double mass)
    : patellaObj(patellaObj), currentPosition(0.0, 0.0, 0.0), 
    velocity(0.0, 0.0, 0.0), acceleration(0.0, 0.0, 0.0), 
    totalForce(0.0, 0.0, 0.0), mass(mass) {}

// Method to simulate patella movement
void PatellaSimulation::simulate(const std::vector<vec3>& separatorPoints) {
    double minVelocity = 0.001;
    double minAcceleration = 0.001;
    double timeStep = 1.0;

    do {
        // Perform one simulation step
        step(timeStep);

        // Calculate acceleration based on the total force and mass
        acceleration.x = totalForce.x / mass;
        acceleration.y = totalForce.y / mass;
        acceleration.z = totalForce.z / mass;

        // Update velocity based on acceleration
        velocity.x += acceleration.x * timeStep;
        velocity.y += acceleration.y * timeStep;
        velocity.z += acceleration.z * timeStep;

        // Print debug information
        std::cout << "Patella Simulation Step:" << std::endl;
        std::cout << "  Current Position: " << currentPosition << std::endl;
        std::cout << "  Velocity: " << velocity << std::endl;
        std::cout << "  Acceleration: " << acceleration << std::endl;

    } while (velocity.length() > minVelocity || acceleration.length() > minAcceleration); // Threshold values to determine when to stop
}

// Method to simulate one time step
void PatellaSimulation::step(double timeStep) {

}

// Method to add a spring to the simulation
void PatellaSimulation::addSpring(Spring* spring) {
    springs.push_back(spring);
}

// Method to get the list of springs
const std::vector<Spring*>& PatellaSimulation::getSprings() const {
    return springs;
}

// Method to get the new position of the patella object
vec3 PatellaSimulation::getNewPosition() const {
    return currentPosition;
}