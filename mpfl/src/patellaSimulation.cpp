#include "patellaSimulation.h"
#include <iostream>

// Constructor for PatellaSimulation
PatellaSimulation::PatellaSimulation(STL* patellaObj, double mass)
    : patellaObj(patellaObj), currentPosition(0.0, 0.0, 0.0), 
    velocity(0.0, 0.0, 0.0), acceleration(0.0, 0.0, 0.0), 
    totalForce(0.0, 0.0, 0.0), mass(mass) {}

// Method to simulate patella movement
void PatellaSimulation::simulate(const std::vector<vec3>& separatorPoints) {
    double minVelocity = 0.01;
    double minAcceleration = 0.01;
    double timeStep = 0.1;
    int stepNumber = 0;
    int maxSteps = 750;

    // Initialize the current position based on the patella object
    currentPosition = vec3(patellaObj->objToWorldTransform[0][3], patellaObj->objToWorldTransform[1][3], patellaObj->objToWorldTransform[2][3]);
    vec3 patellaBeforeCoords = currentPosition;

    do {
        // Perform one simulation step
        step(timeStep);
        stepNumber++;

        // Print debug information
        if (stepNumber % 50 == 0) {
            std::cout << "Patella Simulation Step " << stepNumber << std::endl;
            std::cout << "  Current Position: " << currentPosition << std::endl;
            std::cout << "  Velocity: " << velocity.length() << std::endl;
            std::cout << "  Acceleration: " << acceleration.length() << std::endl;
        }
       
    //} while ((velocity.length() > minVelocity || acceleration.length() > minAcceleration) && stepNumber < maxSteps);
    } while (velocity.length() > minVelocity || acceleration.length() > minAcceleration);

    if (stepNumber >= maxSteps) {
        std::cout << "Simulation stopped after reaching the maximum number of steps" << std::endl;
    }

    // Calculate and print the distance from the patellaBeforeCoords to the new patella coordinates
    vec3 patellaAfterCoords = currentPosition;
    double distanceMoved = (patellaAfterCoords - patellaBeforeCoords).length();
    std::cout << "Distance moved: " << distanceMoved << std::endl;
    std::cout << "Before: " << patellaBeforeCoords << std::endl;
    std::cout << "After: " << patellaAfterCoords << std::endl;
}

// Method to simulate one time step
void PatellaSimulation::step(double timeStep) {

    // Clear the total force
    totalForce = vec3(0.0, 0.0, 0.0);
    vec3 torque = vec3(0.0, 0.0, 0.0);

    // Recalculate the forces from all the springs in the list
    for (Spring* spring : springs) {
        vec3 springVector = currentPosition - vec3(spring->tendon_X, spring->tendon_Y, spring->tendon_Z);
        double updatedLength = springVector.length();
        spring->update(timeStep, updatedLength);

        totalForce.x = totalForce.x + spring->totalForce.x;
        totalForce.y = totalForce.y + spring->totalForce.y;
        totalForce.z = totalForce.z + spring->totalForce.z;

        // Calculate lever arm (distance from center of mass to force application point)
        vec3 leverArm = springVector;

        // Calculate torque as r x F (lever arm cross force)
        vec3 springTorque = leverArm ^ spring->totalForce;
        torque.x += springTorque.x;
        torque.y += springTorque.y;
        torque.z += springTorque.z;
    }

    // Recalculate the acceleration based on the total force and mass
    acceleration.x = totalForce.x / mass;
    acceleration.y = totalForce.y / mass;
    acceleration.z = totalForce.z / mass;

    const double accelerationDamping = 0.95;
    acceleration.x *= accelerationDamping;
    acceleration.y *= accelerationDamping;
    acceleration.z *= accelerationDamping;


    // Update velocity based on acceleration
    velocity.x += acceleration.x * timeStep;
    velocity.y += acceleration.y * timeStep;
    velocity.z += acceleration.z * timeStep;

    const double velocityDamping = 0.5;
    velocity.x *= velocityDamping;
    velocity.y *= velocityDamping;
    velocity.z *= velocityDamping;

    // Update the current position based on velocity and timeStep
    currentPosition.x += velocity.x * timeStep;
    currentPosition.y += velocity.y * timeStep;
    currentPosition.z += velocity.z * timeStep;

}

// Method to add a spring to the simulation
void PatellaSimulation::addSpring(Spring* spring) {
    springs.push_back(spring);
}

// Method to get the list of springs
const std::vector<Spring*>& PatellaSimulation::getSprings() const {
    return springs;
}

// Method to get the new transformation matrix of the patella object
mat4 PatellaSimulation::getNewPosition() const {
    // Create a translation matrix based on the current position
    mat4 translationMatrix = translate(currentPosition.x, currentPosition.y, currentPosition.z);
    return translationMatrix;
}