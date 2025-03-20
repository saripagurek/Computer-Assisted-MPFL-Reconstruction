#include "patellaSimulation.h"
#include <iostream>

// Constructor for PatellaSimulation
PatellaSimulation::PatellaSimulation(STL* patellaObj, double mass)
    : patellaObj(patellaObj), currentPosition(0.0, 0.0, 0.0), 
    velocity(0.0, 0.0, 0.0), acceleration(0.0, 0.0, 0.0), 
    totalForce(0.0, 0.0, 0.0), mass(mass) {}

// Method to simulate patella movement
void PatellaSimulation::simulate(const seq<vec3>& separatorPoints, const seq<vec3>& separatorNormals, const float& correctionAmount) {
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
        step(timeStep, separatorPoints, separatorNormals, correctionAmount);
        stepNumber++;

        // Print debug information
        /*if (stepNumber % 50 == 0) {
            std::cout << "Patella Simulation Step " << stepNumber << std::endl;
            std::cout << "  Current Position: " << currentPosition << std::endl;
            std::cout << "  Velocity: " << velocity.length() << std::endl;
            std::cout << "  Acceleration: " << acceleration.length() << std::endl;
        }*/
       
    //} while ((velocity.length() > minVelocity || acceleration.length() > minAcceleration) && stepNumber < maxSteps);
    } while (velocity.length() > minVelocity || acceleration.length() > minAcceleration);

    /*if (stepNumber >= maxSteps) {
        std::cout << "Simulation stopped after reaching the maximum number of steps" << std::endl;
    }*/

    // Calculate and print the distance from the patellaBeforeCoords to the new patella coordinates
    vec3 patellaAfterCoords = currentPosition;
    double distanceMoved = (patellaAfterCoords - patellaBeforeCoords).length();
    std::cout << "Distance moved: " << distanceMoved << std::endl;
    std::cout << "Before: " << patellaBeforeCoords << std::endl;
    std::cout << "After: " << patellaAfterCoords << std::endl;
}

// Method to simulate one time step
void PatellaSimulation::step(double timeStep, const seq<vec3>& separatorPoints, const seq<vec3>& separatorNormals, const float& correctionAmount) {

    // Clear the total force
    totalForce = vec3(0.0, 0.0, 0.0);

    // Recalculate the forces from all the springs in the list
    for (Spring* spring : springs) {
        vec3 springVector = currentPosition - vec3(spring->tendon_X, spring->tendon_Y, spring->tendon_Z);
        double updatedLength = springVector.length();
        spring->update(timeStep, updatedLength);

        totalForce.x = totalForce.x + spring->totalForce.x;
        totalForce.y = totalForce.y + spring->totalForce.y;
        totalForce.z = totalForce.z + spring->totalForce.z;
    }

    //std::cout << "Total Force: " << totalForce << std::endl;

    // Get the skeletal force and add it to the total force
   vec3 skeletalForce = getSkeletalForce(separatorPoints, separatorNormals, correctionAmount);
    totalForce.x += skeletalForce.x;
    totalForce.y += skeletalForce.y;
    totalForce.z += skeletalForce.z;

    // Print skeletal force for debugging
    //std::cout << "Skeletal Force: " << skeletalForce << std::endl;

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

// Method to calculate total force from skeletal points on the cartilage towards the patella
vec3 PatellaSimulation::getSkeletalForce(const seq<vec3>& separatorPoints, const seq<vec3>& separatorNormals, const float& correctionAmount) {
    vec3 totalForce(0.0, 0.0, 0.0);
    double weight = 0.0000001;  // Weight factor to control force intensity
    double minDistance = 0.001;  // Minimum effective distance to avoid excessive force
    double maxDistance = 150.0;
    double bufferDistance = 50.0;  // Buffer distance to account for patella size and mass

    // Validate input sizes
    if (separatorPoints.size() != separatorNormals.size()) {
        std::cerr << "Error: separatorPoints and separatorNormals sizes do not match. Points: " 
                  << separatorPoints.size() << ", Normals: " << separatorNormals.size() << std::endl;
        return totalForce;
    }

    for (size_t i = 0; i < separatorPoints.size(); ++i) {
        vec3 point = separatorPoints[i];
        vec3 normal = separatorNormals[i];

        // Vector from point to the patella
        vec3 pointToPatella = currentPosition - point;
        double distance = pointToPatella.length();

        // Check if the distance exceeds the maximum threshold
        if (distance > maxDistance) {
            continue;
        }

        // Project the pointToPatella vector onto the normal using the dot product operator
        double projection = pointToPatella * normal;

        // Apply a repulsion force if the projection is negative
        if (projection < 0) {
            double forceMagnitude = 0.0;

            // Enhanced repulsion using exponential increase when distance is less than buffer
            if (distance < bufferDistance) {
                forceMagnitude = weight * exp((bufferDistance - distance) * 5.0);
            } else {
                forceMagnitude = weight / (distance * distance + minDistance * minDistance);
            }

            // Scale the force magnitude by the correctionAmount component-wise
            double correctionScaledX = std::max(static_cast<double>(correctionAmount) * fabs(normal.x), 1e-12);
            double correctionScaledY = std::max(static_cast<double>(correctionAmount) * fabs(normal.y), 1e-12);
            double correctionScaledZ = std::max(static_cast<double>(correctionAmount) * fabs(normal.z), 1e-12);

            // Normalize the normal vector
            normal.normalize();

            // Calculate the final force components
            double forceX = normal.x * forceMagnitude * correctionScaledX;
            double forceY = normal.y * forceMagnitude * correctionScaledY;
            double forceZ = normal.z * forceMagnitude * correctionScaledZ;

            /*if (i % 100 == 0) {
                std::cout << "Point index " << i << ": Point = " << point << ", Normal = " << normal << std::endl;
                std::cout << "Distance = " << distance << std::endl;
                std::cout << "Force components: X = " << forceX << ", Y = " << forceY << ", Z = " << forceZ << std::endl;
                std::cout << "Force Magnitude: " << forceMagnitude << std::endl;
                std::cout << "CorrectionScaled: X = " << correctionScaledX << ", Y = " << correctionScaledY << ", Z = " << correctionScaledZ << std::endl;
            }*/

            // Add the force components to the total force
            totalForce.x += forceX;
            totalForce.y += forceY;
            totalForce.z += forceZ;
        }
    }

    return totalForce;
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