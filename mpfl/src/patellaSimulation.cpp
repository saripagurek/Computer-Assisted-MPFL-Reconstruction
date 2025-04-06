#include "patellaSimulation.h"
#include "headers.h"
#include "main.h"
#include <iostream>

// Constructor for PatellaSimulation
PatellaSimulation::PatellaSimulation(STL* patellaObj, double mass)
    : patellaObj(patellaObj), 
      currentPosition(vec3(patellaObj->objToWorldTransform[0][3], 
                           patellaObj->objToWorldTransform[1][3], 
                           patellaObj->objToWorldTransform[2][3])),
      velocity(0.0, 0.0, 0.0), 
      acceleration(0.0, 0.0, 0.0), 
      totalForce(0.0, 0.0, 0.0), 
      mass(mass) {}

// Method to simulate patella movement
void PatellaSimulation::simulate(const seq<vec3>& separatorPoints, const seq<vec3>& separatorNormals, const float& correctionAmount, const vec3& rotationAxis) {
    double minVelocity = 0.01;
    double minAcceleration = 0.01;
    double timeStep = 0.1;
    int stepNumber = 0;

    // Initialize the current position based on the patella object
    currentPosition = vec3(anim->patellaObj->objToWorldTransform[0][3], anim->patellaObj->objToWorldTransform[1][3], anim->patellaObj->objToWorldTransform[2][3]);
    vec3 patellaBeforeCoords = currentPosition;

    //Assign rotation axis
    this->rotationAxis = rotationAxis;

    do {
        // Perform one simulation step
        step(timeStep, separatorPoints, separatorNormals, correctionAmount);
        stepNumber++;

        // Print debug information
        /*if (stepNumber % 100 == 0) {
            std::cout << "Patella Simulation Step " << stepNumber << std::endl;
            std::cout << "  Current Position: " << currentPosition << std::endl;
            std::cout << "  Velocity: " << velocity.length() << std::endl;
            std::cout << "  Acceleration: " << acceleration.length() << std::endl;
        }*/
       
    } while (velocity.length() > minVelocity || acceleration.length() > minAcceleration);

}

// Method to simulate one time step
void PatellaSimulation::step(double timeStep, const seq<vec3>& separatorPoints, const seq<vec3>& separatorNormals, const float& correctionAmount) {

    // Clear the total force
    totalForce = vec3(0.0, 0.0, 0.0);
    vec3 totalTorque(0.0, 0.0, 0.0);
    double forceScaling = 0.1; 

    // Recalculate the forces from all the springs in the list
    for (Spring* spring : springs) {
        vec3 patellaSurface(spring->patella_X, spring->patella_Y, spring->patella_Z);
        vec3 tendonPoint(spring->tendon_X, spring->tendon_Y, spring->tendon_Z);
        vec3 springVector = currentPosition - vec3(spring->tendon_X, spring->tendon_Y, spring->tendon_Z);
        double updatedLength = springVector.length();
        spring->update(timeStep, updatedLength);

        totalForce.x = totalForce.x + spring->totalForce.x * forceScaling;
        totalForce.y = totalForce.y + spring->totalForce.y * forceScaling;
        totalForce.z = totalForce.z + spring->totalForce.z * forceScaling;

        // Compute r as vector from patella surface attachment point to tendon attachment point
        vec3 r = tendonPoint - patellaSurface;

        // Accumulate torque: r cross F
        vec3 springTorque = r ^ spring->totalForce;
        totalTorque.x += springTorque.x;
        totalTorque.y += springTorque.y;
        totalTorque.z += springTorque.z;
    }

    // Get the skeletal force and add it to the total force
    vec3 skeletalForce = getSkeletalForce(separatorPoints, separatorNormals, correctionAmount);
    totalForce.x += skeletalForce.x;
    totalForce.y += skeletalForce.y;
    totalForce.z += skeletalForce.z;

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

    const double velocityDamping = 0.95;
    velocity.x *= velocityDamping;
    velocity.y *= velocityDamping;
    velocity.z *= velocityDamping;

    // Update the current position based on velocity and timeStep
    currentPosition.x += velocity.x * timeStep;
    currentPosition.y += velocity.y * timeStep;
    currentPosition.z += velocity.z * timeStep;

    // Calculate the rotation angle based on the torque magnitude
    double rotationScale = 0.00001; // Adjust scaling factor as needed
    rotationAngle = totalTorque.length() * rotationScale;

    // Set the rotation axis to be the normalized torque direction
if (rotationAngle > 0.0f) {
    vec3 normalizedTorque = totalTorque.normalize();
    rotationAxis.x = normalizedTorque.x;
    rotationAxis.y = normalizedTorque.y;
    rotationAxis.z = normalizedTorque.z;
} else {
    rotationAxis.x = 0.0f;
    rotationAxis.y = 1.0f;
    rotationAxis.z = 0.0f; // Default axis if no torque
}

    /*std::cout << "Skeletal Force: " << skeletalForce << std::endl;
    std::cout << "Torque: " << totalTorque << std::endl;
    std::cout << "Rotation Angle: " << rotationAngle << std::endl;
    std::cout << "Force: " << totalForce << std::endl;*/

}

// Method to calculate total force from skeletal points on the cartilage towards the patella
vec3 PatellaSimulation::getSkeletalForce(const seq<vec3>& separatorPoints, const seq<vec3>& separatorNormals, const float& correctionAmount) {
    vec3 totalForce(0.0, 0.0, 0.0);
    double weight = 0.00000001;  // Weight factor to control force intensity
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

// Method to get the updated transformation matrix of the patella object
mat4 PatellaSimulation::getNewPosition() const {
    // Copy the original transformation matrix
    mat4 updatedTransform = patellaObj->objToWorldTransform;

    // Update the translation (position) components with the current position
    updatedTransform[0][3] = currentPosition.x;
    updatedTransform[1][3] = currentPosition.y;
    updatedTransform[2][3] = currentPosition.z;

    // Generate the rotation matrix based on the calculated angle and axis
    mat4 rotationMatrix = rotate(rotationAngle, rotationAxis);

    // Combine translation and rotation
    updatedTransform = rotationMatrix * updatedTransform;

    //Print before and after coordinates for debugging
    vec3 patellaBeforeCoords = vec3(patellaObj->objToWorldTransform[0][3], patellaObj->objToWorldTransform[1][3], patellaObj->objToWorldTransform[2][3]);
    vec3 patellaAfterCoords = vec3(updatedTransform[0][3], updatedTransform[1][3], updatedTransform[2][3]);
    std::cout << "Patella Before: " << patellaBeforeCoords << std::endl;
    std::cout << "Patella After: " << patellaAfterCoords << std::endl;
    std::cout << "Rotation Angle: " << rotationAngle << std::endl;
    std::cout << "Rotation Axis: " << rotationAxis << std::endl;
    
    double distanceMoved = sqrt(pow(patellaAfterCoords.x - patellaBeforeCoords.x, 2) +
                                 pow(patellaAfterCoords.y - patellaBeforeCoords.y, 2) +
                                 pow(patellaAfterCoords.z - patellaBeforeCoords.z, 2));
    std::cout << "Distance Moved: " << distanceMoved << std::endl;

    return updatedTransform;
}