#ifndef SPRING_H
#define SPRING_H

#include <cmath>
#include "linalg.h"

class Segs; // Forward declaration of Segs class

class Spring {
public:
    double springConstant; // Spring constant (k)
    double dampingCoefficient; // Damping coefficient
    double tendon_X, tendon_Y, tendon_Z; // Anchor point 1 coordinates
    double patella_X, patella_Y, patella_Z; // Anchor point 2 coordinates
    double velocity_X, velocity_Y, velocity_Z; // Velocity of the patella point
    double weight; // Weight parameter
    int frameCounter; // Frame counter
    vec3 patellaWorldPos; // Patella position in world coordinates

    // Intermediate values
    double restLength;
    double currentLength;
    double displacement;
    double springForceMagnitude;
    vec3 direction;
    vec3 springForce;
    vec3 dampingForce;
    vec3 totalForce;

    // Helper function to calculate the rest length based on anchor points
    double calculateRestLength() const;

public:
    // Constructor
    Spring(double k, double damping, double t_X, double t_Y, double t_Z, double p_X, double p_Y, double p_Z, double weight);

    // Setters
    void setSpringConstant(double k);
    void setDampingCoefficient(double damping);
    void setAnchorPoints(double t_X, double t_Y, double t_Z, double p_X, double p_Y, double p_Z);
    void setVelocity(double v_X, double v_Y, double v_Z);
    void setWeight(double w);

    // Getters
    double getSpringConstant() const;
    double getDampingCoefficient() const;
    double getRestLength() const;
    double getWeight() const;

    // Method to reposition the spring's anchor points
    void reposition(const vec3 &newPatellaXYZ, const vec3 &newtendonXYZ);

    // Method to update the spring's state based on time interval
    void update(double deltaTime, double distance);

    // Method to draw the spring as a cylinder
    void drawSpring(mat4 &WCS_to_VCS, mat4 &WCS_to_CCS, vec3 &lightDirVCS, const vec4 &colour);

    // Method to calculate the force matrix exerted by the spring
    // mat4 calculateForceMatrix() const;
};


#endif // SPRING_H
