#include "spring.h"

// Constructor
Spring::Spring(double k, double damping, double f_X, double f_Y, double f_Z, double p_X, double p_Y, double p_Z)
    : springConstant(k), dampingCoefficient(damping), femur_X(f_X), femur_Y(f_Y), femur_Z(f_Z),
      patella_X(p_X), patella_Y(p_Y), patella_Z(p_Z), velocity_X(0.0), velocity_Y(0.0), velocity_Z(0.0) {}

// Private helper function
double Spring::calculateRestLength() const {
    return std::sqrt(
        std::pow(patella_X - femur_X, 2) +
        std::pow(patella_Y - femur_Y, 2) +
        std::pow(patella_Z - femur_Z, 2)
    );
}

// Setters
void Spring::setSpringConstant(double k) {
    springConstant = k;
}

void Spring::setDampingCoefficient(double damping) {
    dampingCoefficient = damping;
}

void Spring::setAnchorPoints(double f_X, double f_Y, double f_Z, double p_X, double p_Y, double p_Z) {
    femur_X = f_X;
    femur_Y = f_Y;
    femur_Z = f_Z;
    patella_X = p_X;
    patella_Y = p_Y;
    patella_Z = p_Z;
}

void Spring::setVelocity(double v_X, double v_Y, double v_Z) {
    velocity_X = v_X;
    velocity_Y = v_Y;
    velocity_Z = v_Z;
}

// Getters
double Spring::getSpringConstant() const {
    return springConstant;
}

double Spring::getDampingCoefficient() const {
    return dampingCoefficient;
}

double Spring::getRestLength() const {
    return calculateRestLength();
}

// Method to calculate the force exerted by the spring
void Spring::update(double deltaTime) {
    double restLength = calculateRestLength();
    double currentLength = std::sqrt(
        std::pow(patella_X - femur_X, 2) +
        std::pow(patella_Y - femur_Y, 2) +
        std::pow(patella_Z - femur_Z, 2)
    );

    double displacement = currentLength - restLength;
    double springForce = -springConstant * displacement;

    // Calculate damping force
    double dampingForce_X = -dampingCoefficient * velocity_X;
    double dampingForce_Y = -dampingCoefficient * velocity_Y;
    double dampingForce_Z = -dampingCoefficient * velocity_Z;

    // Total force
    double totalForce_X = springForce * (patella_X - femur_X) / currentLength + dampingForce_X;
    double totalForce_Y = springForce * (patella_Y - femur_Y) / currentLength + dampingForce_Y;
    double totalForce_Z = springForce * (patella_Z - femur_Z) / currentLength + dampingForce_Z;

    // Update velocity
    velocity_X += (totalForce_X * deltaTime);
    velocity_Y += (totalForce_Y * deltaTime);
    velocity_Z += (totalForce_Z * deltaTime);

    // Update position
    patella_X += velocity_X * deltaTime;
    patella_Y += velocity_Y * deltaTime;
    patella_Z += velocity_Z * deltaTime;
}
