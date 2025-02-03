#ifndef SPRING_H
#define SPRING_H

#include <cmath>

class Spring {
private:
    double springConstant; // Spring constant (k)
    double dampingCoefficient; // Damping coefficient
    double femur_X, femur_Y, femur_Z; // Anchor point 1 coordinates
    double patella_X, patella_Y, patella_Z; // Anchor point 2 coordinates
    double velocity_X, velocity_Y, velocity_Z; // Velocity of the patella point

    // Helper function to calculate the rest length based on anchor points
    double calculateRestLength() const;

public:
    // Constructor
    Spring(double k, double damping, double f_X, double f_Y, double f_Z, double p_X, double p_Y, double p_Z);

    // Setters
    void setSpringConstant(double k);
    void setDampingCoefficient(double damping);
    void setAnchorPoints(double f_X, double f_Y, double f_Z, double p_X, double p_Y, double p_Z);
    void setVelocity(double v_X, double v_Y, double v_Z);

    // Getters
    double getSpringConstant() const;
    double getDampingCoefficient() const;
    double getRestLength() const;

    // Method to update the spring's state based on time interval
    void update(double deltaTime);
};

#endif // SPRING_H
