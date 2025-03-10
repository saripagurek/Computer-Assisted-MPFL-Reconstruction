#include "spring.h"
#include "main.h"
#include <iostream>

// Constructor
Spring::Spring(double k, double damping, double t_X, double t_Y, double t_Z, double p_X, double p_Y, double p_Z, double weight)
    : springConstant(k), dampingCoefficient(damping), tendon_X(t_X), tendon_Y(t_Y), tendon_Z(t_Z),
      patella_X(p_X), patella_Y(p_Y), patella_Z(p_Z), velocity_X(0.0), velocity_Y(0.0), velocity_Z(0.0), weight(weight) {}

// Private helper function
double Spring::calculateRestLength() const {
    return std::sqrt(
        std::pow(patella_X - tendon_X, 2) +
        std::pow(patella_Y - tendon_Y, 2) +
        std::pow(patella_Z - tendon_Z, 2)
    );
}

// Setters
void Spring::setSpringConstant(double k) {
    springConstant = k;
}

void Spring::setDampingCoefficient(double damping) {
    dampingCoefficient = damping;
}

void Spring::setAnchorPoints(double t_X, double t_Y, double t_Z, double p_X, double p_Y, double p_Z) {
    tendon_X = t_X;
    tendon_Y = t_Y;
    tendon_Z = t_Z;
    patella_X = p_X;
    patella_Y = p_Y;
    patella_Z = p_Z;
}

void Spring::setVelocity(double v_X, double v_Y, double v_Z) {
    velocity_X = v_X;
    velocity_Y = v_Y;
    velocity_Z = v_Z;
}

void Spring::setWeight(double w) {
    weight = w;
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

double Spring::getWeight() const {
    return weight;
}

// Method to reposition the spring's anchor points
void Spring::reposition(const vec3 &newtendonXYZ, const vec3 &newPatellaXYZ) {
    if (newPatellaXYZ != vec3(0, 0, 0)) {
        patella_X = newPatellaXYZ.x;
        patella_Y = newPatellaXYZ.y;
        patella_Z = newPatellaXYZ.z;
    }

    if (newtendonXYZ != vec3(0, 0, 0)) {
        tendon_X = newtendonXYZ.x;
        tendon_Y = newtendonXYZ.y;
        tendon_Z = newtendonXYZ.z;
    }
}

// Method to calculate the force exerted by the spring
void Spring::update(double deltaTime, double distance) {
    restLength = calculateRestLength();
    currentLength = std::sqrt(
        std::pow(patella_X - tendon_X, 2) +
        std::pow(patella_Y - tendon_Y, 2) +
        std::pow(patella_Z - tendon_Z, 2)
    );

    displacement = currentLength - restLength;
    springForceMagnitude = -springConstant * displacement;

    direction = vec3(patella_X - tendon_X, patella_Y - tendon_Y, patella_Z - tendon_Z).normalize();
    springForce = springForceMagnitude * direction;

    // Calculate damping force
    dampingForce = vec3(
        -dampingCoefficient * velocity_X,
        -dampingCoefficient * velocity_Y,
        -dampingCoefficient * velocity_Z
    );

    // Total force
    totalForce = springForce + dampingForce;

    // Calculate velocity based on distance and deltaTime
    velocity_X = distance / deltaTime;
    velocity_Y = distance / deltaTime;
    velocity_Z = distance / deltaTime;

    // Update velocity
    velocity_X += (totalForce.x * deltaTime);
    velocity_Y += (totalForce.y * deltaTime);
    velocity_Z += (totalForce.z * deltaTime);

    // Update position
    //patella_X += velocity_X * deltaTime;
    //patella_Y += velocity_Y * deltaTime;
    //patella_Z += velocity_Z * deltaTime;
}


void checkGLError(const std::string &msg) {
    GLenum err;
    while ((err = glGetError()) != GL_NO_ERROR) {
        std::cerr << "OpenGL error after " << msg << ": " << err << std::endl;
    }
}

// Function to calculate the area of a triangle given its vertices
float triangleArea(const vec3 &a, const vec3 &b, const vec3 &c) {
    return 0.5f * ((b - a) ^ (c - a)).length();
}

// Helper method to draw the spring as a cylinder
void Spring::drawSpring(mat4 &WCS_to_VCS, mat4 &WCS_to_CCS, vec3 &lightDirVCS, const vec4 &colour) {

    // Print the current velocities of the spring
    /*std::cout << "Spring velocities: " << std::endl;
    std::cout << "  velocity_X: " << velocity_X << std::endl;
    std::cout << "  velocity_Y: " << velocity_Y << std::endl;
    std::cout << "  velocity_Z: " << velocity_Z << std::endl;*/

    //std::cout << "Drawing spring with colour: " << colour << std::endl;

    vec3 p0(tendon_X, tendon_Y, tendon_Z);
    vec3 p1(patella_X, patella_Y, patella_Z);

    // Transform patella point to world coordinates
    p1 = (anim->patellaObj->objToWorldTransform * vec4(p1, 1.0)).toVec3();

    //std::cout << "p0: " << p0 << ", p1: " << p1 << std::endl;

    vec3 x0 = (p1 - p0).perp1().normalize();
    vec3 x1 = x0;

    if (x0 * x1 < 0) // (otherwise directions around circle might be different at two ends)
        x0 = -1 * x0;

    // other perpendiculars to x and axis
    vec3 y0 = (x0 ^ (p1 - p0)).normalize();
    vec3 y1 = (x1 ^ (p1 - p0)).normalize();

    // Build a cylinder
    const int NUM_CYL_FACES = 50;
    const float CYL_RADIUS = 0.5;

    vec3 ps[2 * (NUM_CYL_FACES + 1)];
    vec3 ns[2 * (NUM_CYL_FACES + 1)];

    for (int i = 0; i < NUM_CYL_FACES + 1; i++) {
        float theta = i * (2 * M_PI / (float)NUM_CYL_FACES);

        ns[2 * i] = cos(theta) * x0 + sin(theta) * y0;
        ns[2 * i + 1] = cos(theta) * x1 + sin(theta) * y1;

        ps[2 * i] = p0 + CYL_RADIUS * ns[2 * i];
        ps[2 * i + 1] = p1 + CYL_RADIUS * ns[2 * i + 1];
    }

    springSegs->drawSegs(GL_TRIANGLE_STRIP, &ps[0], colour, &ns[0], 2 * (NUM_CYL_FACES + 1), WCS_to_VCS, WCS_to_CCS, lightDirVCS);

    //glEnable( GL_DEPTH_TEST );

    checkGLError("drawSpring");
}


// Method to calculate the force exerted by the spring
vec3 Spring::calculateForce() {
    return totalForce;
}

// Method to calculate the force matrix exerted by the spring
mat4 Spring::calculateForceMatrix() const {
    vec3 force = totalForce;
    force.x *= weight;
    force.y *= weight;
    force.z *= weight;

    mat4 forceMatrix;

    // Initialize to identity matrix
    for (int i = 0; i < 4; ++i) {
        for (int j = 0; j < 4; ++j) {
            forceMatrix[i][j] = (i == j) ? 1.0f : 0.0f;
        }
    }

    // Set the translation components to the force vector
    forceMatrix[0][3] = force.x;
    forceMatrix[1][3] = force.y;
    forceMatrix[2][3] = force.z;

    return forceMatrix;
}
