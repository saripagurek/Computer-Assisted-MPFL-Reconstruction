#include "spring.h"
#include "main.h"
#include <iostream>

// Constructor
Spring::Spring(double k, double damping, double t_X, double t_Y, double t_Z, double p_X, double p_Y, double p_Z, double weight)
    : springConstant(k), dampingCoefficient(damping), tendon_X(t_X), tendon_Y(t_Y), tendon_Z(t_Z),
      patella_X(p_X), patella_Y(p_Y), patella_Z(p_Z), velocity_X(0.0), velocity_Y(0.0), velocity_Z(0.0), weight(weight), frameCounter(0) {
        restLength = calculateRestLength();
        previousLength = restLength;
        isSimulating = false;
      }

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

void Spring::setSimulating(bool simulating) {
    isSimulating = simulating;
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

    // Update rest length based on new anchor points
    restLength = calculateRestLength();
    previousLength = restLength;
}

// Method to calculate the force exerted by the spring
void Spring::update(double deltaTime, double updatedLength) {

    // Increment frame counter
    frameCounter++;
    totalForce = vec3(0.0, 0.0, 0.0);

    // Skip recalculating currentLength if the spring is in simulation mode
    /*if (!isSimulating) {
        // Get the current patella position from anim->patellaObj
        //patellaWorldPos = (anim->patellaObj->objToWorldTransform * vec4(patella_X, patella_Y, patella_Z, 1.0)).toVec3();

        // Calculate the current length of the spring
        currentLength = std::sqrt(
            std::pow(patellaWorldPos.x - tendon_X, 2) +
            std::pow(patellaWorldPos.y - tendon_Y, 2) +
            std::pow(patellaWorldPos.z - tendon_Z, 2)
        );
    }*/

    currentLength = updatedLength;

    // Calculate displacement from the rest length  
    displacement = currentLength - restLength;

    // Calculate spring velocity as the rate of change of length
    double springVelocity = (currentLength - previousLength) / deltaTime;

    // Clamp the spring velocity to avoid instability
    const double maxSpringVelocity = 5.0;
    if (springVelocity > maxSpringVelocity) {
        springVelocity = maxSpringVelocity;
    } else if (springVelocity < -maxSpringVelocity) {
        springVelocity = -maxSpringVelocity;
    }

    // Calculate direction of the spring force
    direction = vec3(patella_X - tendon_X, patella_Y - tendon_Y, patella_Z - tendon_Z).normalize();

    // Hooke's law: F = -kx
    springForce = -springConstant * displacement * direction;

    // Damping force: F = -bv
    dampingForce = -dampingCoefficient * springVelocity * direction;

    // Total force
    totalForce = springForce + dampingForce;

    // Print debug information every 5 frames
    /*if (frameCounter % 10 == 0) {
        std::cout << "Spring Update:" << std::endl;
        std::cout << "  Current Length: " << currentLength << std::endl;
        std::cout << "  Previous Length: " << previousLength << std::endl;
        std::cout << "  Rest Length: " << restLength << std::endl;
        std::cout << "  Displacement: " << displacement << std::endl;
        std::cout << "  Spring Velocity: " << springVelocity << std::endl;
        std::cout << "  Spring Force: " << springForce << std::endl;
        std::cout << "  Damping Force: " << dampingForce << std::endl;
        std::cout << "  Total Force: " << totalForce << std::endl;
    }*/

    previousLength = currentLength;

}


void checkGLError(const std::string &msg) {
    GLenum err;
    while ((err = glGetError()) != GL_NO_ERROR) {
        std::cerr << "OpenGL error after " << msg << ": " << err << std::endl;
    }
}


// Method to calculate the force matrix exerted by the spring
/*mat4 Spring::calculateForceMatrix() const {

    std::cout << "  Spring Calculate Matrix" <<std::endl;
    std::cout << "  Total Force: " << totalForce << std::endl;

    vec3 scaledForce = totalForce;
    scaledForce.x *= weight;
    scaledForce.y *= weight;
    scaledForce.z *= weight;

    std::cout << "  Scaling force by " << weight << std::endl;
    std::cout << "  Scaled Force: " << scaledForce << std::endl;

    // Calculate the translation matrix
    mat4 translationMatrix;
    for (int i = 0; i < 4; ++i) {
        for (int j = 0; j < 4; ++j) {
            translationMatrix[i][j] = (i == j) ? 1.0f : 0.0f;
        }
    }
    translationMatrix[3][0] = totalForce.x;
    translationMatrix[3][1] = totalForce.y;
    translationMatrix[3][2] = totalForce.z;

    // Print translationMatrix for debugging
    std::cout << "Translation Matrix:" << std::endl;
    for (int i = 0; i < 4; ++i) {
        for (int j = 0; j < 4; ++j) {
            std::cout << translationMatrix[i][j] << " ";
        }
        std::cout << std::endl;
    }

    // Calculate the rotation matrix
    vec3 up = vec3(0, 1, 0);
    vec3 axis = up ^ direction.normalize();
    float angle = acos(up * direction.normalize());
    mat4 rotationMatrix = rotate(angle, axis);

    // Combine translation and rotation matrices
    mat4 forceMatrix = translationMatrix * rotationMatrix;

    // Print forceMatrix for debugging
    std::cout << "Force Matrix:" << std::endl;
    for (int i = 0; i < 4; ++i) {
        for (int j = 0; j < 4; ++j) {
            std::cout << forceMatrix[i][j] << " ";
        }
        std::cout << std::endl;
    }

    // Print how much this translation will move the patella
    vec4 patellaPos = vec4(patella_X, patella_Y, patella_Z, 1.0);
    vec4 translatedPatellaPos = forceMatrix * patellaPos;
    std::cout << "Translated Patella Position: " << translatedPatellaPos.toVec3() << std::endl;
    // Print distance moved
    double distanceMoved = (translatedPatellaPos.toVec3() - patellaPos.toVec3()).length();
    std::cout << "Distance Moved: " << distanceMoved << std::endl;

    return forceMatrix;
}*/


// Helper method to draw the spring as a cylinder
void Spring::drawSpring(mat4 &WCS_to_VCS, mat4 &WCS_to_CCS, vec3 &lightDirVCS, const vec4 &colour) {

    vec3 p0(tendon_X, tendon_Y, tendon_Z);
    vec3 p1(patella_X, patella_Y, patella_Z);

    // Transform patella point to world coordinates
    p1 = (anim->patellaObj->objToWorldTransform * vec4(p1, 1.0)).toVec3();

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

    checkGLError("drawSpring");
}
