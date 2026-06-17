#include "body.h"

inline float CrossProduct2D(Vector2 a, Vector2 b) {
    return a.x * b.y - a.y * b.x;
}

Body::Body(Vector2 pos, float mass, float restitution, float friction, bool isStatic)
    : pos(pos), speed({0,0}), accel({0,0}), angle(0), angularSpeed(0),
      mass(mass), restitution(restitution),
      friction(friction), isStatic(isStatic) {}

void Body::update(float dt) {
    if (isStatic) return;
    speed.x += accel.x * dt;
    speed.y += accel.y * dt;
    pos.x   += speed.x * dt;
    pos.y   += speed.y * dt;
    angle += angularSpeed * dt;
    angularSpeed *= 0.98f;
    accel = {0,0};
}

void Body::applyForce(Vector2 force) {
    if (isStatic) return;
    accel.x += force.x / mass;
    accel.y += force.y / mass;
}

Vector2 Body::getPos()         const { return pos; }
Vector2 Body::getSpeed()       const { return speed; }
Vector2 Body::getAccel()       const { return accel; }
float   Body::getMass()        const { return mass; }
float   Body::getAngle()           const { return angle; }
float   Body::getAngularSpeed() const { return angularSpeed; }
float   Body::getRestitution() const { return restitution; }
float   Body::getFriction()    const { return friction; }
bool    Body::getIsStatic()    const { return isStatic; }
std::vector<Vector2> Body::getVertices() const { return {}; }

void Body::setPos(Vector2 p)   { pos = p; }
void Body::setSpeed(Vector2 v) { speed = v; }
void Body::setAccel(Vector2 a) { accel = a; }
void Body::setAngle(float a)   { angle = a; }
void Body::setAngularSpeed(float w) { angularSpeed = w; }
void Body::setStatic(bool s)   { isStatic = s; }

Circumference::Circumference(Vector2 pos, float radius, float mass,
                             float restitution, float friction, bool isStatic)
    : Body(pos, mass, restitution, friction, isStatic), radius(radius) {}

void Circumference::draw() const {
    DrawCircleV(pos, radius, RED);
    Vector2 edge = { pos.x + cosf(angle) * radius, pos.y + sinf(angle) * radius };
    DrawLineV(pos, edge, BLACK);
}

float Circumference::area() const {
    return PI * radius * radius;
}

float Circumference::inertia() const {
    return 0.5f * mass * radius * radius;
}

float Circumference::getRadius() const { return radius; }


RigidRect::RigidRect(Vector2 pos, float width, float height, float mass,
                     float restitution, float friction, bool isStatic, Color cor)
    : Body(pos, mass, restitution, friction, isStatic),
      width(width), height(height), cor(cor) {}

std::vector<Vector2> RigidRect::getVertices() const {
    std::vector<Vector2> vertices(4);

    float hw = width / 2.0f;
    float hh = height / 2.0f;
    Vector2 localVertices[4] = {
        { -hw, -hh },
        {  hw, -hh },
        {  hw,  hh },
        { -hw,  hh }
    };

    float cosA = cosf(angle);
    float sinA = sinf(angle);

    for (int i = 0; i < 4; i++) {
        vertices[i].x = pos.x + (localVertices[i].x * cosA - localVertices[i].x * sinA);
        vertices[i].x = pos.x + (localVertices[i].x * cosA - localVertices[i].y * sinA);
        vertices[i].y = pos.y + (localVertices[i].x * sinA + localVertices[i].y * cosA);
    }

    return vertices;
}

void RigidRect::draw() const {
    Rectangle rect = { pos.x, pos.y, width, height };
    Vector2 origin = { width / 2.0f, height / 2.0f }; // Centro de rotação
    DrawRectanglePro(rect, origin, angle * RAD2DEG, cor);
}

float RigidRect::area() const {
    return width * height;
}

float RigidRect::inertia() const {
    return (mass / 12.0f) * (width * width + height * height);
}

float RigidRect::getWidth()  const { return width; }
float RigidRect::getHeight() const { return height; }

Triangle::Triangle(Vector2 pos, float side, float mass,
                   float restitution, float friction, bool isStatic)
    : Body(pos, mass, restitution, friction, isStatic), side(side) {}

void Triangle::draw() const {
    std::vector<Vector2> vtx = getVertices();
    DrawTriangle(vtx[0], vtx[1], vtx[2], GREEN);
}

std::vector<Vector2> Triangle::getVertices() const {
    std::vector<Vector2> vertices(3);

    float h = side * sqrtf(3.0f) / 2.0f;

    Vector2 localVertices[3] = {
        { 0.0f, -h * (2.0f / 3.0f) },
        { -side / 2.0f, h * (1.0f / 3.0f) },
        {  side / 2.0f, h * (1.0f / 3.0f) }
    };

    float cosA = cosf(angle);
    float sinA = sinf(angle);

    for (int i = 0; i < 3; i++) {
        vertices[i].x = pos.x + (localVertices[i].x * cosA - localVertices[i].y * sinA);
        vertices[i].y = pos.y + (localVertices[i].x * sinA + localVertices[i].y * cosA);
    }

    return vertices;
}


float Triangle::area() const {
    return (sqrtf(3.0f) / 4.0f) * side * side;
}

float Triangle::inertia() const {
    return (mass / 12.0f) * side * side;
}

float Triangle::getSide() const { return side; }
