#include "body.h"

// ─────────────────────────────────────────
//  Body
// ─────────────────────────────────────────

Body::Body(Vector2 pos, float mass, float restitution, float friction, bool isStatic)
    : pos(pos), speed({0,0}), accel({0,0}),
      mass(mass), restitution(restitution),
      friction(friction), isStatic(isStatic) {}

void Body::update(float dt) {
    if (isStatic) return;
    speed.x += accel.x * dt;
    speed.y += accel.y * dt;
    pos.x   += speed.x * dt;
    pos.y   += speed.y * dt;
    accel = {0, 0};
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
float   Body::getRestitution() const { return restitution; }
float   Body::getFriction()    const { return friction; }
bool    Body::getIsStatic()    const { return isStatic; }

void Body::setPos(Vector2 p)   { pos = p; }
void Body::setSpeed(Vector2 v) { speed = v; }
void Body::setAccel(Vector2 a) { accel = a; }
void Body::setStatic(bool s)   { isStatic = s; }


// ─────────────────────────────────────────
//  Circumference
// ─────────────────────────────────────────

Circumference::Circumference(Vector2 pos, float radius, float mass,
                             float restitution, float friction, bool isStatic)
    : Body(pos, mass, restitution, friction, isStatic), radius(radius) {}

void Circumference::draw() const {
    DrawCircleV(pos, radius, RED);
}

float Circumference::area() const {
    return PI * radius * radius;
}

float Circumference::inertia() const {
    return 0.5f * mass * radius * radius;
}

float Circumference::getRadius() const { return radius; }


// ─────────────────────────────────────────
//  RigidRect
// ─────────────────────────────────────────

RigidRect::RigidRect(Vector2 pos, float width, float height, float mass,
                     float restitution, float friction, bool isStatic)
    : Body(pos, mass, restitution, friction, isStatic),
      width(width), height(height) {}

void RigidRect::draw() const {
    DrawRectangleV({pos.x - width/2, pos.y - height/2}, {width, height}, BLUE);
}

float RigidRect::area() const {
    return width * height;
}

float RigidRect::inertia() const {
    return (mass / 12.0f) * (width * width + height * height);
}

float RigidRect::getWidth()  const { return width; }
float RigidRect::getHeight() const { return height; }


// ─────────────────────────────────────────
//  Triangle
// ─────────────────────────────────────────

Triangle::Triangle(Vector2 pos, float side, float mass,
                   float restitution, float friction, bool isStatic)
    : Body(pos, mass, restitution, friction, isStatic), side(side) {}

void Triangle::draw() const {
    float h  = side * (sqrtf(3.0f) / 2.0f);
    Vector2 v1 = { pos.x,          pos.y - h * 2.0f/3.0f };
    Vector2 v2 = { pos.x - side/2, pos.y + h / 3.0f      };
    Vector2 v3 = { pos.x + side/2, pos.y + h / 3.0f      };
    DrawTriangle(v1, v2, v3, GREEN);
}

float Triangle::area() const {
    return (sqrtf(3.0f) / 4.0f) * side * side;
}

float Triangle::inertia() const {
    return (mass / 12.0f) * side * side;
}

float Triangle::getSide() const { return side; }