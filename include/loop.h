#ifndef LOOP_H_INCLUDED
#define LOOP_H_INCLUDED

#include "proj_headers.h"
#include "body.h"

const int   SCREEN_WIDTH  = 1500;
const int   SCREEN_HEIGHT = 700;
const float GRAVITY       = 500.0f;
const float GROUND_Y      = SCREEN_HEIGHT - 20.0f;

struct CollisionInfo {
    bool    colliding = false;
    Vector2 normal    = {0.0f, 0.0f};
    float   depth     = 0.0f;
};

class Simulation {
private:
    std::vector<Body*> bodies;
    bool paused;
    bool showInfo;

    void applyGravity(float dt);
    void resolveGroundCollision(Body* body);
    void resolveWallCollision(Body* body);
    void resolveBodyCollisions();
    void applyImpulse(Body* A, Body* B, const CollisionInfo& info);

    CollisionInfo detectCollision(Body* A, Body* B);
    CollisionInfo detectCircleCircle(Circumference* a, Circumference* b);
    CollisionInfo detectCirclePoly(Circumference* c, const std::vector<Vector2>& poly);
    CollisionInfo detectPolyPoly(const std::vector<Vector2>& A, const std::vector<Vector2>& B);

    void spawnCircle(Vector2 pos);
    void spawnRectangle(Vector2 pos);
    void spawnTriangle(Vector2 pos);
    void spawnStaticGround();

    void handleInput();
    Body* draggedBody;
    bool draggedBodyOldStatic;

    void drawBodies()  const;
    void drawGround()  const;
    void drawHUD()     const;

public:
    Simulation();
    ~Simulation();
    void run();
};

#endif
