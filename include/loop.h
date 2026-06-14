#ifndef LOOP_H_INCLUDED
#define LOOP_H_INCLUDED

#include "proj_headers.h"
#include "body.h"
#include <vector>

const int   SCREEN_WIDTH  = 1500;
const int   SCREEN_HEIGHT = 700;
const float GRAVITY       = 500.0f;
const float GROUND_Y      = SCREEN_HEIGHT - 20.0f;

// informações de uma colisão detectada
struct CollisionInfo {
    bool    colliding = false;
    Vector2 normal    = {0.0f, 0.0f};  // de A para B
    float   depth     = 0.0f;          // profundidade de penetração
};

class Simulation {
private:
    std::vector<Body*> bodies;
    bool paused;
    bool showInfo;


    // ── física ──────────────────────────────────
    void applyGravity(float dt);
    void resolveGroundCollision(Body* body);
    void resolveWallCollision(Body* body);
    void resolveBodyCollisions();
    void applyImpulse(Body* A, Body* B, const CollisionInfo& info);

    // ── detecção de colisão ──────────────────────
    CollisionInfo detectCollision(Body* A, Body* B);
    CollisionInfo detectCircleCircle(Circumference* a, Circumference* b);
    CollisionInfo detectCirclePoly(Circumference* c, const std::vector<Vector2>& poly);
    CollisionInfo detectPolyPoly(const std::vector<Vector2>& A,
                                 const std::vector<Vector2>& B);

    // ── spawn ────────────────────────────────────
    void spawnCircle(Vector2 pos);
    void spawnRectangle(Vector2 pos);
    void spawnTriangle(Vector2 pos);
    void spawnStaticGround();

    // ── input ────────────────────────────────────
    void handleInput();
    Body* draggedBody;
    bool draggedBodyOldStatic;

    // ── desenho ─────────────────────────────────
    void drawBodies()  const;
    void drawGround()  const;
    void drawHUD()     const;

public:
    Simulation();
    ~Simulation();
    void run();
};

#endif // LOOP_H_INCLUDED
