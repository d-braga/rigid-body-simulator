#include "loop.h"
#include <cmath>
#include <string>
#include <limits>
#include <algorithm>

// ═══════════════════════════════════════════════
//  Funções auxiliares de geometria (livres)
// ═══════════════════════════════════════════════

static Vector2 v2Norm(Vector2 v) {
    float len = sqrtf(v.x*v.x + v.y*v.y);
    if (len < 1e-8f) return {0.0f, 0.0f};
    return {v.x/len, v.y/len};
}

static float v2Dot(Vector2 a, Vector2 b) {
    return a.x*b.x + a.y*b.y;
}

// vértices do RigidRect (AABB centrado em pos)
static std::vector<Vector2> rectVerts(RigidRect* r) {
    float hw = r->getWidth()  / 2.0f;
    float hh = r->getHeight() / 2.0f;
    Vector2 p = r->getPos();
    return { {p.x-hw, p.y-hh}, {p.x+hw, p.y-hh},
             {p.x+hw, p.y+hh}, {p.x-hw, p.y+hh} };
}

// vértices do triângulo equilátero centrado em pos
static std::vector<Vector2> triVerts(Triangle* t) {
    float s = t->getSide();
    float h = s * sqrtf(3.0f) / 2.0f;
    Vector2 p = t->getPos();
    return { {p.x,        p.y - h*2.0f/3.0f},
             {p.x - s/2,  p.y + h/3.0f     },
             {p.x + s/2,  p.y + h/3.0f     } };
}

// normais das arestas de um polígono (eixos SAT)
static std::vector<Vector2> polyAxes(const std::vector<Vector2>& v) {
    std::vector<Vector2> axes;
    int n = (int)v.size();
    for (int i = 0; i < n; i++) {
        Vector2 edge = { v[(i+1)%n].x - v[i].x,
                         v[(i+1)%n].y - v[i].y };
        axes.push_back(v2Norm({-edge.y, edge.x}));
    }
    return axes;
}

// projeção de polígono num eixo
static void projectPoly(const std::vector<Vector2>& v, Vector2 axis,
                         float& mn, float& mx) {
    mn = mx = v2Dot(v[0], axis);
    for (auto& p : v) {
        float d = v2Dot(p, axis);
        mn = std::min(mn, d);
        mx = std::max(mx, d);
    }
}

// projeção de círculo num eixo
static void projectCircle(Vector2 center, float radius, Vector2 axis,
                           float& mn, float& mx) {
    float proj = v2Dot(center, axis);
    mn = proj - radius;
    mx = proj + radius;
}

// ═══════════════════════════════════════════════
//  Constructor / Destructor
// ═══════════════════════════════════════════════

Simulation::Simulation() : paused(false), showInfo(true) {
    InitWindow(SCREEN_WIDTH, SCREEN_HEIGHT, "Rigid Body Simulator");
    SetTargetFPS(60);
    spawnStaticGround();
}

Simulation::~Simulation() {
    for (Body* b : bodies) delete b;
    bodies.clear();
    CloseWindow();
}

// ═══════════════════════════════════════════════
//  Loop principal
// ═══════════════════════════════════════════════

void Simulation::run() {
    while (!WindowShouldClose()) {
        float dt = GetFrameTime();

        handleInput();

        if (!paused) {
            applyGravity(dt);
            for (Body* b : bodies) {
                b->update(dt);
                resolveGroundCollision(b);
                resolveWallCollision(b);
            }
            resolveBodyCollisions();
        }

        BeginDrawing();
            ClearBackground(RAYWHITE);
            drawGround();
            drawBodies();
            drawHUD();
        EndDrawing();
    }
}

// ═══════════════════════════════════════════════
//  Input
// ═══════════════════════════════════════════════

void Simulation::handleInput() {
    Vector2 mouse = GetMousePosition();

    if (IsMouseButtonPressed(MOUSE_BUTTON_LEFT))   spawnCircle(mouse);
    if (IsMouseButtonPressed(MOUSE_BUTTON_RIGHT))  spawnRectangle(mouse);
    if (IsMouseButtonPressed(MOUSE_BUTTON_MIDDLE)) spawnTriangle(mouse);

    if (IsKeyPressed(KEY_P)) paused = !paused;
    if (IsKeyPressed(KEY_I)) showInfo = !showInfo;

    if (IsKeyPressed(KEY_C)) {
        for (int i = 1; i < (int)bodies.size(); i++) delete bodies[i];
        bodies.erase(bodies.begin() + 1, bodies.end());
    }
}

// ═══════════════════════════════════════════════
//  Spawn
// ═══════════════════════════════════════════════

void Simulation::spawnCircle(Vector2 pos) {
    bodies.push_back(new Circumference(pos, 20.0f, 1.0f));
}

void Simulation::spawnRectangle(Vector2 pos) {
    bodies.push_back(new RigidRect(pos, 40.0f, 40.0f, 1.0f));
}

void Simulation::spawnTriangle(Vector2 pos) {
    bodies.push_back(new Triangle(pos, 50.0f, 1.0f));
}

void Simulation::spawnStaticGround() {
    Vector2 gp = { SCREEN_WIDTH / 2.0f, GROUND_Y + 10.0f };
    bodies.push_back(new RigidRect(gp, (float)SCREEN_WIDTH, 20.0f,
                                   0.0f, 0.5f, 0.3f, true));
}

// ═══════════════════════════════════════════════
//  Física — gravidade e colisões com bordas
// ═══════════════════════════════════════════════

void Simulation::applyGravity(float dt) {
    for (Body* b : bodies)
        if (!b->getIsStatic())
            b->applyForce({0.0f, GRAVITY * b->getMass()});
}

void Simulation::resolveGroundCollision(Body* body) {
    if (body->getIsStatic()) return;

    if (Circumference* c = dynamic_cast<Circumference*>(body)) {
        float bottom = c->getPos().y + c->getRadius();
        if (bottom >= GROUND_Y) {
            Vector2 p = c->getPos();
            p.y = GROUND_Y - c->getRadius();
            c->setPos(p);
            Vector2 v = c->getSpeed();
            v.y = -v.y * c->getRestitution();
            v.x *=  1.0f - c->getFriction();
            c->setSpeed(v);
        }
        return;
    }
    if (RigidRect* r = dynamic_cast<RigidRect*>(body)) {
        float bottom = r->getPos().y + r->getHeight() / 2.0f;
        if (bottom >= GROUND_Y) {
            Vector2 p = r->getPos();
            p.y = GROUND_Y - r->getHeight() / 2.0f;
            r->setPos(p);
            Vector2 v = r->getSpeed();
            v.y = -v.y * r->getRestitution();
            v.x *=  1.0f - r->getFriction();
            r->setSpeed(v);
        }
        return;
    }
    if (Triangle* t = dynamic_cast<Triangle*>(body)) {
        float h      = t->getSide() * sqrtf(3.0f) / 2.0f;
        float bottom = t->getPos().y + h / 3.0f;
        if (bottom >= GROUND_Y) {
            Vector2 p = t->getPos();
            p.y = GROUND_Y - h / 3.0f;
            t->setPos(p);
            Vector2 v = t->getSpeed();
            v.y = -v.y * t->getRestitution();
            v.x *=  1.0f - t->getFriction();
            t->setSpeed(v);
        }
    }
}

void Simulation::resolveWallCollision(Body* body) {
    if (body->getIsStatic()) return;

    if (Circumference* c = dynamic_cast<Circumference*>(body)) {
        Vector2 p = c->getPos(), v = c->getSpeed();
        if (p.x - c->getRadius() <= 0) {
            p.x =  c->getRadius();
            v.x = -v.x * c->getRestitution();
        }
        if (p.x + c->getRadius() >= SCREEN_WIDTH) {
            p.x =  SCREEN_WIDTH - c->getRadius();
            v.x = -v.x * c->getRestitution();
        }
        c->setPos(p); c->setSpeed(v);
        return;
    }
    if (RigidRect* r = dynamic_cast<RigidRect*>(body)) {
        Vector2 p = r->getPos(), v = r->getSpeed();
        float hw = r->getWidth() / 2.0f;
        if (p.x - hw <= 0) {
            p.x =  hw;
            v.x = -v.x * r->getRestitution();
        }
        if (p.x + hw >= SCREEN_WIDTH) {
            p.x =  SCREEN_WIDTH - hw;
            v.x = -v.x * r->getRestitution();
        }
        r->setPos(p); r->setSpeed(v);
        return;
    }
    if (Triangle* t = dynamic_cast<Triangle*>(body)) {
        Vector2 p = t->getPos(), v = t->getSpeed();
        float half = t->getSide() / 2.0f;
        if (p.x - half <= 0) {
            p.x =  half;
            v.x = -v.x * t->getRestitution();
        }
        if (p.x + half >= SCREEN_WIDTH) {
            p.x =  SCREEN_WIDTH - half;
            v.x = -v.x * t->getRestitution();
        }
        t->setPos(p); t->setSpeed(v);
    }
}

// ═══════════════════════════════════════════════
//  Detecção de colisão (SAT)
// ═══════════════════════════════════════════════

CollisionInfo Simulation::detectCircleCircle(Circumference* a, Circumference* b) {
    CollisionInfo info;
    float dx   = b->getPos().x - a->getPos().x;
    float dy   = b->getPos().y - a->getPos().y;
    float dist = sqrtf(dx*dx + dy*dy);
    float rsum = a->getRadius() + b->getRadius();
    if (dist >= rsum) return info;

    info.colliding = true;
    info.depth  = rsum - dist;
    info.normal = (dist < 1e-8f) ? Vector2{1.0f, 0.0f}
                                  : Vector2{dx/dist, dy/dist};
    return info;
}

CollisionInfo Simulation::detectCirclePoly(Circumference* c,
                                            const std::vector<Vector2>& poly) {
    CollisionInfo info;
    info.depth = std::numeric_limits<float>::max();

    // eixos das arestas do polígono
    auto axes = polyAxes(poly);

    // eixo extra: do vértice mais próximo ao centro do círculo
    float bestDist = std::numeric_limits<float>::max();
    Vector2 closest = poly[0];
    for (auto& v : poly) {
        float d = (c->getPos().x-v.x)*(c->getPos().x-v.x)
                + (c->getPos().y-v.y)*(c->getPos().y-v.y);
        if (d < bestDist) { bestDist = d; closest = v; }
    }
    axes.push_back(v2Norm({c->getPos().x - closest.x,
                           c->getPos().y - closest.y}));

    for (auto& axis : axes) {
        if (axis.x == 0.0f && axis.y == 0.0f) continue;
        float minC, maxC, minP, maxP;
        projectCircle(c->getPos(), c->getRadius(), axis, minC, maxC);
        projectPoly(poly, axis, minP, maxP);

        if (maxC < minP || maxP < minC) {
            info.colliding = false;
            return info;
        }
        float overlap = std::min(maxC, maxP) - std::max(minC, minP);
        if (overlap < info.depth) {
            info.depth  = overlap;
            info.normal = axis;
        }
    }
    info.colliding = true;
    return info;
}

CollisionInfo Simulation::detectPolyPoly(const std::vector<Vector2>& A,
                                          const std::vector<Vector2>& B) {
    CollisionInfo info;
    info.depth = std::numeric_limits<float>::max();

    // testa eixos de A e depois de B
    for (int pass = 0; pass < 2; pass++) {
        const auto& src = (pass == 0) ? A : B;
        for (auto& axis : polyAxes(src)) {
            float minA, maxA, minB, maxB;
            projectPoly(A, axis, minA, maxA);
            projectPoly(B, axis, minB, maxB);

            if (maxA < minB || maxB < minA) {
                info.colliding = false;
                return info;
            }
            float overlap = std::min(maxA, maxB) - std::max(minA, minB);
            if (overlap < info.depth) {
                info.depth  = overlap;
                info.normal = axis;
            }
        }
    }
    info.colliding = true;
    return info;
}

CollisionInfo Simulation::detectCollision(Body* A, Body* B) {
    CollisionInfo info;

    Circumference* ca = dynamic_cast<Circumference*>(A);
    Circumference* cb = dynamic_cast<Circumference*>(B);
    RigidRect*     ra = dynamic_cast<RigidRect*>(A);
    RigidRect*     rb = dynamic_cast<RigidRect*>(B);
    Triangle*      ta = dynamic_cast<Triangle*>(A);
    Triangle*      tb = dynamic_cast<Triangle*>(B);

    // ── Círculo vs Círculo ───────────────────────
    if (ca && cb)
        return detectCircleCircle(ca, cb);

    // ── Círculo vs Retângulo ─────────────────────
    if (ca && rb)
        return detectCirclePoly(ca, rectVerts(rb));

    if (ra && cb) {
        info = detectCirclePoly(cb, rectVerts(ra));
        // normal precisa apontar de A para B
        info.normal = {-info.normal.x, -info.normal.y};
        return info;
    }

    // ── Círculo vs Triângulo ─────────────────────
    if (ca && tb)
        return detectCirclePoly(ca, triVerts(tb));

    if (ta && cb) {
        info = detectCirclePoly(cb, triVerts(ta));
        info.normal = {-info.normal.x, -info.normal.y};
        return info;
    }

    // ── Retângulo vs Retângulo ───────────────────
    if (ra && rb)
        return detectPolyPoly(rectVerts(ra), rectVerts(rb));

    // ── Retângulo vs Triângulo ───────────────────
    if (ra && tb)
        return detectPolyPoly(rectVerts(ra), triVerts(tb));

    if (ta && rb)
        return detectPolyPoly(triVerts(ta), rectVerts(rb));

    // ── Triângulo vs Triângulo ───────────────────
    if (ta && tb)
        return detectPolyPoly(triVerts(ta), triVerts(tb));

    return info;
}

// ═══════════════════════════════════════════════
//  Resolução por impulso
// ═══════════════════════════════════════════════

void Simulation::applyImpulse(Body* A, Body* B, const CollisionInfo& info) {
    float invMassA = A->getIsStatic() ? 0.0f : 1.0f / A->getMass();
    float invMassB = B->getIsStatic() ? 0.0f : 1.0f / B->getMass();
    float totalInv = invMassA + invMassB;
    if (totalInv < 1e-8f) return;

    // garantir que a normal aponta de A para B
    Vector2 dir = { B->getPos().x - A->getPos().x,
                    B->getPos().y - A->getPos().y };
    Vector2 n   = info.normal;
    if (v2Dot(dir, n) < 0) n = {-n.x, -n.y};

    // ── separar corpos (correção posicional) ─────
    float corrX = n.x * info.depth / totalInv;
    float corrY = n.y * info.depth / totalInv;

    if (!A->getIsStatic()) {
        Vector2 p = A->getPos();
        p.x -= corrX * invMassA;
        p.y -= corrY * invMassA;
        A->setPos(p);
    }
    if (!B->getIsStatic()) {
        Vector2 p = B->getPos();
        p.x += corrX * invMassB;
        p.y += corrY * invMassB;
        B->setPos(p);
    }

    // ── impulso ──────────────────────────────────
    Vector2 relVel = { B->getSpeed().x - A->getSpeed().x,
                       B->getSpeed().y - A->getSpeed().y };
    float relVelN = v2Dot(relVel, n);

    if (relVelN > 0) return;  // já se afastando

    float e = (A->getRestitution() + B->getRestitution()) / 2.0f;
    float j = -(1.0f + e) * relVelN / totalInv;

    if (!A->getIsStatic()) {
        Vector2 v = A->getSpeed();
        v.x -= (j * invMassA) * n.x;
        v.y -= (j * invMassA) * n.y;
        A->setSpeed(v);
    }
    if (!B->getIsStatic()) {
        Vector2 v = B->getSpeed();
        v.x += (j * invMassB) * n.x;
        v.y += (j * invMassB) * n.y;
        B->setSpeed(v);
    }
}

void Simulation::resolveBodyCollisions() {
    for (int i = 0; i < (int)bodies.size(); i++) {
        for (int j = i + 1; j < (int)bodies.size(); j++) {
            Body* A = bodies[i];
            Body* B = bodies[j];

            if (A->getIsStatic() && B->getIsStatic()) continue;

            CollisionInfo info = detectCollision(A, B);
            if (info.colliding)
                applyImpulse(A, B, info);
        }
    }
}

// ═══════════════════════════════════════════════
//  Desenho
// ═══════════════════════════════════════════════

void Simulation::drawBodies() const {
    for (const Body* b : bodies) b->draw();
}

void Simulation::drawGround() const {
    DrawRectangle(0, (int)GROUND_Y, SCREEN_WIDTH, 20, DARKGRAY);
}

void Simulation::drawHUD() const {
    if (showInfo) {
        DrawText("Mouse Esq  -> Circulo",     10,  10, 14, DARKGRAY);
        DrawText("Mouse Dir  -> Retangulo",   10,  28, 14, DARKGRAY);
        DrawText("Mouse Meio -> Triangulo",   10,  46, 14, DARKGRAY);
        DrawText("P -> Pausar | C -> Limpar", 10,  64, 14, DARKGRAY);
        DrawText("I -> Ocultar info",         10,  82, 14, DARKGRAY);

        std::string cnt = "Corpos: " + std::to_string((int)bodies.size() - 1);
        DrawText(cnt.c_str(), 10, 100, 14, DARKGRAY);

        std::string fps = "FPS: " + std::to_string(GetFPS());
        DrawText(fps.c_str(), 10, 118, 14, DARKGRAY);
    }
    if (paused)
        DrawText("PAUSADO", SCREEN_WIDTH/2 - 40, SCREEN_HEIGHT/2, 24, RED);
}

// ═══════════════════════════════════════════════
//  main
// ═══════════════════════════════════════════════

int main() {
    Simulation sim;
    sim.run();
    return 0;
}