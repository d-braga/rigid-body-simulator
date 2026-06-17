#include "loop.h"

inline float CrossProduct2D(Vector2 a, Vector2 b) {
    return a.x * b.y - a.y * b.x;
}

static Vector2 v2Norm(Vector2 v) {
    float len = Vector2Length(v);
    if (len < 1e-8f) return {0.0f, 0.0f};
    return {v.x/len, v.y/len};
}

static float v2Dot(Vector2 a, Vector2 b) {
    return a.x*b.x + a.y*b.y;
}

static std::vector<Vector2> rectVerts(RigidRect* r) {
    float hw = r->getWidth()  / 2.0f;
    float hh = r->getHeight() / 2.0f;
    Vector2 p = r->getPos();
    return { {p.x-hw, p.y-hh}, {p.x+hw, p.y-hh},
             {p.x+hw, p.y+hh}, {p.x-hw, p.y+hh} };
}

static std::vector<Vector2> triVerts(Triangle* t) {
    float s = t->getSide();
    float h = s * sqrtf(3.0f) / 2.0f;
    Vector2 p = t->getPos();
    return { {p.x,        p.y - h*2.0f/3.0f},
             {p.x - s/2,  p.y + h/3.0f     },
             {p.x + s/2,  p.y + h/3.0f     } };
}

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

=static void projectPoly(const std::vector<Vector2>& vtx, Vector2 axis, float& minProj, float& maxProj) {
    minProj = Vector2DotProduct(vtx[0], axis);
    maxProj = minProj;
    for (size_t i = 1; i < vtx.size(); ++i) {
        float dot = Vector2DotProduct(vtx[i], axis);
        if (dot < minProj) minProj = dot;
        if (dot > maxProj) maxProj = dot;
    }
}

=static void projectCircle(Vector2 center, float radius, Vector2 axis, float& minProj, float& maxProj) {
    float projection = Vector2DotProduct(center, axis);
    minProj = projection - radius;
    maxProj = projection + radius;
}

bool isPointInsideBody(Vector2 point, Body* body) {
    Circumference* circ = dynamic_cast<Circumference*>(body);
    if (circ) {
        return CheckCollisionPointCircle(point, body->getPos(), circ->getRadius());
    }

    std::vector<Vector2> vertices = body->getVertices();
    if (!vertices.empty()) {
        return CheckCollisionPointPoly(point, vertices.data(), (int)vertices.size());
    }

    return false;
}

Simulation::Simulation() : paused(false), showInfo(true), draggedBody(nullptr), draggedBodyOldStatic(false) {
    InitWindow(SCREEN_WIDTH, SCREEN_HEIGHT, "Simulador de Corpos Rígidos :)");
    SetTargetFPS(60);
    spawnStaticGround();
}

Simulation::~Simulation() {
    for (Body* b : bodies) delete b;
    bodies.clear();
    CloseWindow();
}

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

void Simulation::handleInput() {
    Vector2 mouse = GetMousePosition();

    if (IsMouseButtonPressed(MOUSE_BUTTON_LEFT)) {
        draggedBody = nullptr;
        for (int i = 0; i < bodies.size() - 1; i++) {
            if (isPointInsideBody(mouse, bodies[i])) {
                draggedBody = bodies[i];
                draggedBodyOldStatic = draggedBody->getIsStatic();
                draggedBody->setStatic(true);
                draggedBody->setSpeed({ 0.0f, 0.0f });
                draggedBody->setAngularSpeed(0.0f);
                break;
            }
        }
        if(draggedBody == nullptr) spawnCircle(mouse);
    }

    if (IsMouseButtonDown(MOUSE_BUTTON_LEFT) && draggedBody != nullptr) {
        draggedBody->setPos(mouse);
        draggedBody->setSpeed({ 0.0f, 0.0f });
        draggedBody->setAngularSpeed(0.0f);
    }

    if (IsMouseButtonReleased(MOUSE_BUTTON_LEFT) && draggedBody != nullptr) {
        draggedBody->setStatic(draggedBodyOldStatic);

        if (!draggedBody->getIsStatic()) {
            float dt = GetFrameTime();
            if (dt > 0.0f) {
                Vector2 mouseDelta = GetMouseDelta();
                Vector2 launchVelocity = {mouseDelta.x / dt, mouseDelta.y / dt};
                float momentumMultiplier = 0.7f;
                launchVelocity = Vector2Scale(launchVelocity, momentumMultiplier);
                draggedBody->setSpeed(launchVelocity);
                float angularFactor = Vector2Length(launchVelocity) * 0.01f;
                draggedBody->setAngularSpeed(angularFactor);
            }
        }
        draggedBody = nullptr;
    }

    if (IsMouseButtonPressed(MOUSE_BUTTON_RIGHT))  spawnRectangle(mouse);
    if (IsMouseButtonPressed(MOUSE_BUTTON_MIDDLE)) spawnTriangle(mouse);
    if (IsKeyPressed(KEY_P)) paused = !paused;
    if (IsKeyPressed(KEY_I)) showInfo = !showInfo;
    if (IsKeyPressed(KEY_C)) {
        for (int i = 1; i < (int)bodies.size(); i++) delete bodies[i];
        bodies.erase(bodies.begin() + 1, bodies.end());
    }
}

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
    bodies.push_back(new RigidRect(gp, (float)SCREEN_WIDTH, 20.0f, 0.0f, 0.5f, 0.3f, true, PURPLE));
}

void Simulation::applyGravity(float dt) {
    for (Body* b : bodies)
        if (!b->getIsStatic())
            b->applyForce({0.0f, GRAVITY * b->getMass()});
}

void Simulation::resolveGroundCollision(Body* body) {
    if (body->getIsStatic()) return;

    float slop = 0.1f;
    float speedThreshold = 4.0f;
    float angularThreshold = 0.3f;

    // ── CASO CÍRCULO ──
    Circumference* circ = dynamic_cast<Circumference*>(body);
    if (circ) {
        slop = 0.1f;

        float radius = circ->getRadius();
        float depth = (body->getPos().y + radius) - GROUND_Y;

        if (depth > slop) {
            body->setPos({ body->getPos().x, body->getPos().y - (depth - slop) });
            Vector2 r = { 0.0f, radius };

            float v_contact_x = body->getSpeed().x - body->getAngularSpeed() * r.y;
            float v_contact_y = body->getSpeed().y;

            float invMass = 1.0f / body->getMass();
            float invInertia = 1.0f / body->inertia();

            float mv = 0.0f;
            if (v_contact_y > 0) {
                mv = -(body->getRestitution()) * v_contact_y / invMass;
                body->setSpeed({ body->getSpeed().x, mv * invMass });
            }

            float tangentMass = invMass + (r.y * r.y) * invInertia;
            float mh = -v_contact_x / tangentMass;
            float maxFriction = fmaxf(mv, 0.0f) * body->getFriction();
            if (maxFriction <= 0.0f) {
              maxFriction = body->getMass() * GRAVITY * GetFrameTime() * body->getFriction();
            }
            mh = Clamp(mh, -maxFriction, maxFriction);

            body->setSpeed({ body->getSpeed().x + mh * invMass, body->getSpeed().y });
            body->setAngularSpeed(body->getAngularSpeed() + (-r.y * mh) * invInertia);
        }
        return;
    }

    std::vector<Vector2> vertices = body->getVertices();

    Vector2 deepestVertex = vertices[0];
    float maxDepth = vertices[0].y - GROUND_Y;

    for (int i = 1; i < vertices.size(); i++) {
        float depth = vertices[i].y - GROUND_Y;
        if (depth > maxDepth) {
            maxDepth = depth;
            deepestVertex = vertices[i];
        }
    }

    if (maxDepth > slop) {
        Vector2 r = Vector2Subtract(deepestVertex, body->getPos());
        float v_contact_x = body->getSpeed().x - body->getAngularSpeed() * r.y;
        float v_contact_y = body->getSpeed().y + body->getAngularSpeed() * r.x;
        float invMass = 1.0f / body->getMass();
        float invInertia = 1.0f / body->inertia();
        float mv = 0.0f;
        if (v_contact_y > 0.0f) {
            float normalMass = invMass + (r.x * r.x) * invInertia;
            mv = -(1.0f + body->getRestitution()) * v_contact_y / normalMass;

            body->setSpeed({ body->getSpeed().x, body->getSpeed().y + mv * invMass });
            body->setAngularSpeed(body->getAngularSpeed() + (r.x * mv) * invInertia);
        }
        v_contact_x = body->getSpeed().x - body->getAngularSpeed() * r.y;
        float tangentMass = invMass + (r.y * r.y) * invInertia;
        float mh = -v_contact_x / tangentMass;

        float maxFriction = fmaxf(mv, 0.0f) * body->getFriction();
        if (maxFriction <= 0.0f) {
            maxFriction = body->getMass() * GRAVITY * GetFrameTime() * body->getFriction();
        }
        mh = Clamp(mh, -maxFriction, maxFriction);

        body->setSpeed({ body->getSpeed().x + mh * invMass, body->getSpeed().y });
        body->setAngularSpeed(body->getAngularSpeed() + (-r.y * mh) * invInertia);
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

CollisionInfo Simulation::detectCircleCircle(Circumference* a, Circumference* b) {
    CollisionInfo info;
    float dx   = b->getPos().x - a->getPos().x;
    float dy   = b->getPos().y - a->getPos().y;
    float dist = sqrtf(dx*dx + dy*dy);
    float rsum = a->getRadius() + b->getRadius();
    if (dist >= rsum) return info;

    info.colliding = true;
    info.depth  = rsum - dist;
    info.normal = (dist < 1e-8f) ? Vector2{1.0f, 0.0f} : Vector2{dx/dist, dy/dist};
    return info;
}

CollisionInfo Simulation::detectCirclePoly(Circumference* c, const std::vector<Vector2>& poly) {
    CollisionInfo info;
    info.colliding = false;
    info.depth = FLT_MAX;
    info.normal = { 0.0f, 0.0f };

    size_t count = poly.size();
    if (count == 0) return info;

    Vector2 circleCenter = c->getPos();
    float circleRadius = c->getRadius();

    std::vector<Vector2> axes;

    for (size_t i = 0; i < count; ++i) {
        Vector2 p1 = poly[i];
        Vector2 p2 = poly[(i + 1) % count];
        Vector2 edge = Vector2Subtract(p2, p1);
        Vector2 normal = { -edge.y, edge.x };
        axes.push_back(Vector2Normalize(normal));
    }

    Vector2 closestVertex = poly[0];
    float minDstSq = Vector2DistanceSqr(circleCenter, poly[0]);

    for (size_t i = 1; i < count; ++i) {
        float dstSq = Vector2DistanceSqr(circleCenter, poly[i]);
        if (dstSq < minDstSq) {
            minDstSq = dstSq;
            closestVertex = poly[i];
        }
    }

    Vector2 axisToVertex = Vector2Subtract(closestVertex, circleCenter);
    if (Vector2LengthSqr(axisToVertex) > 0.0f) {
        axes.push_back(Vector2Normalize(axisToVertex));
    }

    for (Vector2 axis : axes) {
        float minA, maxA;
        float minB, maxB;

        projectPoly(poly, axis, minA, maxA);
        projectCircle(circleCenter, circleRadius, axis, minB, maxB);

        if (maxA < minB || maxB < minA) {
            return CollisionInfo();
        }

        float overlap = fminf(maxA, maxB) - fmaxf(minA, minB);

        if (overlap < info.depth) {
            info.depth = overlap;
            info.normal = axis;
        }
    }

    info.colliding = true;
    Vector2 polyCenter = { 0.0f, 0.0f };
    for (const auto& v : poly) {
        polyCenter = Vector2Add(polyCenter, v);
    }
    polyCenter = Vector2Scale(polyCenter, 1.0f / count);

    Vector2 dirCircleToPoly = Vector2Subtract(polyCenter, circleCenter);
    if (Vector2DotProduct(dirCircleToPoly, info.normal) < 0.0f) {
        info.normal = Vector2Scale(info.normal, -1.0f);
    }

    return info;
}

CollisionInfo Simulation::detectPolyPoly(const std::vector<Vector2>& A, const std::vector<Vector2>& B) {
    CollisionInfo info;
    info.colliding = false;
    info.depth = FLT_MAX;
    info.normal = { 0.0f, 0.0f };

    size_t countA = A.size();
    size_t countB = B.size();

    std::vector<Vector2> axes;

    for (size_t i = 0; i < countA; ++i) {
        Vector2 p1 = A[i];
        Vector2 p2 = A[(i + 1) % countA];
        Vector2 edge = Vector2Subtract(p2, p1);
        Vector2 normal = { -edge.y, edge.x };
        axes.push_back(Vector2Normalize(normal));
    }

    for (size_t i = 0; i < countB; ++i) {
        Vector2 p1 = B[i];
        Vector2 p2 = B[(i + 1) % countB];
        Vector2 edge = Vector2Subtract(p2, p1);
        Vector2 normal = { -edge.y, edge.x };
        axes.push_back(Vector2Normalize(normal));
    }

    for (Vector2 axis : axes) {
        float minA, maxA;
        float minB, maxB;

        projectPoly(A, axis, minA, maxA);
        projectPoly(B, axis, minB, maxB);

        if (maxA < minB || maxB < minA) {
            return CollisionInfo();
        }

        float overlap = fminf(maxA, maxB) - fmaxf(minA, minB);

        if (overlap < info.depth) {
            info.depth = overlap;
            info.normal = axis;
        }
    }

    info.colliding = true;

    Vector2 centerA = { 0, 0 };
    Vector2 centerB = { 0, 0 };
    for (const auto& v : A) { centerA = Vector2Add(centerA, v); }
    for (const auto& v : B) { centerB = Vector2Add(centerB, v); }
    centerA = Vector2Scale(centerA, 1.0f / countA);
    centerB = Vector2Scale(centerB, 1.0f / countB);

    Vector2 dirAtoB = Vector2Subtract(centerB, centerA);
    if (Vector2DotProduct(dirAtoB, info.normal) < 0.0f) {
        info.normal = Vector2Scale(info.normal, -1.0f);
    }

    return info;
}

CollisionInfo Simulation::detectCollision(Body* A, Body* B) {
    Circumference* circA = dynamic_cast<Circumference*>(A);
    Circumference* circB = dynamic_cast<Circumference*>(B);
    if (circA && circB) {
        return detectCircleCircle(circA, circB);
    }

    if (!circA && !circB) {
        return detectPolyPoly(A->getVertices(), B->getVertices());
    }

    if (circA && !circB) {
        return detectCirclePoly(circA, B->getVertices());
    }

    if (circB && !circA) {
        CollisionInfo info = detectCirclePoly(circB, A->getVertices());
        info.normal = Vector2Scale(info.normal, -1.0f);
        return info;
    }

    return CollisionInfo();
}

void Simulation::applyImpulse(Body* A, Body* B, const CollisionInfo& info) {
    if (A->getIsStatic() && B->getIsStatic()) return;

    float invMassA = A->getIsStatic() ? 0.0f : (1.0f / A->getMass());
    float invMassB = B->getIsStatic() ? 0.0f : (1.0f / B->getMass());

    const float percent = 0.65f;
    const float slop = 0.01f;

    float correctionMagnitude = fmaxf(info.depth - slop, 0.0f) / (invMassA + invMassB) * percent;
    Vector2 correctionVector = Vector2Scale(info.normal, correctionMagnitude);

    if (!A->getIsStatic()) A->setPos(Vector2Subtract(A->getPos(), Vector2Scale(correctionVector, invMassA)));
    if (!B->getIsStatic()) B->setPos(Vector2Add(B->getPos(), Vector2Scale(correctionVector, invMassB)));

    Vector2 contactPoint = Vector2Lerp(A->getPos(), B->getPos(), 0.5f);

    Vector2 rA = Vector2Subtract(contactPoint, A->getPos());
    Vector2 rB = Vector2Subtract(contactPoint, B->getPos());

    Vector2 vA_contact = { A->getSpeed().x - A->getAngularSpeed() * rA.y,
                           A->getSpeed().y + A->getAngularSpeed() * rA.x };
    Vector2 vB_contact = { B->getSpeed().x - B->getAngularSpeed() * rB.y,
                           B->getSpeed().y + B->getAngularSpeed() * rB.x };

    Vector2 relVel = Vector2Subtract(vB_contact, vA_contact);
    float velAlongNormal = Vector2DotProduct(relVel, info.normal);
    if (velAlongNormal > 0.0f) return;

    float e = fminf(A->getRestitution(), B->getRestitution());

    if (fabsf(velAlongNormal) < 20.0f) {
        e = 0.0f;
    }

    auto CrossProduct2D = [](Vector2 v1, Vector2 v2) { return v1.x * v2.y - v1.y * v2.x; };

    float rA_cross_N = CrossProduct2D(rA, info.normal);
    float rB_cross_N = CrossProduct2D(rB, info.normal);

    float invInertiaA = A->getIsStatic() ? 0.0f : (1.0f / A->inertia());
    float invInertiaB = B->getIsStatic() ? 0.0f : (1.0f / B->inertia());

    float totalInverseMass = invMassA + invMassB +
                             (rA_cross_N * rA_cross_N) * invInertiaA +
                             (rB_cross_N * rB_cross_N) * invInertiaB;

    float impulseScalar = -(1.0f + e) * velAlongNormal / totalInverseMass;
    Vector2 impulse = Vector2Scale(info.normal, impulseScalar);

    if (!A->getIsStatic()) {
        A->setSpeed(Vector2Subtract(A->getSpeed(), Vector2Scale(impulse, invMassA)));
        float torqueA = CrossProduct2D(rA, Vector2Scale(impulse, -1.0f));
        A->setAngularSpeed(A->getAngularSpeed() + torqueA * invInertiaA);
    }
    if (!B->getIsStatic()) {
        B->setSpeed(Vector2Add(B->getSpeed(), Vector2Scale(impulse, invMassB)));
        float torqueB = CrossProduct2D(rB, impulse);
        B->setAngularSpeed(B->getAngularSpeed() + torqueB * invInertiaB);
    }
    Vector2 tangent = { -info.normal.y, info.normal.x };
    float velAlongTangent = Vector2DotProduct(relVel, tangent);

    float rA_cross_T = CrossProduct2D(rA, tangent);
    float rB_cross_T = CrossProduct2D(rB, tangent);
    float tangentShared = invMassA + invMassB +
                          (rA_cross_T * rA_cross_T) * invInertiaA +
                          (rB_cross_T * rB_cross_T) * invInertiaB;

    float frictionScalar = -velAlongTangent / tangentShared;

    float mu = (A->getFriction() + B->getFriction()) / 2.0f;
    frictionScalar = fmaxf(-impulseScalar * mu, fminf(frictionScalar, impulseScalar * mu));

    Vector2 frictionImpulse = Vector2Scale(tangent, frictionScalar);

    if (!A->getIsStatic()) {
        A->setSpeed(Vector2Subtract(A->getSpeed(), Vector2Scale(frictionImpulse, invMassA)));
        A->setAngularSpeed(A->getAngularSpeed() + CrossProduct2D(rA, Vector2Scale(frictionImpulse, -1.0f)) * invInertiaA);
    }
    if (!B->getIsStatic()) {
        B->setSpeed(Vector2Add(B->getSpeed(), Vector2Scale(frictionImpulse, invMassB)));
        B->setAngularSpeed(B->getAngularSpeed() + CrossProduct2D(rB, frictionImpulse) * invInertiaB);
    }
}

void Simulation::resolveBodyCollisions() {
    for (int i = 0; i < (int)bodies.size(); i++) {
        for (int j = i + 1; j < (int)bodies.size(); j++) {
            Body* A = bodies[i];
            Body* B = bodies[j];

            if (A->getIsStatic() && B->getIsStatic()) continue;
            if(Vector2Length(Vector2Subtract(A->getPos(), B->getPos())) < 10.f) continue;
            CollisionInfo info = detectCollision(A, B);
            if (info.colliding)
                applyImpulse(A, B, info);
        }
    }

    float angularThreshold = 0.5f;
    float speedThreshold = 1.5f;

    for (int i = 0; i < (int)bodies.size(); i++) {
        if (fabsf(bodies[i]->getSpeed().x) < speedThreshold) bodies[i]->setSpeed({ 0.0f, bodies[i]->getSpeed().y });
        if (fabsf(bodies[i]->getSpeed().y) < speedThreshold) bodies[i]->setSpeed({ bodies[i]->getSpeed().x, 0.0f });
        if (fabsf(bodies[i]->getAngularSpeed()) < angularThreshold) bodies[i]->setAngularSpeed(0.0f);

    }
}

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
    if (paused) DrawText("PAUSADO", SCREEN_WIDTH/2 - 40, SCREEN_HEIGHT/2, 24, RED);
}

int main() {
    Simulation sim;
    sim.run();
    return 0;
}
