#include "loop.h"
#include <cmath>
#include <string>
#include <limits>
#include <algorithm>

// ═══════════════════════════════════════════════
//  Funções auxiliares de geometria (livres)
// ═══════════════════════════════════════════════

inline float CrossProduct2D(Vector2 a, Vector2 b) {
    return a.x * b.y - a.y * b.x;
}

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
static void projectPoly(const std::vector<Vector2>& vtx, Vector2 axis, float& minProj, float& maxProj) {
    minProj = Vector2DotProduct(vtx[0], axis);
    maxProj = minProj;
    for (size_t i = 1; i < vtx.size(); ++i) {
        float dot = Vector2DotProduct(vtx[i], axis);
        if (dot < minProj) minProj = dot;
        if (dot > maxProj) maxProj = dot;
    }
}

// projeção de círculo num eixo
static void projectCircle(Vector2 center, float radius, Vector2 axis, float& minProj, float& maxProj) {
    float projection = Vector2DotProduct(center, axis);
    minProj = projection - radius;
    maxProj = projection + radius;
}

// Função auxiliar para verificar se o ponto do mouse está dentro de um polígono ou círculo
bool isPointInsideBody(Vector2 point, Body* body) {
    // Se for um círculo
    Circumference* circ = dynamic_cast<Circumference*>(body);
    if (circ) {
        return CheckCollisionPointCircle(point, body->getPos(), circ->getRadius());
    }

    // Se for um polígono (Quadrado ou Triângulo), usamos os vértices reais rotacionados
    std::vector<Vector2> vertices = body->getVertices();
    if (!vertices.empty()) {
        // CheckCollisionPointPoly é uma função nativa da Raylib (incluída em raylib.h)
        return CheckCollisionPointPoly(point, vertices.data(), (int)vertices.size());
    }

    return false;
}

// ═══════════════════════════════════════════════
//  Constructor / Destructor
// ═══════════════════════════════════════════════

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

    // ── 1. CLIQUE INICIAL: Seleciona um corpo existente ──
    if (IsMouseButtonPressed(MOUSE_BUTTON_LEFT)) {
        draggedBody = nullptr;

        for (int i = (int)bodies.size() - 1; i >= 0; --i) {
            // Passa a variável 'mouse' atualizada para a função auxiliar
            if (isPointInsideBody(mouse, bodies[i])) {
                draggedBody = bodies[i];
                draggedBodyOldStatic = draggedBody->getIsStatic();

                // Torna temporariamente estático para não sofrer ação da gravidade no arrasto
                draggedBody->setStatic(true);
                draggedBody->setSpeed({ 0.0f, 0.0f });
                draggedBody->setAngularSpeed(0.0f);
                break;
            }
        }
    }

    // ── 2. SEGURANDO O MOUSE: Atualiza posição ──
    if (IsMouseButtonDown(MOUSE_BUTTON_LEFT) && draggedBody != nullptr) {
        draggedBody->setPos(mouse);

        // Mantemos a velocidade zerada durante o arrasto para evitar detecções fantasmas,
        // mas a posição está a mudar quadro a quadro acompanhando o cursor do rato.
        draggedBody->setSpeed({ 0.0f, 0.0f });
        draggedBody->setAngularSpeed(0.0f);
    }

    // ── 3. SOLTANDO O MOUSE: Calcula o momento e lança o objeto ──
    if (IsMouseButtonReleased(MOUSE_BUTTON_LEFT) && draggedBody != nullptr) {
        // Restaura o estado estático original do objeto
        draggedBody->setStatic(draggedBodyOldStatic);

        if (!draggedBody->getIsStatic()) {
            float dt = GetFrameTime();

            if (dt > 0.0f) {
                // GetMouseDelta() dá o deslocamento do mouse desde o frame passado
                Vector2 mouseDelta = GetMouseDelta();

                // Velocidade = Espaço / Tempo (pixels por segundo)
                Vector2 launchVelocity = { mouseDelta.x / dt, mouseDelta.y / dt };

                // Conservação parcial do momento do lançamento (85%)
                float momentumMultiplier = 0.85f;
                launchVelocity = Vector2Scale(launchVelocity, momentumMultiplier);

                // Aplica a velocidade linear de lançamento ao bloco
                draggedBody->setSpeed(launchVelocity);

                // Adiciona velocidade angular baseada na velocidade horizontal do lançamento
                float angularFactor = launchVelocity.x * 0.01f;
                draggedBody->setAngularSpeed(angularFactor);
            }
        }

        draggedBody = nullptr;
    }

    if (IsMouseButtonPressed(MOUSE_BUTTON_LEFT))   spawnCircle(mouse);
    if (IsMouseButtonPressed(MOUSE_BUTTON_RIGHT))  spawnRectangle(mouse);
    if (IsMouseButtonPressed(MOUSE_BUTTON_MIDDLE)) spawnTriangle(mouse);

    if (IsKeyPressed(KEY_P)) paused = !paused;
    if (IsKeyPressed(KEY_I)) showInfo = !showInfo;

    if (IsKeyPressed(KEY_C)) {
        for (int i = 1; i < (int)bodies.size(); i++) delete bodies[i]; //i==0 é o chão (rigRectangle), por isso não apago ele.
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
                                   0.0f, 0.5f, 0.3f, true, PURPLE)); //massa zero para não participar da física.
}

// ═══════════════════════════════════════════════
//  Física — gravidade e colisões com bordas
// ═══════════════════════════════════════════════

void Simulation::applyGravity(float dt) {
    for (Body* b : bodies)
        if (!b->getIsStatic())
            b->applyForce({0.0f, GRAVITY * b->getMass()}); //note que estamos lidando com ponteiros para objetos, por isso o ->
}

void Simulation::resolveGroundCollision(Body* body) {
    if (body->getIsStatic()) return;

    // Constantes de estabilização física calibradas
    const float slop = 0.1f;             // Tolerância permitida de penetração
    const float velocityThreshold = 4.0f; // Aumentado ligeiramente para segurar o escorregamento
    const float angularThreshold = 0.2f;  // Limiar para travar rotações parasitas e assentar a face

    // ── CASO CÍRCULO ──
    Circumference* circ = dynamic_cast<Circumference*>(body);
    if (circ) {
        float radius = circ->getRadius();
        float depth = (body->getPos().y + radius) - GROUND_Y;

        if (depth > slop) {
            body->setPos({ body->getPos().x, body->getPos().y - (depth - slop) });
            Vector2 r = { 0.0f, radius };

            float v_contact_x = body->getSpeed().x - body->getAngularSpeed() * r.y;
            float v_contact_y = body->getSpeed().y;

            float invMass = 1.0f / body->getMass();
            float invInertia = 1.0f / body->inertia();

            float jN = 0.0f;
            if (v_contact_y > 0) {
                jN = -(1.0f + body->getRestitution()) * v_contact_y / invMass;
                body->setSpeed({ body->getSpeed().x, body->getSpeed().y + jN * invMass });
            }

            v_contact_x = body->getSpeed().x - body->getAngularSpeed() * r.y;
            float tangentMass = invMass + (r.y * r.y) * invInertia;
            float jT = -v_contact_x / tangentMass;
            float maxFriction = fmaxf(jN, 0.0f) * body->getFriction();
            if (maxFriction <= 0.0f) {
                maxFriction = body->getMass() * GRAVITY * GetFrameTime() * body->getFriction();
            }
            jT = fmaxf(-maxFriction, fminf(jT, maxFriction));

            body->setSpeed({ body->getSpeed().x + jT * invMass, body->getSpeed().y });
            body->setAngularSpeed(body->getAngularSpeed() + (-r.y * jT) * invInertia);

            if (fabsf(body->getSpeed().x) < velocityThreshold) body->setSpeed({ 0.0f, body->getSpeed().y });
            if (fabsf(body->getSpeed().y) < velocityThreshold) body->setSpeed({ body->getSpeed().x, 0.0f });
            if (fabsf(body->getAngularSpeed()) < 0.05f) body->setAngularSpeed(0.0f);
        }
        return;
    }

    // ── CASO POLÍGONOS (RigidRect e Triangle) ──
    std::vector<Vector2> vertices = body->getVertices();
    if (vertices.empty()) return;

    // Encontrar o vértice mais profundo
    Vector2 deepestVertex = vertices[0];
    float maxDepth = vertices[0].y - GROUND_Y;

    for (size_t i = 1; i < vertices.size(); ++i) {
        float depth = vertices[i].y - GROUND_Y;
        if (depth > maxDepth) {
            maxDepth = depth;
            deepestVertex = vertices[i];
        }
    }

    if (maxDepth > slop) {
        // 1. Correção Posicional
        body->setPos({ body->getPos().x, body->getPos().y - (maxDepth - slop) });

        // IMPORTANTE: Calculamos o braço de alavanca com base na posição ATUALIZADA do corpo
        Vector2 r = Vector2Subtract(deepestVertex, body->getPos());

        // 2. Velocidade no ponto de contacto exato
        float v_contact_x = body->getSpeed().x - body->getAngularSpeed() * r.y;
        float v_contact_y = body->getSpeed().y + body->getAngularSpeed() * r.x;

        float invMass = 1.0f / body->getMass();
        float invInertia = 1.0f / body->inertia();

        // ── IMPULSO NORMAL VERTICAL ──
        float jN = 0.0f;
        if (v_contact_y > 0.0f) {
            float normalMass = invMass + (r.x * r.x) * invInertia;
            jN = -(1.0f + body->getRestitution()) * v_contact_y / normalMass;

            body->setSpeed({ body->getSpeed().x, body->getSpeed().y + jN * invMass });
            body->setAngularSpeed(body->getAngularSpeed() + (r.x * jN) * invInertia);
        }

        // Recalcular a velocidade de contacto horizontal após o impulso vertical
        v_contact_x = body->getSpeed().x - body->getAngularSpeed() * r.y;

        // ── IMPULSO TANGENCIAL (ATRITO) ──
        float tangentMass = invMass + (r.y * r.y) * invInertia;
        float jT = -v_contact_x / tangentMass;

        float maxFriction = fmaxf(jN, 0.0f) * body->getFriction();
        if (maxFriction <= 0.0f) {
            // Se já estiver no chão a deslizar, simulamos o peso normal (Massa * Gravidade * dt)
            maxFriction = body->getMass() * GRAVITY * GetFrameTime() * body->getFriction();
        }
        jT = fmaxf(-maxFriction, fminf(jT, maxFriction));

        body->setSpeed({ body->getSpeed().x + jT * invMass, body->getSpeed().y });
        body->setAngularSpeed(body->getAngularSpeed() + (-r.y * jT) * invInertia);

        // ── FILTRO EXTRA PARA EVITAR DESLIZAMENTO INFINITO (ESTABILIZAÇÃO DE FACE) ──
        // Se a velocidade linear e a rotação forem muito baixas, paramos o escorregamento
        // e ajudamos o polígono a assentar a sua face de forma firme no chão.
        if (fabsf(body->getSpeed().x) < velocityThreshold) {
            body->setSpeed({ 0.0f, body->getSpeed().y });
        }
        if (fabsf(body->getSpeed().y) < velocityThreshold) {
            body->setSpeed({ body->getSpeed().x, 0.0f });
        }

        // Se a rotação for minúscula, mata o balanço residual, impedindo-o de ficar equilibrado num vértice
        if (fabsf(body->getAngularSpeed()) < angularThreshold) {
            body->setAngularSpeed(0.0f);
        }
    }
}

// colisão com as 'paredes' da janela
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

// ═══════════════════════════════════════════════════
//  Detecção de colisão - Teorema dos Eixos Paralelos
// ═══════════════════════════════════════════════════

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

    // 1. ADICIONA AS NORMAIS DAS FACES DO POLÍGONO COMO EIXOS
    for (size_t i = 0; i < count; ++i) {
        Vector2 p1 = poly[i];
        Vector2 p2 = poly[(i + 1) % count];
        Vector2 edge = Vector2Subtract(p2, p1);
        Vector2 normal = { -edge.y, edge.x }; // Perpendicular
        axes.push_back(Vector2Normalize(normal));
    }

    // 2. ENCONTRA O VÉRTICE MAIS PRÓXIMO DO CÍRCULO E ADICIONA COMO EIXO DE TESTE
    // (Crucial para quando o círculo bate exatamente na quina de um quadrado)
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

    // 3. TESTA TODOS OS EIXOS USANDO O SAT
    for (Vector2 axis : axes) {
        float minA, maxA;
        float minB, maxB;

        // Projeta o polígono usando a função que você já possui
        projectPoly(poly, axis, minA, maxA);
        // Projeta o círculo
        projectCircle(circleCenter, circleRadius, axis, minB, maxB);

        // Se houver uma lacuna (gap) em qualquer eixo, NÃO há colisão
        if (maxA < minB || maxB < minA) {
            return CollisionInfo();
        }

        // Calcula a profundidade da sobreposição neste eixo
        float overlap = fminf(maxA, maxB) - fmaxf(minA, minB);

        // Queremos armazenar a menor penetração geométrica
        if (overlap < info.depth) {
            info.depth = overlap;
            info.normal = axis;
        }
    }

    // Se passou por todos os eixos, a colisão é real
    info.colliding = true;

    // Garante que a normal aponte consistentemente na perspectiva correta (de A para B)
    // Na assinatura padrão de detectCollision: A costuma ser o Círculo e B o Polígono
    // Vamos garantir que a normal aponte do centro do círculo para o centro do polígono
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

    // Eixos do Polígono A
    for (size_t i = 0; i < countA; ++i) {
        Vector2 p1 = A[i];
        Vector2 p2 = A[(i + 1) % countA];
        Vector2 edge = Vector2Subtract(p2, p1);
        Vector2 normal = { -edge.y, edge.x };
        axes.push_back(Vector2Normalize(normal));
    }

    // Eixos do Polígono B
    for (size_t i = 0; i < countB; ++i) {
        Vector2 p1 = B[i];
        Vector2 p2 = B[(i + 1) % countB];
        Vector2 edge = Vector2Subtract(p2, p1);
        Vector2 normal = { -edge.y, edge.x };
        axes.push_back(Vector2Normalize(normal));
    }

    // Testar todos os eixos usando SAT
    for (Vector2 axis : axes) {
        float minA, maxA;
        float minB, maxB;

        // Chamada atualizada para projectPoly
        projectPoly(A, axis, minA, maxA);
        projectPoly(B, axis, minB, maxB);

        // Se houver uma lacuna (gap), não há colisão
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

    // Garante a direção correta da normal (de A para B)
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
    // 1. CASO: Circunferência vs Circunferência
    Circumference* circA = dynamic_cast<Circumference*>(A);
    Circumference* circB = dynamic_cast<Circumference*>(B);
    if (circA && circB) {
        return detectCircleCircle(circA, circB);
    }

    // 2. CASO: Polígono vs Polígono (Quadrado vs Quadrado, Triângulo vs Triângulo, Quadrado vs Triângulo)
    // Se nenhum deles for uma circunferência, tratamo-los a ambos como polígonos genéricos via SAT
    if (!circA && !circB) {
        return detectPolyPoly(A->getVertices(), B->getVertices());
    }

    // 3. CASO: Circunferência vs Polígono (Quadrado ou Triângulo)
    if (circA && !circB) {
        // A é círculo, B é polígono (RigidRect ou Triangle)
        return detectCirclePoly(circA, B->getVertices());
    }

    if (circB && !circA) {
        // B é círculo, A é polígono (RigidRect ou Triangle)
        CollisionInfo info = detectCirclePoly(circB, A->getVertices());

        // CRUCIAL: Como a função detectCirclePoly calcula a normal de círculo->polígono,
        // e aqui invertemos a ordem dos fatores (A é polígono e B é círculo),
        // precisamos de inverter a normal para manter a convenção global de "A para B".
        info.normal = Vector2Scale(info.normal, -1.0f);
        return info;
    }

    return CollisionInfo(); // Sem colisão por predefinição
}

// ═══════════════════════════════════════════════
//  Resolução por impulso
// ═══════════════════════════════════════════════

void Simulation::applyImpulse(Body* A, Body* B, const CollisionInfo& info) {
    if (A->getIsStatic() && B->getIsStatic()) return;

    float invMassA = A->getIsStatic() ? 0.0f : (1.0f / A->getMass());
    float invMassB = B->getIsStatic() ? 0.0f : (1.0f / B->getMass());

    // ── STEP 1: CORREÇÃO POSICIONAL SEVERA (Impede o encaixe/afundamento em pilhas) ──
    // Aumentamos o percent para 0.65f (65% da penetração resolvida instantaneamente por frame)
    // Reduzimos o slop para 0.01f para deixar os encaixes imperceptíveis e bem justos
    const float percent = 0.65f;
    const float slop = 0.01f;

    float correctionMagnitude = fmaxf(info.depth - slop, 0.0f) / (invMassA + invMassB) * percent;
    Vector2 correctionVector = Vector2Scale(info.normal, correctionMagnitude);

    if (!A->getIsStatic()) A->setPos(Vector2Subtract(A->getPos(), Vector2Scale(correctionVector, invMassA)));
    if (!B->getIsStatic()) B->setPos(Vector2Add(B->getPos(), Vector2Scale(correctionVector, invMassB)));

    // ── STEP 2: CÁLCULO DE IMPACTO LINEAR E ANGULAR ──
    // Encontra o ponto médio da colisão baseado na profundidade real para melhor torque em pilhas
    Vector2 contactPoint = Vector2Lerp(A->getPos(), B->getPos(), 0.5f);

    Vector2 rA = Vector2Subtract(contactPoint, A->getPos());
    Vector2 rB = Vector2Subtract(contactPoint, B->getPos());

    Vector2 vA_contact = { A->getSpeed().x - A->getAngularSpeed() * rA.y,
                           A->getSpeed().y + A->getAngularSpeed() * rA.x };
    Vector2 vB_contact = { B->getSpeed().x - B->getAngularSpeed() * rB.y,
                           B->getSpeed().y + B->getAngularSpeed() * rB.x };

    Vector2 relVel = Vector2Subtract(vB_contact, vA_contact);
    float velAlongNormal = Vector2DotProduct(relVel, info.normal);

    // Se eles já estão se separando na velocidade linear, ignora
    if (velAlongNormal > 0.0f) return;

    float e = fminf(A->getRestitution(), B->getRestitution());

    // Para pilhas estáveis, se a velocidade de impacto for muito baixa, zeramos a restituição (e = 0)
    // Isso evita que caixas na base da pilha fiquem quicando microscopicamente para sempre
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

    // ── STEP 3: ATRITO LATERAL (ESSENCIAL PARA TRAVAR A PILHA LATERALMENTE) ──
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
