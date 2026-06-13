#ifndef BODY_H_INCLUDED
#define BODY_H_INCLUDED

#include "proj_headers.h"

class Body {
protected:
    Vector2 pos;
    Vector2 speed;
    Vector2 accel;
    float mass;
    float restitution;
    float friction;
    bool  isStatic;

public:
    Body(Vector2 pos, float mass, float restitution, float friction, bool isStatic);
    virtual ~Body() = default;

    virtual void update(float dt);
    void applyForce(Vector2 force);

    virtual void  draw()    const = 0;
    virtual float area()    const = 0;
    virtual float inertia() const = 0;

    Vector2 getPos()         const;
    Vector2 getSpeed()       const;
    Vector2 getAccel()       const;
    float   getMass()        const;
    float   getRestitution() const;
    float   getFriction()    const;
    bool    getIsStatic()    const;

    void setPos(Vector2 p);
    void setSpeed(Vector2 v);
    void setAccel(Vector2 a);
    void setStatic(bool s);
};


class Circumference : public Body {
private:
    float radius;
public:
    Circumference(Vector2 pos, float radius, float mass,
                  float restitution = 0.5f, float friction = 0.3f, bool isStatic = false);

    void  draw()    const override;
    float area()    const override;
    float inertia() const override;
    float getRadius() const;
};


class RigidRect : public Body {      // renomeado para evitar conflito com raylib
private:
    float width, height;
    Color cor;
public:
    RigidRect(Vector2 pos, float width, float height, float mass,
              float restitution = 0.5f, float friction = 0.3f, bool isStatic = false, Color cor=BLUE);

    void  draw()    const override;
    float area()    const override;
    float inertia() const override;
    float getWidth()  const;
    float getHeight() const;
};


class Triangle : public Body {
private:
    float side;
public:
    Triangle(Vector2 pos, float side, float mass,
             float restitution = 0.5f, float friction = 0.3f, bool isStatic = false);

    void  draw()    const override;
    float area()    const override;
    float inertia() const override;
    float getSide() const;
};

#endif // BODY_H_INCLUDED
