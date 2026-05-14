#include "proj_headers.h"
using namespace std;

int main(){
    const int screenWidth = 800;
    const int screenHeight = 450;

    InitWindow(screenWidth, screenHeight, "raylib [core] example - basic window");
    SetTargetFPS(60);
    while (!WindowShouldClose())
    {
        BeginDrawing();
            Vector2 vetor;
            vetor.x = 1;
            vetor.y = 1;
            ClearBackground(RAYWHITE);

        EndDrawing();
    }
    CloseWindow();
}
