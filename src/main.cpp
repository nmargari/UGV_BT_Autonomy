#include "raylib.h"
#include "behaviortree_cpp/bt_factory.h"

int main()
{
    InitWindow(1280, 720, "UGV Behavior Tree Simulator");
    SetTargetFPS(60);

    while(!WindowShouldClose())
    {
        BeginDrawing();

        ClearBackground(RAYWHITE);

        DrawText("Hello from Raylib!", 400, 340, 20, DARKGRAY);

        EndDrawing();
    }
    
    CloseWindow();

    return 0;
}