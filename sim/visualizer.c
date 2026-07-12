#include "raylib.h"
#include "visualizer.h"

static Camera3D camera;

void visualizer_init(void) {
    InitWindow(1280, 720, "Drone 3D Flight Sim");
    SetTargetFPS(60);
    camera = (Camera3D){ { 20.0f, 20.0f, 20.0f }, { 0.0f, 0.0f, 0.0f }, { 0.0f, 1.0f, 0.0f }, 45.0f, 0 };
}

void visualizer_render(const quad_state_t *state) {
    if (WindowShouldClose()) return;

    BeginDrawing();
        ClearBackground(RAYWHITE);
        BeginMode3D(camera);
            DrawGrid(10, 1.0f);
            // Render drone based on passed state
            DrawCube((Vector3){state->position.x, state->position.z, state->position.y}, 1.0f, 0.2f, 1.0f, BLUE);
        EndMode3D();
    EndDrawing();
}

void visualizer_close(void) {
    CloseWindow();
}


