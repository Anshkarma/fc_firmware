#include "raylib.h"
#include "rlgl.h"   // OpenGL primitive access for matrix rotation
#include "visualizer.h"
#include "plant.h"
#include <string.h>
#include <stdbool.h>
#include <math.h>

static Camera3D camera;
static char scenario_title[64] = "Initializing...";
static bool window_open = false;

/* ==================================================================
 * MANUAL MATH FUNCTIONS TO AVOID raymath.h REDEFINITION CONFLICTS
 * ================================================================== */
static void Manual_QuaternionNormalize(Quaternion *q) {
    float length = sqrtf(q->x*q->x + q->y*q->y + q->z*q->z + q->w*q->w);
    if (length == 0.0f) {
        q->x = 0.0f; q->y = 0.0f; q->z = 0.0f; q->w = 1.0f;
    } else {
        float ilength = 1.0f / length;
        q->x *= ilength;
        q->y *= ilength;
        q->z *= ilength;
        q->w *= ilength;
    }
}

static void Manual_QuaternionToAxisAngle(Quaternion q, Vector3 *outAxis, float *outAngle) {
    // Prevent acosf domain errors by clamping w
    if (q.w > 1.0f) q.w = 1.0f;
    if (q.w < -1.0f) q.w = -1.0f;
    
    *outAngle = 2.0f * acosf(q.w);
    float s = sqrtf(1.0f - q.w * q.w);
    
    if (s < 0.001f) {
        outAxis->x = 1.0f;
        outAxis->y = 0.0f;
        outAxis->z = 0.0f;
    } else {
        outAxis->x = q.x / s;
        outAxis->y = q.y / s;
        outAxis->z = q.z / s;
    }
}

/* ==================================================================
 * VISUALIZER IMPLEMENTATION
 * ================================================================== */

void visualizer_set_title(const char* title) {
    if (!title) return;
    strncpy(scenario_title, title, sizeof(scenario_title) - 1);
    scenario_title[sizeof(scenario_title) - 1] = '\0';

    /* If the window already exists, update it live */
    if (window_open) {
        SetWindowTitle(scenario_title);
    }
}

void visualizer_init(void) {
    InitWindow(1280, 720, scenario_title);
    window_open = true;
    SetTargetFPS(60);
    camera = (Camera3D){ { 20.0f, 20.0f, 20.0f }, { 0.0f, 0.0f, 0.0f }, { 0.0f, 1.0f, 0.0f }, 45.0f, CAMERA_PERSPECTIVE };
}

void visualizer_render(const quad_state_t *state) {
    if (!window_open || WindowShouldClose()) return;

    /* Firmware quat_t is {w,x,y,z}; raylib's Quaternion is {x,y,z,w}. */
    Quaternion q = {
        state->orientation.x, state->orientation.y,
        state->orientation.z, state->orientation.w
    };
    Manual_QuaternionNormalize(&q);

    Vector3 axis;
    float angle;
    Manual_QuaternionToAxisAngle(q, &axis, &angle);

    /* FIX: Our Plant uses Z-UP (Altitude is positive Z).
     * Raylib is right-handed, Y-up.
     * To map Plant (X, Y, Z) -> Raylib (Xr, Yr, Zr) with Determinant +1:
     * We use: (X, Z, -Y)
     * This keeps Up as Up, and prevents mirrored rotations! */
    Vector3 pos_r  = { state->position.x,  state->position.z, -state->position.y };
    Vector3 axis_r = { axis.x,             axis.z,            -axis.y            };

    BeginDrawing();
        ClearBackground(RAYWHITE);
        BeginMode3D(camera);
            DrawGrid(10, 1.0f);

            rlPushMatrix();
                rlTranslatef(pos_r.x, pos_r.y, pos_r.z);
                rlRotatef(angle * RAD2DEG, axis_r.x, axis_r.y, axis_r.z);

                // Drone body
                DrawCube((Vector3){0.0f, 0.0f, 0.0f}, 1.0f, 0.2f, 1.0f, BLUE);
                // Forward-heading indicator (Red arm pointing X-Forward)
                DrawLine3D((Vector3){0, 0, 0}, (Vector3){1.5f, 0, 0}, RED);
            rlPopMatrix();

        EndMode3D();
        
        // Dynamic HUD 
        DrawRectangle(10, 10, 320, 30, Fade(SKYBLUE, 0.5f));
        DrawText(scenario_title, 20, 15, 20, DARKBLUE);
        DrawFPS(10, 50);
    EndDrawing();
}

void visualizer_close(void) {
    if (window_open) {
        CloseWindow();
        window_open = false;
    }
}
