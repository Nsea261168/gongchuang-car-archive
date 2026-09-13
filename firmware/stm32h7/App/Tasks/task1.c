#include "task1.h"

#include "app_scheduler.h"
#include "task_contract.h"
#include "ws2812.h"
#include "lcd.h"

#include <string.h>

#define CUBE_X                8U
#define CUBE_Y               39U
#define CUBE_W              150U
#define CUBE_H              159U
#define CUBE_CENTER_X        75.0f
#define CUBE_CENTER_Y        79.0f
#define CUBE_SCALE           38.0f
#define CUBE_FRAME_MS        50U
#define CUBE_INFO_MS        250U

enum
{
  CUBE_PIXEL_BLACK = 0,
  CUBE_FACE_LIGHT_BLUE,
  CUBE_AXIS_X_RED,
  CUBE_AXIS_Y_GREEN,
  CUBE_AXIS_Z_BLUE,
  CUBE_ORIGIN_WHITE
};

typedef struct
{
  uint8_t vertex[4];
  uint8_t color;
} CubeFace_t;

static uint8_t sCubeOld[CUBE_W * CUBE_H];
static uint8_t sCubeNew[CUBE_W * CUBE_H];

static const int8_t sCubeVertex[8][3] =
{
  {-1, -1, -1}, { 1, -1, -1}, {-1,  1, -1}, { 1,  1, -1},
  {-1, -1,  1}, { 1, -1,  1}, {-1,  1,  1}, { 1,  1,  1}
};

/* All six faces use the same pale blue; the body axes carry XYZ colors. */
static const CubeFace_t sCubeFace[6] =
{
  {{0U, 4U, 6U, 2U}, CUBE_FACE_LIGHT_BLUE},
  {{1U, 3U, 7U, 5U}, CUBE_FACE_LIGHT_BLUE},
  {{0U, 1U, 5U, 4U}, CUBE_FACE_LIGHT_BLUE},
  {{2U, 6U, 7U, 3U}, CUBE_FACE_LIGHT_BLUE},
  {{0U, 2U, 3U, 1U}, CUBE_FACE_LIGHT_BLUE},
  {{4U, 5U, 7U, 6U}, CUBE_FACE_LIGHT_BLUE}
};

static const uint16_t sCubePalette[6] =
{
  BLACK, 0x65FFU, RED, GREEN, BLUE, WHITE
};

static int16_t Task1_Abs16(int16_t value)
{
  return (value < 0) ? (int16_t)-value : value;
}

static int16_t Task1_Round(float value)
{
  return (int16_t)((value >= 0.0f) ? (value + 0.5f) : (value - 0.5f));
}

static void Task1_CubePlot(int16_t x, int16_t y, uint8_t color)
{
  uint8_t dx;
  uint8_t dy;

  /* A two-pixel-wide edge remains visible on the small high-density LCD. */
  for (dy = 0U; dy < 2U; dy++)
  {
    for (dx = 0U; dx < 2U; dx++)
    {
      int16_t px = (int16_t)(x + dx);
      int16_t py = (int16_t)(y + dy);
      if ((px >= 0) && (px < (int16_t)CUBE_W) &&
          (py >= 0) && (py < (int16_t)CUBE_H))
        sCubeNew[(uint32_t)py * CUBE_W + (uint32_t)px] = color;
    }
  }
}

static int32_t Task1_EdgeValue(int16_t ax, int16_t ay,
                               int16_t bx, int16_t by,
                               int16_t px, int16_t py)
{
  return (int32_t)(px - ax) * (int32_t)(by - ay) -
         (int32_t)(py - ay) * (int32_t)(bx - ax);
}

static void Task1_CubeTriangle(const int16_t a[2], const int16_t b[2],
                               const int16_t c[2], uint8_t color)
{
  int32_t area = Task1_EdgeValue(a[0], a[1], b[0], b[1], c[0], c[1]);
  int16_t min_x = a[0];
  int16_t max_x = a[0];
  int16_t min_y = a[1];
  int16_t max_y = a[1];
  int16_t x;
  int16_t y;

  if (area == 0) return;

  if (b[0] < min_x) min_x = b[0];
  if (c[0] < min_x) min_x = c[0];
  if (b[0] > max_x) max_x = b[0];
  if (c[0] > max_x) max_x = c[0];
  if (b[1] < min_y) min_y = b[1];
  if (c[1] < min_y) min_y = c[1];
  if (b[1] > max_y) max_y = b[1];
  if (c[1] > max_y) max_y = c[1];
  if (min_x < 0) min_x = 0;
  if (min_y < 0) min_y = 0;
  if (max_x >= (int16_t)CUBE_W) max_x = (int16_t)CUBE_W - 1;
  if (max_y >= (int16_t)CUBE_H) max_y = (int16_t)CUBE_H - 1;

  for (y = min_y; y <= max_y; y++)
  {
    for (x = min_x; x <= max_x; x++)
    {
      int32_t e0 = Task1_EdgeValue(a[0], a[1], b[0], b[1], x, y);
      int32_t e1 = Task1_EdgeValue(b[0], b[1], c[0], c[1], x, y);
      int32_t e2 = Task1_EdgeValue(c[0], c[1], a[0], a[1], x, y);
      if (((e0 >= 0) && (e1 >= 0) && (e2 >= 0)) ||
          ((e0 <= 0) && (e1 <= 0) && (e2 <= 0)))
        sCubeNew[(uint32_t)y * CUBE_W + (uint32_t)x] = color;
    }
  }
}

static void Task1_CubeLine(int16_t x0, int16_t y0, int16_t x1, int16_t y1,
                           uint8_t color)
{
  int16_t dx = Task1_Abs16((int16_t)(x1 - x0));
  int16_t sx = (x0 < x1) ? 1 : -1;
  int16_t dy = (int16_t)-Task1_Abs16((int16_t)(y1 - y0));
  int16_t sy = (y0 < y1) ? 1 : -1;
  int16_t error = (int16_t)(dx + dy);

  while (1)
  {
    int16_t twice_error;
    Task1_CubePlot(x0, y0, color);
    if ((x0 == x1) && (y0 == y1)) break;
    twice_error = (int16_t)(2 * error);
    if (twice_error >= dy)
    {
      error = (int16_t)(error + dy);
      x0 = (int16_t)(x0 + sx);
    }
    if (twice_error <= dx)
    {
      error = (int16_t)(error + dx);
      y0 = (int16_t)(y0 + sy);
    }
  }
}

static void Task1_ProjectCube(const float q[4], int16_t point[8][2],
                              float depth[8], int16_t axis_point[4][2])
{
  float q0 = q[0];
  float q1 = q[1];
  float q2 = q[2];
  float q3 = q[3];
  float r00 = 1.0f - 2.0f * (q2 * q2 + q3 * q3);
  float r01 = 2.0f * (q1 * q2 - q0 * q3);
  float r02 = 2.0f * (q1 * q3 + q0 * q2);
  float r10 = 2.0f * (q1 * q2 + q0 * q3);
  float r11 = 1.0f - 2.0f * (q1 * q1 + q3 * q3);
  float r12 = 2.0f * (q2 * q3 - q0 * q1);
  float r20 = 2.0f * (q1 * q3 - q0 * q2);
  float r21 = 2.0f * (q2 * q3 + q0 * q1);
  float r22 = 1.0f - 2.0f * (q1 * q1 + q2 * q2);
  uint8_t index;

  for (index = 0U; index < 8U; index++)
  {
    float x = (float)sCubeVertex[index][0];
    float y = (float)sCubeVertex[index][1];
    float z = (float)sCubeVertex[index][2];
    float world_x = r00 * x + r01 * y + r02 * z;
    float world_y = r10 * x + r11 * y + r12 * z;
    float world_z = r20 * x + r21 * y + r22 * z;

    depth[index] = world_x + world_y + world_z;

    /* Fixed isometric camera: no trigonometry is needed per frame. */
    point[index][0] = Task1_Round(CUBE_CENTER_X + CUBE_SCALE *
                                  0.7071068f * (world_x - world_y));
    point[index][1] = Task1_Round(CUBE_CENTER_Y + CUBE_SCALE *
                                  (0.4082483f * (world_x + world_y) -
                                   0.8164966f * world_z));
  }

  axis_point[0][0] = Task1_Round(CUBE_CENTER_X);
  axis_point[0][1] = Task1_Round(CUBE_CENTER_Y);
  for (index = 0U; index < 3U; index++)
  {
    float local_x = (index == 0U) ? 1.8f : 0.0f;
    float local_y = (index == 1U) ? 1.8f : 0.0f;
    float local_z = (index == 2U) ? 1.8f : 0.0f;
    float world_x = r00 * local_x + r01 * local_y + r02 * local_z;
    float world_y = r10 * local_x + r11 * local_y + r12 * local_z;
    float world_z = r20 * local_x + r21 * local_y + r22 * local_z;
    axis_point[index + 1U][0] = Task1_Round(CUBE_CENTER_X + CUBE_SCALE *
        0.7071068f * (world_x - world_y));
    axis_point[index + 1U][1] = Task1_Round(CUBE_CENTER_Y + CUBE_SCALE *
        (0.4082483f * (world_x + world_y) - 0.8164966f * world_z));
  }
}

static void Task1_RenderCube(const float quaternion[4])
{
  int16_t point[8][2];
  int16_t axis_point[4][2];
  float depth[8];
  float face_depth[6];
  uint8_t visible[3];
  uint8_t face;
  uint8_t order;
  uint16_t row;

  memset(sCubeNew, 0, sizeof(sCubeNew));
  Task1_ProjectCube(quaternion, point, depth, axis_point);

  for (face = 0U; face < 6U; face++)
  {
    const CubeFace_t *cube_face = &sCubeFace[face];
    face_depth[face] = depth[cube_face->vertex[0]] +
                       depth[cube_face->vertex[1]] +
                       depth[cube_face->vertex[2]] +
                       depth[cube_face->vertex[3]];
  }

  /* Of each opposite pair, the face with the greater camera depth is visible. */
  visible[0] = (face_depth[0] > face_depth[1]) ? 0U : 1U;
  visible[1] = (face_depth[2] > face_depth[3]) ? 2U : 3U;
  visible[2] = (face_depth[4] > face_depth[5]) ? 4U : 5U;

  /* Painter order keeps a nearly edge-on face behind the nearest face. */
  for (order = 0U; order < 2U; order++)
  {
    uint8_t next;
    for (next = (uint8_t)(order + 1U); next < 3U; next++)
    {
      if (face_depth[visible[order]] > face_depth[visible[next]])
      {
        uint8_t swap = visible[order];
        visible[order] = visible[next];
        visible[next] = swap;
      }
    }
  }

  for (order = 0U; order < 3U; order++)
  {
    const CubeFace_t *cube_face = &sCubeFace[visible[order]];
    const int16_t *a = point[cube_face->vertex[0]];
    const int16_t *b = point[cube_face->vertex[1]];
    const int16_t *c = point[cube_face->vertex[2]];
    const int16_t *d = point[cube_face->vertex[3]];
    Task1_CubeTriangle(a, b, c, cube_face->color);
    Task1_CubeTriangle(a, c, d, cube_face->color);
  }

  /* Dark borders preserve the cube silhouette beneath the body axes. */
  for (order = 0U; order < 3U; order++)
  {
    const CubeFace_t *cube_face = &sCubeFace[visible[order]];
    uint8_t side;
    for (side = 0U; side < 4U; side++)
    {
      uint8_t first = cube_face->vertex[side];
      uint8_t second = cube_face->vertex[(uint8_t)((side + 1U) & 3U)];
      Task1_CubeLine(point[first][0], point[first][1],
                     point[second][0], point[second][1], CUBE_PIXEL_BLACK);
    }
  }

  /* Three positive body axes emerge from the local origin and rotate with
   * the same quaternion as the cube.  Drawing them after the faces and their
   * borders makes the zero-to-tip direction unambiguous. */
  Task1_CubeLine(axis_point[0][0], axis_point[0][1],
                 axis_point[1][0], axis_point[1][1], CUBE_AXIS_X_RED);
  Task1_CubeLine(axis_point[0][0], axis_point[0][1],
                 axis_point[2][0], axis_point[2][1], CUBE_AXIS_Y_GREEN);
  Task1_CubeLine(axis_point[0][0], axis_point[0][1],
                 axis_point[3][0], axis_point[3][1], CUBE_AXIS_Z_BLUE);
  Task1_CubePlot(axis_point[1][0], axis_point[1][1], CUBE_AXIS_X_RED);
  Task1_CubePlot(axis_point[2][0], axis_point[2][1], CUBE_AXIS_Y_GREEN);
  Task1_CubePlot(axis_point[3][0], axis_point[3][1], CUBE_AXIS_Z_BLUE);
  Task1_CubePlot(axis_point[0][0], axis_point[0][1], CUBE_ORIGIN_WHITE);

  /* Send one continuous changed span per scanline.  This removes obsolete
   * pixels while avoiding thousands of LCD address commands per solid face. */
  for (row = 0U; row < CUBE_H; row++)
  {
    int16_t first = -1;
    int16_t last = -1;
    uint16_t column;
    uint32_t row_offset = (uint32_t)row * CUBE_W;
    for (column = 0U; column < CUBE_W; column++)
    {
      if (sCubeOld[row_offset + column] != sCubeNew[row_offset + column])
      {
        if (first < 0) first = (int16_t)column;
        last = (int16_t)column;
      }
    }
    if (first >= 0)
    {
      LCD_Address_Set((uint16_t)(CUBE_X + first), (uint16_t)(CUBE_Y + row),
                      (uint16_t)(CUBE_X + last), (uint16_t)(CUBE_Y + row));
      for (column = (uint16_t)first; column <= (uint16_t)last; column++)
      {
        LCD_WR_DATA(sCubePalette[sCubeNew[row_offset + column]]);
      }
    }
  }
  memcpy(sCubeOld, sCubeNew, sizeof(sCubeOld));
}

static void Task1_FormatSigned1(float value, uint8_t text[7])
{
  uint32_t scaled;

  if (value > 999.9f) value = 999.9f;
  if (value < -999.9f) value = -999.9f;
  text[0] = (value < 0.0f) ? '-' : '+';
  if (value < 0.0f) value = -value;
  scaled = (uint32_t)(value * 10.0f + 0.5f);
  text[1] = (uint8_t)('0' + (scaled / 1000U) % 10U);
  text[2] = (uint8_t)('0' + (scaled / 100U) % 10U);
  text[3] = (uint8_t)('0' + (scaled / 10U) % 10U);
  text[4] = '.';
  text[5] = (uint8_t)('0' + scaled % 10U);
  text[6] = 0U;
}

static const char *Task1_StateText(uint8_t state)
{
  if (state == BMI088_MOTION_STABLE) return "STABLE";
  if (state == BMI088_MOTION_MOVING) return "MOVING";
  if (state == BMI088_MOTION_IMPACT) return "IMPACT";
  return "CAL";
}

static uint16_t Task1_StateColor(uint8_t state)
{
  if (state == BMI088_MOTION_STABLE) return GREEN;
  if (state == BMI088_MOTION_MOVING) return CYAN;
  if (state == BMI088_MOTION_IMPACT) return RED;
  return YELLOW;
}

static void Task1_DrawFrame(void)
{
  memset(sCubeOld, 0, sizeof(sCubeOld));
  memset(sCubeNew, 0, sizeof(sCubeNew));
  LCD_WR_REG(0x28U);
  LCD_Fill(0U, 0U, LCD_W, LCD_H, BLACK);
  LCD_ShowString(10U, 7U, (const uint8_t *)"BMI088 / 3D CUBE",
                 CYAN, BLACK, 24U, 0U);
  LCD_DrawLine(8U, 35U, 271U, 35U, CYAN);
  LCD_DrawRectangle(6U, 37U, 159U, 199U, DARKBLUE);
  LCD_DrawLine(164U, 40U, 164U, 229U, DARKBLUE);

  LCD_ShowString(174U, 46U, (const uint8_t *)"ROLL", GRAY, BLACK, 16U, 0U);
  LCD_ShowString(174U, 84U, (const uint8_t *)"PITCH", GRAY, BLACK, 16U, 0U);
  LCD_ShowString(174U, 122U, (const uint8_t *)"YAW", GRAY, BLACK, 16U, 0U);
  LCD_ShowString(174U, 160U, (const uint8_t *)"RATE", GRAY, BLACK, 16U, 0U);

  LCD_ShowString(10U, 210U, (const uint8_t *)"X", RED, BLACK, 16U, 0U);
  LCD_ShowString(27U, 210U, (const uint8_t *)"Y", GREEN, BLACK, 16U, 0U);
  LCD_ShowString(44U, 210U, (const uint8_t *)"Z", BLUE, BLACK, 16U, 0U);
  LCD_ShowString(61U, 210U, (const uint8_t *)"BODY AXES", GRAY, BLACK, 16U, 0U);
  LCD_ShowString(174U, 216U, (const uint8_t *)"LEFT BACK",
                 GRAY, BLACK, 16U, 0U);
  LCD_WR_REG(0x29U);
}

static void Task1_DrawInfo(const BMI088_Attitude_t *imu, uint16_t rate_hz)
{
  uint8_t text[7];
  uint16_t state_color = Task1_StateColor(imu->motion_state);

  Task1_FormatSigned1(imu->roll_deg, text);
  LCD_ShowString(174U, 64U, text, WHITE, BLACK, 16U, 0U);
  Task1_FormatSigned1(imu->pitch_deg, text);
  LCD_ShowString(174U, 102U, text, WHITE, BLACK, 16U, 0U);
  Task1_FormatSigned1(imu->yaw_deg, text);
  LCD_ShowString(174U, 140U, text, WHITE, BLACK, 16U, 0U);

  LCD_Fill(174U, 178U, 270U, 195U, BLACK);
  LCD_ShowIntNum(174U, 178U, rate_hz, 4U, WHITE, BLACK, 16U);
  LCD_ShowString(210U, 178U, (const uint8_t *)"Hz", GRAY, BLACK, 16U, 0U);
  LCD_Fill(174U, 198U, 270U, 215U, BLACK);
  LCD_ShowString(174U, 198U, (const uint8_t *)Task1_StateText(imu->motion_state),
                 state_color, BLACK, 16U, 0U);
}

static uint32_t sFrameTick;
static uint32_t sInfoTick;
static uint32_t sRateTick;
static uint32_t sLastSamples;
static uint16_t sRateHz;

static void Task1_Enter(uint32_t now_ms)
{
  sFrameTick = now_ms;
  sInfoTick = now_ms;
  sRateTick = now_ms;
  sLastSamples = AppScheduler_GetAttitude()->sample_count;
  sRateHz = 0U;
  Task1_DrawFrame();
}

static void Task1_Tick(uint32_t now_ms, InputEvent_t event)
{
  BMI088_Attitude_t snapshot;
  (void)event;
  snapshot = *AppScheduler_GetAttitude();
  if ((uint32_t)(now_ms - sRateTick) >= 1000U)
  {
    uint32_t elapsed = (uint32_t)(now_ms - sRateTick);
    uint32_t samples = snapshot.sample_count - sLastSamples;
    sRateHz = (uint16_t)((samples * 1000U) / elapsed);
    sLastSamples = snapshot.sample_count;
    sRateTick = now_ms;
  }
  if ((uint32_t)(now_ms - sFrameTick) >= CUBE_FRAME_MS)
  {
    sFrameTick = now_ms;
    Task1_RenderCube(snapshot.quaternion);
  }
  if ((uint32_t)(now_ms - sInfoTick) >= CUBE_INFO_MS)
  {
    sInfoTick = now_ms;
    Task1_DrawInfo(&snapshot, sRateHz);
  }
  if (snapshot.status != BMI088_ATTITUDE_OK)
    WS2812_Ctrl(WS2812_BRIGHTNESS_MAX, 0U, 0U);
  else if (snapshot.motion_state == BMI088_MOTION_STABLE)
    WS2812_Ctrl(0U, WS2812_BRIGHTNESS_MAX, 0U);
  else if (snapshot.motion_state == BMI088_MOTION_IMPACT)
    WS2812_Ctrl(WS2812_BRIGHTNESS_MAX, 0U, 0U);
  else
    WS2812_Ctrl(0U, WS2812_BRIGHTNESS_MAX, WS2812_BRIGHTNESS_MAX);
}

static void Task1_Exit(void) {}

const AppTask_t Task1_Definition = {Task1_Enter, Task1_Tick, Task1_Exit};
