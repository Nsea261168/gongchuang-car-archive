#include "task4.h"

#include "app_scheduler.h"
#include "chassis_motion.h"
#include "lcd.h"
#include "ld06.h"
#include "songjia_motor.h"
#include "ws2812.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

#define T4_FIELD_MM                    3000.0f
#define T4_PI                            3.14159265f
/* Empirical LD06/vehicle alignment from Task0: 90=vehicle front,
   180=right, 270=rear, 0=left.  The rear 90 degrees are blind. */
#define T4_VEHICLE_FRONT_BIN            90U
#define T4_VEHICLE_REAR_BIN            270U
#define T4_REAR_BLIND_HALF_DEG          45
#define T4_LIDAR_OFFSET_X_MM             0.0f
#define T4_LIDAR_OFFSET_Y_MM             0.0f
#define T4_LIDAR_YAW_DEG                 0.0f

#define T4_MIN_DISTANCE_MM              80U
#define T4_MAX_DISTANCE_MM            4600U
#define T4_MIN_CONFIDENCE               10U
#define T4_MAX_POINTS                  300U
#define T4_MAX_CLUSTERS                 20U
#define T4_MAX_SEGMENTS                 20U
#define T4_SPLIT_STACK_SIZE             32U
#define T4_CLUSTER_MAX_BIN_GAP           3U
#define T4_CLUSTER_LINK_BASE_MM         90.0f
#define T4_CLUSTER_LINK_SCALE            0.055f
#define T4_LINE_MIN_POINTS               5U
#define T4_LINE_MIN_LENGTH_MM          150.0f
#define T4_LINE_MAX_RMS_MM              70.0f
#define T4_SPLIT_RESIDUAL_MM            75.0f
#define T4_MERGE_ANGLE_DEG               8.0f
#define T4_MERGE_GAP_MM                180.0f
#define T4_CORNER_MIN_DEG               70.0f
#define T4_CORNER_MAX_DEG              110.0f
#define T4_CORNER_MAX_GAP_MM            350.0f
#define T4_CORNER_MAX_RANGE_MM         4600.0f
#define T4_LINE_YAW_AGREEMENT_DEG        10.0f
#define T4_WALL_INLIER_MM              150.0f
#define T4_MIN_INLIERS                  10U
#define T4_START_QUADRANT_MM          1500.0f
#define T4_POSITION_TOLERANCE_MM       120.0f
#define T4_PROVISIONAL_POS_GATE_MM     150.0f
#define T4_PROVISIONAL_YAW_GATE_DEG      8.0f
#define T4_TRACK_POS_GATE_MM           400.0f
#define T4_TRACK_YAW_GATE_DEG           20.0f
#define T4_POSE_FILTER_ALPHA             0.4f

#define T4_MATCH_PERIOD_MS              60U
#define T4_DISPLAY_PERIOD_MS           200U
#define T4_LIDAR_FRESH_MS              350U
#define T4_DEGRADED_MS                 500U
#define T4_LOST_MS                    1500U
#define T4_OBSTACLE_MIN_LENGTH_MM       80.0f
#define T4_OBSTACLE_MAX_LENGTH_MM      900.0f
#define T4_OBSTACLE_WALL_GAP_MM        220.0f
#define T4_OBSTACLE_TRACK_GATE_MM      350.0f
#define T4_OBSTACLE_CONFIRM_FRAMES       2U

#define T4_MAP_X0                       10U
#define T4_MAP_Y0                       48U
#define T4_MAP_SIZE                    176U
#define T4_LOCAL_RANGE_MM             3500.0f

typedef enum
{
  T4_POINT_RAW = 0,
  T4_POINT_LINE,
  T4_POINT_WALL,
  T4_POINT_UNMATCHED,
  T4_POINT_OBSTACLE
} T4_PointClass_t;

typedef struct
{
  float bx_mm;
  float by_mm;
  uint16_t range_mm;
  uint16_t bin;
  uint8_t segment;
  uint8_t classification;
} T4_Point_t;

typedef struct
{
  uint16_t first;
  uint16_t end;
} T4_Range_t;

typedef struct
{
  uint16_t first;
  uint16_t count;
  float direction_deg;
  float normal_x;
  float normal_y;
  float normal_distance_mm;
  float length_mm;
  float rms_mm;
} T4_LineSegment_t;

typedef struct
{
  float x_mm;
  float y_mm;
  float yaw_deg;
  float rms_mm;
  float cost;
  uint16_t inliers;
  uint8_t wall_mask;
  uint8_t wall_count;
  uint8_t valid;
} T4_Match_t;

typedef struct
{
  float x_mm;
  float y_mm;
  float distance_mm;
  uint16_t points;
  uint8_t consecutive_frames;
  uint8_t valid;
} T4_ObstacleTrack_t;

static T4_Point_t sPoints[T4_MAX_POINTS];
static T4_Range_t sClusters[T4_MAX_CLUSTERS];
static T4_LineSegment_t sSegments[T4_MAX_SEGMENTS];
static uint16_t sPointCount;
static uint8_t sClusterCount;
static uint8_t sSegmentCount;
static uint8_t sCornerCount;
static uint8_t sBestCornerFirst;
static uint8_t sBestCornerSecond;
static float sBestCornerAngle;
static float sBestLineRms;
static float sBestCornerX;
static float sBestCornerY;

static Pose_Packet_t sPose;
static T4_Match_t sProvisional;
static T4_ObstacleTrack_t sObstacle;
static uint8_t sSelectedStart;
static uint8_t sProvisionalFrames;
static uint32_t sLastScanSequence;
static uint32_t sLastMatchTick;
static uint32_t sLastFixTick;
static uint32_t sDisplayTick;
static float sImuYawAtFix;

static float T4_Wrap180(float angle)
{
  while (angle > 180.0f) angle -= 360.0f;
  while (angle <= -180.0f) angle += 360.0f;
  return angle;
}

static float T4_Wrap90(float angle)
{
  while (angle > 90.0f) angle -= 180.0f;
  while (angle <= -90.0f) angle += 180.0f;
  return angle;
}

static float T4_Clamp(float value, float minimum, float maximum)
{
  if (value < minimum) return minimum;
  if (value > maximum) return maximum;
  return value;
}

static float T4_LineAngleDifference(float a, float b)
{
  float difference = fabsf(T4_Wrap180(a - b));
  if (difference > 90.0f) difference = 180.0f - difference;
  return difference;
}

static float T4_PointDistance(float x1, float y1, float x2, float y2)
{
  float dx = x1 - x2;
  float dy = y1 - y2;
  return sqrtf(dx * dx + dy * dy);
}

static int16_t T4_SignedAngleOffset(uint16_t bin, uint16_t center_bin)
{
  int16_t offset = (int16_t)bin - (int16_t)center_bin;
  if (offset > 180) offset -= 360;
  if (offset < -180) offset += 360;
  return offset;
}

static int16_t T4_SignedBinOffset(uint16_t bin)
{
  return T4_SignedAngleOffset(bin, T4_VEHICLE_FRONT_BIN);
}

static void T4_ToWorld(float vehicle_x, float vehicle_y, float yaw_deg,
                       float bx, float by, float *wx, float *wy)
{
  float angle = yaw_deg * T4_PI / 180.0f;
  float sine = sinf(angle);
  float cosine = cosf(angle);
  *wx = vehicle_x + bx * cosine + by * sine;
  *wy = vehicle_y - bx * sine + by * cosine;
}

static float T4_NearestWall(float x, float y, uint8_t *wall)
{
  float distance[4] = {fabsf(x), fabsf(T4_FIELD_MM - x),
                       fabsf(y), fabsf(T4_FIELD_MM - y)};
  uint8_t best = 0U;
  uint8_t index;
  for (index = 1U; index < 4U; index++)
    if (distance[index] < distance[best]) best = index;
  *wall = best;
  return distance[best];
}

static void T4_AddCluster(uint16_t first, uint16_t end)
{
  if ((end > first) && (sClusterCount < T4_MAX_CLUSTERS))
  {
    sClusters[sClusterCount].first = first;
    sClusters[sClusterCount].end = end;
    sClusterCount++;
  }
}

static void T4_CollectPoints(const LD06_Data_t *scan)
{
  float lidar_yaw = T4_LIDAR_YAW_DEG * T4_PI / 180.0f;
  float lidar_sine = sinf(lidar_yaw);
  float lidar_cosine = cosf(lidar_yaw);
  uint16_t cluster_first = 0U;
  uint16_t step;
  uint16_t bin;
  int16_t previous_step = -1000;
  float previous_x = 0.0f;
  float previous_y = 0.0f;

  sPointCount = 0U;
  sClusterCount = 0U;
  /* Start just beyond the blind rear sector so visible bins remain ordered
     across the 359/0 wrap: 316..359, 0..224. */
  for (step = 0U; (step < LD06_ANGLE_BINS) &&
                  (sPointCount < T4_MAX_POINTS); step++)
  {
    bin = (uint16_t)((T4_VEHICLE_REAR_BIN +
                      T4_REAR_BLIND_HALF_DEG + 1U + step) %
                     LD06_ANGLE_BINS);
    uint16_t range = scan->distance_mm[bin];
    int16_t offset = T4_SignedBinOffset(bin);
    int16_t rear_offset = T4_SignedAngleOffset(bin, T4_VEHICLE_REAR_BIN);
    float angle;
    float raw_x;
    float raw_y;
    float bx;
    float by;
    float link_limit;
    uint8_t continuous;
    if ((rear_offset >= -T4_REAR_BLIND_HALF_DEG) &&
        (rear_offset <= T4_REAR_BLIND_HALF_DEG)) continue;
    if ((range < T4_MIN_DISTANCE_MM) || (range > T4_MAX_DISTANCE_MM) ||
        (scan->confidence[bin] < T4_MIN_CONFIDENCE)) continue;

    angle = (float)offset * T4_PI / 180.0f;
    raw_x = (float)range * sinf(angle);
    raw_y = (float)range * cosf(angle);
    bx = T4_LIDAR_OFFSET_X_MM + raw_x * lidar_cosine + raw_y * lidar_sine;
    by = T4_LIDAR_OFFSET_Y_MM - raw_x * lidar_sine + raw_y * lidar_cosine;
    link_limit = T4_CLUSTER_LINK_BASE_MM +
                 T4_CLUSTER_LINK_SCALE * (float)range;
    continuous = ((previous_step != -1000) &&
                  (((int16_t)step - previous_step) <=
                   (int16_t)T4_CLUSTER_MAX_BIN_GAP) &&
                  (T4_PointDistance(bx, by, previous_x, previous_y) <=
                   link_limit)) ? 1U : 0U;
    if ((continuous == 0U) && (sPointCount != cluster_first))
    {
      T4_AddCluster(cluster_first, sPointCount);
      cluster_first = sPointCount;
    }
    sPoints[sPointCount].bx_mm = bx;
    sPoints[sPointCount].by_mm = by;
    sPoints[sPointCount].range_mm = range;
    sPoints[sPointCount].bin = bin;
    sPoints[sPointCount].segment = 0xFFU;
    sPoints[sPointCount].classification = T4_POINT_RAW;
    previous_step = (int16_t)step;
    previous_x = bx;
    previous_y = by;
    sPointCount++;
  }
  T4_AddCluster(cluster_first, sPointCount);
}

static uint8_t T4_FitRange(uint16_t first, uint16_t end,
                           T4_LineSegment_t *line, uint16_t *split_index,
                           float *maximum_residual)
{
  float mean_x = 0.0f;
  float mean_y = 0.0f;
  float sxx = 0.0f;
  float syy = 0.0f;
  float sxy = 0.0f;
  float trace;
  float root;
  float lambda_min;
  float direction;
  float normal_x;
  float normal_y;
  uint16_t count = (uint16_t)(end - first);
  uint16_t index;

  if (count < T4_LINE_MIN_POINTS) return 0U;
  for (index = first; index < end; index++)
  {
    mean_x += sPoints[index].bx_mm;
    mean_y += sPoints[index].by_mm;
  }
  mean_x /= (float)count;
  mean_y /= (float)count;
  for (index = first; index < end; index++)
  {
    float dx = sPoints[index].bx_mm - mean_x;
    float dy = sPoints[index].by_mm - mean_y;
    sxx += dx * dx;
    syy += dy * dy;
    sxy += dx * dy;
  }
  trace = sxx + syy;
  root = sqrtf((sxx - syy) * (sxx - syy) + 4.0f * sxy * sxy);
  lambda_min = 0.5f * (trace - root);
  direction = 0.5f * atan2f(2.0f * sxy, sxx - syy);
  normal_x = -sinf(direction);
  normal_y = cosf(direction);

  memset(line, 0, sizeof(*line));
  line->first = first;
  line->count = count;
  line->direction_deg = direction * 180.0f / T4_PI;
  line->normal_x = normal_x;
  line->normal_y = normal_y;
  line->normal_distance_mm = mean_x * normal_x + mean_y * normal_y;
  line->rms_mm = sqrtf(fmaxf(0.0f, lambda_min) / (float)count);
  line->length_mm = T4_PointDistance(
      sPoints[first].bx_mm, sPoints[first].by_mm,
      sPoints[end - 1U].bx_mm, sPoints[end - 1U].by_mm);

  *maximum_residual = 0.0f;
  *split_index = first;
  for (index = (uint16_t)(first + 1U); index < (uint16_t)(end - 1U); index++)
  {
    float residual = fabsf((sPoints[index].bx_mm - mean_x) * normal_x +
                           (sPoints[index].by_mm - mean_y) * normal_y);
    if (residual > *maximum_residual)
    {
      *maximum_residual = residual;
      *split_index = index;
    }
  }
  return 1U;
}

static void T4_StoreSegment(const T4_LineSegment_t *line)
{
  uint16_t index;
  if (sSegmentCount >= T4_MAX_SEGMENTS) return;
  sSegments[sSegmentCount] = *line;
  for (index = line->first;
       index < (uint16_t)(line->first + line->count); index++)
  {
    sPoints[index].segment = sSegmentCount;
    sPoints[index].classification = T4_POINT_LINE;
  }
  sSegmentCount++;
}

static void T4_SplitCluster(const T4_Range_t *cluster)
{
  T4_Range_t stack[T4_SPLIT_STACK_SIZE];
  uint8_t stack_count = 0U;
  stack[stack_count++] = *cluster;
  while ((stack_count != 0U) && (sSegmentCount < T4_MAX_SEGMENTS))
  {
    T4_Range_t range = stack[--stack_count];
    T4_LineSegment_t line;
    uint16_t split;
    float maximum_residual;
    if (T4_FitRange(range.first, range.end, &line, &split,
                    &maximum_residual) == 0U) continue;
    if ((maximum_residual > T4_SPLIT_RESIDUAL_MM) &&
        ((uint16_t)(split - range.first) >= T4_LINE_MIN_POINTS) &&
        ((uint16_t)(range.end - split) >= T4_LINE_MIN_POINTS) &&
        (stack_count <= (T4_SPLIT_STACK_SIZE - 2U)))
    {
      stack[stack_count].first = split;
      stack[stack_count].end = range.end;
      stack_count++;
      stack[stack_count].first = range.first;
      stack[stack_count].end = (uint16_t)(split + 1U);
      stack_count++;
    }
    else if ((line.length_mm >= T4_LINE_MIN_LENGTH_MM) &&
             (line.rms_mm <= T4_LINE_MAX_RMS_MM))
      T4_StoreSegment(&line);
  }
}

static void T4_SortSegments(void)
{
  uint8_t index;
  for (index = 1U; index < sSegmentCount; index++)
  {
    T4_LineSegment_t value = sSegments[index];
    uint8_t position = index;
    while ((position > 0U) &&
           (sSegments[position - 1U].first > value.first))
    {
      sSegments[position] = sSegments[position - 1U];
      position--;
    }
    sSegments[position] = value;
  }
}

static void T4_ReassignSegmentIds(void)
{
  uint8_t segment;
  for (segment = 0U; segment < sSegmentCount; segment++)
  {
    uint16_t index;
    uint16_t end = (uint16_t)(sSegments[segment].first +
                              sSegments[segment].count);
    for (index = sSegments[segment].first; index < end; index++)
      sPoints[index].segment = segment;
  }
}

static void T4_MergeSegments(void)
{
  uint8_t index = 0U;
  T4_SortSegments();
  while ((uint8_t)(index + 1U) < sSegmentCount)
  {
    T4_LineSegment_t *first = &sSegments[index];
    T4_LineSegment_t *second = &sSegments[index + 1U];
    uint16_t first_end = (uint16_t)(first->first + first->count);
    float gap = T4_PointDistance(
        sPoints[first_end - 1U].bx_mm, sPoints[first_end - 1U].by_mm,
        sPoints[second->first].bx_mm, sPoints[second->first].by_mm);
    if ((first_end >= second->first) &&
        (gap <= T4_MERGE_GAP_MM) &&
        (T4_LineAngleDifference(first->direction_deg,
                                second->direction_deg) <=
         T4_MERGE_ANGLE_DEG))
    {
      T4_LineSegment_t merged;
      uint16_t split;
      float maximum_residual;
      uint16_t merged_end = (uint16_t)(second->first + second->count);
      if ((T4_FitRange(first->first, merged_end, &merged, &split,
                       &maximum_residual) != 0U) &&
          (merged.rms_mm <= T4_LINE_MAX_RMS_MM))
      {
        uint8_t move;
        *first = merged;
        for (move = (uint8_t)(index + 1U);
             (uint8_t)(move + 1U) < sSegmentCount; move++)
          sSegments[move] = sSegments[move + 1U];
        sSegmentCount--;
        continue;
      }
    }
    index++;
  }
  T4_ReassignSegmentIds();
}

static void T4_FindCorners(void)
{
  uint8_t first;
  sCornerCount = 0U;
  sBestCornerFirst = 0xFFU;
  sBestCornerSecond = 0xFFU;
  sBestCornerAngle = 0.0f;
  sBestLineRms = 0.0f;
  for (first = 0U; first < sSegmentCount; first++)
  {
    uint8_t second;
    if ((sBestLineRms == 0.0f) ||
        (sSegments[first].rms_mm < sBestLineRms))
      sBestLineRms = sSegments[first].rms_mm;
    for (second = (uint8_t)(first + 1U);
         second < sSegmentCount; second++)
    {
      float angle = T4_LineAngleDifference(sSegments[first].direction_deg,
                                           sSegments[second].direction_deg);
      uint16_t first_end = (uint16_t)(sSegments[first].first +
                                      sSegments[first].count - 1U);
      uint16_t second_end = (uint16_t)(sSegments[second].first +
                                       sSegments[second].count - 1U);
      float gap_a = T4_PointDistance(sPoints[first_end].bx_mm,
                                     sPoints[first_end].by_mm,
                                     sPoints[sSegments[second].first].bx_mm,
                                     sPoints[sSegments[second].first].by_mm);
      float gap_b = T4_PointDistance(sPoints[sSegments[first].first].bx_mm,
                                     sPoints[sSegments[first].first].by_mm,
                                     sPoints[second_end].bx_mm,
                                     sPoints[second_end].by_mm);
      float endpoint_gap = fminf(gap_a, gap_b);
      if ((sSegments[first].length_mm >= T4_LINE_MIN_LENGTH_MM) &&
          (sSegments[second].length_mm >= T4_LINE_MIN_LENGTH_MM) &&
          (angle >= T4_CORNER_MIN_DEG) &&
          (angle <= T4_CORNER_MAX_DEG) &&
          (endpoint_gap <= T4_CORNER_MAX_GAP_MM))
      {
        float current_error = fabsf(angle - 90.0f);
        float best_error = fabsf(sBestCornerAngle - 90.0f);
        sCornerCount++;
        if ((sBestCornerAngle == 0.0f) || (current_error < best_error))
        {
          sBestCornerAngle = angle;
          sBestCornerFirst = first;
          sBestCornerSecond = second;
        }
      }
    }
  }
}

static void T4_PreprocessScan(const LD06_Data_t *scan)
{
  uint8_t cluster;
  T4_CollectPoints(scan);
  sSegmentCount = 0U;
  for (cluster = 0U; cluster < sClusterCount; cluster++)
    T4_SplitCluster(&sClusters[cluster]);
  T4_MergeSegments();
  T4_FindCorners();
  sPose.line_segment_count = sSegmentCount;
  sPose.corner_candidate_count = sCornerCount;
  sPose.best_corner_angle_deg = sBestCornerAngle;
  sPose.line_fit_rms_mm = sBestLineRms;
}

static uint8_t T4_GetCornerGeometry(uint8_t first, uint8_t second,
                                    float *angle, float *endpoint_gap)
{
  uint16_t first_end = (uint16_t)(sSegments[first].first +
                                  sSegments[first].count - 1U);
  uint16_t second_end = (uint16_t)(sSegments[second].first +
                                   sSegments[second].count - 1U);
  float gap_a = T4_PointDistance(sPoints[first_end].bx_mm,
                                 sPoints[first_end].by_mm,
                                 sPoints[sSegments[second].first].bx_mm,
                                 sPoints[sSegments[second].first].by_mm);
  float gap_b = T4_PointDistance(sPoints[sSegments[first].first].bx_mm,
                                 sPoints[sSegments[first].first].by_mm,
                                 sPoints[second_end].bx_mm,
                                 sPoints[second_end].by_mm);
  *angle = T4_LineAngleDifference(sSegments[first].direction_deg,
                                  sSegments[second].direction_deg);
  *endpoint_gap = fminf(gap_a, gap_b);
  return ((sSegments[first].length_mm >= T4_LINE_MIN_LENGTH_MM) &&
          (sSegments[second].length_mm >= T4_LINE_MIN_LENGTH_MM) &&
          (*angle >= T4_CORNER_MIN_DEG) &&
          (*angle <= T4_CORNER_MAX_DEG) &&
          (*endpoint_gap <= T4_CORNER_MAX_GAP_MM)) ? 1U : 0U;
}

static uint8_t T4_IntersectLines(const T4_LineSegment_t *first,
                                 const T4_LineSegment_t *second,
                                 float *x, float *y)
{
  float determinant = first->normal_x * second->normal_y -
                      first->normal_y * second->normal_x;
  if (fabsf(determinant) < 0.15f) return 0U;
  *x = (first->normal_distance_mm * second->normal_y -
        first->normal_y * second->normal_distance_mm) / determinant;
  *y = (first->normal_x * second->normal_distance_mm -
        first->normal_distance_mm * second->normal_x) / determinant;
  return 1U;
}

static void T4_GetMapCorner(uint8_t corner, float *x, float *y,
                            uint8_t *wall_mask)
{
  if ((corner == 1U) || (corner == 3U)) *x = 0.0f;
  else *x = T4_FIELD_MM;
  if ((corner == 1U) || (corner == 2U)) *y = T4_FIELD_MM;
  else *y = 0.0f;
  if (corner == 1U) *wall_mask = 0x09U;
  else if (corner == 2U) *wall_mask = 0x0AU;
  else if (corner == 3U) *wall_mask = 0x05U;
  else *wall_mask = 0x06U;
}

static void T4_GetStartPrior(uint8_t start, float *x, float *y, float *yaw)
{
  static const float start_x[4] = {300.0f, 2700.0f, 300.0f, 2700.0f};
  static const float start_y[4] = {2700.0f, 2700.0f, 300.0f, 300.0f};
  static const float start_yaw[4] = {180.0f, 180.0f, 0.0f, 0.0f};
  uint8_t index = (start >= 1U && start <= 4U) ? (uint8_t)(start - 1U) : 0U;
  *x = start_x[index];
  *y = start_y[index];
  *yaw = start_yaw[index];
}

static uint8_t T4_IsInsideField(const T4_Match_t *match)
{
  return ((match->x_mm >= -T4_POSITION_TOLERANCE_MM) &&
          (match->x_mm <= T4_FIELD_MM + T4_POSITION_TOLERANCE_MM) &&
          (match->y_mm >= -T4_POSITION_TOLERANCE_MM) &&
          (match->y_mm <= T4_FIELD_MM + T4_POSITION_TOLERANCE_MM)) ? 1U : 0U;
}

static uint8_t T4_IsInSelectedQuadrant(const T4_Match_t *match)
{
  if (((sSelectedStart == 1U) || (sSelectedStart == 3U)) &&
      (match->x_mm > T4_START_QUADRANT_MM)) return 0U;
  if (((sSelectedStart == 2U) || (sSelectedStart == 4U)) &&
      (match->x_mm < T4_START_QUADRANT_MM)) return 0U;
  if (((sSelectedStart == 1U) || (sSelectedStart == 2U)) &&
      (match->y_mm < T4_START_QUADRANT_MM)) return 0U;
  if (((sSelectedStart == 3U) || (sSelectedStart == 4U)) &&
      (match->y_mm > T4_START_QUADRANT_MM)) return 0U;
  return 1U;
}

static float T4_LineYaw(const T4_LineSegment_t *line,
                        uint8_t horizontal_wall, float reference_yaw)
{
  float base = line->direction_deg -
               ((horizontal_wall != 0U) ? 0.0f : 90.0f);
  return T4_Wrap180(reference_yaw + T4_Wrap90(base - reference_yaw));
}

static uint8_t T4_MakeCornerMatch(uint8_t first, uint8_t second,
                                  uint8_t first_is_horizontal,
                                  uint8_t map_corner,
                                  float corner_x, float corner_y,
                                  float reference_x, float reference_y,
                                  float reference_yaw, uint8_t initializing,
                                  T4_Match_t *match,
                                  Task4_FailureReason_t *failure)
{
  const T4_LineSegment_t *line_first = &sSegments[first];
  const T4_LineSegment_t *line_second = &sSegments[second];
  float map_corner_x;
  float map_corner_y;
  float yaw_first = T4_LineYaw(line_first, first_is_horizontal,
                               reference_yaw);
  float yaw_second = T4_LineYaw(line_second,
                                (uint8_t)(first_is_horizontal == 0U),
                                reference_yaw);
  float yaw_difference = fabsf(T4_Wrap180(yaw_first - yaw_second));
  float yaw;
  float angle;
  float sine;
  float cosine;
  uint8_t wall_mask;
  if (yaw_difference > T4_LINE_YAW_AGREEMENT_DEG)
  {
    *failure = T4_FAIL_YAW_INCONSISTENT;
    return 0U;
  }
  yaw = T4_Wrap180(reference_yaw +
                   0.5f * (T4_Wrap180(yaw_first - reference_yaw) +
                           T4_Wrap180(yaw_second - reference_yaw)));
  angle = yaw * T4_PI / 180.0f;
  sine = sinf(angle);
  cosine = cosf(angle);
  T4_GetMapCorner(map_corner, &map_corner_x, &map_corner_y, &wall_mask);
  memset(match, 0, sizeof(*match));
  match->x_mm = map_corner_x - corner_x * cosine - corner_y * sine;
  match->y_mm = map_corner_y + corner_x * sine - corner_y * cosine;
  match->yaw_deg = yaw;
  match->rms_mm = sqrtf(0.5f *
      (line_first->rms_mm * line_first->rms_mm +
       line_second->rms_mm * line_second->rms_mm));
  match->inliers = (uint16_t)(line_first->count + line_second->count);
  match->wall_mask = wall_mask;
  match->wall_count = 2U;
  if (T4_IsInsideField(match) == 0U)
  {
    *failure = T4_FAIL_POSE_OUTSIDE;
    return 0U;
  }
  if ((initializing != 0U) && (T4_IsInSelectedQuadrant(match) == 0U))
  {
    *failure = T4_FAIL_POSE_OUTSIDE;
    return 0U;
  }
  if ((initializing == 0U) &&
      ((T4_PointDistance(match->x_mm, match->y_mm,
                         reference_x, reference_y) > T4_TRACK_POS_GATE_MM) ||
       (fabsf(T4_Wrap180(match->yaw_deg - reference_yaw)) >
        T4_TRACK_YAW_GATE_DEG)))
  {
    *failure = T4_FAIL_POSE_OUTSIDE;
    return 0U;
  }
  match->x_mm = T4_Clamp(match->x_mm, 0.0f, T4_FIELD_MM);
  match->y_mm = T4_Clamp(match->y_mm, 0.0f, T4_FIELD_MM);
  match->cost = match->rms_mm + 2.0f * yaw_difference +
                0.01f * T4_PointDistance(match->x_mm, match->y_mm,
                                         reference_x, reference_y) +
                fabsf(T4_Wrap180(match->yaw_deg - reference_yaw));
  match->valid = 1U;
  *failure = T4_FAIL_NONE;
  return 1U;
}

static uint8_t T4_SolveCornerPose(float reference_x, float reference_y,
                                  float reference_yaw, uint8_t initializing,
                                  T4_Match_t *best,
                                  Task4_FailureReason_t *failure)
{
  uint8_t first;
  uint8_t found = 0U;
  uint8_t saw_intersection = 0U;
  uint8_t saw_reasonable_intersection = 0U;
  uint8_t saw_yaw_consistent = 0U;
  best->cost = 1.0e9f;
  *failure = T4_FAIL_NO_RIGHT_ANGLE;
  for (first = 0U; first < sSegmentCount; first++)
  {
    uint8_t second;
    for (second = (uint8_t)(first + 1U); second < sSegmentCount; second++)
    {
      float corner_angle;
      float endpoint_gap;
      float corner_x;
      float corner_y;
      float corner_range;
      uint8_t first_is_horizontal;
      uint8_t first_corner = 1U;
      uint8_t last_corner = 4U;
      uint8_t map_corner;
      if (T4_GetCornerGeometry(first, second, &corner_angle,
                               &endpoint_gap) == 0U) continue;
      if (T4_IntersectLines(&sSegments[first], &sSegments[second],
                            &corner_x, &corner_y) == 0U)
      {
        *failure = T4_FAIL_PARALLEL_LINES;
        continue;
      }
      saw_intersection = 1U;
      corner_range = sqrtf(corner_x * corner_x + corner_y * corner_y);
      if (corner_range > T4_CORNER_MAX_RANGE_MM)
      {
        *failure = T4_FAIL_BAD_INTERSECTION;
        continue;
      }
      saw_reasonable_intersection = 1U;
      for (map_corner = first_corner; map_corner <= last_corner; map_corner++)
      {
        for (first_is_horizontal = 0U; first_is_horizontal <= 1U;
             first_is_horizontal++)
        {
          T4_Match_t candidate;
          Task4_FailureReason_t candidate_failure;
          if ((T4_MakeCornerMatch(first, second, first_is_horizontal,
                                  map_corner, corner_x, corner_y,
                                  reference_x, reference_y, reference_yaw,
                                  initializing, &candidate,
                                  &candidate_failure) != 0U) &&
              (candidate.cost < best->cost))
          {
            *best = candidate;
            sBestCornerFirst = first;
            sBestCornerSecond = second;
            sBestCornerAngle = corner_angle;
            sBestCornerX = corner_x;
            sBestCornerY = corner_y;
            found = 1U;
          }
          if (candidate_failure != T4_FAIL_YAW_INCONSISTENT)
            saw_yaw_consistent = 1U;
        }
      }
    }
  }
  if (found != 0U)
  {
    *failure = T4_FAIL_NONE;
    return 1U;
  }
  if ((saw_reasonable_intersection != 0U) &&
      (saw_yaw_consistent == 0U)) *failure = T4_FAIL_YAW_INCONSISTENT;
  else if (saw_reasonable_intersection != 0U) *failure = T4_FAIL_POSE_OUTSIDE;
  else if (saw_intersection != 0U) *failure = T4_FAIL_BAD_INTERSECTION;
  return 0U;
}

static uint8_t T4_SolveSingleWall(float reference_yaw, T4_Match_t *best)
{
  uint8_t segment;
  uint8_t found = 0U;
  best->cost = 1.0e9f;
  for (segment = 0U; segment < sSegmentCount; segment++)
  {
    const T4_LineSegment_t *line = &sSegments[segment];
    uint8_t horizontal;
    if (line->length_mm < T4_LINE_MIN_LENGTH_MM) continue;
    for (horizontal = 0U; horizontal <= 1U; horizontal++)
    {
      float yaw = T4_LineYaw(line, horizontal, reference_yaw);
      float yaw_delta = fabsf(T4_Wrap180(yaw - reference_yaw));
      float radians;
      float sine;
      float cosine;
      float world_normal_x;
      float world_normal_y;
      uint8_t side;
      if (yaw_delta > T4_TRACK_YAW_GATE_DEG) continue;
      radians = yaw * T4_PI / 180.0f;
      sine = sinf(radians);
      cosine = cosf(radians);
      world_normal_x = line->normal_x * cosine + line->normal_y * sine;
      world_normal_y = -line->normal_x * sine + line->normal_y * cosine;
      for (side = 0U; side <= 1U; side++)
      {
        T4_Match_t candidate;
        float wall_coordinate = (side == 0U) ? 0.0f : T4_FIELD_MM;
        memset(&candidate, 0, sizeof(candidate));
        candidate.x_mm = sPose.x_mm;
        candidate.y_mm = sPose.y_mm;
        candidate.yaw_deg = yaw;
        if (horizontal != 0U)
        {
          if (fabsf(world_normal_y) < 0.7f) continue;
          candidate.y_mm = wall_coordinate -
                           line->normal_distance_mm / world_normal_y;
          candidate.wall_mask = (side == 0U) ? 0x04U : 0x08U;
        }
        else
        {
          if (fabsf(world_normal_x) < 0.7f) continue;
          candidate.x_mm = wall_coordinate -
                           line->normal_distance_mm / world_normal_x;
          candidate.wall_mask = (side == 0U) ? 0x01U : 0x02U;
        }
        if ((T4_IsInsideField(&candidate) == 0U) ||
            (T4_PointDistance(candidate.x_mm, candidate.y_mm,
                              sPose.x_mm, sPose.y_mm) >
             T4_TRACK_POS_GATE_MM)) continue;
        candidate.x_mm = T4_Clamp(candidate.x_mm, 0.0f, T4_FIELD_MM);
        candidate.y_mm = T4_Clamp(candidate.y_mm, 0.0f, T4_FIELD_MM);
        candidate.rms_mm = line->rms_mm;
        candidate.inliers = line->count;
        candidate.wall_count = 1U;
        candidate.cost = line->rms_mm + yaw_delta +
                         0.02f * T4_PointDistance(candidate.x_mm,
                                                 candidate.y_mm,
                                                 sPose.x_mm, sPose.y_mm);
        candidate.valid = 1U;
        if (candidate.cost < best->cost)
        {
          *best = candidate;
          found = 1U;
        }
      }
    }
  }
  return found;
}

static void T4_ClassifyPoints(const T4_Match_t *match)
{
  uint16_t index;
  uint8_t segment;
  for (index = 0U; index < sPointCount; index++)
  {
    float wx;
    float wy;
    float residual;
    uint8_t wall;
    T4_ToWorld(match->x_mm, match->y_mm, match->yaw_deg,
               sPoints[index].bx_mm, sPoints[index].by_mm, &wx, &wy);
    residual = T4_NearestWall(wx, wy, &wall);
    sPoints[index].classification =
        (residual <= T4_WALL_INLIER_MM) ? T4_POINT_WALL : T4_POINT_UNMATCHED;
  }
  for (segment = 0U; segment < sSegmentCount; segment++)
  {
    T4_LineSegment_t *candidate = &sSegments[segment];
    float center_x = 0.0f;
    float center_y = 0.0f;
    float residual;
    uint8_t wall;
    uint16_t index_end;
    if ((candidate->length_mm < T4_OBSTACLE_MIN_LENGTH_MM) ||
        (candidate->length_mm > T4_OBSTACLE_MAX_LENGTH_MM)) continue;
    index_end = (uint16_t)(candidate->first + candidate->count);
    for (index = candidate->first; index < index_end; index++)
    {
      float wx;
      float wy;
      T4_ToWorld(match->x_mm, match->y_mm, match->yaw_deg,
                 sPoints[index].bx_mm, sPoints[index].by_mm, &wx, &wy);
      center_x += wx;
      center_y += wy;
    }
    center_x /= (float)candidate->count;
    center_y /= (float)candidate->count;
    residual = T4_NearestWall(center_x, center_y, &wall);
    if ((residual > T4_OBSTACLE_WALL_GAP_MM) &&
        (center_x > 0.0f) && (center_x < T4_FIELD_MM) &&
        (center_y > 0.0f) && (center_y < T4_FIELD_MM))
      for (index = candidate->first; index < index_end; index++)
        sPoints[index].classification = T4_POINT_OBSTACLE;
  }
}

static void T4_UpdateObstacle(const T4_Match_t *match)
{
  float center_x = 0.0f;
  float center_y = 0.0f;
  float nearest = 1.0e9f;
  uint16_t count = 0U;
  uint16_t index;
  for (index = 0U; index < sPointCount; index++)
  {
    float wx;
    float wy;
    if (sPoints[index].classification != T4_POINT_OBSTACLE) continue;
    T4_ToWorld(match->x_mm, match->y_mm, match->yaw_deg,
               sPoints[index].bx_mm, sPoints[index].by_mm, &wx, &wy);
    center_x += wx;
    center_y += wy;
    if ((float)sPoints[index].range_mm < nearest)
      nearest = (float)sPoints[index].range_mm;
    count++;
  }
  if (count == 0U)
  {
    sObstacle.consecutive_frames = 0U;
    sObstacle.valid = 0U;
  }
  else
  {
    uint8_t same = 0U;
    center_x /= (float)count;
    center_y /= (float)count;
    if ((sObstacle.consecutive_frames != 0U) &&
        (T4_PointDistance(center_x, center_y,
                          sObstacle.x_mm, sObstacle.y_mm) <=
         T4_OBSTACLE_TRACK_GATE_MM)) same = 1U;
    if (same != 0U)
    {
      if (sObstacle.consecutive_frames < 255U)
        sObstacle.consecutive_frames++;
    }
    else sObstacle.consecutive_frames = 1U;
    sObstacle.x_mm = center_x;
    sObstacle.y_mm = center_y;
    sObstacle.distance_mm = nearest;
    sObstacle.points = count;
    sObstacle.valid =
        (sObstacle.consecutive_frames >= T4_OBSTACLE_CONFIRM_FRAMES) ? 1U : 0U;
  }
  sPose.dynamic_points = (count > 255U) ? 255U : (uint8_t)count;
  sPose.obstacle_confirmed = sObstacle.valid;
  if (sObstacle.valid != 0U)
  {
    sPose.enemy_x_mm = sObstacle.x_mm;
    sPose.enemy_y_mm = sObstacle.y_mm;
    sPose.enemy_distance_mm = sObstacle.distance_mm;
  }
  else
  {
    sPose.enemy_x_mm = 0.0f;
    sPose.enemy_y_mm = 0.0f;
    sPose.enemy_distance_mm = 0.0f;
  }
}

static void T4_PublishMatch(T4_Match_t *match, const LD06_Data_t *scan,
                            uint32_t now_ms, uint8_t provisional)
{
  const BMI088_Attitude_t *imu = AppScheduler_GetAttitude();
  sPose.x_mm = match->x_mm;
  sPose.y_mm = match->y_mm;
  sPose.yaw_deg = match->yaw_deg;
  sPose.rms_mm = match->rms_mm;
  sPose.inlier_count = match->inliers;
  sPose.wall_count = match->wall_count;
  sPose.lidar_sequence = scan->scan_sequence;
  sPose.timestamp_ms = now_ms;
  sPose.sequence++;
  sPose.start_candidate = sSelectedStart;
  sPose.selected_start = sSelectedStart;
  if (provisional != 0U)
  {
    sPose.status = POSE_PROVISIONAL;
    sPose.localization_stage = T4_STAGE_PROVISIONAL;
    sPose.valid = 0U;
  }
  else
  {
    sPose.status = (match->wall_count >= 2U) ?
                   POSE_FIX_HIGH : POSE_FIX_LOW;
    sPose.localization_stage = T4_STAGE_FIXED;
    sPose.valid = 1U;
    sLastFixTick = now_ms;
    if (imu != 0) sImuYawAtFix = imu->yaw_deg;
  }
  T4_ClassifyPoints(match);
  if (provisional == 0U) T4_UpdateObstacle(match);
}

static void T4_SetFailure(Task4_FailureReason_t reason)
{
  sPose.last_failure_reason = (uint8_t)reason;
}

static void T4_ProcessInitialization(const LD06_Data_t *scan, uint32_t now_ms)
{
  T4_Match_t match;
  float start_x;
  float start_y;
  float start_yaw;
  Task4_FailureReason_t failure;
  if (sPointCount < T4_MIN_INLIERS)
  {
    sProvisionalFrames = 0U;
    sPose.localization_stage = T4_STAGE_FIND_LINES;
    T4_SetFailure(T4_FAIL_TOO_FEW_POINTS);
    return;
  }
  if (sSegmentCount < 2U)
  {
    sProvisionalFrames = 0U;
    sPose.localization_stage = T4_STAGE_FIND_LINES;
    T4_SetFailure(T4_FAIL_NO_LINE);
    return;
  }
  if (sCornerCount == 0U)
  {
    sProvisionalFrames = 0U;
    sPose.localization_stage = T4_STAGE_FIND_CORNER;
    T4_SetFailure(T4_FAIL_NO_RIGHT_ANGLE);
    return;
  }
  T4_GetStartPrior(sSelectedStart, &start_x, &start_y, &start_yaw);
  if (T4_SolveCornerPose(start_x, start_y, start_yaw, 1U,
                         &match, &failure) == 0U)
  {
    sProvisionalFrames = 0U;
    sPose.localization_stage = T4_STAGE_FIND_CORNER;
    T4_SetFailure(failure);
    return;
  }
  if ((sProvisionalFrames != 0U) &&
      (T4_PointDistance(match.x_mm, match.y_mm,
                        sProvisional.x_mm, sProvisional.y_mm) <=
       T4_PROVISIONAL_POS_GATE_MM) &&
      (fabsf(T4_Wrap180(match.yaw_deg - sProvisional.yaw_deg)) <=
       T4_PROVISIONAL_YAW_GATE_DEG))
  {
    sProvisionalFrames++;
    T4_SetFailure(T4_FAIL_NONE);
  }
  else
  {
    sProvisionalFrames = 1U;
    T4_SetFailure((sPose.localization_stage == T4_STAGE_PROVISIONAL) ?
                  T4_FAIL_NOT_CONSECUTIVE : T4_FAIL_NONE);
  }
  sProvisional = match;
  sPose.corner_body_x_mm = sBestCornerX;
  sPose.corner_body_y_mm = sBestCornerY;
  sPose.best_corner_angle_deg = sBestCornerAngle;
  sPose.line_fit_rms_mm = match.rms_mm;
  if (sProvisionalFrames >= 2U)
    T4_PublishMatch(&match, scan, now_ms, 0U);
  else
    T4_PublishMatch(&match, scan, now_ms, 1U);
}

static void T4_Degrade(uint32_t now_ms)
{
  uint32_t age = (uint32_t)(now_ms - sLastFixTick);
  if (age < T4_DEGRADED_MS) return;
  if ((sPose.valid != 0U) && (age < T4_LOST_MS))
  {
    sPose.status = POSE_DEGRADED;
    sPose.localization_stage = T4_STAGE_DEGRADED;
  }
  else
  {
    sPose.status = POSE_LOST;
    sPose.localization_stage = T4_STAGE_LOST;
    sPose.valid = 0U;
  }
  T4_SetFailure(T4_FAIL_TRACKING_TIMEOUT);
}

static void T4_ProcessTracking(const LD06_Data_t *scan, uint32_t now_ms)
{
  const BMI088_Attitude_t *imu = AppScheduler_GetAttitude();
  T4_Match_t match;
  Task4_FailureReason_t failure;
  float predicted_yaw = sPose.yaw_deg;
  if ((imu != 0) && (imu->bias_ready != 0U))
    predicted_yaw = T4_Wrap180(sPose.yaw_deg + imu->yaw_deg - sImuYawAtFix);
  if (T4_SolveCornerPose(sPose.x_mm, sPose.y_mm, predicted_yaw, 0U,
                         &match, &failure) != 0U)
  {
    match.x_mm = sPose.x_mm + T4_POSE_FILTER_ALPHA *
                 (match.x_mm - sPose.x_mm);
    match.y_mm = sPose.y_mm + T4_POSE_FILTER_ALPHA *
                 (match.y_mm - sPose.y_mm);
    match.yaw_deg = T4_Wrap180(sPose.yaw_deg + T4_POSE_FILTER_ALPHA *
                    T4_Wrap180(match.yaw_deg - sPose.yaw_deg));
    sPose.corner_body_x_mm = sBestCornerX;
    sPose.corner_body_y_mm = sBestCornerY;
    sPose.best_corner_angle_deg = sBestCornerAngle;
    sPose.line_fit_rms_mm = match.rms_mm;
    T4_SetFailure(T4_FAIL_NONE);
    T4_PublishMatch(&match, scan, now_ms, 0U);
  }
  else if (T4_SolveSingleWall(predicted_yaw, &match) != 0U)
  {
    match.x_mm = sPose.x_mm + T4_POSE_FILTER_ALPHA *
                 (match.x_mm - sPose.x_mm);
    match.y_mm = sPose.y_mm + T4_POSE_FILTER_ALPHA *
                 (match.y_mm - sPose.y_mm);
    match.yaw_deg = T4_Wrap180(sPose.yaw_deg + T4_POSE_FILTER_ALPHA *
                    T4_Wrap180(match.yaw_deg - sPose.yaw_deg));
    T4_SetFailure(T4_FAIL_NONE);
    T4_PublishMatch(&match, scan, now_ms, 0U);
  }
  else
  {
    T4_SetFailure(failure);
    T4_Degrade(now_ms);
  }
}

static uint16_t T4_MapX(float x_mm)
{
  int32_t pixel = (int32_t)(x_mm * (float)T4_MAP_SIZE / T4_FIELD_MM + 0.5f);
  if (pixel < 0) pixel = 0;
  if (pixel > T4_MAP_SIZE) pixel = T4_MAP_SIZE;
  return (uint16_t)(T4_MAP_X0 + pixel);
}

static uint16_t T4_MapY(float y_mm)
{
  int32_t pixel = (int32_t)(y_mm * (float)T4_MAP_SIZE / T4_FIELD_MM + 0.5f);
  if (pixel < 0) pixel = 0;
  if (pixel > T4_MAP_SIZE) pixel = T4_MAP_SIZE;
  return (uint16_t)(T4_MAP_Y0 + T4_MAP_SIZE - pixel);
}

static void T4_ClearMap(void)
{
  LCD_Fill(T4_MAP_X0 + 1U, T4_MAP_Y0 + 1U,
           T4_MAP_X0 + T4_MAP_SIZE - 1U,
           T4_MAP_Y0 + T4_MAP_SIZE - 1U, BLACK);
}

static void T4_DrawBoundary(void)
{
  LCD_DrawRectangle(T4_MAP_X0, T4_MAP_Y0,
                    T4_MAP_X0 + T4_MAP_SIZE,
                    T4_MAP_Y0 + T4_MAP_SIZE, GRAY);
}

static void T4_DrawLocalScan(void)
{
  const float scale = (float)(T4_MAP_SIZE / 2U - 6U) / T4_LOCAL_RANGE_MM;
  const int16_t center_x = (int16_t)(T4_MAP_X0 + T4_MAP_SIZE / 2U);
  const int16_t center_y = (int16_t)(T4_MAP_Y0 + T4_MAP_SIZE / 2U);
  uint16_t index;
  T4_ClearMap();
  LCD_DrawLine((uint16_t)center_x, (uint16_t)center_y,
               (uint16_t)center_x, T4_MAP_Y0 + 4U, DARKBLUE);
  for (index = 0U; index < sPointCount; index++)
  {
    int16_t x = center_x + (int16_t)(sPoints[index].bx_mm * scale);
    int16_t y = center_y - (int16_t)(sPoints[index].by_mm * scale);
    uint16_t color = (sPoints[index].segment == 0xFFU) ? GRAY : DARKBLUE;
    if (sPoints[index].segment == sBestCornerFirst) color = CYAN;
    if (sPoints[index].segment == sBestCornerSecond) color = YELLOW;
    if ((x > (int16_t)T4_MAP_X0) &&
        (x < (int16_t)(T4_MAP_X0 + T4_MAP_SIZE)) &&
        (y > (int16_t)T4_MAP_Y0) &&
        (y < (int16_t)(T4_MAP_Y0 + T4_MAP_SIZE)))
      LCD_DrawPoint((uint16_t)x, (uint16_t)y, color);
  }
  LCD_Fill((uint16_t)(center_x - 2), (uint16_t)(center_y - 2),
           (uint16_t)(center_x + 2), (uint16_t)(center_y + 2), WHITE);
  T4_DrawBoundary();
}

static void T4_DrawGlobalScan(void)
{
  uint16_t index;
  T4_ClearMap();
  for (index = 0U; index < sPointCount; index++)
  {
    float wx;
    float wy;
    uint16_t color;
    T4_ToWorld(sPose.x_mm, sPose.y_mm, sPose.yaw_deg,
               sPoints[index].bx_mm, sPoints[index].by_mm, &wx, &wy);
    if ((wx < 0.0f) || (wx > T4_FIELD_MM) ||
        (wy < 0.0f) || (wy > T4_FIELD_MM)) continue;
    color = (sPoints[index].classification == T4_POINT_WALL) ? GREEN :
            (sPoints[index].classification == T4_POINT_OBSTACLE) ? RED :
            YELLOW;
    LCD_DrawPoint(T4_MapX(wx), T4_MapY(wy), color);
  }
  {
    uint16_t car_x = T4_MapX(sPose.x_mm);
    uint16_t car_y = T4_MapY(sPose.y_mm);
    float angle = sPose.yaw_deg * T4_PI / 180.0f;
    int16_t head_x = (int16_t)car_x + (int16_t)(sinf(angle) * 12.0f);
    int16_t head_y = (int16_t)car_y - (int16_t)(cosf(angle) * 12.0f);
    LCD_Fill((uint16_t)(car_x - 2U), (uint16_t)(car_y - 2U),
             (uint16_t)(car_x + 2U), (uint16_t)(car_y + 2U), WHITE);
    if (head_x < (int16_t)T4_MAP_X0) head_x = (int16_t)T4_MAP_X0;
    if (head_x > (int16_t)(T4_MAP_X0 + T4_MAP_SIZE))
      head_x = (int16_t)(T4_MAP_X0 + T4_MAP_SIZE);
    if (head_y < (int16_t)T4_MAP_Y0) head_y = (int16_t)T4_MAP_Y0;
    if (head_y > (int16_t)(T4_MAP_Y0 + T4_MAP_SIZE))
      head_y = (int16_t)(T4_MAP_Y0 + T4_MAP_SIZE);
    LCD_DrawLine(car_x, car_y, (uint16_t)head_x, (uint16_t)head_y, CYAN);
  }
  T4_DrawBoundary();
}

static const char *T4_StageText(void)
{
  switch ((Task4_LocalizationStage_t)sPose.localization_stage)
  {
    case T4_STAGE_SELECT_START: return "SELECT";
    case T4_STAGE_WAIT_SCAN: return "WAIT";
    case T4_STAGE_FIND_LINES: return "LINES";
    case T4_STAGE_FIND_CORNER: return "CORNER";
    case T4_STAGE_PROVISIONAL: return "PROVIS";
    case T4_STAGE_FIXED: return
        (sPose.status == POSE_FIX_HIGH) ? "FIX HI" : "FIX LO";
    case T4_STAGE_DEGRADED: return "DEGRAD";
    case T4_STAGE_LOST: return "LOST";
    default: return "UNKNOWN";
  }
}

static const char *T4_FailureText(void)
{
  switch ((Task4_FailureReason_t)sPose.last_failure_reason)
  {
    case T4_FAIL_NONE: return "OK";
    case T4_FAIL_NO_SCAN: return "NO SCAN";
    case T4_FAIL_TOO_FEW_POINTS: return "FEW PTS";
    case T4_FAIL_NO_LINE: return "NO LINE";
    case T4_FAIL_NO_RIGHT_ANGLE: return "NO 90";
    case T4_FAIL_PARALLEL_LINES: return "PARALLEL";
    case T4_FAIL_BAD_INTERSECTION: return "BAD CROSS";
    case T4_FAIL_YAW_INCONSISTENT: return "YAW DIFF";
    case T4_FAIL_POSE_OUTSIDE: return "OUTSIDE";
    case T4_FAIL_NOT_CONSECUTIVE: return "NOT CONS";
    case T4_FAIL_TRACKING_TIMEOUT: return "TIMEOUT";
    default: return "UNKNOWN";
  }
}

#if CHASSIS_ESCAPE_TEST_ENABLE
static const char *T4_EscapeStageText(void)
{
  const ChassisEscapeRequest_t *request = ChassisEscapeTest_GetRequest();
  switch ((ChassisEscapeStage_t)request->stage)
  {
    case CHASSIS_ESCAPE_WAIT_SCAN: return "WAIT ESC";
    case CHASSIS_ESCAPE_STRAIGHT: return "ESCAPE";
    case CHASSIS_ESCAPE_BRAKE: return "ESC BRAKE";
    case CHASSIS_ESCAPE_RELOCALIZE: return "RELOCAL";
    case CHASSIS_ESCAPE_FAILED:
      return (request->safety_reason == 7U) ? "CAL REQ" :
             (request->safety_reason == 8U) ? "ESC LIMIT" : "ESC FAIL";
    case CHASSIS_ESCAPE_PAUSED_SAFETY: return "ESC PAUSE";
    default: return "ESC TEST";
  }
}
#endif

static void T4_DrawFrame(void)
{
  LCD_WR_REG(0x28U);
  LCD_Fill(0U, 0U, LCD_W, LCD_H, BLACK);
  LCD_ShowString(10U, 5U, (const uint8_t *)"LIDAR CORNER MAP",
                 CYAN, BLACK, 16U, 0U);
  T4_DrawBoundary();
  LCD_ShowString(190U, 45U, (const uint8_t *)"ST", GRAY, BLACK, 12U, 0U);
  LCD_ShowString(190U, 63U, (const uint8_t *)"MD", GRAY, BLACK, 12U, 0U);
  LCD_ShowString(190U, 81U, (const uint8_t *)"S", GRAY, BLACK, 12U, 0U);
  LCD_ShowString(190U, 99U, (const uint8_t *)"P", GRAY, BLACK, 12U, 0U);
  LCD_ShowString(190U, 117U, (const uint8_t *)"L/C", GRAY, BLACK, 12U, 0U);
  LCD_ShowString(190U, 135U, (const uint8_t *)"A", GRAY, BLACK, 12U, 0U);
  LCD_ShowString(190U, 153U, (const uint8_t *)"X", GRAY, BLACK, 12U, 0U);
  LCD_ShowString(190U, 171U, (const uint8_t *)"Y", GRAY, BLACK, 12U, 0U);
  LCD_ShowString(190U, 189U, (const uint8_t *)"YAW", GRAY, BLACK, 12U, 0U);
  LCD_ShowString(190U, 207U, (const uint8_t *)"C", GRAY, BLACK, 12U, 0U);
  LCD_ShowString(190U, 225U, (const uint8_t *)"ERR", GRAY, BLACK, 12U, 0U);
  LCD_WR_REG(0x29U);
}

static void T4_DrawDiagnostics(uint32_t now_ms)
{
  const LD06_Data_t *scan = LD06_GetSnapshot();
#if CHASSIS_ESCAPE_TEST_ENABLE
  const ChassisEscapeRequest_t *escape_request =
      ChassisEscapeTest_GetRequest();
#endif
  const char *stage_text = T4_StageText();
  char text[20];
  uint32_t age = (scan->scan_sequence == 0U) ? 9999U :
                 (uint32_t)(now_ms - scan->scan_complete_ms);
  uint8_t global = ((sPose.localization_stage == T4_STAGE_PROVISIONAL) ||
                    (sPose.localization_stage == T4_STAGE_FIXED) ||
                    (sPose.localization_stage == T4_STAGE_DEGRADED) ||
                     (sPose.localization_stage == T4_STAGE_LOST)) ? 1U : 0U;
  LCD_Fill(212U, 43U, 279U, 239U, BLACK);
#if CHASSIS_ESCAPE_TEST_ENABLE
  if (escape_request->active != 0U)
    stage_text = T4_EscapeStageText();
#endif
  LCD_ShowString(212U, 45U, (const uint8_t *)stage_text,
                 (sPose.localization_stage == T4_STAGE_FIXED) ? GREEN :
                 YELLOW, BLACK, 12U, 0U);
  LCD_ShowString(212U, 63U,
                 (const uint8_t *)((global != 0U) ? "GLOBAL" : "LOCAL"),
                 (global != 0U) ? GREEN : GRAY, BLACK, 12U, 0U);
  (void)snprintf(text, sizeof(text), "%u", sSelectedStart);
  LCD_ShowString(212U, 81U, (const uint8_t *)text, WHITE, BLACK, 12U, 0U);
  (void)snprintf(text, sizeof(text), "%u/%lu", sPointCount,
                 (unsigned long)age);
  LCD_ShowString(212U, 99U, (const uint8_t *)text, WHITE, BLACK, 12U, 0U);
  (void)snprintf(text, sizeof(text), "%u/%u", sSegmentCount, sCornerCount);
  LCD_ShowString(212U, 117U, (const uint8_t *)text, WHITE, BLACK, 12U, 0U);
  (void)snprintf(text, sizeof(text), "%ld", (long)sBestCornerAngle);
  LCD_ShowString(212U, 135U, (const uint8_t *)text, WHITE, BLACK, 12U, 0U);
  (void)snprintf(text, sizeof(text), "%ld", (long)sPose.x_mm);
  LCD_ShowString(212U, 153U, (const uint8_t *)text, WHITE, BLACK, 12U, 0U);
  (void)snprintf(text, sizeof(text), "%ld", (long)sPose.y_mm);
  LCD_ShowString(212U, 171U, (const uint8_t *)text, WHITE, BLACK, 12U, 0U);
  (void)snprintf(text, sizeof(text), "%ld", (long)sPose.yaw_deg);
  LCD_ShowString(212U, 189U, (const uint8_t *)text, WHITE, BLACK, 12U, 0U);
  (void)snprintf(text, sizeof(text), "%ld/%ld",
                 (long)sPose.corner_body_x_mm,
                 (long)sPose.corner_body_y_mm);
  LCD_ShowString(212U, 207U, (const uint8_t *)text, WHITE, BLACK, 12U, 0U);
  LCD_ShowString(212U, 225U, (const uint8_t *)T4_FailureText(),
                 (sPose.last_failure_reason == T4_FAIL_NONE) ? GREEN : RED,
                 BLACK, 12U, 0U);
}

static void T4_DrawStartSelection(void)
{
  char text[12];
  T4_ClearMap();
#if CHASSIS_ESCAPE_TEST_ENABLE
  LCD_ShowString(38U, 83U, (const uint8_t *)"ESCAPE TEST",
#else
  LCD_ShowString(38U, 83U, (const uint8_t *)"SELECT START",
#endif
                 CYAN, BLACK, 16U, 0U);
  (void)snprintf(text, sizeof(text), "S%u", sSelectedStart);
  LCD_ShowString(82U, 116U, (const uint8_t *)text,
                 WHITE, BLACK, 24U, 0U);
  LCD_ShowString(24U, 158U, (const uint8_t *)"UP/DN SELECT",
                 GRAY, BLACK, 16U, 0U);
  LCD_ShowString(24U, 181U, (const uint8_t *)"CENTER START",
                 YELLOW, BLACK, 16U, 0U);
  T4_DrawBoundary();
}

static void T4_UpdateLed(void)
{
  if (sPose.status == POSE_FIX_HIGH)
    WS2812_Ctrl(0U, WS2812_BRIGHTNESS_MAX, 0U);
  else if ((sPose.localization_stage == T4_STAGE_LOST) ||
           (sPose.last_failure_reason == T4_FAIL_NO_SCAN))
    WS2812_Ctrl(WS2812_BRIGHTNESS_MAX, 0U, 0U);
  else
    WS2812_Ctrl(WS2812_BRIGHTNESS_MAX, WS2812_BRIGHTNESS_MAX, 0U);
}

static void T4_UpdateMotion(uint32_t now_ms)
{
  ChassisMotionPose_t pose;

  pose.x_mm = sPose.x_mm;
  pose.y_mm = sPose.y_mm;
  pose.yaw_deg = sPose.yaw_deg;
  pose.fixed = ((sPose.valid != 0U) &&
                (sPose.status == POSE_FIX_HIGH) &&
                (sPose.localization_stage == T4_STAGE_FIXED)) ? 1U : 0U;
  pose.lidar_fresh = LD06_IsFresh(now_ms, T4_LIDAR_FRESH_MS);
  ChassisMotion_Update(now_ms, &pose);
}

static void T4_ResetLocalization(uint32_t now_ms)
{
  uint8_t selected = sSelectedStart;
  memset(&sPose, 0, sizeof(sPose));
  memset(&sProvisional, 0, sizeof(sProvisional));
  memset(&sObstacle, 0, sizeof(sObstacle));
  sSelectedStart = selected;
  sPose.selected_start = selected;
  sPose.start_candidate = selected;
  sPose.status = POSE_INIT_AMBIGUOUS;
  sPose.localization_stage = T4_STAGE_WAIT_SCAN;
  sPose.last_failure_reason = T4_FAIL_NO_SCAN;
  sProvisionalFrames = 0U;
  sLastScanSequence = 0U;
  sLastMatchTick = now_ms;
  sLastFixTick = now_ms;
  sPointCount = 0U;
  sSegmentCount = 0U;
  sCornerCount = 0U;
  sBestCornerX = 0.0f;
  sBestCornerY = 0.0f;
}

static void Task4_Enter(uint32_t now_ms)
{
  memset(&sPose, 0, sizeof(sPose));
  memset(&sObstacle, 0, sizeof(sObstacle));
  sSelectedStart = 1U;
  sPose.selected_start = sSelectedStart;
  sPose.localization_stage = T4_STAGE_SELECT_START;
  sPose.status = POSE_INIT_AMBIGUOUS;
  sDisplayTick = now_ms;
  sPointCount = 0U;
  sSegmentCount = 0U;
  sCornerCount = 0U;
  sBestCornerX = 0.0f;
  sBestCornerY = 0.0f;
  ChassisMotion_Reset();
#if CHASSIS_ESCAPE_TEST_ENABLE
  ChassisEscapeTest_Abort();
#endif
  SongjiaMotor_StartEncoderPolarityConfig(1U, now_ms);
  T4_DrawFrame();
  T4_DrawStartSelection();
  T4_DrawDiagnostics(now_ms);
  T4_UpdateLed();
}

static void Task4_Tick(uint32_t now_ms, InputEvent_t event)
{
  const LD06_Data_t *scan = LD06_GetSnapshot();
  if (sPose.localization_stage == T4_STAGE_SELECT_START)
  {
    if (event == INPUT_EVENT_UP)
      sSelectedStart = (sSelectedStart >= 4U) ? 1U :
                       (uint8_t)(sSelectedStart + 1U);
    else if (event == INPUT_EVENT_DOWN)
      sSelectedStart = (sSelectedStart <= 1U) ? 4U :
                       (uint8_t)(sSelectedStart - 1U);
    else if (event == INPUT_EVENT_CENTER)
    {
      T4_ResetLocalization(now_ms);
#if CHASSIS_ESCAPE_TEST_ENABLE
      ChassisMotion_Reset();
      ChassisEscapeTest_Start(sSelectedStart, now_ms);
#else
      ChassisMotion_Start(sSelectedStart, now_ms);
#endif
      T4_ClearMap();
      T4_DrawBoundary();
      return;
    }
    if ((event == INPUT_EVENT_UP) || (event == INPUT_EVENT_DOWN))
    {
      sPose.selected_start = sSelectedStart;
      T4_DrawStartSelection();
      T4_DrawDiagnostics(now_ms);
    }
    return;
  }

  if (event == INPUT_EVENT_CENTER)
  {
    T4_ResetLocalization(now_ms);
#if CHASSIS_ESCAPE_TEST_ENABLE
    ChassisMotion_Reset();
    ChassisEscapeTest_Start(sSelectedStart, now_ms);
#else
    ChassisMotion_Start(sSelectedStart, now_ms);
#endif
    return;
  }
  if ((LD06_IsFresh(now_ms, T4_LIDAR_FRESH_MS) != 0U) &&
      (scan->scan_sequence != sLastScanSequence) &&
      ((uint32_t)(now_ms - sLastMatchTick) >= T4_MATCH_PERIOD_MS))
  {
    sLastScanSequence = scan->scan_sequence;
    sLastMatchTick = now_ms;
    T4_PreprocessScan(scan);
    if ((sPose.localization_stage == T4_STAGE_WAIT_SCAN) ||
        (sPose.localization_stage == T4_STAGE_FIND_LINES) ||
        (sPose.localization_stage == T4_STAGE_FIND_CORNER) ||
        (sPose.localization_stage == T4_STAGE_PROVISIONAL))
      T4_ProcessInitialization(scan, now_ms);
    else
      T4_ProcessTracking(scan, now_ms);
  }
  if (LD06_IsFresh(now_ms, T4_LIDAR_FRESH_MS) == 0U)
  {
    if ((sPose.localization_stage == T4_STAGE_WAIT_SCAN) ||
        (sPose.localization_stage == T4_STAGE_FIND_LINES) ||
        (sPose.localization_stage == T4_STAGE_FIND_CORNER) ||
        (sPose.localization_stage == T4_STAGE_PROVISIONAL))
    {
      sProvisionalFrames = 0U;
      sPose.localization_stage = T4_STAGE_WAIT_SCAN;
      T4_SetFailure(T4_FAIL_NO_SCAN);
    }
    else
    {
      T4_SetFailure(T4_FAIL_NO_SCAN);
      T4_Degrade(now_ms);
    }
  }
  else if (((sPose.localization_stage == T4_STAGE_FIXED) ||
            (sPose.localization_stage == T4_STAGE_DEGRADED)) &&
           ((uint32_t)(now_ms - sLastFixTick) >= T4_DEGRADED_MS))
    T4_Degrade(now_ms);

  T4_UpdateMotion(now_ms);

  if ((uint32_t)(now_ms - sDisplayTick) >= T4_DISPLAY_PERIOD_MS)
  {
    sDisplayTick = now_ms;
    if ((sPose.localization_stage == T4_STAGE_PROVISIONAL) ||
        (sPose.localization_stage == T4_STAGE_FIXED) ||
        (sPose.localization_stage == T4_STAGE_DEGRADED) ||
        (sPose.localization_stage == T4_STAGE_LOST))
      T4_DrawGlobalScan();
    else
      T4_DrawLocalScan();
    T4_DrawDiagnostics(now_ms);
    T4_UpdateLed();
  }
}

static void Task4_Exit(void)
{
#if CHASSIS_ESCAPE_TEST_ENABLE
  ChassisEscapeTest_Abort();
#endif
  ChassisMotion_Abort();
  SongjiaMotor_CancelEncoderPolarityConfig();
}

const Pose_Packet_t *Task4_GetPose(void)
{
  return &sPose;
}

void Task4_FillChassisSafetyInput(ChassisSafetyInput_t *input,
                                  uint32_t now_ms,
                                  uint8_t require_feedback,
                                  uint32_t feedback_timeout_ms,
                                  float obstacle_stop_distance_mm)
{
  if (input == 0) return;
  input->localization_usable =
      ((sPose.valid != 0U) &&
       (sPose.status == POSE_FIX_HIGH) &&
       (sPose.localization_stage == T4_STAGE_FIXED)) ? 1U : 0U;
  input->lidar_fresh = LD06_IsFresh(now_ms, T4_LIDAR_FRESH_MS);
  input->require_feedback = require_feedback;
  input->feedback_timeout_ms = feedback_timeout_ms;
  input->obstacle_distance_mm =
      (sPose.obstacle_confirmed != 0U) ? sPose.enemy_distance_mm : 0.0f;
  input->stop_distance_mm = obstacle_stop_distance_mm;
}

#if CHASSIS_ESCAPE_TEST_ENABLE
void Task4_FillChassisEscapeInput(ChassisEscapeInput_t *input,
                                  uint32_t now_ms)
{
  const Pose_Packet_t *pose = &sPose;
  const SongjiaMotorFeedback_t *feedback = SongjiaMotor_GetFeedback();
  const BMI088_Attitude_t *imu = AppScheduler_GetAttitude();
  const LD06_Data_t *scan = LD06_GetSnapshot();
  uint16_t bin;

  if (input == 0) return;
  memset(input, 0, sizeof(*input));
  input->selected_start = sSelectedStart;
  input->fixed = ((pose->valid != 0U) &&
                  (pose->status == POSE_FIX_HIGH) &&
                  (pose->localization_stage == T4_STAGE_FIXED)) ? 1U : 0U;
  input->lidar_fresh = LD06_IsFresh(now_ms, T4_LIDAR_FRESH_MS);
  input->encoder_fresh = SongjiaMotor_IsFeedbackFresh(now_ms, 150U);
  input->obstacle_stop = ((pose->obstacle_confirmed != 0U) &&
                          (pose->enemy_distance_mm <= 250.0f)) ? 1U : 0U;
  for (bin = 0U; bin < LD06_ANGLE_BINS; bin++)
  {
    int16_t front_offset = T4_SignedAngleOffset(bin, T4_VEHICLE_FRONT_BIN);
    if ((front_offset >= -20) && (front_offset <= 20) &&
        (scan->distance_mm[bin] >= T4_MIN_DISTANCE_MM) &&
        (scan->distance_mm[bin] <= 250U) &&
        (scan->confidence[bin] >= T4_MIN_CONFIDENCE))
    {
      input->obstacle_stop = 1U;
      break;
    }
  }
  input->yaw_deg = (imu != 0) ? imu->yaw_deg : 0.0f;
  input->encoder_sequence = feedback->sequence;
  input->left_encoder_delta = feedback->encoder_20ms[0];
  input->right_encoder_delta = feedback->encoder_20ms[1];
}
#endif

const AppTask_t Task4_Definition =
{
  Task4_Enter,
  Task4_Tick,
  Task4_Exit
};
