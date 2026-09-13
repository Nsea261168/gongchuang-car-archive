#ifndef TASK4_H
#define TASK4_H
#include <stdint.h>
#include "task_contract.h"
#include "app_scheduler.h"
#include "../Tests/chassis_escape_test.h"
typedef enum
{
  POSE_LOST = 0,
  POSE_DEGRADED,
  POSE_FIX_LOW,
  POSE_FIX_HIGH,
  POSE_INIT_AMBIGUOUS,
  POSE_PROVISIONAL
} Pose_Status_t;

typedef enum
{
  T4_STAGE_SELECT_START = 0,
  T4_STAGE_WAIT_SCAN,
  T4_STAGE_FIND_LINES,
  T4_STAGE_FIND_CORNER,
  T4_STAGE_PROVISIONAL,
  T4_STAGE_FIXED,
  T4_STAGE_DEGRADED,
  T4_STAGE_LOST
} Task4_LocalizationStage_t;

typedef enum
{
  T4_FAIL_NONE = 0,
  T4_FAIL_NO_SCAN,
  T4_FAIL_TOO_FEW_POINTS,
  T4_FAIL_NO_LINE,
  T4_FAIL_NO_RIGHT_ANGLE,
  T4_FAIL_PARALLEL_LINES,
  T4_FAIL_BAD_INTERSECTION,
  T4_FAIL_YAW_INCONSISTENT,
  T4_FAIL_POSE_OUTSIDE,
  T4_FAIL_NOT_CONSECUTIVE,
  T4_FAIL_TRACKING_TIMEOUT
} Task4_FailureReason_t;
/* Map origin is lower-left; +x is right, +y is up, yaw is clockwise from +y. */
typedef struct {
  float x_mm,y_mm,yaw_deg;
  float enemy_x_mm,enemy_y_mm,enemy_distance_mm;
  float rms_mm;
  float candidate_margin;
  uint32_t timestamp_ms,sequence;
  uint32_t lidar_sequence;
  uint16_t inlier_count;
  uint8_t status,wall_count,dynamic_points,start_candidate,valid;
  uint8_t obstacle_confirmed;
  uint8_t selected_start;
  uint8_t localization_stage;
  uint8_t line_segment_count;
  uint8_t corner_candidate_count;
  uint8_t last_failure_reason;
  float best_corner_angle_deg;
  float line_fit_rms_mm;
  float corner_body_x_mm;
  float corner_body_y_mm;
} Pose_Packet_t;
const Pose_Packet_t *Task4_GetPose(void);
void Task4_FillChassisSafetyInput(ChassisSafetyInput_t *input,
                                  uint32_t now_ms,
                                  uint8_t require_feedback,
                                  uint32_t feedback_timeout_ms,
                                  float obstacle_stop_distance_mm);
#if CHASSIS_ESCAPE_TEST_ENABLE
void Task4_FillChassisEscapeInput(ChassisEscapeInput_t *input,
                                   uint32_t now_ms);
#endif
extern const AppTask_t Task4_Definition;
#endif
