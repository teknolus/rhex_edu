#ifndef RHEXAPI_H_
#define RHEXAPI_H_

#include <memory>
#include <string>
#include <gst/gst.h>

const unsigned int RHEXAPI_MSGLOG_MSG_SIZE=128;
struct MsgLogData_t {
  double  time;	// message time stamp (seconds)
  unsigned  seq;				    // message sequence number
  char      msg[RHEXAPI_MSGLOG_MSG_SIZE];	  // message body
  unsigned char prio;				// message priority
};
  
/** Structure to hold IMU data, including 3D acceleration and angular velocity */
struct IMUData_t {
  struct linear_acceleration_t {
    double x;
    double y;
    double z;
  } acc;
  struct angular_velocity_t {
    double roll;
    double pitch;
    double yaw;
  } gyro;
  long long timestamp_ms;
  bool upside_down;
};

/** Structure to hold GPS data. */
struct GPSData_t {
  bool   locked = false;
  double latitude = 39.9205;
  double longitude = 32.8183;
  double altitude = 900;
  double hdop = 100.0; // meters
  double vdop = 100.0; // meters
  short  tracking = 0; // Satellites being tracked
};

/** Structure to hold leg positions */
struct LegData_t {
  double positions[6];
  long long timestamp_ms;
};

/** Structure to miscellaneous information on the robot state. */
struct RobotMiscInfo_t {
  double leg_motor_temperatures[6];
  double extra_sensors[6];
  double voltage;
  double current;
  bool flipped;
  bool motors_rdy;
  bool ins_rdy;
  bool gps_rdy;
  bool pose_rdy;
};

/* Commands to SitMode */
struct StandCommand_t {
  float manual_offset;
  float pitch_angle;
  bool do_sit;
};

/* Type of walking behavior. */
enum class WalkType { 
  SILENT_WALK, 
    EFFICIENT_WALK, 
    FAST_WALK, 
    WALK_4, 
    WALK_5, 
    WALK_6 };

/* Movement direction for walking. */
enum class MovementDirection { FORWARD, STOP, BACKWARD };

/* Commands for WalkMode */
struct WalkCommand_t {
  WalkType type;
  MovementDirection direction;
  float turning_speed = 0;
  float incline = 0;
  float manualOffset = 0;
};

/* Type of stairs for Stair Mode */
enum class StairsType { SHALLOW, STEEP };

/* Action to start for Stair Mode. */
enum class StairsAction { ASCEND, STOP, DESCEND };

/* Commands for Stair Mode */
struct StairsCommand_t {
  StairsAction action;
  StairsType type;
  bool exit_trigger;
};

/** Command for wheel mode */
struct WheeledCommand_t {
  float forward_speed;
  float turning_speed;
};

/** Command for Hill Mode */
enum class HillAction { CLIMBDOWN, STOP, CLIMBUP };

struct HillCommand_t {
  HillAction action;
  float speed;
  float turn_speed;
  float incline;
};

/** Type of calibration to be performed */
enum class CalibrationType { GROUND, SWITCH, MANUAL };

/** Command for calibration mode */
struct CalibrationCommand_t {
  CalibrationType type;
};

/** Current status for calibration */
enum class CalibrationStatus { NOT_CALIBRATED, CALIBRATING, CALIBRATED, UNKNOWN };

/** Current compound state for calibration */
struct CalibrationState_t {
  CalibrationStatus leg_status[6];
};

/** Command for patrol mode */
struct PatrolCommand_t {
  bool start;
  bool homing;
  bool abort;
  bool clear_points;
};

/** Current status for Patrol Mode */
enum class PatrolStatus { IDLE, SETUP, MOVING, WAITGPS, DONE, ABORTED };

/** Overall state for Patrol mode */
struct PatrolState_t {
  PatrolStatus status;
  GPSData_t target_location;
  int num_points;
  int cur_point;
  bool cmdack;
};

/** Command for homing mode */
struct HomingCommand_t {
  bool start;
  bool abort;
  bool clear_points;
};

/** Current status for homing mode */
enum class HomingStatus { IDLE, SETUP, RUNNING, DONE, ABORTED };

/** Overall state for homing mode */
struct HomingState_t {
  HomingStatus status;
  GPSData_t target_location;
};

/** Command for obstacle mode */
enum class ObstacleCommand { PUSH_FRONT, PUSH_MIDDLE, JUMP };

/** Command for StepUp mode */
enum class StepUpCommand { STEP, EXIT };

/** Command for flip mode */
struct FlipCommand_t {
  bool start;
  bool abort;
};

/** Enumerated type for available modes on RHex */
enum class RHexMode {
  START,
  SIT,
  STAND,
  WALK,
  STAIRS,
  WHEELED,
  HILL,
  CALIBRATION,
  PATROL,
  HOMING,
  OBSTACLE,
  STEP_UP,
  FLIP
};

/** Terrain type selection */
enum class TerrainType { CONCRETE, GRASS, MUD, GRAVEL };

/** Parameters for multimedia connections */
struct MultimediaParameters_t {
  unsigned short front_camera_receiver_port;
  unsigned short rear_camera_receiver_port;
  unsigned short robot_microphone_receiver_port;
};

struct SwapRequest_t {
  bool request;
  std::string username;
  long long timestamp_ms;
};

/**
/brief API interface class for TSK-RHex control

This class follows a private implementation pattern to minimize
header-induced recompilation when private implementation details
change.
*/
class RHexAPI {
public:
  RHexAPI(std::string datalink_interface_name,
          MultimediaParameters_t parameters);
  ~RHexAPI();
  RHexAPI(RHexAPI &&rhs);
  RHexAPI &operator=(RHexAPI &&rhs);

  int Connect(std::string robot_IP, unsigned short robot_port,
              unsigned short local_port=3000);
  int Connect(std::string robot_IP, unsigned short robot_port,
              std::string username, std::string password,
              std::string ca_cert_path);
  void Disconnect();
  bool IsConnected();
  void SendPing();
  void Shutdown();

  SwapRequest_t CheckConnectionSwapRequest();  // placeholder
  void SwapConnection(bool allow);   // placeholder

  void Suspend();
  void KillMotors();

  void SetMode(RHexMode mode);
  RHexMode GetMode();

  void SetStandCommand(StandCommand_t stand_cmd);
  StandCommand_t GetCurrentStandCommand();

  void SetWalkCommand(WalkCommand_t walk_cmd);
  WalkCommand_t GetCurrentWalkCommand();

  void SetStairsCommand(StairsCommand_t stairs_cmd);
  StairsCommand_t GetCurrentStairsCommand();

  void SetCalibrationCommand(CalibrationCommand_t calib_cmd);
  CalibrationState_t GetCalibrationState();

  void SetHomingCommand(HomingCommand_t homing_cmd);
  void AddHomingLocation(GPSData_t homing_location);
  HomingState_t GetHomingState();

  void SetPatrolCommand(PatrolCommand_t patrol_cmd);
  void AddPatrolLocation(GPSData_t patrol_location);
  PatrolState_t GetPatrolState();

  void SetObstacleCommand(ObstacleCommand obstacle_cmd);
  void SetStepUpCommand(StepUpCommand step_up_cmd);

  void SetWheeledCommand(WheeledCommand_t wheeled_cmd);
  WheeledCommand_t GetCurrentWheeledCommand();

  void SetHillCommand(HillCommand_t hill_cmd);
  HillCommand_t GetCurrentHillCommand();

  void SetFlipCommand(FlipCommand_t flip_cmd);
  FlipCommand_t GetCurrentFlipCommand();

  void SetFrontLightBrightness(int brightness);
  int GetFrontLightBrightness();

  void SetRearLightBrightness(int brightness);
  int GetRearLightBrightness();

  void SelfDestruct();

  IMUData_t GetIMUData();
  GPSData_t GetGPSData();
  LegData_t GetLegPositions();
  RobotMiscInfo_t GetMiscInformation();
  double GetBatterySoC(void);

  void SetHomeLocation(GPSData_t home_location); // placeholder, deprecated
  void GoHome();                                 // placeholder, deprecated

  void SetInvertedFlag(bool inverted); // placeholder
  bool GetInvertedFlag();              // placeholder

  void SetTerrainType(TerrainType type); // placeholder
  TerrainType GetTerrainType();          // placeholder

  void PlayFrontCameraStream();
  void PlayRearCameraStream();
  void PlayRobotMicrophoneStream();
  void PlayOCUMicrophoneStream();

  void PauseFrontCameraStream();
  void PauseRearCameraStream();
  void PauseRobotMicrophoneStream();
  void PauseOCUMicrophoneStream();

  void StartRecordingFrontCameraStream(std::string filepath);
  void StartRecordingRearCameraStream(std::string filepath);
  void StartRecordingMicrophoneStream(std::string filepath);

  void StopRecordingFrontCameraStream();
  void StopRecordingRearCameraStream();
  void StopRecordingMicrophoneStream();

  GstElement *GetFrontCameraStreamQmlGlSink();
  GstElement *GetRearCameraStreamQmlGlSink();

  void SetFrontCameraStreamWindow(unsigned long wid);
  void SetRearCameraStreamWindow(unsigned long wid);

  bool IsFrontCameraStreamInitialized();
  bool IsRearCameraStreamInitialized();
  bool IsRobotMicrophoneStreamInitialized();
  bool IsOCUMicrophoneStreamInitialized();

  // New additions
  void SetQtQmlOutput( bool enable );
  
  void GetCalibStatus(bool *calib);
  void SetFrontCameraStreamOverlay( bool enable );
  void SetRearCameraStreamOverlay( bool enable );

  int  GetNumSpeakers(void);
  int  GetNumMicrophones(void);
  int  GetNumCameras(void);
  std::string GetCameraName(int index);
  void PlayCameraStream(int index);
  void UpdateCameraState(int index);
  void PauseCameraStream(int index);
  void StartRecordingCameraStream(int index, std::string filepath);
  void StopRecordingCameraStream(int index);
  void SetCameraStreamWindow(int index, unsigned long window_id);
  void SetCameraStreamOverlay( int index, bool enable );
  void SetCameraTilt( int index, double tilt ); // Full tilt range is [-1,1]
  bool GetCameraTilt( int index );
  GstPad *GetCameraStreamPad( int index );

  int  GetNumLEDs(void);
  std::string GetLEDName( int index );
  int  GetLEDBrightness( int index );
  void SetLEDBrightness( int index, int b );

  bool GetMsgLog( MsgLogData_t &msg );
 
private:
  class Impl;
  std::unique_ptr<Impl> pimpl;
};

#endif // RHEXAPI_H_
