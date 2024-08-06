#include "RHexAPI.h"
#include <pybind11/pybind11.h>
#include <pybind11/stl.h> // include to handle the std::vector

namespace py = pybind11;

PYBIND11_MODULE(pyrhexapi, m) {
    // Bind IMUData_t class
    py::class_<IMUData_t> imu_data(m, "IMUData_t");

    // Bind linear_acceleration_t struct within the scope of IMUData_t
    py::class_<IMUData_t::linear_acceleration_t>(imu_data, "linear_acceleration_t")
        .def(py::init<>())
        .def_readwrite("x", &IMUData_t::linear_acceleration_t::x)
        .def_readwrite("y", &IMUData_t::linear_acceleration_t::y)
        .def_readwrite("z", &IMUData_t::linear_acceleration_t::z);

    // Bind angular_velocity_t struct within the scope of IMUData_t
    py::class_<IMUData_t::angular_velocity_t>(imu_data, "angular_velocity_t")
        .def(py::init<>())
        .def_readwrite("roll", &IMUData_t::angular_velocity_t::roll)
        .def_readwrite("pitch", &IMUData_t::angular_velocity_t::pitch)
        .def_readwrite("yaw", &IMUData_t::angular_velocity_t::yaw);

    // Complete the binding of IMUData_t class
    imu_data
        .def(py::init<>())
        .def_readwrite("acc", &IMUData_t::acc)
        .def_readwrite("gyro", &IMUData_t::gyro)
        .def_readwrite("timestamp_ms", &IMUData_t::timestamp_ms)
        .def_readwrite("upside_down", &IMUData_t::upside_down);
    
    // Bind the GPSData_t class
    py::class_<GPSData_t>(m, "GPSData_t")
        .def(py::init<>())
        .def_readwrite("locked", &GPSData_t::locked)
        .def_readwrite("latitude", &GPSData_t::latitude)
        .def_readwrite("longitude", &GPSData_t::longitude)
        .def_readwrite("altitude", &GPSData_t::altitude)
        .def_readwrite("hdop", &GPSData_t::hdop)
        .def_readwrite("vdop", &GPSData_t::vdop)
        .def_readwrite("tracking", &GPSData_t::tracking);

    // Bind StandCommand_t class
    py::class_<StandCommand_t>(m, "StandCommand_t")
        .def(py::init<>())
        .def_readwrite("manual_offset", &StandCommand_t::manual_offset)
        .def_readwrite("pitch_angle", &StandCommand_t::pitch_angle)
        .def_readwrite("do_sit", &StandCommand_t::do_sit);
    
    // Bind WalkType enum
    py::enum_<WalkType>(m, "WalkType")
        .value("SILENT_WALK", WalkType::SILENT_WALK)
        .value("EFFICIENT_WALK", WalkType::EFFICIENT_WALK)
        .value("FAST_WALK", WalkType::FAST_WALK)
        .value("WALK_4", WalkType::WALK_4)
        .value("WALK_5", WalkType::WALK_5)
        .value("WALK_6", WalkType::WALK_6)
        .export_values();
    
    // Bind MovementDirection enum
    py::enum_<MovementDirection>(m, "MovementDirection")
        .value("FORWARD", MovementDirection::FORWARD)
        .value("STOP", MovementDirection::STOP)
        .value("BACKWARD", MovementDirection::BACKWARD)
        .export_values();
    
    // Bind WalkCommand_t class
    py::class_<WalkCommand_t>(m, "WalkCommand_t")
        .def(py::init<>())
        .def_readwrite("type", &WalkCommand_t::type)
        .def_readwrite("direction", &WalkCommand_t::direction)
        .def_readwrite("turning_speed", &WalkCommand_t::turning_speed)
        .def_readwrite("incline", &WalkCommand_t::incline)
        .def_readwrite("manualOffset", &WalkCommand_t::manualOffset);
    
    // Bind CalibrationType enum
    py::enum_<CalibrationType>(m, "CalibrationType")
        .value("GROUND", CalibrationType::GROUND)
        .value("SWITCH", CalibrationType::SWITCH)
        .value("MANUAL", CalibrationType::MANUAL)
        .export_values();

    // Bind CalibrationCommand_t class
    py::class_<CalibrationCommand_t>(m, "CalibrationCommand_t")
        .def(py::init<>())
        .def_readwrite("type", &CalibrationCommand_t::type);

    // Bind CalibrationStatus enum
    py::enum_<CalibrationStatus>(m, "CalibrationStatus")
        .value("NOT_CALIBRATED", CalibrationStatus::NOT_CALIBRATED)
        .value("CALIBRATING", CalibrationStatus::CALIBRATING)
        .value("CALIBRATED", CalibrationStatus::CALIBRATED)
        .value("UNKNOWN", CalibrationStatus::UNKNOWN)
        .export_values();

    // Bind CalibrationState_t class
    py::class_<CalibrationState_t>(m, "CalibrationState_t")
        .def(py::init<>())
        .def("leg_status", [](CalibrationState_t &self) { return py::make_iterator(self.leg_status, self.leg_status + 6); }, py::keep_alive<0, 1>());

    // Bind RHexMode enum
    py::enum_<RHexMode>(m, "RHexMode")
        .value("START", RHexMode::START)
        .value("SIT", RHexMode::SIT)
        .value("STAND", RHexMode::STAND)
        .value("WALK", RHexMode::WALK)
        .value("STAIRS", RHexMode::STAIRS)
        .value("WHEELED", RHexMode::WHEELED)
        .value("HILL", RHexMode::HILL)
        .value("CALIBRATION", RHexMode::CALIBRATION)
        .value("PATROL", RHexMode::PATROL)
        .value("HOMING", RHexMode::HOMING)
        .value("OBSTACLE", RHexMode::OBSTACLE)
        .value("STEP_UP", RHexMode::STEP_UP)
        .value("FLIP", RHexMode::FLIP)
        .export_values();

    // Bind MultimediaParameters_t class
    py::class_<MultimediaParameters_t>(m, "MultimediaParameters_t")
        .def(py::init<>())
        .def_readwrite("front_camera_receiver_port", &MultimediaParameters_t::front_camera_receiver_port)
        .def_readwrite("rear_camera_receiver_port", &MultimediaParameters_t::rear_camera_receiver_port)
        .def_readwrite("robot_microphone_receiver_port", &MultimediaParameters_t::robot_microphone_receiver_port);
    
    // Bind RHexAPI class
    py::class_<RHexAPI>(m, "RHexAPI")
        .def(py::init<std::string, MultimediaParameters_t>())
        //.def("__del__", [](RHexAPI &self){}) // if you want to add a custom destructor use this, however pybind11 handles deafult destructor automatically
        .def("Connect",
             static_cast<int (RHexAPI::*)(std::string, unsigned short, unsigned short)>(&RHexAPI::Connect),
             py::arg("robot_IP"), py::arg("robot_port"), py::arg("local_port") = 3000)
        .def("Connect",
             static_cast<int (RHexAPI::*)(std::string, unsigned short, std::string, std::string, std::string)>(&RHexAPI::Connect),
             py::arg("robot_IP"), py::arg("robot_port"), py::arg("username"), py::arg("password"), py::arg("ca_cert_path"))
        .def("Disconnect", &RHexAPI::Disconnect)
        .def("Shutdown", &RHexAPI::Shutdown)
        .def("Suspend", &RHexAPI::Suspend)
        .def("KillMotors", &RHexAPI::KillMotors)
        .def("SetMode", &RHexAPI::SetMode)
        .def("GetMode", &RHexAPI::GetMode)
        .def("SetWalkCommand", &RHexAPI::SetWalkCommand)
        .def("GetCurrentWalkCommand", &RHexAPI::GetCurrentWalkCommand)
        .def("SetCalibrationCommand", &RHexAPI::SetCalibrationCommand)
        .def("GetCalibrationState", &RHexAPI::GetCalibrationState)
        .def("GetIMUData", &RHexAPI::GetIMUData)
        .def("GetGPSData", &RHexAPI::GetGPSData)
        .def("GetBatterySoC", &RHexAPI::GetBatterySoC);

}

