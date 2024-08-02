#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>
#include <chrono>
#include <fstream>
#include <netinet/in.h>
#include <unistd.h>
#include "RHexAPI.h"

bool isRobotConnected() {
    std::ifstream conn_file("/tmp/rhex_connection_status");
    if (conn_file.is_open()) {
        std::string status;
        std::getline(conn_file, status);
        conn_file.close();
        return (status == "connected");
    }
    return false;
}

void handleClient(int client_socket, RHexAPI& rhex) {
    char buffer[1024] = {0};
    read(client_socket, buffer, 1024);
    WalkCommand_t wc, cwc;
    wc.type = WalkType::EFFICIENT_WALK;
    wc.incline = 0;
    wc.turning_speed = 0.0;

    std::string command(buffer);
    if (command == "CONNECT") {
        try {
            std::string intf = "lo";
            std::string hostname = "127.0.0.1";
            int port = 5000;

            rhex.Connect(hostname.c_str(), port, 5001);
            std::cout << "Connected successfully.\n";
            std::string response = "Robot connected successfully.";
            send(client_socket, response.c_str(), response.length(), 0);

            // Update connection status file
            std::ofstream conn_file("/tmp/rhex_connection_status");
            if (conn_file.is_open()) {
                conn_file << "connected";
                conn_file.close();
            } else {
                std::cerr << "Unable to update connection status file\n";
            }
        } catch (const std::runtime_error& e) {
            std::string error = "Robot connection error: " + std::string(e.what());
            send(client_socket, error.c_str(), error.length(), 0);
        }
    } else if (command == "STAND") {
        try {
            rhex.SetMode(RHexMode::STAND);
            while (rhex.GetMode() != RHexMode::STAND) {
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
            }
            std::string response = "Robot is now in STAND mode.";
            send(client_socket, response.c_str(), response.length(), 0);
        } catch (const std::runtime_error& e) {
            std::string error = "Exception caught: " + std::string(e.what());
            send(client_socket, error.c_str(), error.length(), 0);
        }
    } else if (command == "SIT") {
        try {
            rhex.SetMode(RHexMode::SIT);
            while (rhex.GetMode() != RHexMode::SIT) {
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
            }
            std::string response = "Robot is now in SIT mode.";
            send(client_socket, response.c_str(), response.length(), 0);
        } catch (const std::runtime_error& e) {
            std::string error = "Exception caught: " + std::string(e.what());
            send(client_socket, error.c_str(), error.length(), 0);
        }
    } else if (command == "DISCONNECT") {
        std::cout << "Disconnecting from the robot...\n";
        rhex.SetMode(RHexMode::SIT);
        while (rhex.GetMode() != RHexMode::SIT) {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
        rhex.Disconnect();
        std::string response = "Robot disconnected.";
        send(client_socket, response.c_str(), response.length(), 0);

        // Update connection status file
        std::ofstream conn_file("/tmp/rhex_connection_status");
        if (conn_file.is_open()) {
            conn_file << "disconnected";
            conn_file.close();
        } else {
            std::cerr << "Unable to update connection status file\n";
        }
    } else if (command == "WALK") {
        rhex.SetMode(RHexMode::WALK);
        while (rhex.GetMode() != RHexMode::WALK) usleep(100000);
        std::string response = "Robot is now in WALK mode.";
        send(client_socket, response.c_str(), response.length(), 0);
    } else if (command == "FORWARD") {
        wc.direction = MovementDirection::FORWARD;     
        wc.turning_speed = 0.5;
        rhex.SetWalkCommand(wc);
    } else if (command == "BACKWARD") {
        wc.direction = MovementDirection::BACKWARD;
        wc.turning_speed = 0.0;
        rhex.SetWalkCommand(wc);
    } else if (command == "STOP") {
        wc.direction = MovementDirection::STOP;
        wc.turning_speed = 0.0;
        rhex.SetWalkCommand(wc);
    } else if (command == "RIGHT") {
        wc.direction = MovementDirection::STOP;
        wc.turning_speed = 0.15;
        rhex.SetWalkCommand(wc);
    } else if (command == "LEFT") {
        wc.direction = MovementDirection::STOP;
        wc.turning_speed = -0.15;
        rhex.SetWalkCommand(wc);
    }else if (command == "CALIBRATE") {
        try {
            while (true) {
                rhex.SetMode(RHexMode::SIT);
                while (rhex.GetMode() != RHexMode::SIT) {
                    usleep(100000);
                }
                sleep(1);
                
                rhex.SetMode(RHexMode::CALIBRATION);
                while (rhex.GetMode() != RHexMode::CALIBRATION) {
                    usleep(100000);
                }
                CalibrationCommand_t cc;
                cc.type = CalibrationType::GROUND;
                rhex.SetCalibrationCommand(cc);
                sleep(1);
                bool calibrated = false;
                int retries = 0;
                
                while (!calibrated) {
                    retries++;
                    CalibrationState_t cs = rhex.GetCalibrationState();
                    calibrated = true;
                    for (int i = 0; i < 6; i++) {
                        if (cs.leg_status[i] != CalibrationStatus::CALIBRATED) {
                            calibrated = false;
                        }
                    }
                    if (retries > 300) {
                        break;
                    }
                    usleep(10000);
                }

                if (calibrated) {
                    std::string response = "Calibration successful.";
                    send(client_socket, response.c_str(), response.length(), 0);
                    break;
                }
            }
        } catch (const std::runtime_error& e) {
            std::string error = "Exception caught during calibration: " + std::string(e.what());
            send(client_socket, error.c_str(), error.length(), 0);
        }
    }  else {
        std::string response = "Invalid command.";
        send(client_socket, response.c_str(), response.length(), 0);
    }
    
    close(client_socket);
}

int main(int argc, char* argv[]) {


    MultimediaParameters_t mm;
    mm.front_camera_receiver_port = 5500;
    mm.rear_camera_receiver_port = 5501;
    mm.robot_microphone_receiver_port = 5502;

    std::string intf = "lo";

    RHexAPI rhex(intf.c_str(), mm);

    int server_fd, new_socket;
    struct sockaddr_in address;
    int opt = 1;
    int addrlen = sizeof(address);

    if ((server_fd = socket(AF_INET, SOCK_STREAM, 0)) == 0) {
        perror("socket failed");
        return -1;
    }

    if (setsockopt(server_fd, SOL_SOCKET, SO_REUSEADDR | SO_REUSEPORT, &opt, sizeof(opt))) {
        perror("setsockopt");
        close(server_fd);
        return -1;
    }

    address.sin_family = AF_INET;
    address.sin_addr.s_addr = INADDR_ANY;
    address.sin_port = htons(8080);

    if (bind(server_fd, (struct sockaddr*)&address, sizeof(address)) < 0) {
        perror("bind failed");
        close(server_fd);
        return -1;
    }

    if (listen(server_fd, 3) < 0) {
        perror("listen");
        close(server_fd);
        return -1;
    }

    while (true) {
        if ((new_socket = accept(server_fd, (struct sockaddr*)&address, (socklen_t*)&addrlen)) < 0) {
            perror("accept");
            close(server_fd);
            return -1;
        }
        std::thread(handleClient, new_socket, std::ref(rhex)).detach();
    }

    rhex.Disconnect();
    return 0;
}
