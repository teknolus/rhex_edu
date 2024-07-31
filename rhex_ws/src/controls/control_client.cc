#include <iostream>
#include <string>
#include <algorithm>
#include <cctype>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <unistd.h>

void sendCommand(const std::string& command) {
    std::string server_address = "127.0.0.1";
    int port = 8080;

    int sock = 0;
    struct sockaddr_in serv_addr;

    if ((sock = socket(AF_INET, SOCK_STREAM, 0)) < 0) {
        std::cerr << "Socket creation error\n";
        return;
    }

    serv_addr.sin_family = AF_INET;
    serv_addr.sin_port = htons(port);

    if (inet_pton(AF_INET, server_address.c_str(), &serv_addr.sin_addr) <= 0) {
        std::cerr << "Invalid address/ Address not supported\n";
        close(sock);
        return;
    }

    if (connect(sock, (struct sockaddr*)&serv_addr, sizeof(serv_addr)) < 0) {
        std::cerr << "Connection Failed\n";
        close(sock);
        return;
    }

    send(sock, command.c_str(), command.length(), 0);
    char buffer[1024] = {0};
    read(sock, buffer, 1024);
    std::cout << "Response from server: " << buffer << std::endl;

    close(sock);
}

int main(int argc, char* argv[]) {
    if (argc != 2) {
        std::cerr << "Usage: " << argv[0] << " <command>\n";
        std::cerr << "Valid commands: connect, disconnect, sit, stand\n";
        return -1;
    }

    std::string command(argv[1]);
    std::transform(command.begin(), command.end(), command.begin(), ::toupper);

    sendCommand(command);
    return 0;
}
