#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <cstring>
#include <iostream>
#include <string>

class ServerSocket {
public:
    ServerSocket(int port) {
        // Creating socket
        serverSocket = socket(AF_INET, SOCK_STREAM, 0);
        if (serverSocket < 0) {
            throw std::runtime_error("Failed to create socket");
        }

        // Specifying the address
        serverAddress.sin_family = AF_INET;
        serverAddress.sin_port = htons(port);
        serverAddress.sin_addr.s_addr = INADDR_ANY;

        // Binding socket
        if (bind(serverSocket, (struct sockaddr*)&serverAddress, sizeof(serverAddress)) < 0) {
            throw std::runtime_error("Failed to bind socket");
        }

        // Listening to the assigned socket
        if (listen(serverSocket, 5) < 0) {
            throw std::runtime_error("Failed to listen on socket");
        }
    }

    ~ServerSocket() {
        // Closing the socket
        close(serverSocket);
    }

    void acceptConnection() {
        clientSocket = accept(serverSocket, nullptr, nullptr);
        if (clientSocket < 0) {
            throw std::runtime_error("Failed to accept connection");
        }
    }

    std::string receiveMessage() {
        char buffer[1024] = {0};
        int bytesReceived = recv(clientSocket, buffer, sizeof(buffer), 0);
        if (bytesReceived < 0) {
            throw std::runtime_error("Failed to receive message");
        }
        return std::string(buffer, bytesReceived);
    }

private:
    int serverSocket;
    int clientSocket;
    sockaddr_in serverAddress;
};

// Usage example
int main() {
    try {
        ServerSocket server(8080);
        std::cout << "Server is listening on port 8080..." << std::endl;

        server.acceptConnection();
        std::cout << "Client connected!" << std::endl;

        while (true) {
            std::string message = server.receiveMessage();
            std::cout << "Message from client: " << message << std::endl;
        }
    } catch (const std::exception& ex) {
        std::cerr << "Error: " << ex.what() << std::endl;
    }

    return 0;
}
