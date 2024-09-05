#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>
#include <cstring>
#include <iostream>

int main() {
    // Creating the socket
    int clientSocket = socket(AF_INET, SOCK_STREAM, 0);
    if (clientSocket < 0) {
        std::cerr << "Socket creation failed!" << std::endl;
        return 1;
    }
    std::cout <<clientSocket << std::endl;

    // Specifying the server address
    sockaddr_in serverAddress;
    serverAddress.sin_family = AF_INET;
    serverAddress.sin_port = htons(8080);
    serverAddress.sin_addr.s_addr = INADDR_ANY;

    // Connecting to the server
    if (connect(clientSocket, (struct sockaddr*)&serverAddress, sizeof(serverAddress)) < 0) {
        std::cerr << "Connection failed!" << std::endl;
        close(clientSocket);
        return 1;
    }

    // Continuously send data
    const char* message = "Hello, server!";
    while (true) {
        ssize_t bytesSent = send(clientSocket, message, strlen(message), 0);
        if (bytesSent < 0) {
            std::cerr << "Send failed!" << std::endl;
            break;
        }

        std::cout << "Message sent: " << message << std::endl;

        // Add a delay if you want to control the rate of sending messages
        sleep(1);
    }

    // Closing the socket
    close(clientSocket);

    return 0;
}
