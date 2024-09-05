#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <cstring>
#include <iostream>
#include <map>
#include <mutex>
#include <thread>
#include <vector>

std::mutex mtx;
std::map<std::string, std::vector<int>> subscribers;

void handlePublisher(int publisherSocket) {
    char buffer[1024];
    while (true) {
        int n = recv(publisherSocket, buffer, sizeof(buffer) - 1, 0);
        if (n <= 0) {
            break;  // Connection closed or error
        }

        buffer[n] = '\0';
        std::string command(buffer);

        if (command.find("PUBLISH") == 0) {
            size_t firstSpace = command.find(' ', 8);
            std::string topic = command.substr(8, firstSpace - 8);
            std::string data = command.substr(firstSpace + 1);

            // Broadcast message to all subscribers of the topic
            std::lock_guard<std::mutex> lock(mtx);
            if (subscribers.find(topic) != subscribers.end()) {
                for (int subscriber : subscribers[topic]) {
                    send(subscriber, data.c_str(), data.size(), 0);
                }
            }
        }
    }
    close(publisherSocket);
}

void handleSubscriber(int subscriberSocket) {
    char buffer[1024];
    int n = recv(subscriberSocket, buffer, sizeof(buffer) - 1, 0);
    if (n <= 0) {
        return;  // Connection closed or error
    }

    buffer[n] = '\0';
    std::string command(buffer);

    if (command.find("SUBSCRIBE") == 0) {
        std::string topic = command.substr(10);

        // Add subscriber to the list for this topic
        std::lock_guard<std::mutex> lock(mtx);
        subscribers[topic].push_back(subscriberSocket);
    }

    // Keep connection open to continue receiving messages
    while (true) {
        int n = recv(subscriberSocket, buffer, sizeof(buffer) - 1, 0);
        if (n <= 0) {
            break;  // Connection closed or error
        }
    }

    close(subscriberSocket);
}

int main() {
    int serverSocket = socket(AF_INET, SOCK_STREAM, 0);
    struct sockaddr_in serverAddr;

    serverAddr.sin_family = AF_INET;
    serverAddr.sin_port = htons(7891);
    serverAddr.sin_addr.s_addr = INADDR_ANY;

    bind(serverSocket, (struct sockaddr *)&serverAddr, sizeof(serverAddr));
    listen(serverSocket, 5);

    while (true) {
        std::cout << "waiting for connection" << std::endl;
        struct sockaddr_in clientAddr;
        socklen_t addr_size = sizeof(clientAddr);
        // int clientSocket = accept(serverSocket, (struct sockaddr *)&clientAddr, &addr_size);
        int clientSocket = accept(serverSocket, nullptr, nullptr);
        std::cout << "connected" << std::endl;

        // Determine if this is a publisher or subscriber based on initial command
        char buffer[1024];
        int n = recv(clientSocket, buffer, sizeof(buffer) - 1, 0);
        if (n <= 0) {
            close(clientSocket);
            continue;
        }

        buffer[n] = '\0';
        std::string command(buffer);
        std::cout << command << std::endl;

        if (command.find("PUBLISH") == 0) {
            std::thread(handlePublisher, clientSocket).detach();
        } else if (command.find("SUBSCRIBE") == 0) {
            std::thread(handleSubscriber, clientSocket).detach();
        } else {
            close(clientSocket);  // Unrecognized command
        }
    }
    std::cout << "closing" << std::endl;

    close(serverSocket);
    return 0;
}
