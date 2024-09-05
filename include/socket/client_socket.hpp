#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <cstring>
#include <iostream>
#include <map>
#include <string>
#include <vector>

class ClientSocket {
   private:
    std::string serverAddress;
    int port;
    std::map<std::string, int> subscriptions;  // Maps topics to sockets
   public:
    ClientSocket(const std::string& serverAddress, int port)
        : serverAddress(serverAddress), port(port) {}

    ~ClientSocket() {
        for (auto& entry : subscriptions) {
            close(entry.second);
        }
    }

    void subscribe(const std::string& topic) {
        int sock = socket(AF_INET, SOCK_STREAM, 0);
        if (sock < 0) {
            throw std::runtime_error("Failed to create socket for topic " + topic);
        }

        sockaddr_in serverAddr;
        serverAddr.sin_family = AF_INET;
        serverAddr.sin_port = htons(port);
        if (inet_pton(AF_INET, serverAddress.c_str(), &serverAddr.sin_addr) <= 0) {
            throw std::runtime_error("Invalid address or address not supported for topic " + topic);
        }

        if (connect(sock, (struct sockaddr*)&serverAddr, sizeof(serverAddr)) < 0) {
            throw std::runtime_error("Connection to the server failed for topic " + topic);
        }

        // Send the topic to the server to subscribe
        send(sock, topic.c_str(), topic.length(), 0);

        subscriptions[topic] = sock;
    }

    std::string receiveMessage(const std::string& topic) {
        if (subscriptions.find(topic) == subscriptions.end()) {
            throw std::runtime_error("Not subscribed to topic " + topic);
        }

        int sock = subscriptions[topic];
        char buffer[1024] = {0};
        int bytesReceived = recv(sock, buffer, sizeof(buffer), 0);
        if (bytesReceived < 0) {
            throw std::runtime_error("Failed to receive message on topic " + topic);
        }
        return std::string(buffer, bytesReceived);
    }

    void unsubscribe(const std::string& topic) {
        if (subscriptions.find(topic) != subscriptions.end()) {
            close(subscriptions[topic]);
            subscriptions.erase(topic);
        } else {
            std::cerr << "Warning: Attempt to unsubscribe from non-existent topic " << topic << std::endl;
        }
    }
};

// Usage example
int main() {
    try {
        ClientSocket client("127.0.0.1", 8080);

        client.subscribe("news");
        client.subscribe("weather");

        std::cout << "Subscribed to news and weather." << std::endl;

        while (true) {
            std::string newsMessage = client.receiveMessage("news");
            std::cout << "News: " << newsMessage << std::endl;

            std::string weatherMessage = client.receiveMessage("weather");
            std::cout << "Weather: " << weatherMessage << std::endl;
        }
    } catch (const std::exception& ex) {
        std::cerr << "Error: " << ex.what() << std::endl;
    }

    return 0;
}
