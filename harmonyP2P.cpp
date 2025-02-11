#include <iostream>
#include <cstring>
#include <sstream>
#include <vector>
#include <array>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <unistd.h>

// Harmony includes
#include "research_interface.h"
#include "arm_controller.h"
#include "overrides.h"

static constexpr int PORT = 12346;
static constexpr int BUFFER_SIZE = 4096; // message buffer
static constexpr char HARMONY_IP[] = "192.168.2.1";

/// Helper: splits a string into tokens (floats).
std::vector<double> splitToDoubles(const std::string &s) {
    std::vector<double> result;
    std::istringstream iss(s);
    double val;
    while (iss >> val) {
        result.push_back(val);
    }
    return result;
}

int main()
{
    // 1) Initialize the Research Interface
    harmony::ResearchInterface info;
    if (!info.init()) {
        std::cerr << "Failed to initialize ResearchInterface" << std::endl;
        return -1;
    }

    // For controlling the right arm, create the appropriate controller
    // If you also want the left arm, do similarly: auto leftCtrl = info.makeLeftArmController();
    auto rightCtrl = harmony::ResearchInterface::makeRightArmController();
    if (!rightCtrl->init()) {
        std::cerr << "Failed to init right arm controller" << std::endl;
        return -1;
    }

    // 2) Setup a UDP socket to listen on port 12346, IP 192.168.2.1
    int sockfd;
    struct sockaddr_in servaddr{}, cliaddr{};
    
    if ((sockfd = socket(AF_INET, SOCK_DGRAM, 0)) < 0) {
        std::cerr << "Socket creation failed" << std::endl;
        return -1;
    }

    memset(&servaddr, 0, sizeof(servaddr));
    servaddr.sin_family = AF_INET;
    servaddr.sin_addr.s_addr = inet_addr(HARMONY_IP);
    servaddr.sin_port = htons(PORT);

    if (bind(sockfd, (const struct sockaddr*)&servaddr, sizeof(servaddr)) < 0) {
        std::cerr << "Bind failed" << std::endl;
        close(sockfd);
        return -1;
    }

    std::cout << "Listening on " << HARMONY_IP << ":" << PORT << " for override commands..." << std::endl;

    // 3) Main loop: receive, parse, respond
    socklen_t len = sizeof(cliaddr);
    char buffer[BUFFER_SIZE];

    while (true) {
        // Wait for incoming command
        int n = recvfrom(sockfd, buffer, BUFFER_SIZE, MSG_WAITALL,
                         (struct sockaddr*)&cliaddr, &len);
        if (n < 0) {
            std::cerr << "recvfrom error\n";
            continue;
        }
        buffer[n] = '\0'; // null-terminate
        std::string cmd(buffer);

        std::cout << "Received command: " << cmd << std::endl;

        // Example command: "SET RARM JOINTOVERRIDE 0.1 10.0 0.2 10.0 0.3 10.0 ..."
        // Parse it (very basic parsing):
        std::istringstream iss(cmd);
        std::string action, object, parameter;
        iss >> action >> object >> parameter; // e.g. "SET", "RARM", "JOINTOVERRIDE"

        // Build a response for debugging
        std::string response;

        if (action == "SET" && object == "RARM" && parameter == "JOINTOVERRIDE") {
            // The rest of the line should be 14 floats: 7 positions + 7 stiffnesses
            std::string remaining;
            getline(iss, remaining); // grab the rest
            auto values = splitToDoubles(remaining);

            if (values.size() == 14) {
                // Construct an array of 7 JointOverride objects
                std::array<harmony::JointOverride, harmony::armJointCount> overrides;
                for (int i = 0; i < 7; i++) {
                    double posRad = values[2*i + 0];
                    double stiff = values[2*i + 1];
                    overrides[i] = harmony::JointOverride{posRad, stiff};
                }
                // Create an ArmJointsOverride
                harmony::ArmJointsOverride armOverride(overrides, true /*enable shoulder constraints*/);

                // Actually apply the override
                rightCtrl->setJointsOverride(armOverride);

                response = "OK: RARM JOINTOVERRIDE applied";
                std::cout << response << std::endl;
            } else {
                response = "ERROR: expected 14 float values after JOINTOVERRIDE";
                std::cerr << response << std::endl;
            }
        }
        else if (action == "SET" && object == "RARM" && parameter == "HARMONY") {
            // e.g. "SET RARM HARMONY" to remove override
            rightCtrl->removeOverride();
            response = "OK: RARM override removed (back to harmony mode)";
            std::cout << response << std::endl;
        }
        else {
            response = "ERROR: Unrecognized command format. Expecting: SET RARM JOINTOVERRIDE ...";
            std::cerr << response << std::endl;
        }

        // Send the response back to the sender
        sendto(sockfd, response.c_str(), response.size(), MSG_CONFIRM,
               (const struct sockaddr*)&cliaddr, len);
    }

    close(sockfd);
    return 0;
}
