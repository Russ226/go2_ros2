#include <arpa/inet.h>
#include <ifaddrs.h>
#include <net/if.h>
#include <stdexcept>
#include <string>
#include <iostream>
#include <cstdint>

static uint32_t IpToU32(const char* ip)
{
    in_addr addr{};
    if (inet_pton(AF_INET, ip, &addr) != 1) {
        throw std::runtime_error("Invalid IP");
    }
    return ntohl(addr.s_addr);
}

std::string DetectInterfaceBySubnet(const std::string& subnet_ip,
                                    const std::string& netmask_ip)
{
    uint32_t subnet = IpToU32(subnet_ip.c_str());
    uint32_t mask   = IpToU32(netmask_ip.c_str());

    struct ifaddrs* ifaddr = nullptr;
    if (getifaddrs(&ifaddr) == -1) {
        throw std::runtime_error("getifaddrs() failed");
    }

    std::string found;

    for (struct ifaddrs* ifa = ifaddr; ifa != nullptr; ifa = ifa->ifa_next) {
        if (!ifa->ifa_addr || !ifa->ifa_netmask) continue;
        if (ifa->ifa_addr->sa_family != AF_INET) continue;
        if (!(ifa->ifa_flags & IFF_UP)) continue;
        if (ifa->ifa_flags & IFF_LOOPBACK) continue;

        auto* addr = reinterpret_cast<sockaddr_in*>(ifa->ifa_addr);
        uint32_t ip = ntohl(addr->sin_addr.s_addr);

        if ((ip & mask) == (subnet & mask)) {
            found = ifa->ifa_name;
            break;
        }
    }

    freeifaddrs(ifaddr);

    if (found.empty()) {
        throw std::runtime_error("No active interface found on requested subnet");
    }

    return found;
}

// int main()
// {
//     try {
//         std::string iface = DetectInterfaceBySubnet("192.168.123.0", "255.255.255.0");
//         std::cout << "Using interface: " << iface << std::endl;

//     } catch (const std::exception& e) {
//         std::cerr << e.what() << std::endl;
//         return 1;
//     }
// }