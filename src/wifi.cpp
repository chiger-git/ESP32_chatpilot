// Copyright (c) 2023 Oleg Kalachev <okalachev@gmail.com>
// Repository: https://github.com/okalachev/flix
// Wi-Fi功能支持
// Wi-Fi support
#include <Arduino.h>


#if WIFI_ENABLED

#include <WiFi.h>
#include <WiFiAP.h>
#include <WiFiUdp.h>

#define WIFI_SSID "Drone_WiFi"
#define WIFI_PASSWORD "12345678"
#define WIFI_UDP_PORT 14550
#define WIFI_UDP_REMOTE_PORT 14550
#define WIFI_UDP_REMOTE_ADDR "255.255.255.255"

extern void print(const char* format, ...);
extern bool mavlinkConnected; // 定义在 mavlink.cpp 中，用于显示连接状态

WiFiUDP udp;

void setupWiFi() {
	print("Setup Wi-Fi\n");
	WiFi.mode(WIFI_AP);// 2026.3.14为提高连接稳定性，添加此行设置为AP模式
	WiFi.softAP(WIFI_SSID, WIFI_PASSWORD);
	udp.begin(WIFI_UDP_PORT);
}

void sendWiFi(const uint8_t *buf, int len) {
    if (WiFi.softAPIP() == IPAddress(0, 0, 0, 0) && WiFi.status() != WL_CONNECTED) return;
	// 获取远程 IP
    IPAddress targetIP = udp.remoteIP();
    // 如果远程 IP 是 0.0.0.0 (表示还没收到过包)，则使用广播地址
    if (targetIP == IPAddress(0, 0, 0, 0)) {
        targetIP.fromString(WIFI_UDP_REMOTE_ADDR);
    }
    udp.beginPacket(targetIP, WIFI_UDP_REMOTE_PORT);
    udp.write(buf, len);
    udp.endPacket();
}

int receiveWiFi(uint8_t *buf, int len) {
	udp.parsePacket();
	return udp.read(buf, len);
}

void printWiFiInfo() {
	print("MAC: %s\n", WiFi.softAPmacAddress().c_str());
	print("SSID: %s\n", WiFi.softAPSSID().c_str());
	print("Password: %s\n", WIFI_PASSWORD);
	print("Clients: %d\n", WiFi.softAPgetStationNum());
	print("Status: %d\n", WiFi.status());
	print("IP: %s\n", WiFi.softAPIP().toString().c_str());
	print("Remote IP: %s\n", udp.remoteIP().toString().c_str());
	print("MAVLink connected: %d\n", mavlinkConnected);
}

#endif
