// Copyright (c) 2023 Oleg Kalachev <okalachev@gmail.com>
// Repository: https://github.com/okalachev/flix
// Wi-Fi功能支持
// Wi-Fi support
#include <Arduino.h>


#if WIFI_ENABLED

#include <esp_wifi.h>
#include <WiFi.h>
#include <WiFiAP.h>
#include <WiFiUdp.h>

#define WIFI_SSID "Drone_WiFi"
#define WIFI_PASSWORD "12345678"
#define WIFI_UDP_PORT 14550
#define WIFI_UDP_REMOTE_PORT 14550
#define WIFI_UDP_REMOTE_ADDR "255.255.255.255"

extern void print(const char* format, ...);
extern uint32_t mavlinkTxMessages;
extern uint32_t mavlinkTxBytes;
extern uint32_t mavlinkRxMessages;
extern uint32_t mavlinkRxBytes;
extern uint32_t mavlinkParseErrors;
extern float mavlinkLastTxTime;
extern float mavlinkLastRxTime;
extern float t;
extern bool mavlinkConnected; // 定义在 mavlink.cpp 中，用于显示连接状态

WiFiUDP udp;
uint32_t wifiTxPackets = 0;
uint32_t wifiTxBytes = 0;
uint32_t wifiTxFails = 0;
uint32_t wifiRxPackets = 0;
uint32_t wifiRxBytes = 0;
uint32_t wifiLastTxMillis = 0;
uint32_t wifiLastRxMillis = 0;

void setupWiFi() {
	print("Setup Wi-Fi\n");
	WiFi.persistent(false);
	WiFi.mode(WIFI_AP);
	WiFi.setSleep(false);
	esp_wifi_set_ps(WIFI_PS_NONE);
	WiFi.softAP(WIFI_SSID, WIFI_PASSWORD, 1, false, 1);
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
    if (udp.endPacket()) {
		wifiTxPackets++;
		wifiTxBytes += len;
		wifiLastTxMillis = millis();
	} else {
		wifiTxFails++;
	}
}

int receiveWiFi(uint8_t *buf, int len) {
	const int packetLen = udp.parsePacket();
	if (!packetLen) return 0;
	const int readLen = udp.read(buf, len);
	if (readLen > 0) {
		wifiRxPackets++;
		wifiRxBytes += readLen;
		wifiLastRxMillis = millis();
	}
	return readLen;
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
	print("UDP tx packets: %lu bytes: %lu fails: %lu age: %.3f s\n",
		wifiTxPackets, wifiTxBytes, wifiTxFails,
		wifiLastTxMillis ? (millis() - wifiLastTxMillis) * 0.001f : NAN);
	print("UDP rx packets: %lu bytes: %lu age: %.3f s\n",
		wifiRxPackets, wifiRxBytes,
		wifiLastRxMillis ? (millis() - wifiLastRxMillis) * 0.001f : NAN);
	print("MAVLink tx msg: %lu bytes: %lu age: %.3f s\n",
		mavlinkTxMessages, mavlinkTxBytes, isfinite(mavlinkLastTxTime) ? t - mavlinkLastTxTime : NAN);
	print("MAVLink rx msg: %lu bytes: %lu parse_err: %lu age: %.3f s\n",
		mavlinkRxMessages, mavlinkRxBytes, mavlinkParseErrors, isfinite(mavlinkLastRxTime) ? t - mavlinkLastRxTime : NAN);
}

#endif
