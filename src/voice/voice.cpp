#include "voice.h"

// 使用硬件串口1 (UART1)
HardwareSerial VoiceSerial(1);

// 引脚定义：将 RX 映射到 GPIO 26，TX 映射到 GPIO 25
const int VOICE_RX_PIN = 26;
const int VOICE_TX_PIN = 25;

void voice_init() {
    // ASRPRO 波特率
    VoiceSerial.begin(115200, SERIAL_8N1, VOICE_RX_PIN, VOICE_TX_PIN);
    Serial.println("Voice module (ASRPRO) initialized on UART1 (RX: GPIO 26, TX: GPIO 25).");
}

void voice_read_command() {
    // 检查是否有数据可读
    if (VoiceSerial.available()) {
        Serial.print("Voice Rx Hex: ");
        while (VoiceSerial.available()) {
            uint8_t data = VoiceSerial.read();
            // 为了美观打印，补充前导 0
            if (data < 0x10) {
                Serial.print("0");
            }
            Serial.print(data, HEX);
            Serial.print(" ");
        }
        Serial.println();
    }
}
