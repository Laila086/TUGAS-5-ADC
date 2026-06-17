#include <Arduino.h>

#if defined(ARDUINO_ARCH_STM32) && defined(USBCON)
#define DBG_SERIAL SerialUSB
#else
#define DBG_SERIAL Serial
#endif

// SRF05 wiring:
// - TRIG: output digital MCU -> Trig SRF05
// - ECHO: input digital MCU <- Echo SRF05
// - LED : indikator jarak
// Catatan: Echo SRF05 level 5V, gunakan pembagi tegangan jika MCU 3.3V.
#if defined(ARDUINO_ARCH_ESP32)
const uint8_t TRIG_PIN = 5;
const uint8_t ECHO_PIN = 18;
const uint8_t LED_PIN = 2;
const bool LED_ACTIVE_HIGH = true;
#else
const uint8_t TRIG_PIN = PB0;
const uint8_t ECHO_PIN = PB1;
// LED eksternal disarankan di PA8 + resistor 220R ke GND.
const uint8_t LED_PIN = PA8;
const bool LED_ACTIVE_HIGH = true;
#endif

const uint8_t SAMPLES = 4;
const uint32_t SAMPLE_INTERVAL_MS = 80;
const uint32_t REPORT_INTERVAL_MS = 500;

const uint32_t ECHO_TIMEOUT_US = 30000;

// Zona perilaku LED:
// > ZONE_BLINK_CM              : LED nyala stabil
// <= ZONE_BLINK_CM dan > ZONE_TOO_CLOSE_CM : LED berkedip (makin dekat makin cepat)
// <= ZONE_TOO_CLOSE_CM         : LED mati
const float ZONE_BLINK_CM = 35.0f;
const float ZONE_TOO_CLOSE_CM = 10.0f;

const uint32_t BLINK_SLOW_MS = 500;
const uint32_t BLINK_FAST_MS = 120;

float distanceBuffer[SAMPLES] = {0.0f};
float distanceSum = 0.0f;
uint8_t distanceIndex = 0;
bool bufferFilled = false;
bool ledOn = false;
bool noEcho = true;
unsigned long lastEchoPulseUs = 0;
unsigned long lastBlinkMs = 0;

enum LedMode {
    LED_MODE_OFF = 0,
    LED_MODE_SOLID,
    LED_MODE_BLINK
};

LedMode ledMode = LED_MODE_OFF;

void pushDistanceSample(float distanceCm) {
    distanceSum -= distanceBuffer[distanceIndex];
    distanceBuffer[distanceIndex] = distanceCm;
    distanceSum += distanceCm;

    distanceIndex++;
    if (distanceIndex >= SAMPLES) {
        distanceIndex = 0;
        bufferFilled = true;
    }
}

float getAverageDistance() {
    uint8_t count = bufferFilled ? SAMPLES : distanceIndex;
    if (count == 0) {
        return 0;
    }
    return distanceSum / (float)count;
}

float readSrf05DistanceCm() {
    digitalWrite(TRIG_PIN, LOW);
    delayMicroseconds(2);
    digitalWrite(TRIG_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIG_PIN, LOW);

    unsigned long echoPulseUs = pulseIn(ECHO_PIN, HIGH, ECHO_TIMEOUT_US);
    if (echoPulseUs == 0) {
        lastEchoPulseUs = 0;
        return -1.0f;
    }

    lastEchoPulseUs = echoPulseUs;

    // Rumus HC-SR04/SRF05 mode cm: jarak = durasi(us) / 58.
    return (float)echoPulseUs / 58.0f;
}

void writeLedState(bool on) {
    digitalWrite(LED_PIN, (on == LED_ACTIVE_HIGH) ? HIGH : LOW);
}

const char *getModeText() {
    if (ledMode == LED_MODE_SOLID) {
        return "ON";
    }
    if (ledMode == LED_MODE_BLINK) {
        return "BLINK";
    }
    return "OFF";
}

const char *getKondisiText(float distanceCm) {
    if (noEcho) {
        return "TIDAK_TERDETEKSI";
    }
    if (distanceCm <= ZONE_TOO_CLOSE_CM) {
        return "SANGAT_DEKAT";
    }
    if (distanceCm <= ZONE_BLINK_CM) {
        return "MENDEKAT";
    }
    return "JAUH";
}

void updateLedByDistance(float distanceCm, unsigned long nowMs) {
    if (distanceCm <= ZONE_TOO_CLOSE_CM) {
        ledMode = LED_MODE_OFF;
        ledOn = false;
        writeLedState(false);
        return;
    }

    if (distanceCm > ZONE_BLINK_CM) {
        ledMode = LED_MODE_SOLID;
        ledOn = true;
        writeLedState(true);
        return;
    }

    ledMode = LED_MODE_BLINK;

    // Semakin dekat ke ZONE_TOO_CLOSE_CM, interval blink makin cepat.
    float normalized = (distanceCm - ZONE_TOO_CLOSE_CM) / (ZONE_BLINK_CM - ZONE_TOO_CLOSE_CM);
    normalized = constrain(normalized, 0.0f, 1.0f);
    uint32_t intervalMs = (uint32_t)(BLINK_FAST_MS + normalized * (BLINK_SLOW_MS - BLINK_FAST_MS));

    if (nowMs - lastBlinkMs >= intervalMs) {
        lastBlinkMs = nowMs;
        ledOn = !ledOn;
        writeLedState(ledOn);
    }
}

void setup() {
    DBG_SERIAL.begin(115200);
    delay(1500);

#if defined(ARDUINO_ARCH_STM32) && defined(USBCON)
    unsigned long usbWaitStart = millis();
    while (!DBG_SERIAL && (millis() - usbWaitStart < 3000)) {
        delay(10);
    }
#endif

    pinMode(TRIG_PIN, OUTPUT);
    pinMode(ECHO_PIN, INPUT);
    digitalWrite(TRIG_PIN, LOW);

    pinMode(LED_PIN, OUTPUT);
    writeLedState(false);

    // Self-test level pin mentah agar kedua kemungkinan wiring LED bisa terlihat.
    digitalWrite(LED_PIN, HIGH);
    delay(300);
    digitalWrite(LED_PIN, LOW);
    delay(300);
    digitalWrite(LED_PIN, HIGH);
    delay(300);
    digitalWrite(LED_PIN, LOW);
    writeLedState(false);

    DBG_SERIAL.println("SRF05 Sensor Jarak - Mode LED Zona Jarak");
}

void loop() {
    static unsigned long lastSampleMs = 0;
    static unsigned long lastReportMs = 0;

    unsigned long now = millis();

    if (now - lastSampleMs >= SAMPLE_INTERVAL_MS) {
        lastSampleMs = now;

        float distanceNow = readSrf05DistanceCm();
        if (distanceNow > 0.0f) {
            noEcho = false;
            pushDistanceSample(distanceNow);
            float distanceAvg = getAverageDistance();
            updateLedByDistance(distanceAvg, now);
        } else {
            noEcho = true;
            // Saat tidak terdeteksi, LED tetap nyala (sesuai kebutuhan).
            ledMode = LED_MODE_SOLID;
            ledOn = true;
            writeLedState(true);
        }
    }

    if (now - lastReportMs >= REPORT_INTERVAL_MS) {
        lastReportMs = now;

        if (noEcho) {
            DBG_SERIAL.println("Jarak=--- cm | Kondisi=TIDAK_TERDETEKSI | LED=ON");
        } else {
            float distanceCm = getAverageDistance();
            const char *kondisiText = getKondisiText(distanceCm);
            const char *modeText = getModeText();

            DBG_SERIAL.printf(
                "Jarak=%.1f cm | Kondisi=%s | Mode=%s | LED=%s\n",
                distanceCm,
                kondisiText,
                modeText,
                ledOn ? "ON" : "OFF"
            );
        }
    }
}
