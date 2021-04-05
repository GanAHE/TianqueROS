/**
 * RC2Serial: Simple PPM -> serial converter on Arduino.
 *
 * Wiring:
 * Pin 3: PPM input
 * Output send via USB
 *
 * Serial output on USB: 115200, 8 data bits, 1 stop bit, no parity
 * Frame contains 2 byte header ("RC") and 16 channels values multiplied by 10 (e.g. 15000 for 1500 us), in uint16_t (big endian):
 * [uint8_t header0] [uint8_t header0] [uint16_t channel0] [uint16_t channel1] ... [uint16_t channel15]
 */

#define PPM_Pin 3  // this must be 2 or 3
#define MULTIPLIER (F_CPU / 1000000 / 8)  // TIMER1 prescaler: 8
#define PPM_CHANNELS_MAX 16

#pragma push(pack, 1)
struct {
    uint8_t header0;
    uint8_t header1;
    uint16_t values[PPM_CHANNELS_MAX];
} data;
#pragma pop(pack)

void setup() {
    Serial.begin(115200);

    pinMode(PPM_Pin, INPUT);
    attachInterrupt(PPM_Pin - 2, read_ppm, FALLING);

    TCCR1A = 0;  // reset TIMER1
    TCCR1B = 0;
    TCCR1B |= (1 << CS11);  // set TIMER1 prescaler: 8
}

void loop() {
    delay(1000);
}

void read_ppm() {
    static byte channel = 0;

    unsigned long counter = TCNT1;
    TCNT1 = 0;

    if (counter > 2500 * MULTIPLIER) {
        data.header0 = 'R';
        data.header1 = 'C';
        for (int i = channel; i < PPM_CHANNELS_MAX; i++) {
            data.values[i] = 0;
        }
        channel = 0;
        Serial.write((byte*)&data, sizeof(data));
    } else {
        if (channel < PPM_CHANNELS_MAX) {
            uint16_t v = counter * 10 / MULTIPLIER;
            data.values[channel++] = (v >> 8 | v << 8); // Swap bytes to big endian
        }
    }
}
