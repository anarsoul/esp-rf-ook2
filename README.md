This is an app for ESP32 to decode the signal from Nexus-TH 433MHz thermal
sensor.

It is a no_std rewrite of [esp-rf-ook](https://github.com/anarsoul/esp-rf-ook)

RXB6 RF receiver is connected to GPIO21 (change it in the code if you need a
different pin). RXB6 outputs high level when it detects carrier, low level when
it detects no carrier.

The app uses RMT module to count number of ticks between edges.

Nexus-TH uses OOK modulation at 433MHz, basic params:
* pulse is 400-600uS (carrier present)
* preamble is >2000 uS (no carrier)
* end of payload is >3000 uS (no carrier)
* zero is 800-1000 uS (no carrier)
* one is 1650-2150 uS (no carrier)

The decoder reads each bit from its period, one rising edge to the next:
1500 uS for a zero and 2500 uS for a one, within 200 uS. The RXB6's slicer
stretches pulses, chops gaps and pulses with glitches, and after the silence
between repeats stays high through the first short gaps, but the real pulses
stay on that grid. Edges that land off the grid are skipped as glitches, and
an edge the receiver lost is recovered from the sum of two periods (3000,
5000, and 4000 when the stuck high is about 2000 uS long). A frame may start
at the capture's first pulse or after a low of at least 2200 uS.

Signal looks like:
```
   |--|                  |--|       |--|                 |--|                             |--|
   |  |                  |  |       |  |                 |  |                             |  |
---|  |------------------|  |-------|  |-----------------|  |-----------------------------|  |
   PULSE    PREAMBLE     PULSE ZERO PULSE    ONE              END OF PAYLOAD
```

Payload is transmitted several times and consists of 36 bits (preamble and EOP
are not counted)


AAAAAAAA BX CC DDDDDDDDDDDD EEEE FFFFFFFF, where:

* A - ID
* B - 1 if battery is OK, 0 if battery low
* X - always zero; a frame with it set is rejected
* C - channel, zero based (0 for channel 1)
* D - temperature * 10 in C. E.g. 123 for 12.3C
* E - always 1111; a frame with anything else is rejected
* F - Humidity. Up to 110 is clamped to 100, above that the frame is rejected

Set following env variables to specify your credentials for WiFi and MQTT:

```
SSID=your_ssid
PASSWORD=your_wifi_password
MQTT_SERVER=your_mqtt_server
MQTT_LOGIN=your_mqtt_login
MQTT_PASSWORD=your_mqtt_password
MQTT_TOPIC=your_mqtt_topic
```

The receiver locks onto the sensor id of the first burst it decodes and
ignores other Nexus-TH sensors from then on. The id changes when the
sensor's battery is replaced; reset the board to lock onto the new one (it
resets itself after five minutes without a published reading).

The app will publish JSON with temperature and humidity data, example:
```
{"time" : "2024-11-02 12:05:31 UTC", "model" : "Nexus-TH", "id" : 174, "channel" : 1, "battery_ok" : true, "temperature_C" : 10.1, "humidity" : 91, "decodes" : 9, "uptime" : 86400123 }
```

`decodes` is how many of the burst's repeats decoded, agreeing with the
published reading or not; the sensor sends about a dozen, so a count that
falls over time is a weakening signal. `uptime` is milliseconds since boot.
