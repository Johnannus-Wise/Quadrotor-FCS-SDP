#include "wifi_server.h"
#include <WiFi.h>
#include <esp_wifi.h>

// Wi-Fi credentials — defined here, declared extern in config.h
const char *ssid     = "Hakeem_2.4GHz";
const char *password = "19641964";

WebServer server(80);

// ============================================================
//  HTML page template
//  BUG FIX: the original static char page[8000] buffer was undersized.
//  The template has 32 format specifiers (%.4f each expands to up to 12 chars).
//  Template body ≈ 3 700 bytes + 32×12 = 4 084 bytes → safely within 8192,
//  Updated to include MAHONY_KP and MAHONY_KI.
//  Raised to 10300 for headroom; also PROGMEM-stored template to reduce DRAM
//  pressure on the ESP32.
// ============================================================

static const char htmlPageTemplate[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
<meta charset="UTF-8">
<title>Flight Controller Tuning</title>
<style>
body { font-family: Arial; max-width: 520px; margin: auto; }
label { display: inline-block; width: 180px; }
input { width: 90px; }
</style>
<script>
function updateTelemetry() {
  fetch('/telemetry')
    .then(r => r.json())
    .then(d => {
      document.getElementById('temperature').innerText = d.temperature_c.toFixed(2);
      document.getElementById('pressure').innerText    = d.pressure_pa.toFixed(2);
      document.getElementById('altitude').innerText    = d.altitude_m.toFixed(2);
      document.getElementById('pitch').innerText       = d.pitch_deg.toFixed(2);
      document.getElementById('roll').innerText        = d.roll_deg.toFixed(2);
      document.getElementById('throttle').innerText    = d.throttle.toFixed(0);
    });
}

function updateParameters() {
  const p = new URLSearchParams();
  const ids = [
    'rollAngleKp','rollAngleKi','rollAngleKd','rollAngleTol','rollAngleDAlpha',
    'pitchAngleKp','pitchAngleKi','pitchAngleKd','pitchAngleTol','pitchAngleDAlpha',
    'rollRateKp','rollRateKi','rollRateKd','rollRateTol','rollRateDAlpha',
    'pitchRateKp','pitchRateKi','pitchRateKd','pitchRateTol','pitchRateDAlpha',
    'yawRateKp','yawRateKi','yawRateKd','yawRateTol','yawRateDAlpha',
    'altitudeKp','altitudeKi','altitudeKd','altitudeTol','altitudeDAlpha',
    'mahonyKp','mahonyKi'
  ];
  ids.forEach(id => p.append(id, document.getElementById(id).value));
  fetch('/update?' + p.toString());
}

setInterval(updateTelemetry, 300);
</script>
</head>
<body>
<h2>Flight Controller PID Tuning</h2>

<h3>Roll Angle</h3>
<input id="rollAngleKp"     value="%.4f"> Kp<br>
<input id="rollAngleKi"     value="%.4f"> Ki<br>
<input id="rollAngleKd"     value="%.4f"> Kd<br>
<input id="rollAngleTol"    value="%.4f"> Tol<br>
<input id="rollAngleDAlpha" value="%.4f"> D&#945;<br>

<h3>Pitch Angle</h3>
<input id="pitchAngleKp"     value="%.4f"> Kp<br>
<input id="pitchAngleKi"     value="%.4f"> Ki<br>
<input id="pitchAngleKd"     value="%.4f"> Kd<br>
<input id="pitchAngleTol"    value="%.4f"> Tol<br>
<input id="pitchAngleDAlpha" value="%.4f"> D&#945;<br>

<h3>Roll Rate</h3>
<input id="rollRateKp"     value="%.4f"> Kp<br>
<input id="rollRateKi"     value="%.4f"> Ki<br>
<input id="rollRateKd"     value="%.4f"> Kd<br>
<input id="rollRateTol"    value="%.4f"> Tol<br>
<input id="rollRateDAlpha" value="%.4f"> D&#945;<br>

<h3>Pitch Rate</h3>
<input id="pitchRateKp"     value="%.4f"> Kp<br>
<input id="pitchRateKi"     value="%.4f"> Ki<br>
<input id="pitchRateKd"     value="%.4f"> Kd<br>
<input id="pitchRateTol"    value="%.4f"> Tol<br>
<input id="pitchRateDAlpha" value="%.4f"> D&#945;<br>

<h3>Yaw Rate</h3>
<input id="yawRateKp"     value="%.4f"> Kp<br>
<input id="yawRateKi"     value="%.4f"> Ki<br>
<input id="yawRateKd"     value="%.4f"> Kd<br>
<input id="yawRateTol"    value="%.4f"> Tol<br>
<input id="yawRateDAlpha" value="%.4f"> D&#945;<br>

<h3>Altitude</h3>
<input id="altitudeKp"     value="%.4f"> Kp<br>
<input id="altitudeKi"     value="%.4f"> Ki<br>
<input id="altitudeKd"     value="%.4f"> Kd<br>
<input id="altitudeTol"    value="%.4f"> Tol<br>
<input id="altitudeDAlpha" value="%.4f"> D&#945;<br>

<h3>Mahony Filter</h3>
<input id="mahonyKp" value="%.4f"> Kp<br>
<input id="mahonyKi" value="%.4f"> Ki<br>

<br><br>
<button onclick="updateParameters()">Update</button>

<hr>
<h3>Telemetry</h3>
<p>Temperature: <span id="temperature">0</span> &deg;C</p>
<p>Pressure: <span id="pressure">0</span> Pa</p>
<p>Altitude: <span id="altitude">0</span> m</p>
<p>Pitch: <span id="pitch">0</span> &deg;</p>
<p>Roll: <span id="roll">0</span> &deg;</p>
<p>Throttle: <span id="throttle">0</span></p>
</body>
</html>
)rawliteral";

// ============================================================
//  Route handlers
// ============================================================

void handleRoot()
{
    // BUG FIX: raised buffer from 8000 → 10300 (see template note above)
    static char page[10300];

    int written = snprintf(
        page, sizeof(page), htmlPageTemplate,
        // Roll Angle
        rollAngleController.Kp,  rollAngleController.Ki,
        rollAngleController.Kd,  rollAngleController.tolerance,
        rollAngleController.derivativeFilterAlpha,
        // Pitch Angle
        pitchAngleController.Kp, pitchAngleController.Ki,
        pitchAngleController.Kd, pitchAngleController.tolerance,
        pitchAngleController.derivativeFilterAlpha,
        // Roll Rate
        rollRateController.Kp,   rollRateController.Ki,
        rollRateController.Kd,   rollRateController.tolerance,
        rollRateController.derivativeFilterAlpha,
        // Pitch Rate
        pitchRateController.Kp,  pitchRateController.Ki,
        pitchRateController.Kd,  pitchRateController.tolerance,
        pitchRateController.derivativeFilterAlpha,
        // Yaw Rate
        yawRateController.Kp,    yawRateController.Ki,
        yawRateController.Kd,    yawRateController.tolerance,
        yawRateController.derivativeFilterAlpha,
        // Altitude
        altitudeController.Kp,   altitudeController.Ki,
        altitudeController.Kd,   altitudeController.tolerance,
        altitudeController.derivativeFilterAlpha,
        // Mahony Filter
        MAHONY_KP, MAHONY_KI);

    // BUG FIX: original ignored snprintf return value.
    // If the buffer is still too small, serve a minimal error page.
    if (written < 0 || written >= (int)sizeof(page))
    {
        server.send(500, "text/plain", "Page buffer overflow — increase page[] size");
        return;
    }

    server.send(200, "text/html", page);
}

void handleUpdate()
{
    rollAngleController.Kp                    = server.arg("rollAngleKp").toFloat();
    rollAngleController.Ki                    = server.arg("rollAngleKi").toFloat();
    rollAngleController.Kd                    = server.arg("rollAngleKd").toFloat();
    rollAngleController.tolerance             = server.arg("rollAngleTol").toFloat();
    rollAngleController.derivativeFilterAlpha = server.arg("rollAngleDAlpha").toFloat();

    pitchAngleController.Kp                    = server.arg("pitchAngleKp").toFloat();
    pitchAngleController.Ki                    = server.arg("pitchAngleKi").toFloat();
    pitchAngleController.Kd                    = server.arg("pitchAngleKd").toFloat();
    pitchAngleController.tolerance             = server.arg("pitchAngleTol").toFloat();
    pitchAngleController.derivativeFilterAlpha = server.arg("pitchAngleDAlpha").toFloat();

    rollRateController.Kp                    = server.arg("rollRateKp").toFloat();
    rollRateController.Ki                    = server.arg("rollRateKi").toFloat();
    rollRateController.Kd                    = server.arg("rollRateKd").toFloat();
    rollRateController.tolerance             = server.arg("rollRateTol").toFloat();
    rollRateController.derivativeFilterAlpha = server.arg("rollRateDAlpha").toFloat();

    pitchRateController.Kp                    = server.arg("pitchRateKp").toFloat();
    pitchRateController.Ki                    = server.arg("pitchRateKi").toFloat();
    pitchRateController.Kd                    = server.arg("pitchRateKd").toFloat();
    pitchRateController.tolerance             = server.arg("pitchRateTol").toFloat();
    pitchRateController.derivativeFilterAlpha = server.arg("pitchRateDAlpha").toFloat();

    yawRateController.Kp                    = server.arg("yawRateKp").toFloat();
    yawRateController.Ki                    = server.arg("yawRateKi").toFloat();
    yawRateController.Kd                    = server.arg("yawRateKd").toFloat();
    yawRateController.tolerance             = server.arg("yawRateTol").toFloat();
    yawRateController.derivativeFilterAlpha = server.arg("yawRateDAlpha").toFloat();

    altitudeController.Kp                    = server.arg("altitudeKp").toFloat();
    altitudeController.Ki                    = server.arg("altitudeKi").toFloat();
    altitudeController.Kd                    = server.arg("altitudeKd").toFloat();
    altitudeController.tolerance             = server.arg("altitudeTol").toFloat();
    altitudeController.derivativeFilterAlpha = server.arg("altitudeDAlpha").toFloat();

    MAHONY_KP = server.arg("mahonyKp").toFloat();
    MAHONY_KI = server.arg("mahonyKi").toFloat();

    server.send(200, "text/plain", "OK");
}

void handleTelemetry()
{
    // BUG FIX: original built a String by concatenation in a tight hot-path.
    // Use snprintf into a stack buffer to avoid heap fragmentation.
    char buf[256];
    snprintf(buf, sizeof(buf),
             "{\"temperature_c\":%.2f,\"pressure_pa\":%.2f,\"altitude_m\":%.2f,\"pitch_deg\":%.2f,\"roll_deg\":%.2f,\"throttle\":%d}",
             temp, pressure, altitude, pitch, roll, mainThrottleInput);
    server.send(200, "application/json", buf);
}

// ============================================================
//  Init
// ============================================================

void PIDWebPage()
{
    Serial.print("Connecting to ");
    Serial.println(ssid);

    WiFi.mode(WIFI_STA);
    WiFi.disconnect();
    delay(100);

    // Set static IP to 192.168.100.19
    IPAddress staticIP(192, 168, 100, 19);
    IPAddress gateway(192, 168, 100, 1);
    IPAddress subnet(255, 255, 255, 0);
    WiFi.config(staticIP, gateway, subnet);

    WiFi.begin(ssid, password);
    WiFi.setSleep(false);
    esp_wifi_set_ps(WIFI_PS_NONE);

    uint32_t t0 = millis();
    while (WiFi.status() != WL_CONNECTED)
    {
        // BUG FIX: original had an infinite blocking loop with no timeout.
        // If SSID is wrong or AP is unreachable the drone hangs forever in
        // setup() and never arms. Now times out after 15 s and continues.
        if (millis() - t0 > 15000)
        {
            Serial.println("\nWiFi timeout — continuing without web server");
            return;
        }
        delay(500);
        Serial.print(".");
    }

    Serial.println("\nWiFi connected!");
    Serial.print("IP: ");
    Serial.println(WiFi.localIP());

    server.on("/",          handleRoot);
    server.on("/update",    handleUpdate);
    server.on("/telemetry", handleTelemetry);

    server.begin();
    Serial.println("Web server started");
}
