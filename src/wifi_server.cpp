#include "wifi_server.h"
#include <WiFi.h>
#include <esp_wifi.h>

// Wi-Fi credentials — defined here, declared extern in config.h
const char *ssid     = "Hakeem_2.4GHz";
const char *password = "19641964";

WebServer server(80);

// ============================================================
//  HTML page template
// ============================================================

const char htmlPageTemplate[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
<meta charset="UTF-8">
<title>Flight Controller Tuning</title>

<style>
body {
  font-family: Arial;
  max-width: 520px;
  margin: auto;
}
label {
  display: inline-block;
  width: 180px;
}
input {
  width: 90px;
}
</style>

<script>
function updateTelemetry() {
  fetch('/telemetry')
    .then(r => r.json())
    .then(d => {
      document.getElementById('roll').innerText = d.roll.toFixed(2);
      document.getElementById('pitch').innerText = d.pitch.toFixed(2);
      document.getElementById('altitude').innerText = d.altitude.toFixed(2);
    });
}

function updateParameters() {
  const p = new URLSearchParams();

  p.append('rollAngleKp',    document.getElementById('rollAngleKp').value);
  p.append('rollAngleKi',    document.getElementById('rollAngleKi').value);
  p.append('rollAngleKd',    document.getElementById('rollAngleKd').value);
  p.append('rollAngleTol',   document.getElementById('rollAngleTol').value);
  p.append('rollAngleDAlpha',document.getElementById('rollAngleDAlpha').value);

  p.append('pitchAngleKp',    document.getElementById('pitchAngleKp').value);
  p.append('pitchAngleKi',    document.getElementById('pitchAngleKi').value);
  p.append('pitchAngleKd',    document.getElementById('pitchAngleKd').value);
  p.append('pitchAngleTol',   document.getElementById('pitchAngleTol').value);
  p.append('pitchAngleDAlpha',document.getElementById('pitchAngleDAlpha').value);

  p.append('rollRateKp',    document.getElementById('rollRateKp').value);
  p.append('rollRateKi',    document.getElementById('rollRateKi').value);
  p.append('rollRateKd',    document.getElementById('rollRateKd').value);
  p.append('rollRateTol',   document.getElementById('rollRateTol').value);
  p.append('rollRateDAlpha',document.getElementById('rollRateDAlpha').value);

  p.append('pitchRateKp',    document.getElementById('pitchRateKp').value);
  p.append('pitchRateKi',    document.getElementById('pitchRateKi').value);
  p.append('pitchRateKd',    document.getElementById('pitchRateKd').value);
  p.append('pitchRateTol',   document.getElementById('pitchRateTol').value);
  p.append('pitchRateDAlpha',document.getElementById('pitchRateDAlpha').value);

  p.append('yawRateKp',    document.getElementById('yawRateKp').value);
  p.append('yawRateKi',    document.getElementById('yawRateKi').value);
  p.append('yawRateKd',    document.getElementById('yawRateKd').value);
  p.append('yawRateTol',   document.getElementById('yawRateTol').value);
  p.append('yawRateDAlpha',document.getElementById('yawRateDAlpha').value);

  p.append('altitudeKp',    document.getElementById('altitudeKp').value);
  p.append('altitudeKi',    document.getElementById('altitudeKi').value);
  p.append('altitudeKd',    document.getElementById('altitudeKd').value);
  p.append('altitudeTol',   document.getElementById('altitudeTol').value);
  p.append('altitudeDAlpha',document.getElementById('altitudeDAlpha').value);

  fetch('/update?' + p.toString());
}

setInterval(updateTelemetry, 300);
</script>
</head>

<body>
<h2>Flight Controller PID Tuning</h2>

<h3>Roll Angle</h3>
<input id="rollAngleKp"     value="%.3f"> Kp<br>
<input id="rollAngleKi"     value="%.3f"> Ki<br>
<input id="rollAngleKd"     value="%.3f"> Kd<br>
<input id="rollAngleTol"    value="%.3f"> Tol<br>
<input id="rollAngleDAlpha" value="%.3f"> D&#945;<br>

<h3>Pitch Angle</h3>
<input id="pitchAngleKp"     value="%.3f"> Kp<br>
<input id="pitchAngleKi"     value="%.3f"> Ki<br>
<input id="pitchAngleKd"     value="%.3f"> Kd<br>
<input id="pitchAngleTol"    value="%.3f"> Tol<br>
<input id="pitchAngleDAlpha" value="%.3f"> D&#945;<br>

<h3>Roll Rate</h3>
<input id="rollRateKp"     value="%.3f"> Kp<br>
<input id="rollRateKi"     value="%.3f"> Ki<br>
<input id="rollRateKd"     value="%.3f"> Kd<br>
<input id="rollRateTol"    value="%.3f"> Tol<br>
<input id="rollRateDAlpha" value="%.3f"> D&#945;<br>

<h3>Pitch Rate</h3>
<input id="pitchRateKp"     value="%.3f"> Kp<br>
<input id="pitchRateKi"     value="%.3f"> Ki<br>
<input id="pitchRateKd"     value="%.3f"> Kd<br>
<input id="pitchRateTol"    value="%.3f"> Tol<br>
<input id="pitchRateDAlpha" value="%.3f"> D&#945;<br>

<h3>Yaw Rate</h3>
<input id="yawRateKp"     value="%.3f"> Kp<br>
<input id="yawRateKi"     value="%.3f"> Ki<br>
<input id="yawRateKd"     value="%.3f"> Kd<br>
<input id="yawRateTol"    value="%.3f"> Tol<br>
<input id="yawRateDAlpha" value="%.3f"> D&#945;<br>

<h3>Altitude</h3>
<input id="altitudeKp"     value="%.3f"> Kp<br>
<input id="altitudeKi"     value="%.3f"> Ki<br>
<input id="altitudeKd"     value="%.3f"> Kd<br>
<input id="altitudeTol"    value="%.3f"> Tol<br>
<input id="altitudeDAlpha" value="%.3f"> D&#945;<br>

<br><br>
<button onclick="updateParameters()">Update</button>

<hr>
<p>Roll: <span id="roll">0</span></p>
<p>Pitch: <span id="pitch">0</span></p>
<p>Altitude: <span id="altitude">0</span></p>

</body>
</html>
)rawliteral";

// ============================================================
//  Route handlers
// ============================================================

void handleRoot()
{
    static char page[8000];

    snprintf(
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
        altitudeController.derivativeFilterAlpha);

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

    server.send(200, "text/plain", "Controller parameters updated");
}

void handleTelemetry()
{
    String json  = "{";
    json += "\"roll\":"     + String(roll,     2) + ",";
    json += "\"pitch\":"    + String(pitch,    2) + ",";
    json += "\"altitude\":" + String(altitude, 2);
    json += "}";
    server.send(200, "application/json", json);
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

    WiFi.begin(ssid, password);
    WiFi.setSleep(false);
    esp_wifi_set_ps(WIFI_PS_NONE);

    while (WiFi.status() != WL_CONNECTED)
    {
        delay(500);
        Serial.print(".");
    }

    Serial.println("\nWiFi connected!");
    Serial.print("ESP32 IP address: ");
    Serial.println(WiFi.localIP());

    server.on("/",          handleRoot);
    server.on("/update",    handleUpdate);
    server.on("/telemetry", handleTelemetry);

    server.begin();
    Serial.println("Web server started");
}
