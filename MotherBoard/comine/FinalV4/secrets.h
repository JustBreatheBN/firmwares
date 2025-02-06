#include <pgmspace.h>
 
#define SECRET

const char AWS_IOT_ENDPOINT[] = "a1bjx53wzzzgiu-ats.iot.ap-south-1.amazonaws.com";       //End Point Name

// Amazon Root CA 1
static const char AWS_CERT_CA[] PROGMEM = R"EOF(
-----BEGIN CERTIFICATE-----
MIIDQTCCAimgAwIBAgITBmyfz5m/jAo54vB4ikPmljZbyjANBgkqhkiG9w0BAQsF
ADA5MQswCQYDVQQGEwJVUzEPMA0GA1UEChMGQW1hem9uMRkwFwYDVQQDExBBbWF6
b24gUm9vdCBDQSAxMB4XDTE1MDUyNjAwMDAwMFoXDTM4MDExNzAwMDAwMFowOTEL
MAkGA1UEBhMCVVMxDzANBgNVBAoTBkFtYXpvbjEZMBcGA1UEAxMQQW1hem9uIFJv
b3QgQ0EgMTCCASIwDQYJKoZIhvcNAQEBBQADggEPADCCAQoCggEBALJ4gHHKeNXj
ca9HgFB0fW7Y14h29Jlo91ghYPl0hAEvrAIthtOgQ3pOsqTQNroBvo3bSMgHFzZM
9O6II8c+6zf1tRn4SWiw3te5djgdYZ6k/oI2peVKVuRF4fn9tBb6dNqcmzU5L/qw
IFAGbHrQgLKm+a/sRxmPUDgH3KKHOVj4utWp+UhnMJbulHheb4mjUcAwhmahRWa6
VOujw5H5SNz/0egwLX0tdHA114gk957EWW67c4cX8jJGKLhD+rcdqsq08p8kDi1L
93FcXmn/6pUCyziKrlA4b9v7LWIbxcceVOF34GfID5yHI9Y/QCB/IIDEgEw+OyQm
jgSubJrIqg0CAwEAAaNCMEAwDwYDVR0TAQH/BAUwAwEB/zAOBgNVHQ8BAf8EBAMC
AYYwHQYDVR0OBBYEFIQYzIU07LwMlJQuCFmcx7IQTgoIMA0GCSqGSIb3DQEBCwUA
A4IBAQCY8jdaQZChGsV2USggNiMOruYou6r4lK5IpDB/G/wkjUu0yKGX9rbxenDI
U5PMCCjjmCXPI6T53iHTfIUJrU6adTrCC2qJeHZERxhlbI1Bjjt/msv0tadQ1wUs
N+gDS63pYaACbvXy8MWy7Vu33PqUXHeeE6V/Uq2V8viTO96LXFvKWlJbYK8U90vv
o/ufQJVtMVT8QtPHRh8jrdkPSHCa2XV4cdFyQzR1bldZwgJcJmApzyMZFo6IQ6XU
5MsI+yMRQ+hDKXJioaldXgjUkK642M4UwtBV8ob2xJNDd2ZhwLnoQdeXeGADbkpy
rqXRfboQnoZsG4q5WTP468SQvvG5
-----END CERTIFICATE-----
)EOF";
 
// Device Certificate                                               //change this
static const char AWS_CERT_CRT[] PROGMEM = R"KEY(
-----BEGIN CERTIFICATE-----
MIIDWjCCAkKgAwIBAgIVAL7GUdIV6TzMeJ61Phntwvv0vc/dMA0GCSqGSIb3DQEB
CwUAME0xSzBJBgNVBAsMQkFtYXpvbiBXZWIgU2VydmljZXMgTz1BbWF6b24uY29t
IEluYy4gTD1TZWF0dGxlIFNUPVdhc2hpbmd0b24gQz1VUzAeFw0yNDA4MTUwNTIx
NTVaFw00OTEyMzEyMzU5NTlaMB4xHDAaBgNVBAMME0FXUyBJb1QgQ2VydGlmaWNh
dGUwggEiMA0GCSqGSIb3DQEBAQUAA4IBDwAwggEKAoIBAQDH/zZw0oNZNwwZFwkK
BdyDXIEtTaEky09EY/PB83cyPOKFlqYoA+1RaPuzvPjDnBVwsWnYzAULk2ri78/Y
NmdhhR0z9Zwd0w5+/ukPpAtsmYvLPx7pqJvZoC5Mk4MyeBltnwq/Xj+FxLnLaq9G
n2gAPqX5R0NVxhX4xoV9MidW0REnAR88aPx5Z1/Yh161LXzo3X6MBm89R+9/IWLv
zBtFx8Ynl0mi4lbBKyN8eHZft28PGihVwOK58lQlxkrUGYs0uaJB0C8g1hYVZyay
7B7er3nQeq61Lv9ePgtJvoDFKY9LYcYhtL9TgHjJM7JtGhihnUqdkNeczEoRZAgA
f7h1AgMBAAGjYDBeMB8GA1UdIwQYMBaAFCw+Wx9kdq5+jZssUN0JR6C0+IpZMB0G
A1UdDgQWBBSLSnvtZrhUkYhH0AmFf4kMDykZpTAMBgNVHRMBAf8EAjAAMA4GA1Ud
DwEB/wQEAwIHgDANBgkqhkiG9w0BAQsFAAOCAQEANY2LQB2vCgkk/d3zKTDsAZ0K
rk91yZYQvYALqe+3scoBConFFgRBEfA1t6aNlbgrgSCMzGvID5snCo4kItBqrGw8
TRLBkRAeR3g2mjPIF9ePDFEvHT3TrUsv+tKrKs04G7j9tIKxGG3jXnSWQh+tqAkm
lBzT7AU9o3Y8ys/yjGGlykdn1l5w6re1u1vffq+9bI4gKWReAxZLvygPQor5jD69
bQsuoITlvk6dXIz/3rPBLSV17pTOeZtXHG4TKUMKwBhZ1CEj7bPlOMrZTHkWlvm/
MItVwMm2TmJIYmLVRUEZeyHOsukZEl1xtsdtlzHR8Wqx5Mag0he8Ij1Rgbh/kA==
-----END CERTIFICATE-----
 
 
)KEY";
 
// Device Private Key                                               //change this
static const char AWS_CERT_PRIVATE[] PROGMEM = R"KEY(
-----BEGIN RSA PRIVATE KEY-----
MIIEpAIBAAKCAQEAx/82cNKDWTcMGRcJCgXcg1yBLU2hJMtPRGPzwfN3MjzihZam
KAPtUWj7s7z4w5wVcLFp2MwFC5Nq4u/P2DZnYYUdM/WcHdMOfv7pD6QLbJmLyz8e
6aib2aAuTJODMngZbZ8Kv14/hcS5y2qvRp9oAD6l+UdDVcYV+MaFfTInVtERJwEf
PGj8eWdf2IdetS186N1+jAZvPUfvfyFi78wbRcfGJ5dJouJWwSsjfHh2X7dvDxoo
VcDiufJUJcZK1BmLNLmiQdAvINYWFWcmsuwe3q950HqutS7/Xj4LSb6AxSmPS2HG
IbS/U4B4yTOybRoYoZ1KnZDXnMxKEWQIAH+4dQIDAQABAoIBAQCMHf6rPqFtcMaW
EXBwyDh9A+MwtTGP0KzhUlGl8Yfx93wQccJJSK8MmUu22k9C5GiVAocQr6ddO9Pn
2HG6nisBxLv3+jx9HLAMQ1VPcvPaCx1IsRlcRJej+bhWSAbj5gaSqZljB4njkzdE
D6NrDCVV/o5TvhyrzFL6b0N4uhmblTFBVrXQTPwAsSKD078EtsWDrs6QhvEvSXk4
VGI4DGKudhpv0ob+LTyl6cO1RX1q307FwkgWvluDhpBi4h6Fjd4JYH0w72ZSLf0A
xn9jhX7YiBmgoYbNZEoJsBe4KZ2Ip/UiiSsszE3/h4A6oEYjXY2t+O9Ub0zLjsJg
mA6GYqVFAoGBAPiwrzKxAYThuHByHduTZm5qooj0SZ8ECN1Qzf7+nU1JhFs/bm4G
O2ldyCetv2Dm9tRzycuTPIGOlX6SZUpIzl9wpggxfHQ3TmabsI0YtacoOd0HS6Vi
nV9bZYLtpPybxxNaDkWqND0Wro+6Hj5+nMhFFIXndq2KtKiAM285tKvTAoGBAM3g
IHlPRIpYRECDTwHPjnmXOTwxlH4pbjb8bsCCk+9WyefBG94wgJ5utU7r7SwPygXh
G6QdfYl23asWdk87tkpGGEKA4eNCUBi0G8XBPImN4vyNcPE1eD4tduF3X41TiGvQ
xkw9uF76mjCjAU9QR7/ob/YTLpd9tTk7AEMOV8WXAoGADjQRHYUMgClgHlP2LpdB
j2bZt5a0KbfSRcmaqkiidUqQm6GIzhHfDMqFZva+amtmh4XBbbYQC5o4IY186Xw1
BTBq2ybwY7bQ7H1R3Q0fOtfKTjxfbqeuM0cGCi9GcAL5oLpQ/FuXNjH4cE8B8THM
BIglfuxUU98LKGCCD4UtM28CgYEAq6AOMlL8hoAy1b0hXhcTZRXGJwIGEm6jbL9f
7KhufvXQ95yqiL3SW0FbcGGJpFO9TM1uZ5AWGUS4YmolUsQqoRxyGO6sYPHlJ9Lt
3BvNRba85WFcKBk1FlB8bFVBmOVsMsUmqmRBkRxE8grRircYOUmiHHe9PZkUPJON
Elmgnh0CgYAQd17DuuUDKGXT0vnCR/DNJAki+vwuyhdqx1ewvNJo/VH0Xy3KjRnm
BpDhwaXZPltEr8k2rWLekh+DReEXF1byaq7sesm8B65DSyZ2XVacDNjR4dHeih1u
kDFpNVc/smfyGh6GLo3AmeSANVDBSIGwSWZByYSej8d0QVJeCNcRiw==
-----END RSA PRIVATE KEY-----
 
)KEY";


const char index_html[] PROGMEM = R"rawliteral(

  <!DOCTYPE html>
<html lang="en">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>Just Breathe Configuration</title>
    <style>
        body {
            font-family: Arial, sans-serif;
            display: flex;
            justify-content: center;
            align-items: center;
            height: 100vh;
            background-color: #f0f0f0;
            margin: 0;
            flex-direction: column;
        }
        .container {
            background-color: #fff;
            padding: 20px;
            border-radius: 10px;
            box-shadow: 0 0 10px rgba(0, 0, 0, 0.1);
            max-width: 400px;
            width: calc(100% - 40px);
            box-sizing: border-box;
            margin-bottom: 20px;
        }
        h2 {
            margin-bottom: 20px;
            font-size: 24px;
            text-align: center;
        }
        label {
            display: block;
            margin-bottom: 5px;
            font-weight: bold;
        }
        input[type="text"], input[type="password"], textarea, select {
            width: 100%;
            padding: 10px;
            margin-bottom: 20px;
            border: 1px solid #ccc;
            border-radius: 5px;
            box-sizing: border-box;
        }
        input[type="submit"], input[type="button"], input[type="button"].connect-btn {
            background-color: #007bff;
            color: white;
            padding: 10px;
            border: none;
            border-radius: 5px;
            cursor: pointer;
            width: 100%;
            font-size: 16px;
            box-sizing: border-box;
            margin-bottom: 10px;
        }
        input[type="submit"]:hover, input[type="button"]:hover {
            background-color: #0056b3;
        }
        .status {
            margin-top: 10px;
            text-align: center;
            font-size: 18px;
        }
        .disabled {
            opacity: 0.6;
            pointer-events: none;
        }
        .aws-connected {
    color: green;
}

.aws-disconnected {
    color: red;
}
.connected {
    color: green;
}

.disconnected {
    color: red;
}

.disabled {
    opacity: 0.6;
    pointer-events: none;
}
    </style>
</head>
<body>
    <div class="container" id="wifi-card">
    <h2>Wi-Fi Configuration</h2>
    <form id="wifi-form">
        <label for="ssid">Select Wi-Fi Network:</label>
        <select id="ssid" name="ssid"></select>
        <label for="password">Wi-Fi Password:</label>
        <input type="password" id="password" name="password">
        <div id="wifi-status" class="status disconnected">Wi-Fi Status: Not Connected</div>
        <input type="button" value="Connect to Wi-Fi" class="connect-btn" onclick="connectToWiFi()">
    </form>
</div>

<div class="container" id="aws-card">
    <div id="aws-status" class="status disconnected">AWS Status: Not Connected</div>
</div>

<div class="container disabled" id="config-card">
    <h2>Device Configuration</h2>
    <form id="config-form">
        <label for="device-id">Device ID:</label>
        <input type="text" id="device-id" name="device-id">
        <input type="submit" value="Save Configuration">
    </form>
    <input type="button" value="Reset Configuration" onclick="resetConfig()">
</div>
    <script>
        document.addEventListener('DOMContentLoaded', function() {
            fetch('/wifi-networks')
                .then(response => response.json())
                .then(data => {
                    const ssidSelect = document.getElementById('ssid');
                    data.networks.forEach(network => {
                        const option = document.createElement('option');
                        option.value = network.ssid;
                        option.textContent = `${network.ssid} (${network.rssi} dBm)`;
                        ssidSelect.appendChild(option);
                    });
                });
        });

        function connectToWiFi() {
            const formData = new FormData(document.getElementById('wifi-form'));

            fetch('/save-config', {
                method: 'POST',
                body: formData
            })
            .then(response => response.json())
            .then(data => {
                alert(data.message);
                // Start checking Wi-Fi connection status
                checkWifiStatus();
            })
            .catch(error => {
                console.error('Error:', error);
                alert('Failed to connect to Wi-Fi.');
            });
        }

function checkWifiStatus() {
    fetch('/wifi-status')
        .then(response => response.json())
        .then(data => {
            const wifiStatusDiv = document.getElementById('wifi-status');
            if (data.connected) {
                wifiStatusDiv.textContent = `Wi-Fi Status: Connected (IP: ${data.ip})`;
                wifiStatusDiv.classList.add('connected');
                wifiStatusDiv.classList.remove('disconnected');
                enableConfigCard();  // Enable the configuration card
            } else {
                wifiStatusDiv.textContent = "Wi-Fi Status: Not Connected";
                wifiStatusDiv.classList.add('disconnected');
                wifiStatusDiv.classList.remove('connected');
                setTimeout(checkWifiStatus, 5000);  // Retry after 5 seconds
            }
        })
        .catch(error => {
            console.error('Error checking Wi-Fi status:', error);
            setTimeout(checkWifiStatus, 5000);  // Retry after 5 seconds
        });
}


function enableConfigCard() {
    const configCard = document.getElementById('config-card');
    configCard.classList.remove('disabled');
}

        function fetchDevices() {
            // Fetch devices or any required data from API once connected to Wi-Fi
            fetch('/fetch-devices')
                .then(response => response.json())
                .then(data => {
                    // Handle device data here (populate a select box or any other UI element if needed)
                })
                .catch(error => {
                    console.error('Error fetching devices:', error);
                });
        }

        document.getElementById('config-form').addEventListener('submit', function(event) {
    event.preventDefault();  // Prevent the default form submission
    const formData = new FormData(this);

    fetch('/save-device-config', {
        method: 'POST',
        body: formData
    })
    .then(response => response.json())
    .then(data => {
        alert(data.message);
        if (data.message.includes('connected to AWS')) {
            alert('Device successfully connected to AWS IoT. Closing configuration page.');
            window.close();  // Close the page if AWS is connected
        } else {
            alert('Configuration saved, but AWS connection failed.');
        }
    })
    .catch(error => {
        console.error('Error:', error);
        alert('Failed to save device configuration.');
    });
});

        function resetConfig() {
            fetch('/reset-config', { method: 'POST' })
                .then(response => response.json())
                .then(data => {
                    alert(data.message);
                })
                .catch(error => {
                    console.error('Error:', error);
                    alert('Failed to reset configuration.');
                });
        }
//function checkWifiStatus() {
//    fetch('/wifi-status')
//        .then(response => response.json())
//        .then(data => {
//            const wifiStatusDiv = document.getElementById('wifi-status');
//            if (data.connected) {
//                wifiStatusDiv.textContent = `Wi-Fi Status: Connected (IP: ${data.ip})`;
//                wifiStatusDiv.classList.add('connected');  // Add connected class
//                enableConfigCard();  // Enable the configuration card
//            } else {
//                wifiStatusDiv.textContent = "Wi-Fi Status: Not Connected";
//                wifiStatusDiv.classList.remove('connected');
//                setTimeout(checkWifiStatus, 5000);  // Retry after 5 seconds
//            }
//        })
//        .catch(error => {
//            console.error('Error checking Wi-Fi status:', error);
//            setTimeout(checkWifiStatus, 5000);  // Retry after 5 seconds
//        });
//}
//
//function enableConfigCard() {
//    const configCard = document.getElementById('config-card');
//    configCard.classList.remove('disabled');  // Remove the 'disabled' class to enable the card
//}

//document.addEventListener('DOMContentLoaded', function() {
//    checkWifiStatus();  // Start checking Wi-Fi status when the page loads
//});


function checkAwsStatus() {
    fetch('/aws-status')
        .then(response => response.json())
        .then(data => {
            const awsStatusDiv = document.getElementById('aws-status');
            if (data.connected) {
                awsStatusDiv.textContent = "AWS Status: Connected";
                awsStatusDiv.classList.add('connected');
                awsStatusDiv.classList.remove('disconnected');
            } else {
                awsStatusDiv.textContent = "AWS Status: Not Connected";
                awsStatusDiv.classList.add('disconnected');
                awsStatusDiv.classList.remove('connected');
                setTimeout(checkAwsStatus, 5000);  // Retry after 5 seconds
            }
        })
        .catch(error => {
            console.error('Error checking AWS status:', error);
            setTimeout(checkAwsStatus, 5000);  // Retry after 5 seconds
        });
}

document.addEventListener('DOMContentLoaded', function() {
    checkWifiStatus();  // Start checking Wi-Fi status when the page loads
    checkAwsStatus();   // Start checking AWS status when the page loads
});



    
    </script>
</body>
</html>

)rawliteral";
