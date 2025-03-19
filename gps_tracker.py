import serial
import requests

# GPS modülüne bağlan (örneğin: /dev/ttyUSB0 veya COM3)
#/dev/ttyUSB0 Linux ve macOS işletim sistemlerinde kullanılıyor.
#COM3 Windows işletim sistemi için kullanılıyor.


# GPS modülüne bağlan
# Baudrate: 9600 (GPS modülünün baudrate'ine göre değişebilir.)
gps = serial.Serial("COM3", baudrate=9600, timeout=1)

# Sunucu adresi
server_url = "http://yourserver.com/update_location"

def parse_gps(data):
    """Gelen GPS NMEA verisini ayristir."""
    if data.startswith("$GPGGA"):  # Sadece GPGGA verisini kullanıyoruz
        parts = data.split(",")
        if len(parts) > 6 and parts[2] and parts[4]:
            lat = float(parts[2]) / 100.0
            lon = float(parts[4]) / 100.0
            return lat, lon
    return None

while True:
    line = gps.readline().decode("utf-8").strip()
    if line:
        location = parse_gps(line)
        if location:
            lat, lon = location
            print(f"Konum: {lat}, {lon}")

            # Sunucuya veriyi gönder
            data = {"latitude": lat, "longitude": lon}
            response = requests.post(server_url, json=data)
            print(f"Sunucu Yaniti: {response.text}")
