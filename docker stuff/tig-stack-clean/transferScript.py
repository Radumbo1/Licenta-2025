import serial
import os
from influxdb_client import InfluxDBClient, Point
from dotenv import load_dotenv

# Load .env file
load_dotenv()

# Read values from .env
INFLUXDB_URL = f"http://{os.getenv('DOCKER_INFLUXDB_INIT_HOST')}:{os.getenv('DOCKER_INFLUXDB_INIT_PORT')}"
INFLUXDB_TOKEN = os.getenv("DOCKER_INFLUXDB_INIT_ADMIN_TOKEN")
INFLUXDB_ORG = os.getenv("DOCKER_INFLUXDB_INIT_ORG")
INFLUXDB_BUCKET = os.getenv("DOCKER_INFLUXDB_INIT_BUCKET")

# Serial port configuration (Windows)
ser = serial.Serial('COM10', 9600)  # Use the COM port from Windows directly (e.g., COM6)
# One pop-up window for searcing and displaying the serial ports and what to use?
# Initialize InfluxDB client
client = InfluxDBClient(url=INFLUXDB_URL, token=INFLUXDB_TOKEN, org=INFLUXDB_ORG)
write_api = client.write_api()

start_influx = False

try:
    while True:
        line = ser.readline().decode('utf-8').strip()
        #print(line)
        if line:
            if line == "START INFLUX":
                start_influx = True
                print("Received START INFLUX")
            elif start_influx:
                write_api.write(bucket=INFLUXDB_BUCKET, org=INFLUXDB_ORG, record=line)
                print(f"Sent to InfluxDB: {line}")
                print(f"URL: {INFLUXDB_URL} | Org: {INFLUXDB_ORG} | Bucket: {INFLUXDB_BUCKET}")
                start_influx = False
except KeyboardInterrupt:
    print("Exiting...")
finally:
    ser.close()
