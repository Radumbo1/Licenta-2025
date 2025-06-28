import firebase_admin
from firebase_admin import credentials, db, firestore
from influxdb_client import InfluxDBClient
import os

# Initialize Firebase
cred = credentials.Certificate('art-tu-test-cloud-database-firebase-adminsdk-fbsvc-cf6cc644e1.json')  # Use the path to your Firebase private key
FIRESTORE_APP = firebase_admin.initialize_app(cred)
FIRESTORE_DB = firestore.client()
DEFAULT_COLLECTION_NAME = "sensor-data"
INFLUXDB_INIT_ADMIN_TOKEN = "f94eb75372110679c9fbd0ffa140fded87140bdc2b911c9641d339669f8efee3"
INFLUXDB_INIT_ORG = "telemetryARTTU"

def upload_to_firebase(data: dict):
    FIRESTORE_DB.collection(DEFAULT_COLLECTION_NAME).document().set(data)


# InfluxDB Configuration (replace with your own values)
influx_url = "http://influxdb:8086"  # InfluxDB URL (Telegraf sends data to this)
influx_bucket = "telemetryDATA"  # The InfluxDB bucket where data is stored

# Connect to InfluxDB
influx_client = InfluxDBClient(url=influx_url, token=INFLUXDB_INIT_ADMIN_TOKEN, org=INFLUXDB_INIT_ORG)
influx_query_api = influx_client.query_api()

# Example query: Get data from the last 1 hour (adjust based on your needs)
query = f'from(bucket: "{influx_bucket}") |> range(start: -1h)'

# Query the data from InfluxDB
result = influx_query_api.query(org=INFLUXDB_INIT_ORG, query=query)

# Process the result and upload to Firebase
for table in result:
    for record in table.records:
        data = {
            'timestamp': record.get_time(),
            'metric_name': record.get_field(),
            'value': record.get_value(),
        }
        upload_to_firebase(data)  # Upload each record to Firebase
