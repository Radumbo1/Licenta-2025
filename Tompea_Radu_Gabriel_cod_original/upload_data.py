import firebase_admin
from firebase_admin import credentials, db, firestore
from influxdb_client import InfluxDBClient
import os
import tkinter as tk
from tkinter import messagebox
import json
from datetime import datetime
import csv
from collections import defaultdict


# initialize Firebase
cred = credentials.Certificate('art-tu-test-cloud-database-firebase-adminsdk-fbsvc-cf6cc644e1.json')
FIRESTORE_APP = firebase_admin.initialize_app(cred)
FIRESTORE_DB = firestore.client()
DEFAULT_COLLECTION_NAME = "sensor-data"
INFLUXDB_INIT_ADMIN_TOKEN = "f94eb75372110679c9fbd0ffa140fded87140bdc2b911c9641d339669f8efee3"
INFLUXDB_INIT_ORG = "telemetryARTTU"

def upload_to_firebase(data: dict):
    FIRESTORE_DB.collection(DEFAULT_COLLECTION_NAME).document().set(data)

# InfluxDB config
influx_url = "http://localhost:8086"
influx_bucket = "telemetryDATA"

# connect to InfluxDB
influx_client = InfluxDBClient(url=influx_url, token=INFLUXDB_INIT_ADMIN_TOKEN, org=INFLUXDB_INIT_ORG)
influx_query_api = influx_client.query_api()

def wait_for_user():
    def on_upload():
        print("Querying InfluxDB for data...")
        query = f'from(bucket: "{influx_bucket}") |> range(start: -1h)'
        print(f"Executing query: {query}")
        result = influx_query_api.query(org=INFLUXDB_INIT_ORG, query=query)
        print(f"result: {result}")

        # group by (timestamp, measurement)
        grouped = defaultdict(dict)
        for table in result:
            for record in table.records:
                ts = str(record.get_time())
                measurement = record.get_measurement()
                key = (ts, measurement)
                grouped[key]['timestamp'] = ts
                grouped[key]['measurement'] = measurement
                grouped[key][record.get_field()] = record.get_value()

        upload_count = 0
        for doc in grouped.values():
            print(f"Uploading grouped data: {doc}")
            upload_to_firebase(doc)
            upload_count += 1

        messagebox.showinfo("Upload", f"Uploaded {upload_count} grouped records to Firestore!")

    def on_download():
        # download and aggregate all data from Firestore
        docs = FIRESTORE_DB.collection(DEFAULT_COLLECTION_NAME).stream()
        all_data = [doc.to_dict() for doc in docs]

        # ensure output directory exists
        output_dir = "./Cloud_Data"
        os.makedirs(output_dir, exist_ok=True)

        # save aggregated data to a CSV file with timestamp
        filename = f"cloud_data_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"
        filepath = os.path.join(output_dir, filename)

        if all_data:
            # save aggregated data to a JSON file with timestamp
            json_filename = f"cloud_data_{datetime.now().strftime('%Y%m%d_%H%M%S')}.json"
            json_filepath = os.path.join(output_dir, json_filename)
            with open(json_filepath, "w", encoding="utf-8") as f:
                json.dump(all_data, f, ensure_ascii=False, indent=2, default=str)
            messagebox.showinfo("Download Complete", f"Data downloaded and saved to:\n{json_filepath}")
        else:
            messagebox.showinfo("No Data", "No data found in Firestore to save.")

    root = tk.Tk()
    root.title("Cloud Data Actions")
    tk.Label(root, text="Press a button to download or upload data from cloud.").pack(padx=20, pady=10)
    tk.Button(root, text="Download Data", command=on_download).pack(pady=5)
    tk.Button(root, text="Upload Data", command=on_upload).pack(pady=5)
    root.mainloop()

wait_for_user()