from influxdb_client import InfluxDBClient

# Connection details
url = "http://172.22.1.1:8086"
token = "mhKc7QAulq4ShrUeaIJ4eu17YeHjyS8z8Ln0K62TBgloQauED2-70Tg7nWjxnimGorY4NkYciPwKeeDOXp0-FA=="
org = "Capstone" # "50ffbc921dc3c6c5"
bucket = "telegraf"

# Create a client
client = InfluxDBClient(url=url, token=token, org=org)

# Define the query you want to run
query = f'''from(bucket: "telegraf")
  |> range(start: v.timeRangeStart, stop: v.timeRangeStop)
  |> filter(fn: (r) => r["_measurement"] == "mqtt_consumer")
  |> filter(fn: (r) => r["topic"] == "robot/motor")
  |> filter(fn: (r) => r["_field"] == "q_current" or r["_field"] == "d_current")
  |> yield(name: "mean")'''

# Query the data
query_api = client.query_api()
result = query_api.query(query)

# Process the results
for table in result:
    for record in table.records:
        print(f'Time: {record.get_time()} Value: {record.get_value()}')
