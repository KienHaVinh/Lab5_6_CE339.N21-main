import numpy as np
import pandas as pd
from sklearn.model_selection import train_test_split
from sklearn.ensemble import RandomForestClassifier
from sklearn.metrics import accuracy_score
from pymongo import MongoClient
import paho.mqtt.client as paho
from paho import mqtt

# MongoDB connection URI
uri = "mongodb+srv://antgroup0905:0905168232@cluster0.jph72eg.mongodb.net/test?retryWrites=true&w=majority"

# MQTT broker configuration 8883
broker_address = "c09422c3110542c394f85fd6edf9bcf5.s2.eu.hivemq.cloud"
client_mqtt = paho.Client(client_id="", userdata=None, protocol=paho.MQTTv5)
client_mqtt.tls_set(tls_version=mqtt.client.ssl.PROTOCOL_TLS)
client_mqtt.username_pw_set(username="20520597", password="Brs13125")
client_mqtt.connect(broker_address, port=8883)
mqtt_topic = "20520597/predict" 


# Connect to MongoDB
client = MongoClient(uri)
db = client.sensor
fire_alert = db.fire_alert

# Initialize temperature and humidity
temperature = 0
gas = 0
fireOccurrence = 0

# Load data from MongoDB to a dictionary df
# Assuming the dataset is stored in a pandas DataFrame named 'df'
# Fetch data from MongoDB collection
data = fire_alert.find()

# Convert MongoDB cursor to a list of dictionaries
data_list = list(data)

# Create a pandas DataFrame from the list of dictionaries
df = pd.DataFrame(data_list)


# The DataFrame should have 'temperature', 'gas', and 'fire_occur' columns
# Split the data into features (X) and target variable (y)
X = df[['temperature', 'gas']]
y = df['fire_occur']

# Split the data into training and testing sets
X_train, X_test, y_train, y_test = train_test_split(X, y, test_size=0.2, random_state=42)

# Create a Random Forest classifier
clf = RandomForestClassifier()

# Train the model
clf.fit(X_train, y_train)

# Make predictions on the test set
y_pred = clf.predict(X_test)

# Check if there is a fire occurrence in the predictions
if 0 in y_pred:
    # Publish a message to the MQTT topic
    client_mqtt.publish("20520597/predict", "Fire Occurrence Predicted")

# Calculate accuracy
accuracy = accuracy_score(y_test, y_pred)
print("Accuracy:", accuracy)