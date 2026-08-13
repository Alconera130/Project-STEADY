#include <Arduino.h>
#include <NimBLEDevice.h>
#include <TinyGPSPlus.h>
#include <Preferences.h>

#include "config.h"
#include "gps.h"

HardwareSerial GPSSerial(1);

TinyGPSPlus gps;
Preferences preferences;

NimBLECharacteristic *gpsCharacteristic = nullptr;

bool deviceConnected = false;

double lastLatitude = 0;
double lastLongitude = 0;
double lastAltitude = 0;

uint32_t lastSatellites = 0;
uint32_t lastFixTime = 0;

unsigned long lastBLEUpdate = 0;

class ServerCallbacks : public NimBLEServerCallbacks {
    void onConnect(NimBLEServer *server, NimBLEConnInfo &connInfo) override {
        deviceConnected = true;
    }

    void onDisconnect(NimBLEServer *server, NimBLEConnInfo &connInfo, int reason) override {
        deviceConnected = false;
        NimBLEDevice::startAdvertising();
    }
};

void sendLastLocation() {
    char message[160];

    snprintf(
        message,
        sizeof(message),
        "LAST,%.7f,%.7f,%.1f,%lu",
        lastLatitude,
        lastLongitude,
        lastAltitude,
        (unsigned long)lastFixTime
    );

    gpsCharacteristic->setValue(message);

    if (deviceConnected) gpsCharacteristic->notify();
}

void sendStatus() {
    char message[100];

    snprintf(
        message,
        sizeof(message),
        "STATUS,%d,%lu",
        gps.location.isValid() ? 1 : 0,
        (unsigned long)lastSatellites
    );

    gpsCharacteristic->setValue(message);

    if (deviceConnected) gpsCharacteristic->notify();
}

class GPSCallbacks : public NimBLECharacteristicCallbacks {
    void onWrite(
        NimBLECharacteristic *characteristic,
        NimBLEConnInfo &connInfo
    ) override {
        std::string value =
        characteristic->getValue();

        if (value == "GET_LAST") sendLastLocation();
        if (value == "GET_STATUS") sendStatus();
    }
};

void saveLocation() {
    preferences.putDouble("lat", lastLatitude);
    preferences.putDouble("lon", lastLongitude);
    preferences.putDouble("alt", lastAltitude);

    preferences.putUInt("sats", lastSatellites);
    preferences.putUInt("time", lastFixTime);
}

void loadLocation() {
    lastLatitude = preferences.getDouble("lat", 0);
    lastLongitude = preferences.getDouble("lon", 0);
    lastAltitude = preferences.getDouble("alt", 0);
    lastSatellites = preferences.getUInt("sats", 0);
    lastFixTime = preferences.getUInt("time", 0);
}

void gpsBegin() {
    GPSSerial.begin(GPS_BAUD, SERIAL_8N1, GPS_RX, GPS_TX);
    preferences.begin("wallet", false);

    loadLocation();

    NimBLEDevice::init(BLE_DEVICE_NAME);
    NimBLEServer *server = NimBLEDevice::createServer();

    server->setCallbacks(new ServerCallbacks());

    NimBLEService *service = server->createService(BLE_SERVICE_UUID);

    gpsCharacteristic = service->createCharacteristic(
        BLE_GPS_UUID,
        NIMBLE_PROPERTY::READ |
        NIMBLE_PROPERTY::WRITE |
        NIMBLE_PROPERTY::NOTIFY
    );

    gpsCharacteristic->setCallbacks(new GPSCallbacks());

    service->start();

    NimBLEAdvertising *advertising = NimBLEDevice::getAdvertising();

    advertising->addServiceUUID(BLE_SERVICE_UUID);
    advertising->setName(BLE_DEVICE_NAME);

    NimBLEDevice::startAdvertising();

    Serial.println("GPS/BLE READY");
}

void gpsUpdate() {
    while (GPSSerial.available()) gps.encode(GPSSerial.read());

    if (gps.location.isUpdated() && gps.location.isValid()) {
        lastLatitude = gps.location.lat();
        lastLongitude = gps.location.lng();

        if (gps.altitude.isValid()) lastAltitude = gps.altitude.meters();
        if (gps.satellites.isValid()) lastSatellites = gps.satellites.value();

        lastFixTime = millis() / 1000;

        saveLocation();

        Serial.print("GPS: ");
        Serial.print(lastLatitude, 7);
        Serial.print(",");
        Serial.println(lastLongitude, 7);
    }

    if (deviceConnected && millis() - lastBLEUpdate >= 2000) {
        lastBLEUpdate = millis();

        sendLastLocation();
    }
}