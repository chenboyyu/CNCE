#include <Wire.h>
#include <MPU6050.h>
#include <Adafruit_BMP085.h>
#include <WiFi.h>
#include <WiFiUdp.h>

#define I2C_SDA 8
#define I2C_SCL 9

// ===== WiFi 設定 =====
const char* ssid     = "AIR";
const char* password = "00000000";

char   gcs_ip[16] = "192.168.137.1";
const int   gcs_port = 8888;
const int local_port = 8889;

#define BUILTIN_LED 2

MPU6050 mpu;
Adafruit_BMP085 bmp;
WiFiUDP udp;

unsigned long lastSend = 0;
const int sendInterval = 50;

// 歸零
float zero_pitch = 0, zero_roll = 0, zero_yaw = 0;

// 互補濾波
float filt_pitch = 0, filt_roll = 0;
float alpha = 0.96;
bool  first_run = true;

// 磁力計 raw
int16_t mag_x = 0, mag_y = 0, mag_z = 0;
bool mag_ok = false;
bool mag_is_qmc = false;
uint8_t mag_addr = 0;

// BMP180
bool bmp_ok = false;
float temperature = 0, pressure = 0;

// ==============================================================

void i2c_write_byte(uint8_t dev, uint8_t reg, uint8_t val) {
  Wire.beginTransmission(dev);
  Wire.write(reg);
  Wire.write(val);
  Wire.endTransmission();
}

uint8_t i2c_read_byte(uint8_t dev, uint8_t reg) {
  Wire.beginTransmission(dev);
  Wire.write(reg);
  Wire.endTransmission(false);
  Wire.requestFrom(dev, (uint8_t)1);
  if (Wire.available()) return Wire.read();
  return 0;
}

// ==============================================================

float calc_yaw(float pitch_deg, float roll_deg) {
  if (!mag_ok) return 0;
  float p = pitch_deg * PI / 180.0;
  float r = roll_deg  * PI / 180.0;

  float mx = mag_x * cos(p) + mag_z * sin(p);
  float my = mag_x * sin(r) * sin(p) + mag_y * cos(r) - mag_z * sin(r) * cos(p);

  float yaw = atan2(-my, mx) * 180.0 / PI;
  if (yaw < 0) yaw += 360;
  return yaw;
}

void calc_attitude(float ax, float ay, float az, float &pitch, float &roll) {
  pitch = atan2(-ax, sqrt(ay * ay + az * az)) * 180.0 / PI;
  roll  = atan2(ay, az) * 180.0 / PI;
}

// ==============================================================

void i2c_scan() {
  Serial.print("I2C scan:");
  for (uint8_t addr = 1; addr < 127; addr++) {
    Wire.beginTransmission(addr);
    if (Wire.endTransmission() == 0) {
      Serial.print(" 0x"); Serial.print(addr, HEX);
    }
  }
  Serial.println();
}

void scan_aux_bus() {
  // 使用 MPU6050 I2C Master 掃描 AUX 匯流排
  i2c_write_byte(0x68, 0x37, 0x00); // 關 bypass
  i2c_write_byte(0x68, 0x6A, 0x20); // I2C_MST_EN
  i2c_write_byte(0x68, 0x23, 0x07); // I2C_MST_CTRL
  delay(10);

  Serial.print("AUX bus scan:");
  uint8_t addrs[] = {0x0C, 0x0D, 0x0E, 0x1C, 0x1D, 0x1E, 0x1F, 0x2C, 0x30, 0x3C};
  for (int i = 0; i < 10; i++) {
    uint8_t a = addrs[i];
    // 設 slave 0 從該位址讀 1 byte
    i2c_write_byte(0x68, 0x25, a | 0x80); // SLAVE0_ADDR + read
    i2c_write_byte(0x68, 0x26, 0x00);      // SLAVE0_REG = 0x00
    i2c_write_byte(0x68, 0x27, 0x81);      // SLAVE0_CTRL: 1 byte, enable
    delay(5);

    // 檢查 NACK 位元 (I2C_MST_STATUS bit 4)
    uint8_t status = i2c_read_byte(0x68, 0x36);
    if (!(status & 0x10)) {
      // 無 NACK → 裝置存在
      uint8_t data = i2c_read_byte(0x68, 0x49); // EXT_SENS_DATA_00
      Serial.print(" 0x"); Serial.print(a, HEX);
      if (data) { Serial.print("(0x"); Serial.print(data, HEX); Serial.print(")"); }
    }
  }
  // 恢復 bypass
  i2c_write_byte(0x68, 0x6A, 0x00);
  i2c_write_byte(0x68, 0x37, 0x02);
  Serial.println();
  delay(50);
}

void enable_bypass() {
  // 方法 A: 用函式庫
  mpu.setI2CBypassEnabled(true);
  delay(20);

  // 方法 B: 再補直刷確保生效
  i2c_write_byte(0x68, 0x6A, 0x00); // USER_CTRL: 關閉主模式
  i2c_write_byte(0x68, 0x37, 0x02); // INT_PIN_CFG: Bypass 開啟
  delay(10);

  // 驗證暫存器
  Serial.print("USER_CTRL(0x6A)=");
  Serial.print(i2c_read_byte(0x68, 0x6A), HEX);
  Serial.print(" INT_PIN_CFG(0x37)=");
  Serial.println(i2c_read_byte(0x68, 0x37), HEX);

  i2c_write_byte(0x68, 0x6B, 0x00); // PWR_MGMT_1: 喚醒
  delay(100);
}

void init_magnetometer() {
  Wire.beginTransmission(0x1E);
  if (Wire.endTransmission() == 0) {
    mag_addr = 0x1E;
    mag_is_qmc = false;
    i2c_write_byte(0x1E, 0x00, 0x18);
    delay(3);
    i2c_write_byte(0x1E, 0x01, 0x20);
    delay(3);
    i2c_write_byte(0x1E, 0x02, 0x00);
    delay(10);
    mag_ok = true;
    Serial.println("Magnetometer: HMC5883L found at 0x1E");
    return;
  }

  Wire.beginTransmission(0x0D);
  if (Wire.endTransmission() == 0) {
    mag_addr = 0x0D;
    mag_is_qmc = true;
    i2c_write_byte(0x0D, 0x09, 0x0D);
    delay(10);
    mag_ok = true;
    Serial.println("Magnetometer: QMC5883L found at 0x0D");
    return;
  }

  Wire.beginTransmission(0x2C);
  if (Wire.endTransmission() == 0) {
    mag_addr = 0x2C;
    mag_is_qmc = false;
    i2c_write_byte(0x2C, 0x00, 0x18);
    delay(3);
    i2c_write_byte(0x2C, 0x01, 0x20);
    delay(3);
    i2c_write_byte(0x2C, 0x02, 0x00);
    delay(10);
    mag_ok = true;
    Serial.println("Magnetometer: found at 0x2C (trying HMC5883L protocol)");
    return;
  }

  Serial.println("Magnetometer: none found");
}

void read_magnetometer() {
  if (mag_is_qmc) {
    Wire.beginTransmission(mag_addr);
    Wire.write(0x00);
    Wire.endTransmission(false);
    Wire.requestFrom(mag_addr, (uint8_t)6);
    if (Wire.available() >= 6) {
      int16_t xl = Wire.read(), xh = Wire.read();
      int16_t yl = Wire.read(), yh = Wire.read();
      int16_t zl = Wire.read(), zh = Wire.read();
      mag_x = (xh << 8) | xl;
      mag_y = (yh << 8) | yl;
      mag_z = (zh << 8) | zl;
    }
  } else {
    Wire.beginTransmission(mag_addr);
    Wire.write(0x03);
    Wire.endTransmission(false);
    Wire.requestFrom(mag_addr, (uint8_t)6);
    if (Wire.available() >= 6) {
      mag_x = (Wire.read() << 8) | Wire.read();
      mag_z = (Wire.read() << 8) | Wire.read();
      mag_y = (Wire.read() << 8) | Wire.read();
    }
  }
}

// ==============================================================

void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("\n=== Glider ESP32-S3 ===");

  pinMode(BUILTIN_LED, OUTPUT);
  digitalWrite(BUILTIN_LED, LOW);

  // 強制重置引腳狀態，避免 ESP32-S3 內建強上拉干擾
  pinMode(I2C_SDA, INPUT);
  pinMode(I2C_SCL, INPUT);

  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(100000); // 降速至 100kHz，確保老舊磁力計穩定

  i2c_scan(); // 第一次掃描（僅 MPU6050 + BMP180）

  // ---- MPU6050 ----
  mpu.initialize();
  if (!mpu.testConnection()) {
    Serial.println("MPU6050 connection failed!");
    while (1) delay(10);
  }
  Serial.println("MPU6050 OK");
  mpu.setFullScaleAccelRange(MPU6050_ACCEL_FS_2);
  mpu.setFullScaleGyroRange(MPU6050_GYRO_FS_250);

  // ---- 直刷暫存器開啟 Bypass ----
  enable_bypass();

  i2c_scan(); // 第二次掃描（應多出 0x1E 或 0x0D）

  // ---- 掃描 MPU6050 背後 AUX 匯流排 ----
  scan_aux_bus();

  // ---- 磁力計 ----
  init_magnetometer();

  // ---- BMP180 ----
  if (bmp.begin()) {
    bmp_ok = true;
    Serial.println("BMP180 OK");
  } else {
    Serial.println("BMP180 not found (optional)");
  }

  // ---- WiFi ----
  Serial.print("Connecting to WiFi");
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println();
  Serial.print("Connected! IP: ");
  Serial.println(WiFi.localIP());

  udp.begin(local_port);
  Serial.print("UDP on port ");
  Serial.println(local_port);
}

// ==============================================================

void loop() {
  if (millis() - lastSend >= sendInterval) {
    lastSend = millis();

    // ---- 讀取 MPU6050 raw ----
    int16_t ax, ay, az, gx, gy, gz;
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

    float a_x = ax / 16384.0 * 9.80665;
    float a_y = ay / 16384.0 * 9.80665;
    float a_z = az / 16384.0 * 9.80665;

    float g_x = gx / 131.0; // deg/s
    float g_y = gy / 131.0;
    float g_z = gz / 131.0;

    // ---- 讀磁力計 ----
    if (mag_ok) read_magnetometer();

    // ---- 讀 BMP180 (每 10 次) ----
    static int bmp_count = 0;
    if (bmp_ok && (++bmp_count % 10 == 0)) {
      temperature = bmp.readTemperature();
      pressure = bmp.readPressure() / 100.0;
    }

    // ---- 姿態計算 ----
    float accel_pitch, accel_roll;
    calc_attitude(a_x, a_y, a_z, accel_pitch, accel_roll);

    float dt = sendInterval / 1000.0;
    if (first_run) {
      filt_pitch = accel_pitch;
      filt_roll  = accel_roll;
      first_run = false;
    } else {
      filt_pitch = alpha * (filt_pitch + g_y * dt) + (1 - alpha) * accel_pitch;
      filt_roll  = alpha * (filt_roll  + g_x * dt) + (1 - alpha) * accel_roll;
    }

    float pitch = filt_pitch - zero_pitch;
    float roll  = filt_roll  - zero_roll;
    float yaw   = calc_yaw(filt_pitch, filt_roll) - zero_yaw;
    if (yaw < 0) yaw += 360;

    // ---- 發送 UDP ----
    char buf[240];
    snprintf(buf, sizeof(buf),
      "DATA,%.3f,%.3f,%.3f,%.6f,%.6f,%.6f,%.2f,%.2f,%.2f,%d,%d,%d,%.1f,%.1f",
      a_x, a_y, a_z,
      g_x, g_y, g_z,
      pitch, roll, yaw,
      mag_x, mag_y, mag_z,
      temperature, pressure);

    udp.beginPacket(gcs_ip, gcs_port);
    udp.print(buf);
    udp.endPacket();

    Serial.println(buf);
  }

  // ---- 接收命令 ----
  int pkt = udp.parsePacket();
  if (pkt) {
    char cmd[64];
    int len = udp.read(cmd, sizeof(cmd) - 1);
    if (len > 0) {
      cmd[len] = '\0';
      String s = String(cmd);
      Serial.print("CMD: ");
      Serial.println(s);

      if (s == "$LED_ON#") {
        digitalWrite(BUILTIN_LED, HIGH);
      } else if (s == "$LED_OFF#") {
        digitalWrite(BUILTIN_LED, LOW);
      } else if (s.startsWith("$FILTER:")) {
        float v = s.substring(8, s.length() - 1).toFloat();
        if (v >= 0.0 && v <= 1.0) { alpha = v; first_run = true; }
      } else if (s == "$ZERO#") {
        int16_t ax, ay, az, gx, gy, gz;
        mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
        float a_x = ax / 16384.0 * 9.80665;
        float a_y = ay / 16384.0 * 9.80665;
        float a_z = az / 16384.0 * 9.80665;
        float p, r;
        calc_attitude(a_x, a_y, a_z, p, r);
        zero_pitch = p;
        zero_roll  = r;
        zero_yaw   = calc_yaw(p, r);
        digitalWrite(BUILTIN_LED, HIGH);
        delay(100);
        digitalWrite(BUILTIN_LED, LOW);
        Serial.println("  -> Zero set");
      } else if (s.startsWith("$GCS_IP:")) {
        String ipStr = s.substring(8, s.length() - 1);
        ipStr.toCharArray(gcs_ip, sizeof(gcs_ip));
        Serial.print("  -> GCS IP: ");
        Serial.println(gcs_ip);
      }
    }
  }
}
