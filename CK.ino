// ===================================================================
// NHÀ KÍNH THÔNG MINH TRỒNG DÂU TÂY - MÃ ĐỀ 12
// Môn: Lập trình hệ thống (INF1133)
// Bộ điều khiển: ESP32 DevKit V1
// IoT Platform: E-ra (era.io)
// Phiên bản: 1.0 - Đầy đủ Câu 1 + Câu 2 + Câu 3
// ===================================================================

/* Download ERa library: https://github.com/eoh-jsc/era-lib/releases/latest */

// Enable debug console
#define ERA_DEBUG

/* MQTT host mặc định của E-ra */
#define DEFAULT_MQTT_HOST "mqtt1.eoh.io"

/* Auth Token - Lấy từ E-ra App hoặc Dashboard */
#define ERA_AUTH_TOKEN "8a602076-7d6c-4f72-a0d6-e119851166d3"

// =============================================================
// THƯ VIỆN
// =============================================================
#include <Arduino.h>
#include <ERa.hpp>
#include <DHT.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>

// =============================================================
// CẤU HÌNH WI-FI
// =============================================================
const char ssid[] = "Fablab 2.4G";
const char pass[] = "Fira@2024";

// =============================================================
// CHÂN GPIO (PIN MAPPING)
// =============================================================
// Cảm biến
#define PIN_DHT         4     // DHT11 - Nhiệt độ & độ ẩm KK
#define PIN_SOIL_ADC    34    // Soil Moisture - đọc ADC (chỉ input)
#define PIN_LDR_ADC     33    // Quang trở (LDR) - đọc ADC
#define PIN_TRIG        5     // HC-SR04 TRIG
#define PIN_ECHO        18    // HC-SR04 ECHO

// Thiết bị đầu ra (Relay/MOSFET)
#define PIN_RELAY_BOM   25    // Relay kích bơm nước
#define PIN_RELAY_QUAT  26    // Relay kích quạt
#define PIN_RELAY_LED   27    // MOSFET kích LED grow

// LCD I2C
#define I2C_SDA          21    // LCD I2C SDA
#define I2C_SCL          22    // LCD I2C SCL
#define LCD_ADDR        0x27  // Địa chỉ I2C của LCD (0x27 hoặc 0x3F)
#define LCD_COLS        16
#define LCD_ROWS        2

// =============================================================
// NGƯỠNG TIÊU CHUẨN DÂU TÂY (Smart Agriculture)
// =============================================================
// Nhiệt độ: 18-26°C (tối ưu: 22°C)
#define TEMP_MIN        18.0
#define TEMP_MAX        26.0
// Độ ẩm KK: 60-80%
#define HUM_AIR_MIN     60.0
#define HUM_AIR_MAX     80.0
// Độ ẩm đất: 60-80% (ngưỡng đã chuyển đổi sang %)
#define SOIL_DRY        60.0   // (%) dưới -> cần tưới
#define SOIL_WET        80.0   // (%) trên -> đủ nước
// Ánh sáng: 60-100% (>=60% là đủ sáng cho dâu tây)
#define LIGHT_MIN       60.0
// Mực nước bể: dưới 30% là cần bổ sung
#define WATER_LOW       30.0

// =============================================================
// KHAI BÁO ĐỐI TƯỢNG
// =============================================================
#define DHTTYPE DHT11
DHT dht(PIN_DHT, DHTTYPE);
LiquidCrystal_I2C lcd(LCD_ADDR, LCD_COLS, LCD_ROWS);

WiFiClient mbTcpClient;
// Bien trang thai LCD (chi dung ben trong module LCD - dead code vi khong ai set = true)
static bool lcdNeedReinit = false;
// =============================================================
// CẤU TRÚC DỮ LIỆU CẢM BIẾN
// =============================================================
struct SensorData {
    float nhietDo;       // °C
    float doAmKK;        // %
    float doAmDat;       // % (đã chuyển đổi ADC -> %)
    float anhSang;       // %
    float mucNuoc;       // % (mực nước trong bể)
    float khoangCach;    // cm (khoảng cách đo bằng HC-SR04)
};

struct DeviceState {
    bool bom;            // Bơm nước
    bool quat;           // Quạt
    bool led;            // LED grow
    bool cheDoAuto;      // true = AUTO, false = MANUAL
    int  canhBao;        // 0=Bình thường, 1=Hơi nóng, 2=Nóng, 3=Thiếu nước
};

SensorData  sensors = {0, 0, 0, 0, 0, 0};
DeviceState devices = {false, false, false, true, 0};

// Biến đếm thời gian
unsigned long lastSensorRead  = 0;
unsigned long lastSendToEra    = 0;
unsigned long lastLCDUpdate   = 0;
const unsigned long INTERVAL_SENSOR  = 2000;   // Đọc cảm biến mỗi 2 giây
const unsigned long INTERVAL_ERA     = 3000;   // Gửi E-ra mỗi 3 giây (realtime + ổn định)
const unsigned long INTERVAL_LCD     = 2000;   // Cập nhật LCD mỗi 2 giây

// =============================================================
// E-RA VIRTUAL PIN MAPPING (khớp với Dashboard E-ra)
// =============================================================
// Input (cảm biến + cảnh báo) - ESP gửi lên
#define Temp    V0
#define Hum     V1
#define Soil    V2
#define Light   V3
#define Water   V4
#define CBao    V9
// Output (thiết bị + chế độ) - E-ra gửi xuống
#define Bom     V5
#define Quat    V6
#define Led     V7
#define Chedo   V8

// =============================================================
// MODULE 1: ĐỌC CẢM BIẾN
// =============================================================

// Đọc DHT11 (nhiệt độ + độ ẩm không khí) - DHT11 cần >=1s giữa 2 lần đọc
void docDHT11() {
    static unsigned long lastDHT = 0;
    if (millis() - lastDHT < 1500) return;
    lastDHT = millis();

    float t = dht.readTemperature();
    float h = dht.readHumidity();
    if (!isnan(t)) sensors.nhietDo = t;
    if (!isnan(h)) sensors.doAmKK  = h;
}

// Đọc cảm biến độ ẩm đất (analog 0-4095 -> 0-100%)
void docSoil() {
    // Dummy read để ổn định ADC sau khi switch channel (giảm nhiễu ~10%)
    analogRead(PIN_SOIL_ADC);
    delayMicroseconds(100);
    int raw = analogRead(PIN_SOIL_ADC);
    if (raw == 0 || raw == 4095) return;  // dây lỏng hoặc ADC ngắn mạch
    // Soil sensor: ướt ~ 1200 (3.3V), khô ~ 4095 (0V)
    // Map: raw cao = khô, raw thấp = ướt
    float percent = map(raw, 1200, 4095, 100, 0);
    sensors.doAmDat = constrain(percent, 0, 100);
}

// Đọc cảm biến ánh sáng LDR (analog 0-4095 -> 0-100%)
void docLDR() {
    // Đọc nhiều lần để lấy trung bình (ổn định hơn)
    const int NUM_SAMPLES = 10;
    long sum = 0;
    for (int i = 0; i < NUM_SAMPLES; i++) {
        sum += analogRead(PIN_LDR_ADC);
        delayMicroseconds(100);
    }
    int raw = sum / NUM_SAMPLES;

    // GND -- LDR -- ADC -- 10K -- 3.3V
    // Tối → LDR cao → ADC voltage cao → raw cao → % ánh sáng thấp
    // Sáng → LDR thấp → ADC voltage thấp → raw thấp → % ánh sáng cao
    float percent = map(raw, 0, 4095, 100, 0);
    sensors.anhSang = constrain(percent, 0, 100);
}

// Đọc khoảng cách HC-SR04 (cm)
float docKhoangCach() {
    digitalWrite(PIN_TRIG, LOW);
    delayMicroseconds(2);
    digitalWrite(PIN_TRIG, HIGH);
    delayMicroseconds(10);
    digitalWrite(PIN_TRIG, LOW);

    long duration = pulseIn(PIN_ECHO, HIGH, 30000); // timeout 30ms
    if (duration == 0) return -1;  // lỗi
    float distance = duration * 0.034 / 2.0;
    return distance;
}

// Đọc mực nước (HC-SR04 đặt trên đỉnh bể)
void docMucNuoc() {
    // Lấy trung bình 5 mẫu để giảm nhiễu (HC-SR04 dễ bị nhiễu do relay)
    const int NUM_SAMPLES = 5;
    float sum = 0;
    int valid = 0;
    for (int i = 0; i < NUM_SAMPLES; i++) {
        float d = docKhoangCach();
        if (d > 0 && d < 50) {  // bỏ giá trị lỗi (timeout hoặc quá xa)
            sum += d;
            valid++;
        }
        delay(50);  // delay giữa các lần đo
    }
    if (valid == 0) {
        sensors.khoangCach = -1;
        sensors.mucNuoc = 0;
        return;
    }
    float d = sum / valid;
    sensors.khoangCach = d;
    // Sensor đặt cách đáy bể 12.5cm, mặt nước max cao 9cm
    // OFFSET ĐÃ XÓA - sensor đọc khá chính xác, offset 2.5 gây sai lệch lớn
    const float KHOANG_CACH_DEN_DAY = 12.5;   // cm - sensor → đáy bể
    const float CHIEU_CAO_MUC_MAX  = 9.0;     // cm - mặt nước max từ đáy
    float percent = ((KHOANG_CACH_DEN_DAY - d) / CHIEU_CAO_MUC_MAX) * 100.0;
    sensors.mucNuoc = constrain(percent, 0, 100);
}

// Đọc tất cả cảm biến
void docTatCaCamBien() {
    // Stagger các cảm biến 30ms để tránh peak dòng đồng thời
    // (DHT 2.5mA + Soil 5mA + LDR 0.5mA + HC-SR04 15mA peak = ~23mA spike)
    docDHT11();
    delay(30);
    docSoil();
    delay(30);
    docLDR();
    delay(30);
    docMucNuoc();
}

// =============================================================
// MODULE 2: ĐIỀU KHIỂN THIẾT BỊ (RELAY)
// =============================================================
void khoiTaoThietBi() {
    pinMode(PIN_RELAY_BOM,  OUTPUT);
    pinMode(PIN_RELAY_QUAT, OUTPUT);
    pinMode(PIN_RELAY_LED,  OUTPUT);
    pinMode(PIN_TRIG, OUTPUT);
    pinMode(PIN_ECHO, INPUT);

    digitalWrite(PIN_RELAY_BOM,  HIGH);  // Tắt (active LOW)
    digitalWrite(PIN_RELAY_QUAT, HIGH);
    digitalWrite(PIN_RELAY_LED,  HIGH);  // Tat LED (active LOW) - nhat quan voi CKCAU4.ino
}

// ACTIVE LOW: HIGH = tắt, LOW = bật (module relay phổ thông)
void setBom(bool on)  {
    devices.bom = on;
    digitalWrite(PIN_RELAY_BOM, on ? LOW : HIGH);
}
void setQuat(bool on) {
    devices.quat = on;
    digitalWrite(PIN_RELAY_QUAT, on ? LOW : HIGH);
}
void setLed(bool on)  {
    devices.led = on;
    digitalWrite(PIN_RELAY_LED, on ? LOW : HIGH);
}

// =============================================================
// MODULE 3: LOGIC TỰ ĐỘNG (AUTO MODE)
// =============================================================
void logicTuDong() {
    if (!devices.cheDoAuto) return;  // MANUAL thì không can thiệp

    // QUAT: hysteresis 22-26 do C (4 do vung dem), tranh bat/tat khi nhiet do bien dong quanh TEMP_MAX
    //   Dang ON: chi tat khi nhiet do < 22 (TEMP_MAX - 4)
    //   Dang OFF: chi bat khi nhiet do > 26 (TEMP_MAX)
    if (sensors.nhietDo > TEMP_MAX) {
        setQuat(true);
    } else if (sensors.nhietDo <= TEMP_MAX - 4.0) {
        setQuat(false);
    }

    // BOM: chi theo do am dat (HC-SR04 chi de canh bao be can)
    // Hysteresis: SOIL_DRY=60 (bat) -> SOIL_WET=80 (tat) -> giua = giu nguyen
    if (sensors.doAmDat < SOIL_DRY) {
        setBom(true);
    } else if (sensors.doAmDat >= SOIL_WET) {
        setBom(false);
    }

    // LED: bật khi SÁNG CAO (> LIGHT_MIN), tắt khi TỐI (<= LIGHT_MIN - 20)
    // ánh sáng > 60% → bật, ánh sáng <= 40% → tắt (hysteresis 20% chống flickering)
    if (sensors.anhSang > LIGHT_MIN) {
        setLed(true);
    } else if (sensors.anhSang <= LIGHT_MIN - 20.0) {
        setLed(false);
    }
}

// =============================================================
// MODULE 4: XÁC ĐỊNH CẢNH BÁO
// =============================================================
int xacDinhCanhBao() {
    // Ưu tiên: Nóng > Hơi nóng > Thiếu nước > Bình thường
    if (sensors.nhietDo >= TEMP_MAX + 4.0) return 2;     // NÓNG (>=30°C)
    if (sensors.nhietDo > TEMP_MAX)         return 1;     // HƠI NÓNG (26-30°C)
    if (sensors.mucNuoc < WATER_LOW)        return 3;     // THIẾU NƯỚC
    return 0;                                             // BÌNH THƯỜNG
}

// =============================================================
// MODULE 5: GỬI/NHẬN DỮ LIỆU E-RA
// =============================================================

/* Callback khi ESP32 nhận lệnh từ E-ra - có debounce chống flickering */
ERA_WRITE(Chedo) {
    int val = param.getInt();
    bool newState = (val == 1);
    if (newState != devices.cheDoAuto) {
        devices.cheDoAuto = newState;
        ERa.virtualWrite(Chedo, devices.cheDoAuto ? 1 : 0);
        Serial.printf("[ERA] V8 (AUTO) = %d\n", devices.cheDoAuto ? 1 : 0);
    }
}

ERA_WRITE(Bom) {
    int val = param.getInt();
    bool newState = (val == 1);
    if (!devices.cheDoAuto && newState != devices.bom) {
        setBom(newState);
        ERa.virtualWrite(Bom, devices.bom ? 1 : 0);
        Serial.printf("[ERA] V5 (BOM) = %d\n", devices.bom ? 1 : 0);
    }
}

ERA_WRITE(Quat) {
    int val = param.getInt();
    bool newState = (val == 1);
    Serial.printf("[ERA] Quat received: val=%d, cheDoAuto=%d, quat=%d\n",
                  val, devices.cheDoAuto, devices.quat);
    if (!devices.cheDoAuto && newState != devices.quat) {
        setQuat(newState);
        Serial.printf("[ERA] Quat set: GPIO26=%d, sync ERa=%d\n",
                      digitalRead(PIN_RELAY_QUAT), devices.quat);
        ERa.virtualWrite(Quat, devices.quat ? 1 : 0);
    }
}

ERA_WRITE(Led) {
    int val = param.getInt();
    bool newState = (val == 1);
    if (!devices.cheDoAuto && newState != devices.led) {
        devices.led = newState;
        setLed(newState);
        ERa.virtualWrite(Led, devices.led ? 1 : 0);
        Serial.printf("[ERA] V7 (LED) = %d\n", devices.led ? 1 : 0);
    }
}

// Hàm gửi dữ liệu cảm biến lên E-ra (chỉ gửi khi thay đổi + sync mỗi 30s)
void guiDuLieuLenEra() {
    static float lastTemp = -999.0, lastHum = -999.0, lastSoil = -999.0;
    static float lastLight = -999.0, lastWater = -999.0;
    static int lastCBao = -1;
    static bool lastBom = false, lastQuat = false, lastLed = false;
    static bool lastCheDo = true;

    unsigned long now = millis();
    bool forceSync = (now % 30000) < 3000;  // ép gửi đầu mỗi chu kỳ 30s

    // Chỉ gửi nếu thay đổi > ngưỡng hoặc đến chu kỳ sync
    if (forceSync || abs(sensors.nhietDo - lastTemp) > 0.1) {
        ERa.virtualWrite(Temp, sensors.nhietDo);
        lastTemp = sensors.nhietDo;
    }
    if (forceSync || abs(sensors.doAmKK - lastHum) > 1) {
        ERa.virtualWrite(Hum, sensors.doAmKK);
        lastHum = sensors.doAmKK;
    }
    if (forceSync || abs(sensors.doAmDat - lastSoil) > 1) {
        ERa.virtualWrite(Soil, sensors.doAmDat);
        lastSoil = sensors.doAmDat;
    }
    if (forceSync || abs(sensors.anhSang - lastLight) > 1) {
        ERa.virtualWrite(Light, sensors.anhSang);
        lastLight = sensors.anhSang;
    }
    if (forceSync || abs(sensors.mucNuoc - lastWater) > 1) {
        ERa.virtualWrite(Water, sensors.mucNuoc);
        lastWater = sensors.mucNuoc;
    }

    // Thiết bị + chế độ + cảnh báo: gửi khi thay đổi state
    if (forceSync || devices.bom  != lastBom) { ERa.virtualWrite(Bom,  devices.bom  ? 1 : 0); lastBom  = devices.bom;  }
    if (forceSync || devices.quat != lastQuat) { ERa.virtualWrite(Quat, devices.quat ? 1 : 0); lastQuat = devices.quat; }
    if (forceSync || devices.led  != lastLed)  { ERa.virtualWrite(Led,  devices.led  ? 1 : 0); lastLed  = devices.led;  }
    if (forceSync || devices.cheDoAuto != lastCheDo) {
        ERa.virtualWrite(Chedo, devices.cheDoAuto ? 1 : 0);
        lastCheDo = devices.cheDoAuto;
    }
    if (forceSync || devices.canhBao != lastCBao) {
        ERa.virtualWrite(CBao, devices.canhBao);
        lastCBao = devices.canhBao;
    }
}

// =============================================================
// MODULE 6: HIỂN THỊ LCD 16x2
// =============================================================

// Safe helper: in 1 dòng LCD có retry I2C (chống nhiễu relay làm lock bus).
// Neu relay dam nhieu lam I2C lock (SDA ket LOW), se re-init Wire de recover.
void lcdSafePrint(const char* line, uint8_t row) {
    // Kiem tra bus truoc khi gui - neu NACK thi re-init Wire
    bool ok = false;
    for (int retry = 0; retry < 3; retry++) {
        Wire.beginTransmission(LCD_ADDR);
        uint8_t err = Wire.endTransmission();
        if (err == 0) { ok = true; break; }
        // NACK hoac loi bus -> re-init I2C
        Wire.begin(I2C_SDA, I2C_SCL, 100000);
        delay(5);
    }
    // Neu van loi thi bo qua lan in nay (se thu lai o chu ky sau)
    lcd.setCursor(0, row);
    lcd.print(line);
}

void hienThiLCD() {
    // Rate limit: chỉ in tối đa 2 lần/giây
    static unsigned long lastLCDCall = 0;
    unsigned long now = millis();
    if (now - lastLCDCall < 500) return;
    lastLCDCall = now;

    static char lastLine1[17] = "";
    static char lastLine2[17] = "";
    static unsigned long lastRetry = 0;

    // FIX Bug 1: Định kỳ (mỗi 5s) reset cache để buộc gửi lại nội dung
    // (trước đây code in " " gây giật LCD - nay bỏ hẳn)
    if (now - lastRetry > 5000) {
        lastRetry = now;
        lastLine1[0] = '\0';
        lastLine2[0] = '\0';
    }

    char line1[17], line2[17];
    snprintf(line1, sizeof(line1), "T:%4.1f H:%2.0f%% S:%2.0f%%",
             sensors.nhietDo, sensors.doAmKK, sensors.doAmDat);

    const char* cbStr;
    switch (devices.canhBao) {
        case 1: cbStr = "!N"; break;
        case 2: cbStr = "!!"; break;
        case 3: cbStr = "!W"; break;
        default: cbStr = "OK"; break;
    }
    snprintf(line2, sizeof(line2), "L:%3.0f%% W:%2.0f%% %s",
             sensors.anhSang, sensors.mucNuoc, cbStr);

    if (strcmp(line1, lastLine1) != 0) {
        lcdSafePrint(line1, 0);
        strcpy(lastLine1, line1);
    }
    if (strcmp(line2, lastLine2) != 0) {
        lcdSafePrint(line2, 1);
        strcpy(lastLine2, line2);
    }
}

// =============================================================
// CALLBACK KẾT NỐI E-RA
// =============================================================
ERA_CONNECTED() {
    ERA_LOG(ERA_PSTR("ERa"), ERA_PSTR("Đã kết nối E-ra thành công!"));
}

ERA_DISCONNECTED() {
    ERA_LOG(ERA_PSTR("ERa"), ERA_PSTR("Mất kết nối, đang thử lại..."));
}

// =============================================================
// SETUP
// =============================================================
void setup() {
    Serial.begin(115200);
    Serial.println("\n=== NHA KINH THONG MINH - ESP32 ===");

    // FIX Reset khi dùng Adapter 5V: tắt brownout detector
    // Adapter no-name 2A thực tế ~1-1.5A, sụt áp tạm thời khi relay/WiFi peak
    // → brownout reset. Tắt để ESP32 không tự restart liên tục.
    // LƯU Ý: đây là giải pháp tạm, vẫn nên đổi adapter 3A chính hãng.
    WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0);

    // FIX Bug 6: Bỏ setCpuFrequencyMhz(80) - để nguyên 160MHz mặc định
    // Lý do: ERa WiFi trên Core 0 với tight timing, CPU chậm → MQTT ping timeout → reset
    // Relay noise nên giải quyết bằng I2C retry + delay, không hạ CPU

    // Lưu ý: KHÔNG disableCore0WDT/Core1WDT
    // Vì ERa chạy task WiFi trên Core 0, tắt WDT sẽ phá vỡ kết nối

    khoiTaoThietBi();
    pinMode(PIN_DHT, INPUT);  // INPUT (mac dinh), neu mach khong co R 10k pull-up ngoai thi them INPUT_PULLUP
    dht.begin();
    // KHONG can delay 2000 vi da co lastDHT check (>=1500ms) ben trong docDHT11

    // FIX Bug 3: Wire.begin() + setClock() để đảm bảo I2C ổn định + sẵn sàng cho retry sau này
    Wire.begin(I2C_SDA, I2C_SCL, 100000);  // I2C 100kHz ro rang
    Wire.setClock(100000);                  // ép clock về 100kHz chắc chắn
    lcd.init();
    lcd.backlight();
    lcd.setCursor(0, 0);
    lcd.print(" NHA KINH THONG");
    lcd.setCursor(0, 1);
    lcd.print(" MINH - DAU TAY ");
    delay(2000);  // Giu man hinh chao 2s de test hien thi (chi 1 lan o setup, khong anh huong loop)
    lcd.clear();

    ERa.setModbusClient(mbTcpClient);
    ERa.setScanWiFi(true);
    ERa.begin(ssid, pass);

    // FIX Reset khi dùng Adapter 5V: giảm WiFi TX power
    // Mặc định ESP32 19.5dBm (~250mA peak), giảm xuống 17dBm (~170mA peak)
    // Tiết kiệm ~80mA peak khi WiFi truyền, đủ xa cho nhà kính trong nhà
    WiFi.setTxPower(WIFI_POWER_17dBm);

    Serial.println("Khoi tao hoan tat!");
}

// =============================================================
// LOOP
// =============================================================
void loop() {
    ERa.run();

    unsigned long now = millis();

    if (now - lastSensorRead >= INTERVAL_SENSOR) {
        lastSensorRead = now;
        docTatCaCamBien();
        devices.canhBao = xacDinhCanhBao();

        // In sensor debug mỗi 5 giây (tránh flood Serial)
        static unsigned long lastSensorPrint = 0;
        if (now - lastSensorPrint > 5000) {
            lastSensorPrint = now;
            Serial.printf("[SENSOR] T=%.1f H=%.0f S=%.0f L=%.0f W=%.0f CB=%d\n",
                          sensors.nhietDo, sensors.doAmKK, sensors.doAmDat,
                          sensors.anhSang, sensors.mucNuoc, devices.canhBao);
        }
    }

    logicTuDong();

    if (now - lastSendToEra >= INTERVAL_ERA) {
        lastSendToEra = now;
        guiDuLieuLenEra();
        // FIX Bug 4: delay nhỏ giữa I2C và cảm biến để tránh nhiễu ADC chồng I2C
        delay(20);
    }

    if (now - lastLCDUpdate >= INTERVAL_LCD) {
        lastLCDUpdate = now;
        hienThiLCD();
    }
}
