
#include <Arduino.h>
#include <FastLED.h>
#include <NimBLEDevice.h>

int g_pin_sck   = 33;   
int g_pin_dout  = 32;  
#define BAT_ADC_PIN     34   

// ============================================================
// 2. CẤU HÌNH DẢI LED WS2812B (CHÍNH XÁC GPIO 18)
// ============================================================
#define LED_PIN         18  
#define NUM_LEDS        8
#define LED_TYPE        WS2812B
#define COLOR_ORDER     GRB
#define BRIGHTNESS      50   

CRGB leds[NUM_LEDS];

// ============================================================
// 3. CẤU HÌNH BLE UUID (Nordic UART Service chuẩn)
// ============================================================
#define SERVICE_UUID        "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define CHARACTERISTIC_UUID "beb5483e-36e1-4688-b7f5-ea07361b26a8"  // TX: ESP32 -> App
#define RX_CHAR_UUID        "beb5483e-36e1-4688-b7f5-ea07361b26a9"  // RX: App -> ESP32

NimBLEServer*         pServer         = nullptr;
NimBLECharacteristic* pCharacteristic = nullptr;
bool deviceConnected = false;

const float SCALE_FACTOR = 200.0f;   // 100 xung = 1 Pa (Chuẩn xác với MPS20N0040D)
const float K_FLOW       = 0.0085f;  // Hệ số lưu lượng ống thổi
const float LED_THRESHOLDS[NUM_LEDS] = {
    5.0f,   // Bóng 1 (Đỏ)
    10.0f,  // Bóng 2 (Đỏ)
    20.0f,  // Bóng 3 (Đỏ)
    35.0f,  // Bóng 4 (Vàng)
    50.0f,  // Bóng 5 (Vàng)
    75.0f,  // Bóng 6 (Vàng)
    100.0f, // Bóng 7 (Xanh lá)
    150.0f  // Bóng 8 (Xanh lá)
};

// ============================================================
// 5. BIẾN CHIA SẺ GIỮA 2 CORE (SHARED MEMORY)
// ============================================================
volatile long  g_raw_val   = 0;
volatile long  g_tare      = 0;
volatile bool  g_tare_done = false;
volatile float g_dp_pa     = 0.0f;
volatile float g_flow_ls   = 0.0f;
volatile float g_fvc_l     = 0.0f;
volatile float g_pf_ls     = 0.0f;
volatile int   g_bat_pct   = 100;
volatile bool  g_sensor_ok = false;

float fvc_local = 0.0f;
float pf_local  = 0.0f;

// ============================================================
// 6. BỘ LỌC TRUNG BÌNH TRƯỢT (MOVING AVERAGE FILTER)
// ============================================================
const int NUM_READINGS = 15;
long  ma_readings[NUM_READINGS];
int   ma_index = 0;
long  ma_total = 0;
bool  ma_initialized = false;

// ============================================================
// 7. TASK HANDLES
// ============================================================
TaskHandle_t hTaskBLE    = nullptr;
TaskHandle_t hTaskSensor = nullptr;

// ============================================================
// HÀM: ĐỌC 1 MẪU DỮ LIỆU TỪ CHIP ADC HX710B VỚI CHÂN TUỲ CHỌN
// ============================================================
bool read_hx710_single(int sck_pin, int dout_pin, long &out_val) {
    // Chờ DOUT xuống LOW (dữ liệu sẵn sàng) với timeout 250ms
    uint32_t t0 = millis();
    while (digitalRead(dout_pin) == HIGH) {
        if (millis() - t0 > 250) {
            return false; // Timeout: chưa có dữ liệu hoặc tuột dây
        }
        delayMicroseconds(50);
    }

    // Đọc 24 bit dữ liệu
    portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;
    portENTER_CRITICAL(&mux);

    long raw = 0;
    for (int i = 23; i >= 0; i--) {
        digitalWrite(sck_pin, HIGH);
        delayMicroseconds(1);
        if (digitalRead(dout_pin)) {
            raw |= (1L << i);
        }
        digitalWrite(sck_pin, LOW);
        delayMicroseconds(1);
    }

    // Xung thứ 25: chốt kết quả và giữ Gain 128
    digitalWrite(sck_pin, HIGH);
    delayMicroseconds(1);
    digitalWrite(sck_pin, LOW);
    delayMicroseconds(1);

    portEXIT_CRITICAL(&mux);

    // Chuyển mã bù 2 sang 32-bit có dấu
    if (raw & 0x800000L) {
        raw |= 0xFF000000L;
    }
    if (raw == 0L || raw == -1L || raw == 0xFFFFFFL) {
        return false;
    }

    out_val = raw;
    return true;
}

// ============================================================
// HÀM: TỰ ĐỘNG DÒ TÌM VÀ CẤU HÌNH ĐÚNG CHÂN SCK / DOUT
// ============================================================
void auto_detect_hx710_pins() {
    Serial.println("\n[SENSOR AUTO-DETECT] Dang kiem tra ket noi cam bien...");

    long test_val = 0;

    // Kịch bản 1: SCK=33, DOUT=32 (theo Altium chuẩn)
    pinMode(33, OUTPUT); digitalWrite(33, LOW);
    pinMode(32, INPUT);
    delay(50);
    if (read_hx710_single(33, 32, test_val)) {
        g_pin_sck  = 33;
        g_pin_dout = 32;
        g_sensor_ok = true;
        Serial.printf(">>> [KET QUA] Phat hien cam bien dung chuan: SCK = GPIO 33, DOUT = GPIO 32! (Raw = %ld)\n", test_val);
        return;
    }

    // Kịch bản 2: Cắm ngược chân trên Breadboard: SCK=32, DOUT=33
    pinMode(32, OUTPUT); digitalWrite(32, LOW);
    pinMode(33, INPUT);
    delay(50);
    if (read_hx710_single(32, 33, test_val)) {
        g_pin_sck  = 32;
        g_pin_dout = 33;
        g_sensor_ok = true;
        Serial.printf(">>> [KET QUA] Phat hien ban da CAM NGUOC 2 day SCK va DOUT tren Breadboard!\n");
        Serial.printf("    => Da tu dong sua trong code thanh: SCK = GPIO 32, DOUT = GPIO 33! (Raw = %ld)\n", test_val);
        return;
    }

    // Nếu cả 2 đều không được: Báo trạng thái chi tiết để người dùng kiểm tra dây
    pinMode(33, INPUT);
    pinMode(32, INPUT);
    int st32 = digitalRead(32);
    int st33 = digitalRead(33);
    Serial.println(">>> [CANH BAO NGUY HIEM] Khong tim thay chip HX710B tren ca 2 chan GPIO 32 va 33!");
    Serial.printf("    Trang thai chan hien tai: D32 = %d | D33 = %d\n", st32, st33);
    if (st32 == 0 && st33 == 0) {
        Serial.println("    => Nguyen nhan: Ca 2 chan deu bi muc 0V! Co the module chua co nguon VCC, hoac long day GND/VCC!");
    } else if (st32 == 1 && st33 == 1) {
        Serial.println("    => Nguyen nhan: Ca 2 chan deu o muc 3.3V (Timeout)! Kiem tra xem da cam dung lo hang D32/D33 tren breadboard chua!");
    }
    // Mặc định giữ 33 và 32
    g_pin_sck  = 33;
    g_pin_dout = 32;
    pinMode(g_pin_sck, OUTPUT); digitalWrite(g_pin_sck, LOW);
    pinMode(g_pin_dout, INPUT);
}

// ============================================================
// HÀM: ĐỌC RAW THEO CHÂN ĐÃ NHẬN DIỆN
// ============================================================
bool hx710b_read_raw(long &out_val) {
    return read_hx710_single(g_pin_sck, g_pin_dout, out_val);
}

// ============================================================
// HÀM: ZERO-TARE (LẤY MỐC 0 ÁP SUẤT MÔI TRƯỜNG KHI KHỞI ĐỘNG)
// ============================================================
void perform_zero_tare() {
    Serial.println("\n-------------------------------------------------------------");
    Serial.println("[TARE] Dang doi cam bien on dinh dien ap (2 giay)... Vui long KHONG THOI!");

    // Đợi 2 giây cho cảm biến MPS20N0040D và mạch HX710B ổn định nhiệt & điện áp hoàn toàn
    for (int i = 0; i < 20; i++) {
        long discard = 0;
        hx710b_read_raw(discard);
        vTaskDelay(pdMS_TO_TICKS(100));
    }

    Serial.println("[TARE] Dang thu thap 30 mau chuan moc 0 ap suat...");
    long sum = 0;
    int valid_count = 0;
    int attempts = 0;

    // Thu thập 30 mẫu hợp lệ khi điện áp đã ổn định
    while (valid_count < 30 && attempts < 100) {
        long raw = 0;
        if (hx710b_read_raw(raw)) {
            sum += raw;
            valid_count++;
        }
        attempts++;
        vTaskDelay(pdMS_TO_TICKS(25));
    }

    if (valid_count > 0) {
        g_tare = sum / valid_count;
        g_tare_done = true;
        g_sensor_ok = true;
        Serial.printf(">>> [TARE THANH CONG] Moc 0 chuan: g_tare = %ld (Trung binh %d mau)\n", g_tare, valid_count);
    } else {
        Serial.println("[TARE] CHUA CO DU LIEU: Se tu dong lay moc 0 khi nhan mau hop le dau tien.");
        g_tare = 0;
        g_tare_done = false;
        g_sensor_ok = false;
    }
    Serial.println("-------------------------------------------------------------\n");
}

// ============================================================
// HÀM: QUY ĐỔI RAW → ÁP SUẤT (Pa) → LƯU LƯỢNG (L/s)
// ============================================================
float raw_to_pa(long raw) {
    if (!g_tare_done || g_tare == 0) return 0.0f;
    long diff = raw - g_tare;
    // Ngưỡng vùng chết khử nhiễu môi trường: dao động dưới 300 xung coi như dp = 0
    if (labs(diff) < 300) return 0.0f;
    return (float)diff / SCALE_FACTOR;
}

float pa_to_flow(float dp) {
    if (dp < 2.0f) return 0.0f; // Chỉ đo lưu lượng khi THỔI RA (dp dương >= 2 Pa)
    return K_FLOW * sqrtf(dp);
}

// ============================================================
// HÀM: TÍCH PHÂN DUNG TÍCH PHỔI FVC (Lít)
// ============================================================
void integrate_fvc(float dp, float fl, uint32_t dt_ms) {
    if (dp >= LED_THRESHOLDS[0]) { // Bắt đầu vượt ngưỡng bóng 1 (đang có hơi THỔI RA)
        fvc_local += fl * (dt_ms / 1000.0f);
        if (fl > pf_local) {
            pf_local = fl; // Cập nhật Peak Flow cao nhất
        }
    }
}

// ============================================================
// HÀM ĐO PIN LỌC ĐA TẦNG (EMA + OVERSAMPLING)
// ============================================================
int read_battery_pct() {
    static float bat_filtered_pct = -1.0f; 
    long sum = 0;
    for (int i = 0; i < 64; i++) {
        sum += analogRead(BAT_ADC_PIN);
        delayMicroseconds(50);
    }
    
    float raw_avg = (float)sum / 64.0f;
    
    float v = (raw_avg / 4095.0f) * 3.3f * 2.05f;
    float pct;
    if      (v >= 4.15f) pct = 100.0f;
    else if (v >= 3.95f) pct = 80.0f  + (v - 3.95f) / 0.20f * 20.0f;
    else if (v >= 3.70f) pct = 50.0f  + (v - 3.70f) / 0.25f * 30.0f;
    else if (v >= 3.45f) pct = 15.0f  + (v - 3.45f) / 0.25f * 35.0f;
    else if (v >= 3.20f) pct =  0.0f  + (v - 3.20f) / 0.25f * 15.0f;
    else                 pct =  0.0f;

    pct = constrain(pct, 0.0f, 100.0f);

    if (bat_filtered_pct < 0.0f) {
        bat_filtered_pct = pct;
    } else {
        bat_filtered_pct = 0.85f * bat_filtered_pct + 0.15f * pct;
    }

    return (int)(bat_filtered_pct + 0.5f);
}
void update_led_bar(float dp) {
    FastLED.clear();
    if (dp < LED_THRESHOLDS[0]) {
        leds[0] = CRGB::Yellow; 
        FastLED.show();
        return;
    }

    int n_on = 0;
    for (int i = 0; i < NUM_LEDS; i++) {
        if (dp >= LED_THRESHOLDS[i]) {
            n_on = i + 1;
        } else {
            break;
        }
    }

    for (int i = 0; i < n_on; i++) {
        if (i < 3) {
            leds[i] = CRGB::Red; 
        } else if( i>=3 && i<6){
                leds[i] = CRGB::Yellow;}
         else {
            leds[i] = CRGB::Red;  
        }
    }

    FastLED.show();
}


class MyServerCallbacks : public NimBLEServerCallbacks {
    void onConnect(NimBLEServer*, NimBLEConnInfo&) override {
        deviceConnected = true;
        Serial.println("\n>>> [BLE EVENT] App dien thoai DA KET NOI thanh cong!");
    }
    void onDisconnect(NimBLEServer*, NimBLEConnInfo&, int) override {
        deviceConnected = false;
        Serial.println("\n>>> [BLE EVENT] App da NGAT KET NOI. He thong dang phat song lai...");
    }
};

class MyRxCallbacks : public NimBLECharacteristicCallbacks {
    void onWrite(NimBLECharacteristic* pC, NimBLEConnInfo&) override {
        std::string cmd = pC->getValue();
        if (cmd == "TARE") {
            g_tare_done = false;
            fvc_local   = 0.0f;
            pf_local    = 0.0f;
            Serial.println("[CMD] Nhan lenh TARE: Dang can lai moc 0 ap suat...");
        } else if (cmd == "RESET_FVC") {
            fvc_local = 0.0f;
            pf_local  = 0.0f;
            Serial.println("[CMD] Nhan lenh RESET_FVC: Da reset thong so lan thoi.");
        }
    }
};

// ============================================================
// TASK 1: XỬ LÝ BLUETOOTH (CHẠY TRÊN CORE 0)
// ============================================================
void TaskBLECode(void* pvParameters) {
    NimBLEDevice::init("Smart_Spirometer");
    NimBLEDevice::setMTU(128);
    pServer = NimBLEDevice::createServer();
    pServer->setCallbacks(new MyServerCallbacks());
    pServer->advertiseOnDisconnect(true);

    NimBLEService* pSvc = pServer->createService(SERVICE_UUID);

    // TX Characteristic: Gửi dữ liệu telemetry lên App qua Notify
    pCharacteristic = pSvc->createCharacteristic(
        CHARACTERISTIC_UUID, NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::NOTIFY);

    // RX Characteristic: Nhận lệnh từ App
    NimBLECharacteristic* pRx = pSvc->createCharacteristic(
        RX_CHAR_UUID, NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::WRITE_NR);
    pRx->setCallbacks(new MyRxCallbacks());

    pServer->start();
    NimBLEDevice::getAdvertising()->addServiceUUID(SERVICE_UUID);
    NimBLEDevice::getAdvertising()->start();

    Serial.println("=============================================================");
    Serial.printf("[BLE] MAC      : %s\n", NimBLEDevice::getAddress().toString().c_str());
    Serial.printf("[BLE] Svc UUID : %s\n", SERVICE_UUID);
    Serial.printf("[BLE] TX  UUID : %s\n", CHARACTERISTIC_UUID);
    Serial.printf("[BLE] RX  UUID : %s\n", RX_CHAR_UUID);
    Serial.println("[BLE] Da bat dau phat song BLE. San sang ket noi!");
    Serial.println("=============================================================");

    char payload[96];
    for (;;) {
        if (deviceConnected && g_tare_done) {
            snprintf(payload, sizeof(payload),
                "{\"dp\":%.1f,\"fl\":%.2f,\"fvc\":%.3f,\"pf\":%.2f,\"bat\":%d}",
                (float)g_dp_pa, (float)g_flow_ls,
                (float)g_fvc_l, (float)g_pf_ls, (int)g_bat_pct);
            pCharacteristic->setValue(payload);
            pCharacteristic->notify();

            // In trực tiếp gói tin vừa bắn qua Bluetooth lên Serial Monitor
            Serial.printf(">>> [BLE TX] %s\n", payload);
        }
        vTaskDelay(pdMS_TO_TICKS(200)); // Gửi telemetry 5 lần/giây (5Hz)
    }
}

// ============================================================
// TASK 2: ĐỌC CẢM BIẾN & XỬ LÝ LED (CHẠY TRÊN CORE 1)
// ============================================================
void TaskSensorCode(void* pvParameters) {
    auto_detect_hx710_pins();
    perform_zero_tare();

    uint32_t t_last     = millis();
    uint32_t t_led      = 0;
    uint32_t t_bat      = 0;
    uint32_t t_serial   = 0;
    uint32_t t_idle     = 0;
    uint32_t t_neg_idle = 0;

    for (;;) {
        if (!g_tare_done) {
            perform_zero_tare();
        }

        uint32_t now = millis();
        uint32_t dt  = now - t_last;
        t_last       = now;

        // 1. Đọc dữ liệu từ cảm biến HX710B
        long raw = 0;
        if (hx710b_read_raw(raw)) {
            g_raw_val = raw;
            g_sensor_ok = true;
            if (g_tare == 0 || !g_tare_done) {
                g_tare = raw;
                g_tare_done = true;
                Serial.printf("\n>>> [AUTO-TARE] Da chot moc 0 ban dau: Tare = %ld <<<\n\n", g_tare);
            }

            // Lọc trung bình trượt 15 mẫu
            if (!ma_initialized) {
                for (int i = 0; i < NUM_READINGS; i++) ma_readings[i] = raw;
                ma_total = raw * NUM_READINGS;
                ma_initialized = true;
            }
            ma_total -= ma_readings[ma_index];
            ma_readings[ma_index] = raw;
            ma_total += raw;
            ma_index = (ma_index + 1) % NUM_READINGS;
            long raw_filtered = ma_total / NUM_READINGS;

            // ============================================================
            // THUẬT TOÁN BÁM TRÔI & TỰ ĐỘNG THOÁT TRẠNG THÁI THỔI
            // ============================================================
            static uint32_t t_slope_check = 0;
            static long raw_at_start = 0;
            static bool is_blowing = false;
            static uint32_t t_steady_blow = 0;

            long diff_now = raw_filtered - g_tare;
            float dp_candidate = (float)diff_now / SCALE_FACTOR;

            if (raw_at_start == 0) raw_at_start = raw_filtered;

            // Kiểm tra biến thiên áp suất mỗi 200ms
            if (now - t_slope_check >= 200) {
                float dp_diff_step = (float)(raw_filtered - raw_at_start) / SCALE_FACTOR;
                if (is_blowing && dp_diff_step < -2.0f) {
                    if (dp_candidate < 10.0f) { 
                        is_blowing = false;
                        g_tare = raw_filtered; 
                    }
                }
                if (dp_diff_step > 3.0f && dp_candidate >= LED_THRESHOLDS[0]) {
                    is_blowing = true;
                    t_steady_blow = now;
                }

                // 3. Nếu không thổi (trạng thái nghỉ hoặc trôi lừ đừ < 1.5 Pa)
                if (!is_blowing || (dp_candidate < LED_THRESHOLDS[0])) {
                    if (fabsf(dp_diff_step) < 1.5f) {
                        is_blowing = false;
                        // Kéo Tare mềm bám theo nhiệt độ
                        g_tare = (long)(0.75f * g_tare + 0.25f * raw_filtered);
                    }
                }
                if (is_blowing && fabsf(dp_diff_step) < 0.8f) {
                    if (now - t_steady_blow > 7000) {
                        is_blowing = false; // Tự thoát trạng thái thổi
                        g_tare = raw_filtered;
                    }
                } else {
                    t_steady_blow = now;
                }

                raw_at_start = raw_filtered;
                t_slope_check = now;
            }
            float dp = 0.0f;
            if (is_blowing && dp_candidate >= LED_THRESHOLDS[0]) {
                dp = dp_candidate;
            } else {
                dp = 0.0f;
                if (raw_filtered < g_tare - 150) {
                    g_tare = raw_filtered;
                }
            }

            float fl = pa_to_flow(dp);
            integrate_fvc(dp, fl, dt);
            // Cập nhật biến toàn cục
            g_dp_pa   = dp;
            g_flow_ls = fl;
            g_fvc_l   = fvc_local;
            g_pf_ls   = pf_local;
        } else {
            // Không đọc được mẫu mới
            g_sensor_ok = false;
        }
        if (Serial.available()) {
            char c = Serial.read();
            if (c == 't' || c == 'T') {
                Serial.println("\n>>> [SERIAL TARE] Dang can lai moc 0 ap suat theo yeu cau...");
                g_tare_done = false;
                fvc_local = 0.0f;
                pf_local  = 0.0f;
            }
        }
        if (now - t_led >= 50) {
            t_led = now;
            update_led_bar((float)g_dp_pa);
        }

        // 3. Cập nhật % dung lượng pin (chu kỳ 5 giây)
        if (now - t_bat >= 5000) {
            t_bat = now;
            g_bat_pct = read_battery_pct();
        }

        // 4. In thông số truyền Bluetooth & Cảm biến ra Serial Monitor định kỳ (250ms)
        if (now - t_serial >= 250) {
            t_serial = now;

            long diff = (long)g_raw_val - (long)g_tare;
            int n_leds = 0;
            float dp_now = (float)g_dp_pa;

            // CHỈ TÍNH SỐ ĐÈN SÁNG KHI THỔI RA (dp DƯƠNG >= NGƯỠNG BÓNG 1)
            if (dp_now >= LED_THRESHOLDS[0]) {
                for (int i = 0; i < NUM_LEDS; i++) {
                    if (dp_now >= LED_THRESHOLDS[i]) {
                        n_leds = i + 1;
                    } else {
                        break;
                    }
                }
            }

            for (int i = 0; i < 8; i++) {
                bar[i + 1] = (i < n_leds) ? '#' : '-';
            }

            Serial.printf("[MONITOR] Raw: %-8ld | Tare: %-8ld | Diff: %+7ld | dP: %5.1f Pa | LED: %s (%d/8) | BLE: %s\n",
                          (long)g_raw_val, (long)g_tare, diff,
                          (float)g_dp_pa, bar, n_leds,
                          deviceConnected ? "DA KET NOI" : "CHO KET NOI");

            // Cảnh báo nếu cảm biến mất kết nối
            if (!g_sensor_ok && g_tare_done) {
                Serial.println("   >>> [CANH BAO] Cam bien HX710B mat ket noi hoac bi tre! Kiem tra day DOUT / SCK!");
            }

            // Báo hiệu nổi bật CHỈ KHI NGƯỜI DÙNG THỰC SỰ THỔI RA (dp DƯƠNG)
            if (dp_now >= LED_THRESHOLDS[0]) {
                Serial.printf("   >>> [DANG THOI] Diff: %+ld xung | dP: %.1f Pa | Flow: %.2f L/s | Sang: %d/8 LED\n",
                              diff, dp_now, (float)g_flow_ls, n_leds);
            }
        }

        vTaskDelay(pdMS_TO_TICKS(10)); // Vòng lặp cảm biến ~100Hz
    }
}

// ============================================================
// KHỞI ĐỘNG HỆ THỐNG (SETUP)
// ============================================================
void setup() {
    Serial.begin(115200);
    delay(500);
    Serial.println("\n=============================================================");
    Serial.println("  SMART SPIROMETER - THIET BI DO CHUC NANG PHOI v2.3");
    Serial.println("=============================================================");

    // Cấu hình ADC đo pin
    analogReadResolution(12);
    analogSetAttenuation(ADC_11db);

    FastLED.addLeds<LED_TYPE, LED_PIN, COLOR_ORDER>(leds, NUM_LEDS).setCorrection(TypicalLEDStrip);
    FastLED.setBrightness(BRIGHTNESS);
    FastLED.clear();
    FastLED.show();

    // ========================================================
    // TEST PHẦN CỨNG 2 MÀU XANH LÁ & ĐỎ KHI KHỞI ĐỘNG:
    // ========================================================
    Serial.println("[LED TEST] 1. Bat toan bo 8 bong XANH LA (0.8 giay)...");
    fill_solid(leds, NUM_LEDS, CRGB::Red);
    FastLED.show();
    delay(800);

    Serial.println("[LED TEST] 2. Bat toan bo 8 bong DO (0.8 giay)...");
    fill_solid(leds, NUM_LEDS, CRGB::Red);
    FastLED.show();
    delay(800);

    FastLED.clear();
    leds[0] = CRGB::Yellow;
    FastLED.show();
    Serial.println("[LED TEST] Xong! Bong so 1 dang sang DO SAN SANG.");

    // Khởi tạo 2 Task FreeRTOS trên 2 Core
    xTaskCreatePinnedToCore(TaskBLECode,    "TaskBLE",    10000, nullptr, 1, &hTaskBLE,    0);
    xTaskCreatePinnedToCore(TaskSensorCode, "TaskSensor",  8192, nullptr, 2, &hTaskSensor, 1);

    Serial.println("[SETUP] Khoi tao xong cac Task! Dang bat dau do...");
}

// ============================================================
// LOOP (FreeRTOS quản lý, loopTask xóa để giải phóng RAM)
// ============================================================
void loop() {
    vTaskDelete(nullptr);
}
