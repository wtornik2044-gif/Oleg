/* ESP32-CAM + Edge Impulse
 * Обнаружение загрязнения и передача цифрового сигнала на Arduino Mega.
 *
 * Подключение к Arduino Mega:
 *   ESP32-CAM GPIO13  -> Arduino Mega D44
 *   ESP32-CAM GND     -> Arduino Mega GND
 *   ESP32-CAM 5V      -> стабильное питание 5 В
 */

#include <Kyrtka1111-project-1_inferencing.h>
#include "edge-impulse-sdk/dsp/image/image.hpp"
#include "esp_camera.h"

// ================= НАСТРОЙКИ СИГНАЛА ДЛЯ ARDUINO =================
// GPIO13 выбран как отдельный цифровой выход.
// При обнаружении загрязнения на нём появляется HIGH.
#define SIGNAL_PIN 13

// Сколько держать HIGH для Arduino, мс.
// Долгую паузу 5 секунд лучше делать на Arduino, а не на ESP32-CAM.
const unsigned long SIGNAL_PULSE_MS = 300;

// Порог уверенности модели.
// Если нужный класс найден с вероятностью выше порога, ESP32-CAM подаст сигнал.
const float DETECTION_THRESHOLD = 0.70f;

// Названия классов загрязнения.
// ВАЖНО: названия должны совпадать с классами в Edge Impulse.
// Если у тебя классы называются иначе, поменяй строки ниже.
const char* TARGET_LABEL_1 = "oil";
const char* TARGET_LABEL_2 = "dirt";
const char* TARGET_LABEL_3 = "dirty";
const char* TARGET_LABEL_4 = "line";

// ================= КОНФИГУРАЦИЯ ESP32-CAM AI THINKER =================
#define CAMERA_MODEL_AI_THINKER

#if defined(CAMERA_MODEL_AI_THINKER)
#define PWDN_GPIO_NUM     32
#define RESET_GPIO_NUM    -1
#define XCLK_GPIO_NUM      0
#define SIOD_GPIO_NUM     26
#define SIOC_GPIO_NUM     27

#define Y9_GPIO_NUM       35
#define Y8_GPIO_NUM       34
#define Y7_GPIO_NUM       39
#define Y6_GPIO_NUM       36
#define Y5_GPIO_NUM       21
#define Y4_GPIO_NUM       19
#define Y3_GPIO_NUM       18
#define Y2_GPIO_NUM        5
#define VSYNC_GPIO_NUM    25
#define HREF_GPIO_NUM     23
#define PCLK_GPIO_NUM     22
#else
#error "Camera model not selected"
#endif

// ================= ПАРАМЕТРЫ КАДРА =================
#define EI_CAMERA_RAW_FRAME_BUFFER_COLS  320
#define EI_CAMERA_RAW_FRAME_BUFFER_ROWS  240
#define EI_CAMERA_FRAME_BYTE_SIZE        3

static bool debug_nn = false;
static bool is_initialised = false;
uint8_t *snapshot_buf = nullptr;

static camera_config_t camera_config = {
    .pin_pwdn = PWDN_GPIO_NUM,
    .pin_reset = RESET_GPIO_NUM,
    .pin_xclk = XCLK_GPIO_NUM,
    .pin_sscb_sda = SIOD_GPIO_NUM,
    .pin_sscb_scl = SIOC_GPIO_NUM,

    .pin_d7 = Y9_GPIO_NUM,
    .pin_d6 = Y8_GPIO_NUM,
    .pin_d5 = Y7_GPIO_NUM,
    .pin_d4 = Y6_GPIO_NUM,
    .pin_d3 = Y5_GPIO_NUM,
    .pin_d2 = Y4_GPIO_NUM,
    .pin_d1 = Y3_GPIO_NUM,
    .pin_d0 = Y2_GPIO_NUM,
    .pin_vsync = VSYNC_GPIO_NUM,
    .pin_href = HREF_GPIO_NUM,
    .pin_pclk = PCLK_GPIO_NUM,

    // 10 МГц снижает нагрев и обычно стабильнее для ESP32-CAM.
    .xclk_freq_hz = 10000000,
    .ledc_timer = LEDC_TIMER_0,
    .ledc_channel = LEDC_CHANNEL_0,

    .pixel_format = PIXFORMAT_JPEG,
    .frame_size = FRAMESIZE_QVGA,
    .jpeg_quality = 12,
    .fb_count = 1,
    .fb_location = CAMERA_FB_IN_PSRAM,
    .grab_mode = CAMERA_GRAB_WHEN_EMPTY,
};

// ================= ПРОТОТИПЫ =================
bool ei_camera_init(void);
void ei_camera_deinit(void);
bool ei_camera_capture(uint32_t img_width, uint32_t img_height, uint8_t *out_buf);
static int ei_camera_get_data(size_t offset, size_t length, float *out_ptr);

bool is_target_label(const char* label);
void send_dirty_signal(void);
void print_results(const ei_impulse_result_t &result);
bool detect_dirty(const ei_impulse_result_t &result);

// ================= SETUP =================
void setup() {
    Serial.begin(115200);
    delay(1000);

    pinMode(SIGNAL_PIN, OUTPUT);
    digitalWrite(SIGNAL_PIN, LOW);

    Serial.println();
    Serial.println("ESP32-CAM + Edge Impulse");
    Serial.println("Camera model: AI Thinker");
    Serial.print("Signal output GPIO: ");
    Serial.println(SIGNAL_PIN);

    if (!ei_camera_init()) {
        Serial.println("Failed to initialize camera");
        return;
    }

    Serial.println("Camera initialized");
    Serial.println("Starting inference...");
}

// ================= LOOP =================
void loop() {
    if (ei_sleep(5) != EI_IMPULSE_OK) {
        return;
    }

    snapshot_buf = (uint8_t*)malloc(
        EI_CAMERA_RAW_FRAME_BUFFER_COLS *
        EI_CAMERA_RAW_FRAME_BUFFER_ROWS *
        EI_CAMERA_FRAME_BYTE_SIZE
    );

    if (snapshot_buf == nullptr) {
        Serial.println("ERR: Failed to allocate snapshot buffer");
        delay(1000);
        return;
    }

    ei::signal_t signal;
    signal.total_length = EI_CLASSIFIER_INPUT_WIDTH * EI_CLASSIFIER_INPUT_HEIGHT;
    signal.get_data = &ei_camera_get_data;

    if (!ei_camera_capture(
            (size_t)EI_CLASSIFIER_INPUT_WIDTH,
            (size_t)EI_CLASSIFIER_INPUT_HEIGHT,
            snapshot_buf)) {
        Serial.println("Failed to capture image");
        free(snapshot_buf);
        snapshot_buf = nullptr;
        delay(500);
        return;
    }

    ei_impulse_result_t result = { 0 };
    EI_IMPULSE_ERROR err = run_classifier(&signal, &result, debug_nn);

    if (err != EI_IMPULSE_OK) {
        Serial.print("ERR: Failed to run classifier: ");
        Serial.println(err);
        free(snapshot_buf);
        snapshot_buf = nullptr;
        delay(500);
        return;
    }

    print_results(result);

    if (detect_dirty(result)) {
        Serial.println("DIRTY DETECTED -> signal to Arduino");
        send_dirty_signal();
    } else {
        digitalWrite(SIGNAL_PIN, LOW);
    }

    free(snapshot_buf);
    snapshot_buf = nullptr;

    delay(500);
}

// ================= ЛОГИКА ОБНАРУЖЕНИЯ =================
bool is_target_label(const char* label) {
    if (strcmp(label, TARGET_LABEL_1) == 0) return true;
    if (strcmp(label, TARGET_LABEL_2) == 0) return true;
    if (strcmp(label, TARGET_LABEL_3) == 0) return true;
    if (strcmp(label, TARGET_LABEL_4) == 0) return true;
    return false;
}

bool detect_dirty(const ei_impulse_result_t &result) {
#if EI_CLASSIFIER_OBJECT_DETECTION == 1
    for (uint32_t i = 0; i < result.bounding_boxes_count; i++) {
        ei_impulse_result_bounding_box_t bb = result.bounding_boxes[i];

        if (bb.value == 0) {
            continue;
        }

        if (is_target_label(bb.label) && bb.value >= DETECTION_THRESHOLD) {
            return true;
        }
    }
    return false;
#else
    for (uint16_t i = 0; i < EI_CLASSIFIER_LABEL_COUNT; i++) {
        const char* label = ei_classifier_inferencing_categories[i];
        float value = result.classification[i].value;

        if (is_target_label(label) && value >= DETECTION_THRESHOLD) {
            return true;
        }
    }
    return false;
#endif
}

void send_dirty_signal(void) {
    digitalWrite(SIGNAL_PIN, HIGH);
    delay(SIGNAL_PULSE_MS);
    digitalWrite(SIGNAL_PIN, LOW);
}

void print_results(const ei_impulse_result_t &result) {
    Serial.print("Predictions. DSP: ");
    Serial.print(result.timing.dsp);
    Serial.print(" ms, Classification: ");
    Serial.print(result.timing.classification);
    Serial.print(" ms, Anomaly: ");
    Serial.print(result.timing.anomaly);
    Serial.println(" ms");

#if EI_CLASSIFIER_OBJECT_DETECTION == 1
    for (uint32_t i = 0; i < result.bounding_boxes_count; i++) {
        ei_impulse_result_bounding_box_t bb = result.bounding_boxes[i];

        if (bb.value == 0) {
            continue;
        }

        Serial.print("  ");
        Serial.print(bb.label);
        Serial.print(": ");
        Serial.print(bb.value, 5);
        Serial.print(" [x=");
        Serial.print(bb.x);
        Serial.print(", y=");
        Serial.print(bb.y);
        Serial.print(", w=");
        Serial.print(bb.width);
        Serial.print(", h=");
        Serial.print(bb.height);
        Serial.println("]");
    }
#else
    for (uint16_t i = 0; i < EI_CLASSIFIER_LABEL_COUNT; i++) {
        Serial.print("  ");
        Serial.print(ei_classifier_inferencing_categories[i]);
        Serial.print(": ");
        Serial.println(result.classification[i].value, 5);
    }
#endif

#if EI_CLASSIFIER_HAS_ANOMALY
    Serial.print("Anomaly: ");
    Serial.println(result.anomaly, 3);
#endif
}

// ================= КАМЕРА =================
bool ei_camera_init(void) {
    if (is_initialised) {
        return true;
    }

    esp_err_t err = esp_camera_init(&camera_config);
    if (err != ESP_OK) {
        Serial.printf("Camera init failed with error 0x%x\n", err);
        return false;
    }

    sensor_t *s = esp_camera_sensor_get();

    if (s == nullptr) {
        Serial.println("ERR: Failed to get camera sensor");
        return false;
    }

    // Настройки можно менять под фактическую ориентацию камеры в корпусе.
    // Если изображение перевёрнуто, поменяй 0 на 1.
    s->set_vflip(s, 0);
    s->set_hmirror(s, 0);
    s->set_brightness(s, 1);
    s->set_saturation(s, 0);
    s->set_awb_gain(s, 1);

    is_initialised = true;
    return true;
}

void ei_camera_deinit(void) {
    esp_err_t err = esp_camera_deinit();

    if (err != ESP_OK) {
        Serial.println("Camera deinit failed");
        return;
    }

    is_initialised = false;
}

bool ei_camera_capture(uint32_t img_width, uint32_t img_height, uint8_t *out_buf) {
    if (!is_initialised) {
        Serial.println("ERR: Camera is not initialized");
        return false;
    }

    camera_fb_t *fb = esp_camera_fb_get();

    if (!fb) {
        Serial.println("Camera capture failed");
        return false;
    }

    bool converted = fmt2rgb888(fb->buf, fb->len, PIXFORMAT_JPEG, out_buf);

    esp_camera_fb_return(fb);

    if (!converted) {
        Serial.println("Conversion failed");
        return false;
    }

    if ((img_width != EI_CAMERA_RAW_FRAME_BUFFER_COLS) ||
        (img_height != EI_CAMERA_RAW_FRAME_BUFFER_ROWS)) {
        ei::image::processing::crop_and_interpolate_rgb888(
            out_buf,
            EI_CAMERA_RAW_FRAME_BUFFER_COLS,
            EI_CAMERA_RAW_FRAME_BUFFER_ROWS,
            out_buf,
            img_width,
            img_height
        );
    }

    return true;
}

static int ei_camera_get_data(size_t offset, size_t length, float *out_ptr) {
    size_t pixel_ix = offset * 3;
    size_t pixels_left = length;
    size_t out_ptr_ix = 0;

    while (pixels_left != 0) {
        out_ptr[out_ptr_ix] =
            (snapshot_buf[pixel_ix + 2] << 16) +
            (snapshot_buf[pixel_ix + 1] << 8) +
             snapshot_buf[pixel_ix];

        out_ptr_ix++;
        pixel_ix += 3;
        pixels_left--;
    }

    return 0;
}

#if !defined(EI_CLASSIFIER_SENSOR) || EI_CLASSIFIER_SENSOR != EI_CLASSIFIER_SENSOR_CAMERA
#error "Invalid model for current sensor"
#endif
