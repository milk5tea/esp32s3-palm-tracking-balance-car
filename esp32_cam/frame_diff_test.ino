// ============================================================
// frame_diff_test — ESP32-S3 CAM 帧差法 + 卡尔曼多目标追踪
// 纯串口输出（115200），不联网
// MOTION 数据从板载 SDA 口（GPIO47）发出接 STM32 的 PA10（板上RX口）；USB 串口仅作调试
//
// 验证目标：
//   1. 稳定运行，不卡死、不重启
//   2. 串口持续输出 MOTION:n;id,x,y,area,miss
//   3. 挥手输出 1~3 个目标，ID 稳定
// ============================================================
#include "esp_camera.h"
#include "esp_heap_caps.h"
#include <Arduino.h>

// MOTION 数据串口：复用板载 SDA/SCL 口（GPIO47/48），与 USB 调试串口互不干扰
HardwareSerial VisionSerial(1);

// ========== 摄像头引脚（ESP32-S3 CAM 通用布局） ==========
// 若与你之前跑通例程的引脚不一致，以跑通的为准替换
#define PWDN_GPIO_NUM     -1
#define RESET_GPIO_NUM    -1
#define XCLK_GPIO_NUM     15
#define SIOD_GPIO_NUM     4
#define SIOC_GPIO_NUM     5
#define Y9_GPIO_NUM       16
#define Y8_GPIO_NUM       17
#define Y7_GPIO_NUM       18
#define Y6_GPIO_NUM       12
#define Y5_GPIO_NUM       10
#define Y4_GPIO_NUM       8
#define Y3_GPIO_NUM       9
#define Y2_GPIO_NUM       11
#define VSYNC_GPIO_NUM    6
#define HREF_GPIO_NUM     7
#define PCLK_GPIO_NUM     13

// ========== 视觉数据串口（板载SDA/SCL口，官方例程证实 GPIO47/48 空闲） ==========
#define VISION_TX_PIN  47   // 板载丝印SDA（GPIO47），MOTION数据输出，接STM32的PA10（板上RX口）
#define VISION_RX_PIN  48   // 板载丝印SCL（GPIO48），本程序只发不收，仅占位

// 调试开关：1 = MOTION 同时在 USB 串口回显（联调验证用，正式接车后可改 0 关掉）
#define MOTION_ECHO_USB 1

// ========== 帧差法参数 ==========
#define FRAME_WIDTH     320
#define FRAME_HEIGHT    240
#define DIFF_THRESHOLD  30      // 像素差阈值：高了漏检，低了误报
#define MIN_BLOB_AREA   80      // 最小连通域面积（px²），滤噪点
#define MAX_TARGETS     4       // 最大同时跟踪目标数
#define MAX_MISS        8       // 连续 miss 次数达到该值即删除目标

// 2x2 降采样网格：320x240 -> 160x120
// diff 数组从 75KB 降到 19KB，连通域计算量也降为 1/4
#define GRID_W  (FRAME_WIDTH / 2)   // 160
#define GRID_H  (FRAME_HEIGHT / 2)  // 120
#define GRID_N  (GRID_W * GRID_H)   // 19200

// ========== 卡尔曼跟踪器（恒速模型，标量增益近似） ==========
typedef struct {
  float x, y;
  float vx, vy;
  float P;
  float Q;
  float R;
  int id;
  bool active;
  int miss;
  int area;
} Tracker;

static Tracker trackers[MAX_TARGETS];
static int next_id = 0;
static uint8_t* prev_frame = NULL;

void tracker_init(Tracker* t, float x, float y, int area, int id) {
  t->x = x; t->y = y;
  t->vx = 0; t->vy = 0;
  t->P = 100;
  t->Q = 3.0;
  t->R = 25.0;
  t->id = id;
  t->active = true;
  t->miss = 0;
  t->area = area;
}

void tracker_predict(Tracker* t) {
  t->x += t->vx;
  t->y += t->vy;
  // 限幅在画面内，防止丢失后的惯性预测漂出画面产生负数坐标
  if (t->x < 0) t->x = 0;
  if (t->x > FRAME_WIDTH - 1) t->x = FRAME_WIDTH - 1;
  if (t->y < 0) t->y = 0;
  if (t->y > FRAME_HEIGHT - 1) t->y = FRAME_HEIGHT - 1;
  t->P += t->Q;
}

void tracker_update(Tracker* t, float mx, float my) {
  // 先取新息（测量残差），再做位置修正——速度估计才不会被增益 K 吃掉
  float ix = mx - t->x;
  float iy = my - t->y;
  float K = t->P / (t->P + t->R);
  t->x += K * ix;
  t->y += K * iy;
  t->vx = 0.6 * t->vx + 0.4 * ix;
  t->vy = 0.6 * t->vy + 0.4 * iy;
  t->P *= (1 - K);
}

// ========== 摄像头初始化 ==========
void init_camera() {
  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  config.pin_sscb_sda = SIOD_GPIO_NUM;
  config.pin_sscb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 10000000;  // 10MHz：降低摄像头功耗与电流尖峰，供电不稳时更稳；帧率仍够用
  config.pixel_format = PIXFORMAT_GRAYSCALE;
  config.frame_size = FRAMESIZE_QVGA;
  config.jpeg_quality = 10;
  config.fb_count = 2;
  // 有 PSRAM 时帧缓冲放 PSRAM，避免内部 DRAM 溢出（上次溢出的根源之一）
  config.fb_location = (heap_caps_get_free_size(MALLOC_CAP_SPIRAM) > 0)
                         ? CAMERA_FB_IN_PSRAM : CAMERA_FB_IN_DRAM;

  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("CAM_INIT_ERR: 0x%x\n", err);  // 0x105=找不到摄像头（查引脚/排线/供电）
  }
}

// ========== 连通域检测（160x120 网格 BFS） ==========
int detect_blobs(uint8_t* diff, int* cx, int* cy, int* area) {
  static uint8_t visited[GRID_N];
  static int16_t qx[GRID_N], qy[GRID_N];  // 整格容量，每个格子最多入队一次，绝不越界
  memset(visited, 0, sizeof(visited));

  int count = 0;
  for (int y = 0; y < GRID_H; y++) {
    for (int x = 0; x < GRID_W; x++) {
      int i = y * GRID_W + x;
      if (!diff[i] || visited[i]) continue;

      // BFS 洪泛填充
      int head = 0, tail = 0;
      qx[tail] = x; qy[tail] = y; tail++;
      visited[i] = 1;
      int sx = 0, sy = 0, n = 0;
      while (head < tail) {
        int px = qx[head], py = qy[head];
        head++;
        sx += px; sy += py; n++;

        int nx, ny, ni;
        nx = px + 1; ny = py; ni = ny * GRID_W + nx;
        if (nx < GRID_W && !visited[ni] && diff[ni]) {
          visited[ni] = 1; qx[tail] = nx; qy[tail] = ny; tail++;
        }
        nx = px - 1; ny = py; ni = ny * GRID_W + nx;
        if (nx >= 0 && !visited[ni] && diff[ni]) {
          visited[ni] = 1; qx[tail] = nx; qy[tail] = ny; tail++;
        }
        nx = px; ny = py + 1; ni = ny * GRID_W + nx;
        if (ny < GRID_H && !visited[ni] && diff[ni]) {
          visited[ni] = 1; qx[tail] = nx; qy[tail] = ny; tail++;
        }
        nx = px; ny = py - 1; ni = ny * GRID_W + nx;
        if (ny >= 0 && !visited[ni] && diff[ni]) {
          visited[ni] = 1; qx[tail] = nx; qy[tail] = ny; tail++;
        }
      }

      // 面积过滤 + 质心映射回 320x240 像素坐标
      if (n * 4 >= MIN_BLOB_AREA) {
        cx[count] = sx * 2 / n + 1;
        cy[count] = sy * 2 / n + 1;
        area[count] = n * 4;
        count++;
        if (count >= MAX_TARGETS) return count;
      }
    }
  }
  return count;
}

// ========== 串口输出 ==========
void print_targets() {
  int active_n = 0;
  for (int i = 0; i < MAX_TARGETS; i++) {
    if (trackers[i].active) active_n++;
  }
  VisionSerial.print("MOTION:");
  VisionSerial.print(active_n);
  for (int i = 0; i < MAX_TARGETS; i++) {
    if (trackers[i].active) {
      VisionSerial.print(";");
      VisionSerial.print(trackers[i].id);
      VisionSerial.print(",");
      VisionSerial.print((int)trackers[i].x);
      VisionSerial.print(",");
      VisionSerial.print((int)trackers[i].y);
      VisionSerial.print(",");
      VisionSerial.print(trackers[i].area);
      VisionSerial.print(",");
      VisionSerial.print(trackers[i].miss);
    }
  }
  VisionSerial.println();
#if MOTION_ECHO_USB
  // USB 调试回显，方便在电脑上确认 SDA 口发出的内容
  Serial.print("MOTION:");
  Serial.print(active_n);
  for (int i = 0; i < MAX_TARGETS; i++) {
    if (trackers[i].active) {
      Serial.print(";");
      Serial.print(trackers[i].id);
      Serial.print(",");
      Serial.print((int)trackers[i].x);
      Serial.print(",");
      Serial.print((int)trackers[i].y);
      Serial.print(",");
      Serial.print(trackers[i].area);
      Serial.print(",");
      Serial.print(trackers[i].miss);
    }
  }
  Serial.println();
#endif
}

// ========== 主程序 ==========
void setup() {
  Serial.begin(115200);
  VisionSerial.begin(115200, SERIAL_8N1, VISION_RX_PIN, VISION_TX_PIN);	// MOTION数据走板载SDA口
  delay(500);
  Serial.println("Frame Diff Test Start");
  init_camera();
  for (int i = 0; i < MAX_TARGETS; i++) trackers[i].active = false;
}

void loop() {
  static int fb_err_count = 0;

  camera_fb_t* fb = esp_camera_fb_get();
  if (!fb) {
    fb_err_count++;
    if (fb_err_count > 5) {
      // 连续多次取帧失败，说明摄像头传感器已停摆（多为供电不足导致），尝试重新初始化
      Serial.println("CAM_RESTART");
      esp_camera_deinit();
      init_camera();
      fb_err_count = 0;
      free(prev_frame);   // 重建背景帧
      prev_frame = NULL;
    } else {
      Serial.println("FB_ERR");
    }
    delay(100);
    return;
  }
  fb_err_count = 0;

  // 第一帧：只存背景，不做检测
  if (prev_frame == NULL) {
    prev_frame = (uint8_t*)malloc(fb->len);
    if (!prev_frame) {
      Serial.println("MALLOC_ERR");
      esp_camera_fb_return(fb);
      return;
    }
    memcpy(prev_frame, fb->buf, fb->len);
    esp_camera_fb_return(fb);
    return;
  }

  // 帧差，直接降采样到 160x120 网格
  static uint8_t diff[GRID_N];
  for (int i = 0; i < GRID_N; i++) {
    int x = (i % GRID_W) * 2;
    int y = (i / GRID_W) * 2;
    int d = abs((int)fb->buf[y * FRAME_WIDTH + x] - (int)prev_frame[y * FRAME_WIDTH + x]);
    diff[i] = (d > DIFF_THRESHOLD) ? 255 : 0;
  }

  int cx[MAX_TARGETS], cy[MAX_TARGETS], area[MAX_TARGETS];
  int n = detect_blobs(diff, cx, cy, area);

  bool matched[MAX_TARGETS] = {false};

  // 最近邻匹配
  for (int i = 0; i < n; i++) {
    int best = -1;
    float best_d = 50.0;  // 匹配距离阈值，ID 乱跳时可调到 80
    for (int j = 0; j < MAX_TARGETS; j++) {
      if (!trackers[j].active || matched[j]) continue;
      float d = fabs(trackers[j].x - cx[i]) + fabs(trackers[j].y - cy[i]);
      if (d < best_d) {
        best_d = d;
        best = j;
      }
    }
    if (best != -1) {
      tracker_update(&trackers[best], cx[i], cy[i]);
      trackers[best].area = area[i];
      trackers[best].miss = 0;
      matched[best] = true;
    } else {
      // 新目标
      for (int j = 0; j < MAX_TARGETS; j++) {
        if (!trackers[j].active) {
          tracker_init(&trackers[j], cx[i], cy[i], area[i], next_id++);
          matched[j] = true;
          break;
        }
      }
    }
  }

  // 未匹配目标：预测一步并累计 miss
  for (int j = 0; j < MAX_TARGETS; j++) {
    if (trackers[j].active && !matched[j]) {
      tracker_predict(&trackers[j]);
      trackers[j].miss++;
      if (trackers[j].miss >= MAX_MISS) trackers[j].active = false;  // miss 达到 8 即删除
    }
  }

  print_targets();

  memcpy(prev_frame, fb->buf, fb->len);
  esp_camera_fb_return(fb);
  delay(33);
}
