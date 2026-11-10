#include <Arduino.h>
/*
 * Cardputer ADV Ragdoll
 * ---------------------
 * Verlet-integration ragdoll that reacts to the onboard IMU.
 *
 * Physics trick: an accelerometer reads (a_device - g). In the device's own
 * frame every free particle therefore feels an acceleration of exactly
 * -accelReading. So one vector handles BOTH gravity (tilting) and shaking.
 * The gyro adds centrifugal + Euler forces so spinning the device flings
 * the ragdoll outward.
 *
 * Libraries: M5Cardputer, M5Unified, M5GFX (install via Library Manager).
 * Board:     "M5Cardputer" (M5Stack board package).
 *
 * Keys:
 *   SPACE  kick the ragdoll        R  reset pose
 *   X / Y  flip screen X / Y axis  S  swap X/Y axes   Z  flip spin direction
 *   G      cycle gravity strength  H  toggle help
 * (Axis settings are saved to flash, so you only calibrate once.)
 */

#include <M5Cardputer.h>
#include <Preferences.h>

// ------------------------------------------------------------------ config
static const int   W = 240;
static const int   H = 135;

static const float G_PX           = 520.0f;  // pixels/s^2 per 1 g
static const float MAX_G          = 6.0f;    // clamp for violent shakes
static const float DAMP           = 0.996f;  // air drag per substep
static const float BOUNCE         = 0.35f;   // wall restitution
static const float FLOOR_FRICTION = 0.12f;
static const float ROT_SCALE      = 0.6f;    // strength of gyro pseudo-forces
static const int   SUBSTEPS       = 4;
static const int   ITERS          = 5;
static const float FRAME_DT       = 1.0f / 60.0f;
static const float DIZZY_THRESH   = 1.4f;    // g above rest to count as a "shake"

#define RGB565(r, g, b) ((((r) & 0xF8) << 8) | (((g) & 0xFC) << 3) | ((b) >> 3))
static const uint16_t C_BG    = RGB565(18, 22, 34);
static const uint16_t C_FLOOR = RGB565(60, 70, 95);
static const uint16_t C_SKIN  = RGB565(255, 205, 160);
static const uint16_t C_TORSO = RGB565(255, 140, 40);
static const uint16_t C_LEGS  = RGB565(70, 120, 230);
static const uint16_t C_FACE  = RGB565(30, 30, 40);
static const uint16_t C_TEXT  = RGB565(200, 210, 230);

// ------------------------------------------------------------------ model
struct Pt   { float x, y, px, py, r; };
struct Link { uint8_t a, b; float len; bool minOnly; bool visible; float w; uint16_t col; };

enum { HEAD, NECK, PELVIS, ELB_L, HAND_L, ELB_R, HAND_R,
       KNEE_L, FOOT_L, KNEE_R, FOOT_R, NP };

Pt pts[NP];

Link links[] = {
  // visible, rigid bones
  { HEAD,   NECK,   9,  false, true, 2.0f, C_SKIN  },
  { NECK,   PELVIS, 22, false, true, 4.5f, C_TORSO },
  { NECK,   ELB_L,  12, false, true, 2.2f, C_SKIN  },
  { ELB_L,  HAND_L, 11, false, true, 2.0f, C_SKIN  },
  { NECK,   ELB_R,  12, false, true, 2.2f, C_SKIN  },
  { ELB_R,  HAND_R, 11, false, true, 2.0f, C_SKIN  },
  { PELVIS, KNEE_L, 14, false, true, 3.0f, C_LEGS  },
  { KNEE_L, FOOT_L, 14, false, true, 2.6f, C_LEGS  },
  { PELVIS, KNEE_R, 14, false, true, 3.0f, C_LEGS  },
  { KNEE_R, FOOT_R, 14, false, true, 2.6f, C_LEGS  },
  // hidden "min distance" links: keep the body from folding in on itself
  { HEAD,   PELVIS, 27, true, false, 0, 0 },
  { ELB_L,  ELB_R,  16, true, false, 0, 0 },
  { KNEE_L, KNEE_R,  9, true, false, 0, 0 },
  { FOOT_L, FOOT_R,  9, true, false, 0, 0 },
  { NECK,   KNEE_L, 21, true, false, 0, 0 },
  { NECK,   KNEE_R, 21, true, false, 0, 0 },
};
static const int NL = sizeof(links) / sizeof(links[0]);

// ------------------------------------------------------------------ state
M5Canvas    canvas(&M5Cardputer.Display);
Preferences prefs;

struct Settings { bool swapXY; int sgnX; int sgnY; int sgnRot; } cfg = { false, 1, -1, 1 };

float    gMul        = 1.0f;
bool     showHelp    = true;
uint32_t helpUntil   = 0;
uint32_t dizzyUntil  = 0;
uint32_t lastFrame   = 0;
float    shakeLevel  = 0;
float    omega = 0, prevOmega = 0, alpha = 0;
float    fAx = 0, fAy = 0, fAz = 1;   // filtered accel (g)

// ------------------------------------------------------------------ helpers
void saveSettings() {
  prefs.putBool("swap", cfg.swapXY);
  prefs.putInt("sx", cfg.sgnX);
  prefs.putInt("sy", cfg.sgnY);
  prefs.putInt("sr", cfg.sgnRot);
}

void loadSettings() {
  cfg.swapXY = prefs.getBool("swap", cfg.swapXY);
  cfg.sgnX   = prefs.getInt("sx", cfg.sgnX);
  cfg.sgnY   = prefs.getInt("sy", cfg.sgnY);
  cfg.sgnRot = prefs.getInt("sr", cfg.sgnRot);
}

void setPt(int i, float x, float y, float r) {
  pts[i] = { x, y, x, y, r };
}

void resetRagdoll() {
  float cx = W * 0.5f;
  setPt(HEAD,   cx,       30, 7);
  setPt(NECK,   cx,       39, 2);
  setPt(PELVIS, cx + 1,   61, 2);
  setPt(ELB_L,  cx - 10,  45, 2);
  setPt(HAND_L, cx - 16,  55, 2);
  setPt(ELB_R,  cx + 10,  45, 2);
  setPt(HAND_R, cx + 16,  55, 2);
  setPt(KNEE_L, cx - 5,   74, 2);
  setPt(FOOT_L, cx - 6,   88, 2);
  setPt(KNEE_R, cx + 7,   74, 2);
  setPt(FOOT_R, cx + 8,   88, 2);
}

void kick() {
  float ix = (random(-100, 101) / 100.0f) * 5.0f;
  float iy = -(random(40, 101) / 100.0f) * 7.0f;
  for (auto &p : pts) {
    p.px -= ix + (random(-100, 101) / 100.0f) * 1.5f;
    p.py -= iy + (random(-100, 101) / 100.0f) * 1.5f;
  }
}

// ------------------------------------------------------------------ physics
void integrate(float dt, float ax, float ay, float w, float a) {
  for (auto &p : pts) {
    float vx = (p.x - p.px) * DAMP;
    float vy = (p.y - p.py) * DAMP;

    // pseudo-forces from rotation of the device frame
    float rx = p.x - W * 0.5f;
    float ry = p.y - H * 0.5f;
    float fx = ax + (w * w * rx + a * ry) * ROT_SCALE;
    float fy = ay + (w * w * ry - a * rx) * ROT_SCALE;

    float nx = p.x + vx + fx * dt * dt;
    float ny = p.y + vy + fy * dt * dt;

    float minX = p.r, maxX = W - p.r;
    float minY = p.r, maxY = H - 2 - p.r;

    float evx = nx - p.x;
    float evy = ny - p.y;

    if (nx < minX)      { nx = minX; evx = -evx * BOUNCE; }
    else if (nx > maxX) { nx = maxX; evx = -evx * BOUNCE; }

    if (ny < minY) {
      ny = minY; evy = -evy * BOUNCE;
    } else if (ny > maxY) {
      ny = maxY;
      evy = (fabsf(evy) < 0.15f) ? 0 : -evy * BOUNCE;
      evx *= (1.0f - FLOOR_FRICTION);           // floor friction
    }

    p.x = nx;  p.y = ny;
    p.px = nx - evx;
    p.py = ny - evy;
  }
}

void solveLinks() {
  for (int i = 0; i < NL; i++) {
    Link &l = links[i];
    Pt &a = pts[l.a];
    Pt &b = pts[l.b];
    float dx = b.x - a.x, dy = b.y - a.y;
    float d = sqrtf(dx * dx + dy * dy);
    if (d < 0.001f) { dx = 0.01f; dy = 0; d = 0.01f; }
    if (l.minOnly && d >= l.len) continue;
    float k = (d - l.len) / d * 0.5f;
    a.x += dx * k;  a.y += dy * k;
    b.x -= dx * k;  b.y -= dy * k;
  }
  for (auto &p : pts) {                          // keep inside the screen
    p.x = constrain(p.x, p.r, (float)W - p.r);
    p.y = constrain(p.y, p.r, (float)H - 2 - p.r);
  }
}

void stepPhysics(float ax, float ay) {
  float dt = FRAME_DT / SUBSTEPS;
  for (int s = 0; s < SUBSTEPS; s++) {
    integrate(dt, ax, ay, omega, alpha);
    for (int i = 0; i < ITERS; i++) solveLinks();
  }
}

// ------------------------------------------------------------------ drawing
void drawFace(bool dizzy) {
  Pt &h = pts[HEAD], &n = pts[NECK];
  float dx = h.x - n.x, dy = h.y - n.y;
  float d = sqrtf(dx * dx + dy * dy);
  if (d < 0.001f) d = 1;
  dx /= d;  dy /= d;                  // "up" direction of the head
  float qx = -dy, qy = dx;            // sideways

  float ex1 = h.x + dx * 1.5f + qx * 2.8f, ey1 = h.y + dy * 1.5f + qy * 2.8f;
  float ex2 = h.x + dx * 1.5f - qx * 2.8f, ey2 = h.y + dy * 1.5f - qy * 2.8f;
  float mx  = h.x - dx * 3.0f,             my  = h.y - dy * 3.0f;

  if (dizzy) {
    for (int e = 0; e < 2; e++) {
      float x = e ? ex2 : ex1, y = e ? ey2 : ey1;
      canvas.drawLine(x - 1.5f, y - 1.5f, x + 1.5f, y + 1.5f, C_FACE);
      canvas.drawLine(x - 1.5f, y + 1.5f, x + 1.5f, y - 1.5f, C_FACE);
    }
    canvas.fillCircle(mx, my, 2, C_FACE);       // "oh no" mouth
  } else {
    canvas.fillCircle(ex1, ey1, 1, C_FACE);
    canvas.fillCircle(ex2, ey2, 1, C_FACE);
    canvas.drawLine(mx - qx * 2, my - qy * 2, mx + qx * 2, my + qy * 2, C_FACE);
  }
}

void draw(bool dizzy) {
  canvas.fillScreen(C_BG);
  canvas.drawFastHLine(0, H - 1, W, C_FLOOR);

  for (int i = 0; i < NL; i++) {
    Link &l = links[i];
    if (!l.visible) continue;
    canvas.drawWideLine(pts[l.a].x, pts[l.a].y, pts[l.b].x, pts[l.b].y, l.w, l.col);
  }
  // hands & feet
  canvas.fillCircle(pts[HAND_L].x, pts[HAND_L].y, 2, C_SKIN);
  canvas.fillCircle(pts[HAND_R].x, pts[HAND_R].y, 2, C_SKIN);
  canvas.fillCircle(pts[FOOT_L].x, pts[FOOT_L].y, 3, C_FACE);
  canvas.fillCircle(pts[FOOT_R].x, pts[FOOT_R].y, 3, C_FACE);
  // head
  canvas.fillCircle(pts[HEAD].x, pts[HEAD].y, 7, C_SKIN);
  drawFace(dizzy);

  // shake meter
  int bw = (int)(constrain(shakeLevel / 4.0f, 0.0f, 1.0f) * W);
  uint16_t mc = dizzy ? RGB565(255, 80, 60) : RGB565(80, 220, 120);
  canvas.fillRect(0, 0, bw, 2, mc);

  if (showHelp || millis() < helpUntil) {
    canvas.setTextColor(C_TEXT);
    canvas.setTextSize(1);
    canvas.setCursor(3, 6);   canvas.print("SPACE kick  R reset  G gravity");
    canvas.setCursor(3, 16);  canvas.print("X/Y flip  S swap  Z spin  H help");
    canvas.setCursor(3, 26);  canvas.printf("g:%.1f", gMul);
  }
  canvas.pushSprite(0, 0);
}

// ------------------------------------------------------------------ input
void handleKeys() {
  if (!M5Cardputer.Keyboard.isChange() || !M5Cardputer.Keyboard.isPressed()) return;
  auto ks = M5Cardputer.Keyboard.keysState();
  for (char c : ks.word) {
    switch (c) {
      case ' ': kick(); break;
      case 'r': resetRagdoll(); break;
      case 'x': cfg.sgnX = -cfg.sgnX; saveSettings(); break;
      case 'y': cfg.sgnY = -cfg.sgnY; saveSettings(); break;
      case 's': cfg.swapXY = !cfg.swapXY; saveSettings(); break;
      case 'z': cfg.sgnRot = -cfg.sgnRot; saveSettings(); break;
      case 'h': showHelp = !showHelp; break;
      case 'g':
        gMul = (gMul < 0.7f) ? 1.0f : (gMul < 1.2f) ? 1.5f : (gMul < 1.7f) ? 0.3f : 0.5f;
        if (gMul == 0.5f) gMul = 0.5f;
        helpUntil = millis() + 1500;
        break;
    }
  }
}

// ------------------------------------------------------------------ main
void setup() {
  auto mcfg = M5.config();
  M5Cardputer.begin(mcfg, true);
  M5Cardputer.Display.setRotation(1);
  M5Cardputer.Display.fillScreen(C_BG);

  if (!M5.Imu.isEnabled()) {
    M5Cardputer.Display.setTextColor(TFT_RED);
    M5Cardputer.Display.setCursor(4, 4);
    M5Cardputer.Display.print("IMU not found!\nUpdate M5Unified /\nM5Cardputer libs.");
    while (true) delay(1000);
  }

  canvas.setColorDepth(16);
  canvas.createSprite(W, H);

  prefs.begin("ragdoll", false);
  loadSettings();

  randomSeed(micros());
  resetRagdoll();
  helpUntil = millis() + 6000;
  showHelp = false;
  helpUntil = millis() + 6000;
}

void loop() {
  M5Cardputer.update();
  handleKeys();

  uint32_t now = millis();
  if (now - lastFrame < 16) return;
  lastFrame = now;

  // ---- read IMU
  M5.Imu.update();
  float ax, ay, az, gx, gy, gz;
  M5.Imu.getAccel(&ax, &ay, &az);
  M5.Imu.getGyro(&gx, &gy, &gz);

  // light low-pass so sensor noise doesn't jitter the doll
  const float k = 0.65f;
  fAx += k * (ax - fAx);
  fAy += k * (ay - fAy);
  fAz += k * (az - fAz);

  // ---- shake detection (deviation of |a| from 1 g)
  float mag = sqrtf(ax * ax + ay * ay + az * az);
  float dev = fabsf(mag - 1.0f);
  shakeLevel += 0.4f * (dev - shakeLevel);
  if (dev > DIZZY_THRESH) dizzyUntil = now + 1500;
  bool dizzy = now < dizzyUntil;

  // ---- map sensor axes to screen axes, then flip into "effective gravity"
  float sx = cfg.sgnX * (cfg.swapXY ? fAy : fAx);
  float sy = cfg.sgnY * (cfg.swapXY ? fAx : fAy);
  float effX = constrain(-sx, -MAX_G, MAX_G);
  float effY = constrain(-sy, -MAX_G, MAX_G);

  // ---- gyro (about the axis pointing out of the screen)
  float w = cfg.sgnRot * gz * DEG_TO_RAD;           // rad/s
  omega += 0.5f * (w - omega);
  float rawAlpha = (omega - prevOmega) / FRAME_DT;
  prevOmega = omega;
  alpha += 0.3f * (constrain(rawAlpha, -150.0f, 150.0f) - alpha);

  // ---- simulate + draw
  stepPhysics(effX * G_PX * gMul, effY * G_PX * gMul);
  draw(dizzy);
}
