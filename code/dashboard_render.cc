#include "dashboard_render.h"
#include <GL/gl.h>
#include <cstdio>
#include <cmath>
#include <cstring>

// ---------- 前置声明 ----------
static void drawSimpleNumber(float x, float y, float size, int c);

// ---------- 辅助：画圆 ----------
static void drawCircle(float cx, float cy, float r, int seg) {
    glBegin(GL_TRIANGLE_FAN);
    glVertex2f(cx, cy);
    for (int i = 0; i <= seg; ++i) {
        float ang = 2.0f * M_PI * i / seg;
        glVertex2f(cx + r * cosf(ang), cy + r * sinf(ang));
    }
    glEnd();
}

// ---------- 辅助：画圆弧 ----------
static void drawArc(float cx, float cy, float r, float startAng, float endAng, int seg) {
    glBegin(GL_TRIANGLE_FAN);
    glVertex2f(cx, cy);
    for (int i = 0; i <= seg; ++i) {
        float t = (float)i / seg;
        float ang = startAng + t * (endAng - startAng);
        glVertex2f(cx + r * cosf(ang), cy + r * sinf(ang));
    }
    glEnd();
}

// ---------- 数字绘制（7 段码） ----------
static void drawSimpleNumber(float x, float y, float size, int c) {
    if (c < 0 || c > 9) return;
    glColor4f(1, 1, 1, 1);
    float w = size * 0.5f, h = size;
    // 7 段码顶点表（相对 0,0 左下角）
    struct { float x1, y1, x2, y2; } seg[7] = {
        {0, h, w, h},      // 上
        {w, h, w, h/2},    // 右上
        {w, h/2, w, 0},    // 右下
        {0, 0, w, 0},      // 下
        {0, 0, 0, h/2},    // 左下
        {0, h/2, 0, h},    // 左上
        {0, h/2, w, h/2}   // 中
    };
    // 各数字亮哪些段
    const unsigned char pat[10] = {
        0b0111111, 0b0000110, 0b1011011, 0b1001111,
        0b1100110, 0b1101101, 0b1111101, 0b0000111,
        0b1111111, 0b1101111
    };
    unsigned char bits = pat[c];
    for (int i = 0; i < 7; ++i) {
        if (bits & (1 << i)) {
            glBegin(GL_LINES);
            glVertex2f(x + seg[i].x1, y + seg[i].y1);
            glVertex2f(x + seg[i].x2, y + seg[i].y2);
            glEnd();
        }
    }
}

// ---------- 绘制文本串 ----------
static void drawText(float x, float y, const char* s) {
    float cur = x;
    for (; *s; ++s) {
        if (*s >= '0' && *s <= '9') {
            drawSimpleNumber(cur, y, 12, *s - '0');
            cur += 10;
        } else if (*s == '.') {
            glBegin(GL_POINTS); glVertex2f(cur + 2, y + 2); glEnd();
            cur += 4;
        } else if (*s == ' ') {
            cur += 6;
        } else {               // 其他字符留空
            cur += 8;
        }
    }
}

// ---------- 构造函数 ----------
DashboardRenderer::DashboardRenderer(int ww, int wh) : width_(ww), height_(wh) {}

// ---------- 主渲染 ----------
void DashboardRenderer::render(const DashboardData& d) {
    glPushAttrib(GL_ALL_ATTRIB_BITS);
    glMatrixMode(GL_PROJECTION); glPushMatrix(); glLoadIdentity();
    glOrtho(0, width_, 0, height_, -1, 1);
    glMatrixMode(GL_MODELVIEW);  glPushMatrix(); glLoadIdentity();

    glDisable(GL_DEPTH_TEST);
    glDisable(GL_LIGHTING);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    // 翻转后的基线
    float base_y = height_ - 30;

    drawSpeedometer(150, base_y - 200, 80, d.speed_kmh);
    drawTachometer(350, base_y - 200, 80, d.rpm);
    drawDigitalDisplay(width_ - 250, base_y - 100, d);

    glDisable(GL_BLEND);
    glPopMatrix(); glMatrixMode(GL_PROJECTION); glPopMatrix(); glMatrixMode(GL_MODELVIEW);
    glPopAttrib();
}

// ---------- 速度表 ----------
void DashboardRenderer::drawSpeedometer(float cx, float cy, float r, double speed) {
    const float maxS = 60;
    // 背景
    glColor4f(0.1f, 0.1f, 0.1f, 0.8f);
    drawCircle(cx, cy, r, 36);
    // 刻度 & 数字
    glColor4f(1, 1, 1, 1);
    for (int i = 0; i <= 60; i += 10) {
        float ang = M_PI * 0.75f - (i / maxS) * M_PI * 1.5f;
        float r1 = r * 0.85f, r2 = r * 0.95f;
        glBegin(GL_LINES);
        glVertex2f(cx + r1 * cosf(ang), cy + r1 * sinf(ang));
        glVertex2f(cx + r2 * cosf(ang), cy + r2 * sinf(ang));
        glEnd();
        char lab[8]; snprintf(lab, sizeof(lab), "%d", i);
        float lx = cx + (r + 10) * cosf(ang) - 4;
        float ly = cy + (r + 10) * sinf(ang) - 3;
        drawText(lx, ly, lab);
    }
    // 指针
    float ang = M_PI * 0.75f - (std::min(float(speed), maxS) / maxS) * M_PI * 1.5f;
    glColor4f(1, 0.2f, 0.2f, 1); glLineWidth(3);
    glBegin(GL_LINES);
    glVertex2f(cx, cy); glVertex2f(cx + r * 0.8f * cosf(ang), cy + r * 0.8f * sinf(ang));
    glEnd(); glLineWidth(1);
    // 中心圆
    glColor4f(0.3f, 0.3f, 0.3f, 1); drawCircle(cx, cy, 5, 14);
    // 当前值
    char txt[32]; snprintf(txt, sizeof(txt), "%.0f km/h", speed);
    drawText(cx - 20, cy - r - 25, txt);
}

// ---------- 转速表 ----------
void DashboardRenderer::drawTachometer(float cx, float cy, float r, double rpm) {
    const float maxR = 4000;
    glColor4f(0.1f, 0.1f, 0.15f, 0.8f); drawCircle(cx, cy, r, 36);
    glColor4f(1, 1, 1, 1);
    for (int i = 0; i <= 4; ++i) {
        float ang = M_PI * 0.75f - (i / 4.0f) * M_PI * 1.5f;
        float r1 = r * 0.85f, r2 = r * 0.95f;
        glBegin(GL_LINES);
        glVertex2f(cx + r1 * cosf(ang), cy + r1 * sinf(ang));
        glVertex2f(cx + r2 * cosf(ang), cy + r2 * sinf(ang));
        glEnd();
        char lab[8]; snprintf(lab, sizeof(lab), "%d", i * 1000);
        drawText(cx + (r + 10) * cosf(ang) - 12, cy + (r + 10) * sinf(ang) - 3, lab);
    }
    // 红线区
    float rs = M_PI * 0.75f - (3000.f / maxR) * M_PI * 1.5f;
    float re = M_PI * 0.75f - (4000.f / maxR) * M_PI * 1.5f;
    glColor4f(1, 0, 0, 0.3f); drawArc(cx, cy, r * 0.9f, rs, re, 16);
    // 指针
    float ang = M_PI * 0.75f - (std::min(float(rpm), maxR) / maxR) * M_PI * 1.5f;
    glColor4f(0.2f, 1, 0.2f, 1); glLineWidth(3);
    glBegin(GL_LINES);
    glVertex2f(cx, cy); glVertex2f(cx + r * 0.8f * cosf(ang), cy + r * 0.8f * sinf(ang));
    glEnd(); glLineWidth(1);
    drawCircle(cx, cy, 5, 14);
    char txt[32]; snprintf(txt, sizeof(txt), "%.0f RPM", rpm);
    drawText(cx - 20, cy - r - 25, txt);
}

// ---------- 数字面板 ----------
void DashboardRenderer::drawDigitalDisplay(float x, float y, const DashboardData& d) {
    const float pw = 220, ph = 140;  // 稍微增加面板大小
    // 背景
    glColor4f(0, 0, 0, 0.8f);
    glBegin(GL_QUADS);
    glVertex2f(x, y); glVertex2f(x + pw, y);
    glVertex2f(x + pw, y + ph); glVertex2f(x, y + ph);
    glEnd();
    // 边框
    glColor4f(0.5f, 0.5f, 0.5f, 1);
    glBegin(GL_LINE_LOOP);
    glVertex2f(x, y); glVertex2f(x + pw, y);
    glVertex2f(x + pw, y + ph); glVertex2f(x, y + ph);
    glEnd();

    // 标题
    glColor4f(1, 1, 0, 1); drawText(x + (pw - 8 * 6) / 2, y + ph - 18, "CAR INFO");

    // 油量
    glColor4f(1, 1, 1, 1);
    char buf[32]; snprintf(buf, sizeof(buf), "FUEL %.0f%%", d.fuel);
    drawText(x + 10, y + 105, buf);
    glColor4f(0.2f, 1, 0.2f, 1);
    float fw = (d.fuel / 100.f) * 120;
    glBegin(GL_QUADS);
    glVertex2f(x + 10, y + 115); glVertex2f(x + 10 + fw, y + 115);
    glVertex2f(x + 10 + fw, y + 125); glVertex2f(x + 10, y + 125);
    glEnd();
    glColor4f(0.5f, 0.5f, 0.5f, 1);
    glBegin(GL_LINE_LOOP);
    glVertex2f(x + 10, y + 115); glVertex2f(x + 130, y + 115);
    glVertex2f(x + 130, y + 125); glVertex2f(x + 10, y + 115);
    glEnd();

    // 温度
    snprintf(buf, sizeof(buf), "TEMP %.0fC", d.temperature);
    drawText(x + 10, y + 75, buf);
    float tr = (d.temperature - 70.f) / 40.f;
    if (tr > 0.8f) glColor4f(1, 0.2f, 0.2f, 1);
    else glColor4f(0.2f, 0.5f, 1, 1);
    float tw = tr * 120; if (tw > 120) tw = 120;
    glBegin(GL_QUADS);
    glVertex2f(x + 10, y + 85); glVertex2f(x + 10 + tw, y + 85);
    glVertex2f(x + 10 + tw, y + 95); glVertex2f(x + 10, y + 95);
    glEnd();
    glColor4f(0.5f, 0.5f, 0.5f, 1);
    glBegin(GL_LINE_LOOP);
    glVertex2f(x + 10, y + 85); glVertex2f(x + 130, y + 85);
    glVertex2f(x + 130, y + 95); glVertex2f(x + 10, y + 95);
    glEnd();

    // 位置信息 - 详细显示
    glColor4f(0.2f, 1.0f, 0.8f, 1.0f);
    char pos_buf[64];
    snprintf(pos_buf, sizeof(pos_buf), "POS %.1f %.1f", d.position_x, d.position_y);
    drawText(x + 10, y + 45, pos_buf);
    
    // 高度信息
    glColor4f(0.8f, 0.8f, 1.0f, 1.0f);
    char alt_buf[32];
    snprintf(alt_buf, sizeof(alt_buf), "ALT %.3f", d.position_z);
    drawText(x + 10, y + 25, alt_buf);
    
    // 速度信息
    glColor4f(1.0f, 0.8f, 0.2f, 1.0f);
    char speed_buf[32];
    snprintf(speed_buf, sizeof(speed_buf), "SPD %.2f m/s", d.speed);
    drawText(x + 120, y + 45, speed_buf);
}
