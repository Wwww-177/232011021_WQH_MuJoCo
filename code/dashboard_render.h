#ifndef MJPC_DASHBOARD_RENDER_H_
#define MJPC_DASHBOARD_RENDER_H_

#include "dashboard_data.h"

class DashboardRenderer {
public:
    DashboardRenderer(int window_width, int window_height);
    void render(const DashboardData& data);

private:
    int width_, height_;

    // 辅助绘制
    void drawSpeedometer(float cx, float cy, float radius, double speed);
    void drawTachometer(float cx, float cy, float radius, double rpm);
    void drawDigitalDisplay(float x, float y, const DashboardData& data);
};

#endif  // MJPC_DASHBOARD_RENDER_H_
