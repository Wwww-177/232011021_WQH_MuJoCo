#ifndef MJPC_DASHBOARD_DATA_H
#define MJPC_DASHBOARD_DATA_H

#include <mujoco/mujoco.h>
#include <cmath>

// 仪表盘数据结构
struct DashboardData {
    double speed;        // 速度 (m/s)
    double speed_kmh;    // 速度 (km/h)
    double rpm;          // 转速 (转/分钟)
    double fuel;         // 油量 (%)
    double temperature;  // 温度 (°C)
    double position_x;   // X位置
    double position_y;   // Y位置
    double position_z;   // Z位置
};

// 从MuJoCo数据中提取仪表盘数据
class DashboardDataExtractor {
public:
    DashboardDataExtractor(const mjModel* model) : m_(model) {
        // 查找车身body的ID
        car_body_id_ = mj_name2id(model, mjOBJ_BODY, "car");
        if (car_body_id_ < 0) {
            printf("警告: 找不到名为'car'的body\n");
        }
        
        // 查找速度传感器
        velocity_sensor_id_ = mj_name2id(model, mjOBJ_SENSOR, "car_velocity");
        if (velocity_sensor_id_ < 0) {
            printf("警告: 找不到名为'car_velocity'的速度传感器\n");
            printf("使用qvel数据代替\n");
        }
        
        printf("仪表盘数据提取器初始化: car_body_id=%d, velocity_sensor_id=%d\n", 
               car_body_id_, velocity_sensor_id_);
    }
    
    // 更新数据 - 修复：确保正确获取速度数据
    void update(const mjData* data, DashboardData& dashboard) {
        double vx = 0.0, vy = 0.0;
        
        if (velocity_sensor_id_ >= 0) {
            // 使用传感器数据
            int sensor_adr = m_->sensor_adr[velocity_sensor_id_];
            if (sensor_adr >= 0 && sensor_adr + 2 < m_->nsensordata) {
                vx = data->sensordata[sensor_adr];
                vy = data->sensordata[sensor_adr + 1];
            } else {
                // 备用方法：直接使用qvel
                if (car_body_id_ >= 0 && car_body_id_ * 6 + 3 < m_->nv) {
                    vx = data->qvel[car_body_id_ * 6 + 3];
                    vy = data->qvel[car_body_id_ * 6 + 4];
                }
            }
        } else if (car_body_id_ >= 0) {
            // 直接使用qvel数据
            if (car_body_id_ * 6 + 3 < m_->nv) {
                vx = data->qvel[car_body_id_ * 6 + 3];
                vy = data->qvel[car_body_id_ * 6 + 4];
            } else if (m_->nv >= 2) {
                // 备选方案
                vx = data->qvel[0];
                vy = data->qvel[1];
            }
        }
        
        // 计算水平速度
        dashboard.speed = sqrt(vx * vx + vy * vy);
        dashboard.speed_kmh = dashboard.speed * 3.6;
        
        // 基础转速 + 速度相关部分
        dashboard.rpm = 800.0 + (dashboard.speed_kmh * 60.0);
        
        // 限制范围
        if (dashboard.rpm < 800) dashboard.rpm = 800;
        if (dashboard.rpm > 8000) dashboard.rpm = 8000;
        
        // 添加一些随机波动使指针更有活力
        static double noise = 0.0;
        noise += 0.1;
        if (noise > 6.28) noise -= 6.28;
        dashboard.rpm += 100.0 * sin(noise);
        
        // 模拟油量
        static double fuel_level = 100.0;
        fuel_level -= dashboard.speed_kmh * 0.00005;
        if (fuel_level < 0) fuel_level = 0;
        dashboard.fuel = fuel_level;
        
        // 模拟温度
        dashboard.temperature = 75.0 + (dashboard.rpm / 8000.0) * 40.0;
        
        // 获取位置
        if (car_body_id_ >= 0 && car_body_id_ < m_->nbody) {
            dashboard.position_x = data->xpos[car_body_id_ * 3];
            dashboard.position_y = data->xpos[car_body_id_ * 3 + 1];
            dashboard.position_z = data->xpos[car_body_id_ * 3 + 2];
        } else {
            dashboard.position_x = data->qpos[0];
            dashboard.position_y = data->qpos[1];
            dashboard.position_z = data->qpos[2];
        }
        
        // 调试输出 - 每30帧输出一次
        static int update_count = 0;
        if (update_count++ % 30 == 0) {
            printf("📊 仪表盘数据: 速度=%.3f m/s (%.1f km/h), RPM=%.0f, 位置=(%.2f,%.2f), vx=%.3f, vy=%.3f\n", 
                   dashboard.speed, dashboard.speed_kmh, dashboard.rpm,
                   dashboard.position_x, dashboard.position_y, vx, vy);
        }
    }
    
private:
    const mjModel* m_;
    int car_body_id_;
    int velocity_sensor_id_;
};

#endif  // MJPC_DASHBOARD_DATA_H
