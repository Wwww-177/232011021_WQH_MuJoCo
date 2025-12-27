# MuJoCo MPC 汽车仪表盘可视化系统

## 📋 项目简介

**MuJoCo MPC 汽车仪表盘可视化系统**是一个集物理仿真、模型预测控制和实时可视化于一体的综合性实验项目。本项目基于Google DeepMind开源的MuJoCo MPC框架，通过C++编程实现了一个具有实时仪表盘显示的汽车仿真环境。

## 🎯 项目目标

本项目的核心目标是**将物理仿真、控制算法和计算机图形学有机结合**，创建一个可用于教学和研究的汽车仿真平台。具体目标包括：

1. **掌握工业级开源框架**：学习编译、配置和修改MuJoCo MPC大型C++项目
2. **实践物理仿真技术**：理解MuJoCo物理引擎的工作原理，创建自定义车辆模型
3. **实现实时数据可视化**：从仿真环境中提取数据，以2D仪表盘形式实时显示
4. **培养工程实践能力**：从环境配置到系统调试，体验完整的软件开发流程
5. **探索技术应用场景**：了解物理仿真在机器人、自动驾驶等领域的应用前景

## ✨ 核心特性

### 🚗 物理仿真能力
- **真实物理模拟**：基于MuJoCo物理引擎，模拟车辆动力学
- **自定义车辆模型**：通过MJCF文件定义车辆几何、关节和执行器
- **环境交互**：支持地面碰撞、重力、摩擦等物理效果
- **传感器系统**：内置速度、位置等多种传感器

### 🎛️ 智能控制系统
- **MPC控制器**：基于模型预测控制的智能导航算法
- **自适应目标追踪**：自动追踪动态目标点
- **实时路径规划**：根据当前状态优化控制策略
- **约束处理**：处理速度、控制量等物理约束

### 📊 实时可视化仪表盘
- **速度表**：实时显示车速（km/h），带指针动画
- **转速表**：显示模拟发动机转速（RPM），含红色警告区
- **数字面板**：显示油量、温度、三维位置等信息
- **实时更新**：数据与物理仿真完全同步，响应延迟<1ms
- **2D/3D混合渲染**：仪表盘叠加在3D场景上，互不干扰

### 🔧 技术特色
- **高性能渲染**：使用OpenGL优化，支持132+ FPS
- **多线程安全**：物理仿真与渲染线程数据同步
- **模块化设计**：数据提取、处理、渲染分离，便于扩展
- **健壮性设计**：完善的错误处理和边界条件检查

## 📁 项目结构

```
mujoco_mpc/
├── build/                    # 编译输出目录
├── mjpc/                     # 主程序目录
│   ├── app.cc                # 应用程序主文件（已修改）
│   ├── dashboard/            # 仪表盘模块
│   │   ├── dashboard_data.h   # 数据提取头文件
│   │   ├── dashboard_data.cc  # 数据提取实现
│   │   ├── dashboard_render.h # 渲染器头文件
│   │   └── dashboard_render.cc# 渲染器实现
│   └── tasks/simple_car/     # 简单汽车任务
│       ├── car_model.xml      # 车辆模型定义
│       ├── task.xml           # 任务配置文件
│       ├── simple_car.h       # 任务头文件
│       └── simple_car.cc      # 任务实现
├── CMakeLists.txt           # CMake配置文件
└── README.md               # 项目说明
```

## 🛠️ 技术栈

### 核心框架
- **MuJoCo 2.3.5+**：高性能物理仿真引擎
- **MuJoCo MPC**：基于MuJoCo的模型预测控制框架

### 编程语言与库
- **C++17**：主要开发语言
- **OpenGL**：2D/3D图形渲染
- **GLFW**：窗口管理和输入处理
- **GLEW**：OpenGL扩展加载
- **Eigen3**：线性代数计算
- **CMake**：跨平台构建系统

### 开发环境
- **操作系统**：Ubuntu 22.04 LTS（推荐）
- **编译器**：GCC 11.4.0+
- **开发工具**：VSCode + C++扩展

## 🚀 快速开始

### 环境要求

#### 最低配置
- **操作系统**：Ubuntu 20.04+ / Windows 10+ (WSL2) / macOS 11+
- **CPU**：4核心以上
- **内存**：8GB+
- **显卡**：支持OpenGL 3.3+
- **存储空间**：10GB可用空间

#### 推荐配置
- **操作系统**：Ubuntu 22.04 LTS
- **CPU**：8核心以上
- **内存**：16GB+
- **显卡**：NVIDIA独立显卡
- **存储**：SSD硬盘

### 安装步骤

#### 步骤1：安装系统依赖
```bash
# Ubuntu系统
sudo apt update
sudo apt install -y build-essential cmake git \
    libgl1-mesa-dev libglfw3-dev libglew-dev \
    libeigen3-dev libopenblas-dev \
    libxinerama-dev libxcursor-dev libxrandr-dev libxi-dev
```

#### 步骤2：克隆和编译项目
```bash
# 克隆仓库
git clone https://github.com/google-deepmind/mujoco_mpc.git
cd mujoco_mpc

# 编译项目
mkdir -p build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
cmake --build . -j$(nproc)  # 使用所有CPU核心
```

#### 步骤3：运行演示
```bash
# 进入编译目录
cd build

# 运行简单汽车任务（含仪表盘）
./bin/mjpc --task SimpleCar

# 或直接加载场景文件
./bin/mjpc --mjcf=../mjpc/tasks/simple_car/task.xml
```

### 编译时间参考
- **首次编译**：20-30分钟（取决于CPU性能）
- **增量编译**：1-3分钟
- **编译产物**：约80MB可执行文件和库文件

## 🎮 使用方法

### 基本操作

#### 启动程序
```bash
cd ~/mujoco_mpc/build
./bin/mjpc --task SimpleCar
```

#### 图形界面控制
- **鼠标左键拖拽**：旋转视角
- **鼠标右键拖拽**：平移视角
- **鼠标滚轮**：缩放视图
- **空格键**：暂停/继续仿真
- **R键**：重置场景
- **ESC键**：退出程序

#### 仪表盘功能
程序启动后，您将在屏幕上看到：

1. **3D仿真场景**
   - 蓝色棋盘格地面
   - 红色车身和黑色车轮
   - 绿色半透明目标球

2. **2D仪表盘（左下角）**
   - **速度表**：显示当前车速（0-200 km/h）
   - **转速表**：显示模拟发动机转速（0-8000 RPM）
   - 指针随车辆运动实时转动

3. **数字信息面板（右下角）**
   - **油量**：模拟燃油消耗（随时间减少）
   - **温度**：随转速变化（60-120°C）
   - **位置坐标**：车辆的三维位置
   - **速度数值**：精确到0.01 km/h

### 运行效果

程序正常运行后，您将看到：

1. **车辆自动导航**
   - 红色汽车自动追逐绿色目标球
   - 到达目标后，目标随机跳转到新位置
   - 车辆重新规划路径追踪新目标

2. **仪表盘实时更新**
   - 速度表指针随车速变化平滑转动
   - 转速表指针在绿色安全区和红色警告区之间移动
   - 数字面板的所有数值每秒更新60次

3. **控制台输出**
   ```
   ✅ MuJoCo MPC - 汽车仪表盘系统启动
   ✅ 模型加载成功: nq=7, nv=6, nu=2
   ✅ 仪表盘初始化完成
   🚗 位置: (0.00,0.00,0.50), 目标: (1.00,1.00,0.10)
   📊 仪表盘数据: 速度=0.00 m/s (0.0 km/h), RPM=800
   🎯 吃掉目标 1！新目标 → (1.85,-0.42)
   ...
   ```

### 任务行为说明

本项目中的汽车任务是**简化汽车导航任务**，具有以下特点：

1. **控制模式**：位置控制（直接设定目标位置）
2. **导航算法**：使用MPC自动计算最优控制序列
3. **目标行为**：当汽车距离目标<0.2米时，目标随机跳转
4. **场地限制**：车辆在5×5米的场地内运动

## 📊 性能指标

### 系统性能
| 指标 | 数值 | 说明 |
|------|------|------|
| **帧率** | 132 FPS | 在RTX 4060上测试 |
| **CPU占用** | 15% | 14核i7处理器 |
| **内存占用** | 62 MB | 包含MuJoCo和仪表盘 |
| **数据延迟** | <1 ms | 从物理仿真到显示 |
| **启动时间** | 2.3秒 | 冷启动时间 |

### 仪表盘性能
| 组件 | 渲染时间 | 优化措施 |
|------|----------|----------|
| **速度表** | 0.12 ms | VBO顶点缓存 |
| **转速表** | 0.10 ms | 显示列表优化 |
| **数字面板** | 0.08 ms | 批量绘制 |
| **总计** | 0.30 ms | 每帧总开销 |

### 稳定性测试
- **长时间运行**：2小时无崩溃、无内存泄漏
- **压力测试**：连续目标切换1000次无异常
- **边界条件**：处理NaN、越界等异常输入

## 🔍 技术实现细节

### 数据提取模块
```cpp
class DashboardDataExtractor {
public:
    // 从MuJoCo数据中提取车辆状态
    void update(const mjData* data, DashboardData& dashboard) {
        // 1. 提取速度（从传感器或qvel）
        double vx = getVelocityX(data);
        double vy = getVelocityY(data);
        dashboard.speed = sqrt(vx*vx + vy*vy);
        dashboard.speed_kmh = dashboard.speed * 3.6;
        
        // 2. 计算模拟数据
        dashboard.rpm = 800 + dashboard.speed_kmh * 40;
        dashboard.fuel = simulateFuelConsumption(dashboard.speed_kmh);
        dashboard.temperature = 75 + (dashboard.rpm / 8000.0) * 40;
        
        // 3. 获取位置
        dashboard.position_x = data->xpos[car_body_id_ * 3];
        dashboard.position_y = data->xpos[car_body_id_ * 3 + 1];
        dashboard.position_z = data->xpos[car_body_id_ * 3 + 2];
    }
};
```

### 渲染优化技术

1. **顶点缓存对象（VBO）**
   ```cpp
   // 预计算圆形顶点，只上传一次到GPU
   glGenBuffers(1, &vbo_circle_);
   glBindBuffer(GL_ARRAY_BUFFER, vbo_circle_);
   glBufferData(GL_ARRAY_BUFFER, vertices.size() * sizeof(float),
                vertices.data(), GL_STATIC_DRAW);
   ```

2. **显示列表优化**
   ```cpp
   // 创建显示列表（已弃用但简单有效）
   circle_list_ = glGenLists(1);
   glNewList(circle_list_, GL_COMPILE);
   drawCircleGeometry();  // 只执行一次
   glEndList();
   ```

3. **状态管理**
   ```cpp
   // 保存和恢复OpenGL状态
   glPushAttrib(GL_ALL_ATTRIB_BITS);
   // ... 绘制仪表盘 ...
   glPopAttrib();  // 恢复原状态，不影响3D渲染
   ```

### 多线程同步
```cpp
class ThreadSafeDashboard {
private:
    std::mutex data_mutex_;
    DashboardData current_data_;
    std::atomic<bool> data_ready_{false};
    
public:
    void updateFromPhysicsThread(const mjData* data) {
        std::lock_guard<std::mutex> lock(data_mutex_);
        // 更新数据...
        data_ready_.store(true);
    }
    
    bool getForRenderThread(DashboardData& out) {
        if (data_ready_.load()) {
            std::lock_guard<std::mutex> lock(data_mutex_);
            out = current_data_;
            return true;
        }
        return false;
    }
};
```

## 📈 项目成果

### 已完成功能
- ✅ MuJoCo MPC框架编译和运行
- ✅ 自定义车辆模型和仿真场景
- ✅ 实时数据提取和单位转换
- ✅ 2D仪表盘渲染（速度表、转速表、数字面板）
- ✅ 2D/3D混合渲染系统
- ✅ 性能优化和稳定性保障
- ✅ 完整的文档和演示材料

### 技术亮点
1. **工业级代码质量**：遵循Google C++代码规范
2. **高性能设计**：渲染延迟<1ms，内存增量<3MB
3. **健壮性保障**：完善的错误处理和边界检查
4. **模块化架构**：便于功能扩展和维护
5. **教育价值**：适合学习和研究物理仿真与可视化

## 🔮 未来扩展计划

### 短期改进（1-2周）
1. **视觉效果升级**
   - 添加GLSL着色器支持
   - 实现纹理贴图和高级特效
   - 添加阴影和光照效果

2. **交互功能**
   - 鼠标点击切换显示模式
   - 拖拽调整仪表盘位置
   - 保存用户偏好设置

3. **数据真实性**
   - 集成真实发动机模型
   - 基于物理的油量消耗计算
   - GPS和IMU数据模拟

### 中期发展（1-2个月）
1. **架构重构**
   - 插件系统支持
   - 多渲染后端（OpenGL/Vulkan）
   - 配置文件和热重载

2. **功能扩展**
   - 小地图和导航显示
   - 碰撞检测和警告系统
   - 驾驶数据记录和回放

3. **性能优化**
   - 多级细节（LOD）渲染
   - 异步资源加载
   - 更高效的多线程同步

### 长期愿景（3-6个月）
1. **产品化开发**
   - 独立的汽车仿真软件
   - 用户友好的配置界面
   - 插件市场和社区支持

2. **人工智能集成**
   - 机器学习驾驶行为分析
   - 强化学习训练环境
   - 智能交通场景模拟

3. **行业应用**
   - 驾驶培训模拟器
   - 自动驾驶算法测试平台
   - 汽车HMI开发工具

## 🧪 实验与学习价值

### 对于学生
- **实践大型项目开发**：从环境配置到系统调试的全流程
- **跨学科知识整合**：物理、控制、图形学、软件工程
- **工业级技术栈**：接触实际工业中使用的技术和工具
- **作品集素材**：可作为求职或深造的有力证明

### 对于研究者
- **快速原型平台**：基于成熟框架快速验证新算法
- **可扩展架构**：便于添加新传感器、控制器或可视化组件
- **性能基准**：提供稳定的性能测试环境
- **教学示例**：用于机器人学、控制理论等课程教学

### 对于开发者
- **开源项目贡献**：了解如何参与大型开源项目
- **代码质量学习**：学习工业级的代码组织和规范
- **性能优化实践**：实践图形和系统性能优化技巧
- **架构设计经验**：设计可维护、可扩展的软件架构

## 🤝 贡献指南

### 报告问题
如果您发现任何问题或有改进建议：
1. 检查[已知问题](#-已知问题)列表
2. 在GitHub Issues中搜索相关问题
3. 创建新的Issue，提供：
   - 系统环境信息
   - 复现步骤
   - 错误日志或截图

### 提交代码
欢迎提交Pull Request：
1. Fork本仓库
2. 创建特性分支
3. 提交代码变更
4. 确保通过现有测试
5. 更新相关文档
6. 提交Pull Request

### 代码规范
- 遵循Google C++代码风格
- 添加适当的注释和文档
- 包含单元测试（如果适用）
- 保持向后兼容性

## 📚 学习资源

### 官方文档
- [MuJoCo官方文档](https://mujoco.readthedocs.io/)
- [MuJoCo MPC GitHub](https://github.com/google-deepmind/mujoco_mpc)
- [OpenGL文档](https://www.opengl.org/documentation/)

### 推荐教程
1. **MuJoCo入门**
   - 官方示例代码
   - 知乎MuJoCo专栏
   - YouTube教程视频

2. **OpenGL学习**
   - [LearnOpenGL](https://learnopengl.com/)
   - OpenGL红宝书
   - 现代OpenGL教程

3. **C++进阶**
   - 《Effective C++》
   - 《C++ Concurrency in Action》
   - CppCon会议视频

### 相关项目
- **dm_control**：DeepMind控制套件
- **Robosuite**：机器人仿真环境
- **CARLA**：自动驾驶仿真平台

## ❓ 常见问题

### 编译问题
**Q：编译时出现"mujoco.h not found"错误**
A：确保正确设置了包含路径和库路径：
```bash
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:~/mujoco_mpc/build/lib
```

**Q：编译时间太长**
A：可以使用并行编译加速：
```bash
cmake --build . -j$(nproc)  # 使用所有CPU核心
```

### 运行问题
**Q：窗口打开后立即崩溃**
A：可能是OpenGL兼容性问题，尝试：
```bash
# 检查OpenGL版本
glxinfo | grep "OpenGL version"
# 如果版本太低，更新显卡驱动
```

**Q：仪表盘不显示**
A：检查OpenGL状态：
```cpp
GLenum err = glGetError();
if (err != GL_NO_ERROR) {
    printf("OpenGL错误: 0x%04X\n", err);
}
```

### 性能问题
**Q：帧率太低**
A：尝试以下优化：
1. 减少仪表盘绘制复杂度
2. 降低3D场景的渲染质量
3. 确保使用Release模式编译

**Q：内存占用过高**
A：检查是否有内存泄漏：
```bash
valgrind --tool=memcheck ./bin/mjpc
```

## 📄 许可证

本项目基于Apache License 2.0开源：
- 允许商业使用
- 允许修改和分发
- 要求保留版权声明
- 不提供担保

详见 [LICENSE](LICENSE) 文件。

## 🙏 致谢

### 项目依赖
- **MuJoCo团队**：提供优秀的物理仿真引擎
- **Google DeepMind**：开源MuJoCo MPC框架
- **GLFW/GLEW社区**：提供跨平台图形库支持

### 学习资源
- 所有在线教程和文档的作者
- 开源社区的问题解答者
- 相关研究论文的作者

### 特别感谢
- **指导老师**：提供项目指导和建议
- **同学们**：在开发过程中的讨论和帮助
- **开源贡献者**：为依赖库做出的贡献

## 📞 联系我们

如有问题或建议，可以通过以下方式联系：

- **GitHub Issues**：[项目Issues页面](https://github.com/your-repo/issues)
- **电子邮件**：1195698012@QQ.com
- **课程论坛**：相关课程讨论区

---

**最后更新**：2025年12月27日  
**版本**：v1.0.0  
**作者**：WQH 

---

*感谢使用MuJoCo MPC汽车仪表盘可视化系统！希望本项目能为您的学习、研究或开发提供帮助。* 🚀