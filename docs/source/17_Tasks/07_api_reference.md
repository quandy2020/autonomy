# 7. API 参考

### 7.1 system::Autonomy

头文件：`autonomy/system/autonomy.hpp`

#### 工厂与生命周期

```cpp
std::unique_ptr<Autonomy> CreateAutonomy(const proto::AutonomyOptions& options);

void Start();
void Configure(const RuntimeOptions& runtime = {});
void Shutdown();
```

#### 导航任务

```cpp
bool NavigateToPose(
    const commsgs::geometry_msgs::PoseStamped& goal,
    const std::function<bool()>& cancel_checker,
    bool keep_alive = true,
    double timeout_sec = 0.0);

bool NavigateThroughPoses(
    const std::vector<commsgs::geometry_msgs::PoseStamped>& goals,
    const std::function<bool()>& cancel_checker,
    bool keep_alive = true,
    double timeout_sec = 0.0);

void ReplanToGoal(const commsgs::geometry_msgs::PoseStamped& goal);
```

#### Server 访问

```cpp
map::MapServer* GetMapServer();
planning::PlannerServer* GetPlanner();
control::ControllerServer* GetController();
```

### 7.2 RuntimeOptions

```cpp
struct RuntimeOptions {
    bool enable_bt_tasks{true};      // 已定义，当前未读取
    bool use_bt_navigation{true};    // Configure 中当前强制 false
    std::string config_directory;
    std::string planner_id;
    std::string controller_id;
    // ...
};
```

### 7.3 NavigatorInterface

```cpp
enum class NavigatorStatus {
    Idle, Running, Completed, Failed, Canceled
};
```

### 7.4 相关文档

- [05 Framework · Autonomy API](../05_Framework/02_quickstart.md)
- [14 Commsgs · nav_msgs](../14_Commsgs/08_nav_planning_msgs.md)
