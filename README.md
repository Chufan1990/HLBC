# HLBC

Hierarchical Lyapunov-based Controller ~~名字我瞎比起的~~ 目前和Apollo的lat_controller的结构相同，原因是LQR是非常成熟~~能追述到Apollo登月了, 80多年了~~的控制算法，是控制问题优化解法的入门。Apollo的代码架构和规范都十分成熟，没有必要重复造轮子。但既然叫*Hierarchical*(结构化)，之后会对算法做结构上的拓展(常见的做法是分为upper和lower controller，同时在上下级中添加增益补偿控制，例如MRAC)，也会不断的扩充不同的控制算法(例如MPC)。

### Introduction
- **input**
    - Chassis *vehicle state info, i.e, linear velocity, angular velocity, linear acceleration, angular acceleration, geer position, etc.*
      - speed_mps *车辆行驶速度，由轮速计提供*
      - gear_location *当前车辆档位 (前进/后退)*
    - LocalizationEstimation *vehicle localization info, i.e, vehicle x/y/z coordination in map frame, vehicle euclidean angular velocity in map frame, vehicle euclidean linear velocity in map frame, etc.*
      - pose *车辆位姿信息*
        - position *x,y,z坐标 (地图坐标系)*
        - orientation *车辆朝向 (地图坐标系,四元数)*
        - linear_velocity *x,y,z方向的线速度 (地图坐标系)*
        - angular_velocity *x,y,z方向的角速度 (地图坐标系)*
        - heading *车辆朝向 (地图坐标系)*
        - linear_acceleration_vrf *x,y,z方向的线加速度(车辆坐标系 IMU 及定位融合结果提供)*
        - linear_angular_vrf *x,y,z方向的角速度 (车辆坐标系 IMU 及定位融合结果提供)*
        - euler_angle *roll, ptich, yaw (地图坐标系)*
    - **ADCTrajectory** *reference trajectory published planning module*
      - trajectory_point *轨迹点，包括位置和速度信息*
        - path_point *轨迹点位置信息*
          - x *x坐标*
          - y *y坐标*
          - z *z坐标*
          - theta *轨迹点朝向*
          - kappa *轨迹点曲率*
          - s *相对径向位移 (相对于第一个轨迹点)*
          - dkappa *曲率对径向位移的一阶导*
          - ddkappa *曲率对径向位移的二阶导*
      - v *轨迹点速度*
      - a *轨迹点加速度*
      - relative_time *相对时间 (相对于第一个轨迹点)*

- **Ouput**:
    - ControlCommand *For LQR, only steering angle in percentage/degree/rad*

#### Structure:
    - conf
    - data
    - include
        - common
            - configs
            - math
            - status
            - util
            - vehicle_state
        - control
            - common
            - controller
        - nodes
        - planning (incomplete)
            - math
                - discretized_points_smoothing
            - reference_line
                - 
    - launch
    - proto
    - src
        - common
            - configs
            - math
            - status
            - util
            - vehicle_state
        - control
            - common
            - controller
        - nodes
        - planning (incomplete)
            - math
                - discretized_points_smoothing
            - reference_line
  
#### Control Process
HLBC目前是以库的形式，提供接口给waypoint_follower/pure_pursuit， pure_pursuit节点可以选择是否调用HLBC或仍旧使用pure_pursuit作为横向控制指令计算的方法。这样设计的原因是为了让HLBC和原pure_pursuit的代码冲突最少，以达到快速合并的效果。但这种方式需要额外的数据拷贝，同时主要传递数据的数据结构存在浪费。HLBC的目的是提供一个用优化的方法解决控制问题的思路，代码运行效率暂时没有考量。

HLBC的入口在*controller_interface.cpp*, 处理的流程在 `bool::ControllerInterface<L, C, T, F>::ComputeControlCommand(const L& localization, const C& chassis, const T& trajectory, const F& feedback, double& ret)`中。该函数为模板函数，`L`, `C`, `T`, `F`分表代表了模板类型**Localization**, **Chassis**, **Trajectory** 和 **Feedback** (autoware中底盘的反馈数据是由多个ros message分别发出，这样做不仅效率低，占用了更多的线程，还可能会导致消息的时间戳不匹配)
```c++
    template <typename L, typename C, typename T, typename F>
    bool ControllerInterface<L, C, T, F>::ComputeControlCommand(
        const L& localization, const C& chassis, const T& trajectory,
        const F& feedback, double& ret) {

    // 1. 读取输入数据后进行数据格式转换，将ros message转换成protobuf
    GetProtoFromMsg(trajectory, local_view_.mutable_trajectory());

    GetProtoFromMsg(feedback, local_view_.mutable_chassis());

    GetProtoFromMsg(localization, chassis, local_view_.mutable_localization());

    // 2. 计算轨迹点的曲率，曲率一阶导，曲率二阶导，相对径向位移和相对时间
    UpdateTrajectoryPoint(&local_view_.localization(),
                            local_view_.mutable_trajectory());

    // 3. 检查数据时候存在
    auto status = CheckInput(&local_view_);

    if (!status.ok()) {
        AERROR_EVERY(100,
                    "controller/controller_agent.cpp, "
                    "ControllerAgent::ComputeControlCommand",
                    "Control input data failed: " << status.error_message());
    } else {
    // 4. 检查数据时间戳是否过期
        Status status_ts = CheckTimestamp(local_view_);
        if (!status_ts.ok()) {
        AERROR(
            "controller/controller_agent.cpp, "
            "ControllerAgent::ComputeControlCommand",
            "Input messages timeout");
        status = status_ts;
        }
    }

    // 5. 计算控制指令
    status = controller_agent_.ComputeControlCommand(
        &local_view_.localization(), &local_view_.chassis(),
        &local_view_.trajectory(), &control_command_);
    if (!status.ok()) {
        AERROR("", status.error_message());
        return false;
    }
    auto vehicle_param =
        common::VehicleConfigHelper::Instance()->GetConfig().vehicle_param();
    ret = control_command_.steering_target() / 100.0 *
            vehicle_param.max_steer_angle();

    return true;
    }
```

在`controller_interface.cpp`中，`ControllerInterface::Init()` 会生成一个`DependencyInjector`并传递给`ControllerAgent`来实现数据的流转，关于依赖注入的解释可以参考[Dependency Injection-依赖注入详解](https://zhuanlan.zhihu.com/p/36181914)，这里不再赘述。同时`ControllerInterface::Init()`还会读取`conf`文件中的配置文件`control_conf.pb.txt`并通过`ControlConf`转递给`ControllerAgent`.
```c++
    bool ControllerInterface<L, C, T, F>::Init() {
    // 1. 生成 DependencyInjector
    injector_ = std::make_shared<DependencyInjector>();

    // 2. 读取 configuration 文件并写入control_conf
    ACHECK(
        !common::util::GetProtoFromFile(FLAGS_control_conf_file, &control_conf_),
        "controller/controller_agent.cpp, ControllerAgent::Init",
        "Unable to load control conf file: " << FLAGS_control_conf_file);

    AINFO("controller/controller_agent.cpp, ControllerAgent::Init",
            "Conf file: " << FLAGS_control_conf_file << " is loaded.");

    ADEBUG("controller/controller_agent.cpp, ControllerAgent::Init",
            "FLAGS_use_control_submodules: " << FLAGS_use_control_submodules);
    // 3. 传入ControllerAgent
    if (!controller_agent_.Init(injector_, &control_conf_).ok()) return false;
    return true;
    }
```

`ControllerAgent`的主要目的是进行多个`Controller`的注册及切换。在*Apollo*中，`Controller`的注册是在初始化函数`ControllerAgent::Init`中由配置文件`control_conf`和一个工厂函数`factory`共同实现的。目前HLBC仅有一个横向控制器，因此省去了工厂函数的实现，仅在初始化函数中注册了一个`LatController`。`ControllerAgent::ComputeControlCommand`仅是`LatController::ComputerControlCommand`的一层wrapper.

`LatController`是HLBC主要的计算函数，包括了状态方程 $x_{k+1} = A_{k}x_{k} + B_{k}u$ 的初始化，更新和离散化，状态向量的`matrix_state`的计算，LQR增益的计算，和前馈增益的计算。矩阵和状态量的更新依赖于`TrajectoryAnalyzer`提供的方法，包括坐标转换，根据绝对时间/相对时间/位置选取轨迹上最近点，样条/线性差值。LQR增益的求解调用`common/math/linear_quadratic_regular.cpp`中的求解器，求解方式为*recursive Algebraic Riccati equations*，收敛条件为$\frac{||P_{k+1} - P_{k}||}{||P_{k+1}||} < tol$，详细说明可参考[Arnold, W.F., III and A.J. Laub, "Generalized Eigenproblem Algorithms and Software for Algebraic Riccati Equations," Proc. IEEE®, 72 (1984), pp. 1746-1754](https://engineering.purdue.edu/AAECourses/aae564/2008/fall/Notes/ArnoldLaub1984)

```c++
    Status LatController::ComputeControlCommand(
        const localization::LocalizationEstimate* localization,
        const canbus::Chassis* chassis,
        const planning::ADCTrajectory* planning_published_trajectory,
        ControlCommand* cmd) {

    // 1. 获取车辆状态
    auto vehicle_state = injector_->vehicle_state();

    ...
    // 2. 生成TrajectoryAnalyer处理轨迹信息
    trajectory_analyzer_ =
        std::move(TrajectoryAnalyzer(&target_tracking_trajectory));

    if (((FLAGS_trajectory_transform_to_com_reverse &&
            vehicle_state->gear() == canbus::Chassis::GEAR_REVERSE) ||
        (FLAGS_trajectory_transform_to_com_drive &&
            vehicle_state->gear() == canbus::Chassis::GEAR_DRIVE)) &&
        enable_look_ahead_back_control_) {
        trajectory_analyzer_.TrajectoryTransformToCOM(lr_);
    }

    // 3. 倒车时重构车辆动力学模型矩阵 A，B
    if (vehicle_state->gear() == canbus::Chassis::GEAR_REVERSE) {
        /**
        * A matrix (Gear Reverse)
        *[0.0, 0.0, 1.0 * v 0.0;
        * 0.0, (-(c_f + c_r) / m) / v, (c_f + c_r) / m,
        * (l_r * c_r - l_f * c_f) / m / v;
        * 0.0, 0.0, 0.0, 1.0;
        * 0.0, ((lr * cr - lf * cf) / i_z) / v, (l_f * c_f - l_r * c_r) / i_z,
        * (-1.0 * (l_f^2 * c_f + l_r^2 * c_r) / i_z) / v;]
        */
        cf_ = -control_conf_->lat_controller_conf().cf();
        cr_ = -control_conf_->lat_controller_conf().cr();
        matrix_a_(0, 1) = 0.0;
        matrix_a_coeff_(0, 2) = 1.0;
    } else {
        /**
        * A matrix (Gear Drive)
        * [0.0, 1.0, 0.0, 0.0;
        * 0.0, (-(c_f + c_r) / m) / v, (c_f + c_r) / m,
        * (l_r * c_r - l_f * c_f) / m / v;
        * 0.0, 0.0, 0.0, 1.0;
        * 0.0, ((lr * cr - lf * cf) / i_z) / v, (l_f * c_f - l_r * c_r) / i_z,
        * (-1.0 * (l_f^2 * c_f + l_r^2 * c_r) / i_z) / v;]
        */
        cf_ = control_conf_->lat_controller_conf().cf();
        cr_ = control_conf_->lat_controller_conf().cr();
        matrix_a_(0, 1) = 1.0;
        matrix_a_coeff_(0, 2) = 0.0;
    }
    matrix_a_(1, 2) = (cf_ + cr_) / mass_;
    matrix_a_(3, 2) = (lf_ * cf_ - lr_ * cr_) / iz_;
    matrix_a_coeff_(1, 1) = -(cf_ + cr_) / mass_;
    matrix_a_coeff_(1, 3) = (lr_ * cr_ - lf_ * cf_) / mass_;
    matrix_a_coeff_(3, 1) = (lr_ * cr_ - lf_ * cf_) / iz_;
    matrix_a_coeff_(3, 3) = -1.0 * (lf_ * lf_ * cf_ + lr_ * lr_ * cr_) / iz_;

    /**
    * b = [0.0, c_f / m, 0.0, l_f * c_f / i_z]^T
    */
    matrix_b_(1, 0) = cf_ / mass_;
    matrix_b_(3, 0) = lf_ * cf_ / iz_;
    matrix_bd_ = matrix_b_ * ts_;


    // 4. 根据行驶方向，离散化控制矩阵 B
    UpdateDrivingOrientation();

    ...

    // 5. 更新横向偏差
    UpdateState(debug);

    // 6. 根据行驶方向，离散化状态矩阵 A
    UpdateMatrix();

    // 7. 更新道路预览模型(道路预览模型的原理是构建一个简化的预测模型，希望用LQR的方式来求解预测问题，但实际上这种简化模型存在较多原理上的问题，并且模型预测控制本身也是很成熟的应用，因此Apollo并未真正使用这个模型，这里仅仅是提供参考，实际控制程序中并未使用)
    UpdateMatrixCompound();
 

    // 8. 更新权重
    int q_param_size = control_conf_->lat_controller_conf().matrix_q_size();
    int reverse_q_param_size =
        control_conf_->lat_controller_conf().reverse_matrix_q_size();

    if (injector_->vehicle_state()->gear() == canbus::Chassis::GEAR_REVERSE) {
        for (int i = 0; i < reverse_q_param_size; i++) {
        matrix_q_(i, i) =
            control_conf_->lat_controller_conf().reverse_matrix_q(i);
        }
    } else {
        for (int i = 0; i < q_param_size; i++) {
        matrix_q_(i, i) = control_conf_->lat_controller_conf().matrix_q(i);
        }
    }

    // 9. 求解LQR增益
    if (FLAGS_enable_gain_scheduler) {
        matrix_q_updated_(0, 0) =
            matrix_q_(0, 0) * lat_err_interpolation_->Interpolate(
                                std::fabs(vehicle_state->linear_velocity()));
        matrix_q_updated_(2, 2) =
            matrix_q_(2, 2) * heading_err_interpolation_->Interpolate(
                                std::fabs(vehicle_state->linear_velocity()));
        common::math::SolveLQRProblem(matrix_adc_, matrix_bdc_, matrix_q_updated_,
                                    matrix_r_, lqr_eps_, lqr_max_iteration_,
                                    &matrix_k_);
    } else {
        common::math::SolveLQRProblem(matrix_adc_, matrix_bdc_, matrix_q_,
                                    matrix_r_, lqr_eps_, lqr_max_iteration_,
                                    &matrix_k_);
    }


    // 10. 计算反馈控制的增益
    const double steer_angle_feedback = -(matrix_k_ * matrix_state_)(0, 0) * 180 /
                                        M_PI * steer_ratio_ /
                                        steer_single_direction_max_degree_ * 100;

    const double steer_angle_feedforward = ComputeFeedForward(debug->curvature());
    
    ...
    // 11. Model Reference Adaptive Control 是自适应控制中一种很常用的方式，目前还未在代码中实现
    debug->set_steer_mrac_enable_status(enable_mrac_);

    ...
    // 12. 输出控制指令
    cmd->set_steering_target(common::math::Clamp(
        steer_angle, pre_steer_angle_ - steer_diff_with_max_rate,
        pre_steer_angle_ + steer_diff_with_max_rate));
    cmd->set_steering_rate(FLAGS_steer_angle_rate);

    pre_steer_angle_ = cmd->steering_target();

    ...
    return Status::OK();
    }
```

### LQR Problem
LQR是线性凸优化(二次规划)的一个基本形式，离散的控制问题可设置为

- 代价函数 *cost function*:  $min_{u} \sum_{0}^{\inf} x_{k}^{T}Qx_{k} + u^{T}Ru$
- 等式约束 *equality constraints*:  $x_{k+1} = A_{k}x_{k} + B_{k}u$ 

**x_{k}** 和 **u**分别代表运动控制中，被控车辆*k*时刻状态*state*和控制指令*u*, **Q** 和 **R** 为权重矩阵。由于运动过程中的，车辆追踪的目标并不总是*0*,即$x_{ref} = 0$, 因此常用状态误差 $e = x_{ref} - x$作为状态量。因此代价函数是状态误差和控制量的加权平均。直观的理解，LQR的目标函数是使误差量和控制量的综合尽可能的小，以此来达到消除偏差的同时，尽可能少的使用大幅度的控制量(对于横向控制来说，就是大角度的转向)。

同时，在动力学方程中，状态误差**e**还包括了横向偏差的变化率`lateral_error_rate`和车辆朝向的变化率`heading_error_rate`。通过调节权重矩阵**Q**的值，可以在代价函数中加入对状态变化率的cost，使得LQR求解时会同时考虑状态的变化快慢，增加车辆运动的平滑性。

前馈控制增益的计算主要依赖于轨迹点的曲率*curvature*，具体的计算为$ff = arctan(L \cdot K)$，其中**L**为轮距`wheel_base`，**K**为对应轨迹点的曲率`kappa`.

