// ESP-IDF v5.4.x + micro_ros_espidf_component Jazzy (UDP Wi-Fi).
// Transport ONLY. All PWM, encoder and odometry are in the 50-Hz motor task.
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <initializer_list>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_err.h"
#include "esp_timer.h"
#include <uros_network_interfaces.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rmw_microros/rmw_microros.h>
#include <rosidl_runtime_c/string_functions.h>
#include <geometry_msgs/msg/twist.h>
#include <std_msgs/msg/float32_multi_array.h>
#include <std_msgs/msg/bool.h>
#include <nav_msgs/msg/odometry.h>
#include <sensor_msgs/msg/imu.h>
#include <sensor_msgs/msg/range.h>
#include "robot_sensors.h"
#include "robot_config.h"
#include "yaw_outer.h"
#include "robot_drive.h"

namespace {
void error(const char *where, rcl_ret_t rc){
    // Error text pointers may be corrupted after transport failure or flood UART.
    printf("RCL ERROR %s: rc=%ld\n",where,(long)rc);
    rcl_reset_error();
}
// Throttle telemetry error logs: low-priority monitoring must never flood UART.
void telemError(const char* topic,rcl_ret_t rc,uint32_t& count,uint64_t& lastLog){
    if(rc==RCL_RET_OK)return;
    ++count;
    const uint64_t now=(uint64_t)esp_timer_get_time();
    if(now-lastLog>5000000ULL){
        lastLog=now;
        printf("TELEMETRY_FAIL %s rc=%ld total=%lu\n",topic,(long)rc,(unsigned long)count);
    }
    rcl_reset_error();
}
void onTwist(const void* input){
    if(!input)return;
    const auto *t=(const geometry_msgs__msg__Twist*)input;
    bool planar=isfinite(t->linear.x)&&isfinite(t->angular.z)&&
        isfinite(t->linear.y)&&isfinite(t->linear.z)&&
        isfinite(t->angular.x)&&isfinite(t->angular.y)&&
        fabs(t->linear.y)<0.00001&&fabs(t->linear.z)<0.00001&&
        fabs(t->angular.x)<0.00001&&fabs(t->angular.y)<0.00001;
    robot_drive_cmd_vel(planar?(float)t->linear.x:NAN,planar?(float)t->angular.z:NAN);
}
void onHeadingEnable(const void* input){
    if(input)robot_drive_heading_enable_request(static_cast<const std_msgs__msg__Bool*>(input)->data);
}
void onHeadingPidSet(const void* input){
    if(!input)return;
    const auto *msg=static_cast<const std_msgs__msg__Float32MultiArray*>(input);
    robot_drive_heading_pid_request(msg->data.data,msg->data.size==3);
}
void onFfPoint(const void* input){
    if(!input)return;
    const auto* m=static_cast<const std_msgs__msg__Float32MultiArray*>(input);
    robot_drive_ff_point_request(m->data.data,m->data.size==4);
}
void onFfScale(const void* input){
    if(!input)return;
    const auto* m=static_cast<const std_msgs__msg__Float32MultiArray*>(input);
    robot_drive_ff_scale_request(m->data.data,m->data.size==3);
}
void onFfSave(const void* input){
    if(input)robot_drive_ff_save_request(static_cast<const std_msgs__msg__Bool*>(input)->data);
}
void onFfReset(const void* input){
    if(input)robot_drive_ff_reset_request(static_cast<const std_msgs__msg__Bool*>(input)->data);
}
void onBenchArm(const void* input){
    if(!input)return;
    const auto* m = static_cast<const std_msgs__msg__Bool*>(input);
    robot_drive_bench_request(m->data);
}
void onPidSet(const void* input){
    if(!input)return;
    const auto *msg=(const std_msgs__msg__Float32MultiArray*)input;
    robot_drive_pid_request(msg->data.data,msg->data.size==6);
}
void copyStamp(builtin_interfaces__msg__Time &time, int64_t ns){
    if(ns<=0){time.sec=0;time.nanosec=0;return;}
    time.sec=(int32_t)(ns/1000000000LL);
    time.nanosec=(uint32_t)(ns%1000000000LL);
}
void setCovariances(nav_msgs__msg__Odometry &o){
    // INITIAL placeholders, NOT statistically calibrated. Non-planar components
    // deliberately have very large uncertainty. Validate from trajectory tests
    // BEFORE using this input in robot_localization EKF.
    for(int i=0;i<36;i++){
        o.pose.covariance[i]=0.0;
        o.twist.covariance[i]=0.0;
    }
    o.pose.covariance[0]=0.02;
    o.pose.covariance[7]=0.05;
    o.pose.covariance[14]=1.0e6;
    o.pose.covariance[21]=1.0e6;
    o.pose.covariance[28]=1.0e6;
    o.pose.covariance[35]=0.08;
    o.twist.covariance[0]=0.03;
    o.twist.covariance[7]=0.05;
    o.twist.covariance[14]=1.0e6;
    o.twist.covariance[21]=1.0e6;
    o.twist.covariance[28]=1.0e6;
    o.twist.covariance[35]=0.10;
}
void microRosTask(void*){
    rcl_allocator_t alloc=rcl_get_default_allocator();
    rcl_init_options_t opts=rcl_get_zero_initialized_init_options();
    rcl_ret_t rc=rcl_init_options_init(&opts,alloc);
    if(rc!=RCL_RET_OK){error("init_options",rc);vTaskDelete(nullptr);return;}
#ifdef CONFIG_MICRO_ROS_ESP_XRCE_DDS_MIDDLEWARE
    auto* rmwOpts=rcl_init_options_get_rmw_init_options(&opts);
    if(rmw_uros_options_set_udp_address(CONFIG_MICRO_ROS_AGENT_IP,CONFIG_MICRO_ROS_AGENT_PORT,rmwOpts)!=RMW_RET_OK){
        printf("Unable to configure UDP Agent\n");vTaskDelete(nullptr);return;
    }
    while(rmw_uros_ping_agent_options(1000,3,rmwOpts)!=RMW_RET_OK){
        printf("Waiting for Agent %s:%s (motors stay stopped without /cmd_vel)\n",
               CONFIG_MICRO_ROS_AGENT_IP,CONFIG_MICRO_ROS_AGENT_PORT);
        vTaskDelay(pdMS_TO_TICKS(1300));
    }
#endif
    rclc_support_t support;
    rc=rclc_support_init_with_options(&support,0,nullptr,&opts,&alloc);
    if(rc!=RCL_RET_OK){error("support",rc);(void)rcl_init_options_fini(&opts);vTaskDelete(nullptr);return;}
    rcl_node_t node=rcl_get_zero_initialized_node();
    rc=rclc_node_init_default(&node,"delivery_robot_motor","",&support);
    if(rc!=RCL_RET_OK){error("node",rc);(void)rclc_support_fini(&support);(void)rcl_init_options_fini(&opts);vTaskDelete(nullptr);return;}
    rcl_subscription_t subCmd=rcl_get_zero_initialized_subscription();
    rcl_subscription_t subPid=rcl_get_zero_initialized_subscription();
    rcl_subscription_t subBench=rcl_get_zero_initialized_subscription();
    rcl_subscription_t subHeadingEnable=rcl_get_zero_initialized_subscription();
    rcl_subscription_t subHeadingPid=rcl_get_zero_initialized_subscription();
    rcl_subscription_t subFfPoint=rcl_get_zero_initialized_subscription();
    rcl_subscription_t subFfScale=rcl_get_zero_initialized_subscription();
    rcl_subscription_t subFfSave=rcl_get_zero_initialized_subscription();
    rcl_subscription_t subFfReset=rcl_get_zero_initialized_subscription();
    rcl_publisher_t pubFf=rcl_get_zero_initialized_publisher();
    rcl_publisher_t pubWheel=rcl_get_zero_initialized_publisher();
    rcl_publisher_t pubPid=rcl_get_zero_initialized_publisher();
    rcl_publisher_t pubOdom=rcl_get_zero_initialized_publisher();
    rcl_publisher_t pubLink=rcl_get_zero_initialized_publisher();
    rcl_publisher_t pubControl=rcl_get_zero_initialized_publisher();
    rcl_publisher_t pubImu=rcl_get_zero_initialized_publisher();
    rcl_publisher_t pubRange=rcl_get_zero_initialized_publisher();
    rcl_publisher_t pubSafety=rcl_get_zero_initialized_publisher();
    rcl_publisher_t pubHeading=rcl_get_zero_initialized_publisher();
    rcl_publisher_t pubTrajectory=rcl_get_zero_initialized_publisher();
    // On a partial ROS init failure, do NOT leave an orphan node on the Agent.
    // Clean up in reverse dependency order and require a fresh session before retry.
    rclc_executor_t exec=rclc_executor_get_zero_initialized_executor();
    bool executorReady=false;
    auto cleanup=[&](){
        if(executorReady) (void)rclc_executor_fini(&exec);
        rcl_publisher_t* pubs[]={&pubFf,&pubTrajectory,&pubHeading,&pubSafety,&pubRange,&pubImu,&pubControl,&pubLink,
                                 &pubOdom,&pubPid,&pubWheel};
        for(auto* pub:pubs) if(rcl_publisher_is_valid(pub))
            (void)rcl_publisher_fini(pub,&node);
        rcl_subscription_t* subs[]={&subFfReset,&subFfSave,&subFfScale,&subFfPoint,&subHeadingPid,&subHeadingEnable,&subBench,&subPid,&subCmd};
        for(auto* sub:subs) if(rcl_subscription_is_valid(sub))
            (void)rcl_subscription_fini(sub,&node);
        if(rcl_node_is_valid(&node)) (void)rcl_node_fini(&node);
        (void)rclc_support_fini(&support);
        (void)rcl_init_options_fini(&opts);
    };
#define CHECK(entity, expr) do{rc=(expr);if(rc!=RCL_RET_OK){ \
    error(entity,rc);printf("MICROROS_SETUP_FAILED at %s. MOTOR STOP by watchdog; restart Agent+ESP32.\n",entity); \
    cleanup();vTaskDelete(nullptr);return;}}while(0)
    CHECK("sub /cmd_vel",rclc_subscription_init_default(&subCmd,&node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs,msg,Twist),"/cmd_vel"));
    CHECK("sub /robot/pid_set",rclc_subscription_init_default(&subPid,&node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs,msg,Float32MultiArray),"/robot/pid_set"));
    CHECK("sub /robot/bench_arm",rclc_subscription_init_default(&subBench,&node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs,msg,Bool),"/robot/bench_arm"));
    CHECK("sub /robot/heading_enable",rclc_subscription_init_default(&subHeadingEnable,&node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs,msg,Bool),"/robot/heading_enable"));
    CHECK("sub /robot/heading_pid_set",rclc_subscription_init_default(&subHeadingPid,&node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs,msg,Float32MultiArray),"/robot/heading_pid_set"));
    CHECK("sub /robot/ff_set",rclc_subscription_init_default(&subFfPoint,&node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs,msg,Float32MultiArray),"/robot/ff_set"));
    CHECK("sub /robot/ff_scale_set",rclc_subscription_init_default(&subFfScale,&node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs,msg,Float32MultiArray),"/robot/ff_scale_set"));
    CHECK("sub /robot/ff_save",rclc_subscription_init_default(&subFfSave,&node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs,msg,Bool),"/robot/ff_save"));
    CHECK("sub /robot/ff_reset",rclc_subscription_init_default(&subFfReset,&node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs,msg,Bool),"/robot/ff_reset"));
    CHECK("pub /robot/ff_status",rclc_publisher_init_best_effort(&pubFf,&node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs,msg,Float32MultiArray),"/robot/ff_status"));
    CHECK("pub /robot/wheel_debug",rclc_publisher_init_best_effort(&pubWheel,&node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs,msg,Float32MultiArray),"/robot/wheel_debug"));
    CHECK("pub /robot/pid_status",rclc_publisher_init_best_effort(&pubPid,&node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs,msg,Float32MultiArray),"/robot/pid_status"));
    CHECK("pub /odom",rclc_publisher_init_default(&pubOdom,&node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs,msg,Odometry),"/odom"));
    CHECK("pub /robot/link_debug",rclc_publisher_init_best_effort(&pubLink,&node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs,msg,Float32MultiArray),"/robot/link_debug"));
    CHECK("pub /robot/control_debug",rclc_publisher_init_best_effort(&pubControl,&node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs,msg,Float32MultiArray),"/robot/control_debug"));
    CHECK("pub /imu/data_raw",rclc_publisher_init_best_effort(&pubImu,&node,
       ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs,msg,Imu),"/imu/data_raw"));
    CHECK("pub /front_range",rclc_publisher_init_best_effort(&pubRange,&node,
       ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs,msg,Range),"/front_range"));
    CHECK("pub /robot/safety_status",rclc_publisher_init_best_effort(&pubSafety,&node,
       ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs,msg,Float32MultiArray),"/robot/safety_status"));
    CHECK("pub /robot/heading_status",rclc_publisher_init_best_effort(&pubHeading,&node,
       ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs,msg,Float32MultiArray),"/robot/heading_status"));
    CHECK("pub /robot/trajectory_status",rclc_publisher_init_best_effort(&pubTrajectory,&node,
       ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs,msg,Float32MultiArray),"/robot/trajectory_status"));
    // V10 debug telemetry uses BEST_EFFORT to avoid reliable UDP ACK stalls.
    // Monitor with --qos-reliability best_effort; report lost samples on host.
    // /odom remains Reliable for EKF/RViz compatibility; bound its ACK wait.
    if(rmw_uros_set_publisher_session_timeout(
           rcl_publisher_get_rmw_handle(&pubOdom),
           RobotConfig::ODOM_RELIABLE_ACK_MS)!=RMW_RET_OK){
        printf("WARN: unable to bound odom ACK wait\n");
    }
    geometry_msgs__msg__Twist cmdInput={};
    float pidInBuffer[6]={};
    std_msgs__msg__Float32MultiArray pidInput={};
    pidInput.data.data=pidInBuffer;
    pidInput.data.size=0;
    pidInput.data.capacity=6;
    float headingPidInBuffer[3]={};
    std_msgs__msg__Float32MultiArray headingPidInput={};
    headingPidInput.data.data=headingPidInBuffer;
    headingPidInput.data.size=0;headingPidInput.data.capacity=3;
    CHECK("executor_init",rclc_executor_init(&exec,&support.context,9,&alloc));
    executorReady=true;
    CHECK("executor_cmd",rclc_executor_add_subscription(&exec,&subCmd,&cmdInput,onTwist,ON_NEW_DATA));
    CHECK("executor_pid",rclc_executor_add_subscription(&exec,&subPid,&pidInput,onPidSet,ON_NEW_DATA));
    std_msgs__msg__Bool benchInput={};
    std_msgs__msg__Bool headingEnabledInput={};
    CHECK("executor_bench",rclc_executor_add_subscription(&exec,&subBench,&benchInput,onBenchArm,ON_NEW_DATA));
    CHECK("executor_heading_enable",rclc_executor_add_subscription(&exec,&subHeadingEnable,&headingEnabledInput,onHeadingEnable,ON_NEW_DATA));
    CHECK("executor_heading_pid",rclc_executor_add_subscription(&exec,&subHeadingPid,&headingPidInput,onHeadingPidSet,ON_NEW_DATA));

    float ffPointInBuffer[4]={};
    std_msgs__msg__Float32MultiArray ffPointInput={};
    ffPointInput.data.data=ffPointInBuffer;ffPointInput.data.size=0;ffPointInput.data.capacity=4;
    float ffScaleInBuffer[3]={};
    std_msgs__msg__Float32MultiArray ffScaleInput={};
    ffScaleInput.data.data=ffScaleInBuffer;ffScaleInput.data.size=0;ffScaleInput.data.capacity=3;
    std_msgs__msg__Bool ffSaveInput={},ffResetInput={};
    CHECK("executor_ff_point",rclc_executor_add_subscription(&exec,&subFfPoint,&ffPointInput,onFfPoint,ON_NEW_DATA));
    CHECK("executor_ff_scale",rclc_executor_add_subscription(&exec,&subFfScale,&ffScaleInput,onFfScale,ON_NEW_DATA));
    CHECK("executor_ff_save",rclc_executor_add_subscription(&exec,&subFfSave,&ffSaveInput,onFfSave,ON_NEW_DATA));
    CHECK("executor_ff_reset",rclc_executor_add_subscription(&exec,&subFfReset,&ffResetInput,onFfReset,ON_NEW_DATA));
    float ffStatusData[43]={};
    std_msgs__msg__Float32MultiArray ffStatus={};
    ffStatus.data.data=ffStatusData;ffStatus.data.size=43;ffStatus.data.capacity=43;
    float wheelData[10]={};
    std_msgs__msg__Float32MultiArray wheelMsg={};
    wheelMsg.data.data=wheelData;wheelMsg.data.capacity=10;wheelMsg.data.size=10;
    float pidStatusData[8]={};
    std_msgs__msg__Float32MultiArray pidStatus={};
    pidStatus.data.data=pidStatusData;pidStatus.data.capacity=8;pidStatus.data.size=8;

    float linkData[6]={};
    std_msgs__msg__Float32MultiArray linkMsg={};
    linkMsg.data.data=linkData;linkMsg.data.size=6;linkMsg.data.capacity=6;
    float controlData[14]={};
    std_msgs__msg__Float32MultiArray controlMsg={};
    controlMsg.data.data=controlData;controlMsg.data.capacity=14;controlMsg.data.size=14;
    float safetyData[12]={};
    std_msgs__msg__Float32MultiArray safetyMsg={};
    safetyMsg.data.data=safetyData;safetyMsg.data.size=12;safetyMsg.data.capacity=12;
    // Dedicated heading diagnostics, preserving the existing 12-field
    // /robot/safety_status contract used by V8 monitoring tools.
    float headingData[42]={};
    std_msgs__msg__Float32MultiArray headingMsg={};
    headingMsg.data.data=headingData;headingMsg.data.size=42;headingMsg.data.capacity=42;
    float trajectoryData[14]={};
    std_msgs__msg__Float32MultiArray trajectoryMsg={};
    trajectoryMsg.data.data=trajectoryData;trajectoryMsg.data.size=14;trajectoryMsg.data.capacity=14;
    sensor_msgs__msg__Imu imuMsg={};
    sensor_msgs__msg__Range rangeMsg={};
    if(!sensor_msgs__msg__Imu__init(&imuMsg) ||
       !sensor_msgs__msg__Range__init(&rangeMsg) ||
       !rosidl_runtime_c__String__assign(&imuMsg.header.frame_id,"imu_link") ||
       !rosidl_runtime_c__String__assign(&rangeMsg.header.frame_id,"front_range_link")){
      printf("ERROR sensor_msgs initialization\n");cleanup();vTaskDelete(nullptr);return;
    }
    imuMsg.orientation_covariance[0]=-1.0; // six-axis chip: NO quaternion
    imuMsg.angular_velocity_covariance[0]=0.0003;
    imuMsg.angular_velocity_covariance[4]=0.0003;
    imuMsg.angular_velocity_covariance[8]=0.0003;
    imuMsg.linear_acceleration_covariance[0]=0.10;
    imuMsg.linear_acceleration_covariance[4]=0.10;
    imuMsg.linear_acceleration_covariance[8]=0.10;
    rangeMsg.radiation_type=sensor_msgs__msg__Range__INFRARED;
    rangeMsg.field_of_view=0.436f; // approximate VL6180X 25 deg, validate sensor field of view
    rangeMsg.min_range=0.001f;
    rangeMsg.max_range=0.10f; // VL6180X nominal 10 cm; no long-range claims
    nav_msgs__msg__Odometry odom={};
    if(!nav_msgs__msg__Odometry__init(&odom) ||
       !rosidl_runtime_c__String__assign(&odom.header.frame_id,"odom") ||
       !rosidl_runtime_c__String__assign(&odom.child_frame_id,"base_link")){
        printf("ERROR odometry message initialization\n");
        sensor_msgs__msg__Imu__fini(&imuMsg);sensor_msgs__msg__Range__fini(&rangeMsg);
        cleanup();vTaskDelete(nullptr);return;
    }
    setCovariances(odom);
    if(rmw_uros_sync_session(250)!=RMW_RET_OK){
        printf("WARN Agent clock sync failed; /odom requires a valid timestamp (check Wi-Fi)\n");
    }
    printf("Robot_Delivery_V12: 9 subscribers incl FF+heading controls, "
           "11 publishers. Check Agent/ROS node info for endpoints.\n");
    printf("VL6180X: OBSERVATION ONLY. No motor stop or speed cap from /front_range.\n");
    printf("V12 wheel_debug, pid_status and ff_status are BEST_EFFORT; use --qos-reliability best_effort\n");
    uint64_t lastWheel=esp_timer_get_time(),lastPid=lastWheel,lastOdom=lastWheel,lastLink=lastWheel,lastControl=lastWheel;
    uint64_t lastFf=lastWheel;
    uint64_t lastImu=lastWheel,lastRange=lastWheel,lastSafety=lastWheel,lastHeading=lastWheel,lastTrajectory=lastWheel;
    uint32_t lastImuSeq=0,lastRangeSeq=0;
    uint64_t lastTimeSync=lastWheel;
    uint64_t lastPublishLog=0;
    bool warnedClock=false;
    uint32_t errFf=0;
    uint32_t errWheel=0,errPid=0,errOdom=0,errLink=0,errControl=0;
    uint32_t errImu=0,errRange=0,errSafety=0,errHeading=0,errTrajectory=0;
    while(true){
        rc=rclc_executor_spin_some(&exec,RCL_MS_TO_NS(5));
        if(rc!=RCL_RET_OK && rc!=RCL_RET_TIMEOUT){
            error("executor disconnected",rc);
            printf("Motor watchdog will stop. Reboot ESP32 after restoring Agent.\n");
            break;
        }
        const uint64_t now=esp_timer_get_time();

        // PERIODIC_CLOCK_RESYNC_V1
        // micro-ROS epoch offset can drift even while the epoch remains valid.
        // Re-sync periodically so /odom and /imu/data_raw stay aligned with
        // the Ubuntu/Agent clock. If the clock is invalid, retry faster.
        {
            const int64_t clockEpoch = rmw_uros_epoch_nanos();
            const bool clockValid =
                clockEpoch > 1600000000000000000LL;

            const uint64_t syncIntervalUs =
                clockValid ? 60000000ULL : 5000000ULL;  // 60 s / 5 s

            if(now - lastTimeSync >= syncIntervalUs){
                lastTimeSync = now;

                const rmw_ret_t syncRc =
                    rmw_uros_sync_session(100);

                if(syncRc == RMW_RET_OK){
                    warnedClock = false;
                    printf("Agent clock re-synchronized\n");
                }else if(!clockValid){
                    printf("WARN Agent clock re-sync failed\n");
                }
            }
        }
        // Prioritize fresh sensor telemetry before slower diagnostic publishers.
        // Match each ROS header timestamp to acquisition time (not publish time).
        if(now-lastImu>=20000ULL){
            lastImu=now;
            ImuSample data;robot_imu_snapshot(data);
            if(data.valid && data.calibrated && data.seq!=lastImuSeq &&
               data.time_us>0 && data.time_us<=now && now-data.time_us<80000ULL){
                int64_t epoch=rmw_uros_epoch_nanos();
                if(epoch>1600000000000000000LL){
                    copyStamp(imuMsg.header.stamp,epoch-int64_t((now-data.time_us)*1000ULL));
                    imuMsg.angular_velocity.x=data.gx;imuMsg.angular_velocity.y=data.gy;
                    imuMsg.angular_velocity.z=data.gz;
                    imuMsg.linear_acceleration.x=data.ax;imuMsg.linear_acceleration.y=data.ay;
                    imuMsg.linear_acceleration.z=data.az;
                    rc=rcl_publish(&pubImu,&imuMsg,nullptr);
                    telemError("imu",rc,errImu,lastPublishLog);
                    if(rc==RCL_RET_OK)lastImuSeq=data.seq;
                }
            }
        }
        if(now-lastRange>=40000ULL){
            lastRange=now;
            RangeReport data;robot_range_snapshot(data);
            if(data.valid && data.seq!=lastRangeSeq && data.time_us<=now &&
               now-data.time_us<180000ULL){
                int64_t epoch=rmw_uros_epoch_nanos();
                if(epoch>1600000000000000000LL){
                    copyStamp(rangeMsg.header.stamp,epoch-int64_t((now-data.time_us)*1000ULL));
                    rangeMsg.range=data.meters;
                    rc=rcl_publish(&pubRange,&rangeMsg,nullptr);
                    telemError("front_range",rc,errRange,lastPublishLog);
                    if(rc==RCL_RET_OK)lastRangeSeq=data.seq;
                }
            }
        }
        if(now-lastHeading>=100000ULL){
            lastHeading=now;
            WheelReport r;robot_drive_get_report(r);
            headingData[0]=r.imuYawDeg;        // relative yaw since boot, degrees
            headingData[1]=r.headingTargetDeg; // target yaw, degrees
            headingData[2]=r.headingErrorDeg;  // wrap(target - actual), degrees
            headingData[3]=r.gyroZ;            // instantaneous rate, rad/s
            headingData[4]=r.yawRequested;     // original angular.z cmd, rad/s
            headingData[5]=r.yawEffective;     // corrected angular.z, rad/s
            headingData[6]=r.yawCorrection;    // bounded correction, rad/s
            headingData[7]=r.yawMode;          // IDLE=0, HOLD=1, TURN=2, FALLBACK=3
            headingData[8]=r.imuFresh;         // fresh sensor data: 1 or 0
            headingData[9]=r.imuAgeMs;         // age of last gyro sample, ms
            // Backward-compatible extension: existing indices 0..9 unchanged.
            headingData[10]=r.headingPidEnabled; // 1=ON 0=OFF
            headingData[11]=r.headingPidActive;  // hysteresis: configured trigger/release
            headingData[12]=r.headingKp;
            headingData[13]=r.headingKi;
            headingData[14]=r.headingKd;
            headingData[15]=r.headingP;          // P contribution rad/s
            headingData[16]=r.headingI;          // I contribution rad/s
            headingData[17]=r.headingD;          // D contribution rad/s
            headingData[18]=r.headingSaturated;  // correction hit cap
            headingData[19]=YawOuter::Controller::HEADING_TRIGGER_RAD*180.0f/RobotConfig::PI_F; // real trigger
            headingData[20]=r.headingSetStatus;  // 0 default,10 pending,1 applied,-1 invalid,-2 busy
            headingData[21]=r.headingSetSeq;
            // Append-only debug extension: old indices 0..21 unchanged.
            headingData[22]=r.imuInitialized;  // 0/1 I2C config done
            headingData[23]=r.imuCalProgress;  // stationary gyro samples 0..150
            headingData[24]=r.imuErrors;       // I2C read/init errors
            headingData[25]=r.imuSeq;          // accepted sensor samples
            headingData[26]=r.imuRawZCounts;   // raw Z gyro counts
            headingData[27]=r.imuBiasZCounts;  // gyro calibration bias counts
            headingData[28]=r.encoderYawRate;  // enc-derived yaw rad/s
            headingData[29]=r.headingGuardReason; // 0=ok,4=sign,5=large error,6=nonfinite
            headingData[30]=r.steeringAuthority; // 0=fallback encoder,1=heading,2=bench
            headingData[31]=r.headingDeltaRpm; // estimated differential RPM contributed by yaw PID
            headingData[32]=r.imuWho;          // WHO_AM_I (MPU6500=112)
            headingData[33]=RobotConfig::IMU_GYRO_Z_SIGN;
            headingData[34]=r.headingSlewLimited; // 1=slew rate limited
            headingData[35]=r.imuCalReject;    // 1=gravity, 2=gyro-moving, 3=both
            headingData[36]=r.imuAccelNorm;    // norm of measured acceleration m/s2
            headingData[37]=r.imuCalRestarts;  // calibration interrupted count
            headingData[38]=r.headingEnableRxSeq;
            headingData[39]=r.headingEnableAppliedSeq;
            headingData[40]=r.headingGuardResetCount;
            headingData[41]=r.headingEnableApplyStatus;
            rc=rcl_publish(&pubHeading,&headingMsg,nullptr);
            telemError("heading_status",rc,errHeading,lastPublishLog);
        }
        if(now-lastTrajectory>=200000ULL){
            lastTrajectory=now;
            WheelReport r;robot_drive_get_report(r);
            trajectoryData[0]=r.pathXRef;       // LOCAL desired x, m
            trajectoryData[1]=r.pathYRef;       // LOCAL desired y, m
            trajectoryData[2]=r.pathXEst;       // LOCAL gyro+encoder estimated x, m
            trajectoryData[3]=r.pathYEst;       // LOCAL gyro+encoder estimated y, m
            trajectoryData[4]=r.pathCrossTrack; // signed lateral error, m
            trajectoryData[5]=r.pathAlongTrack; // signed tangent error, m
            trajectoryData[6]=r.pathCurvature;  // desired curvature, 1/m
            trajectoryData[7]=r.pathRadius;     // desired signed radius, m (0 if straight/spin)
            trajectoryData[8]=r.imuYawDeg;      // measured gyro-integrated yaw, deg
            trajectoryData[9]=r.headingTargetDeg; // commanded yaw, deg
            trajectoryData[10]=r.headingErrorDeg; // wrap(target-measured), deg
            trajectoryData[11]=r.pathCorrection; // LOCAL path feedback rad/s (default 0)
            trajectoryData[12]=r.yawCorrection; // TOTAL angular correction rad/s
            trajectoryData[13]=r.yawMode;       // 0=IDLE,1=straight,2=turn,3=fallback
            rc=rcl_publish(&pubTrajectory,&trajectoryMsg,nullptr);
            telemError("trajectory_status",rc,errTrajectory,lastPublishLog);
        }
        if(now-lastSafety>=200000ULL){
            lastSafety=now;
            WheelReport r;robot_drive_get_report(r);
            RangeReport range;robot_range_snapshot(range);
            safetyData[0]=r.sensorFlags; // observation only: 1 IMU stale; 2 ToF unknown
            safetyData[1]=r.obstacleClearance;
            safetyData[2]=-1.0f; // ToF NEVER caps speed
            safetyData[3]=r.yawCorrection;
            safetyData[4]=r.gyroZ;
            safetyData[5]=r.state;
            safetyData[6]=float(range.range_status);
            safetyData[7]=float(range.errors);
            // V8 extensions preserve all eight V7 indices.
            safetyData[8]=r.interlockReason;
            safetyData[9]=r.benchArmed;
            safetyData[10]=r.benchRemainingMs;
            safetyData[11]=r.cmdFresh;
            rc=rcl_publish(&pubSafety,&safetyMsg,nullptr);
            telemError("safety",rc,errSafety,lastPublishLog);
        }
        if(now-lastWheel>=uint64_t(RobotConfig::DEBUG_MS)*1000){
            lastWheel=now;
            WheelReport r;robot_drive_get_report(r);
            wheelData[0]=r.targetL;wheelData[1]=r.targetR;
            wheelData[2]=r.rpmL;wheelData[3]=r.rpmR;
            wheelData[4]=r.pwmL;wheelData[5]=r.pwmR;
            wheelData[6]=r.distanceL;wheelData[7]=r.distanceR;
            wheelData[8]=r.headingDeg;wheelData[9]=r.state;
            rc=rcl_publish(&pubWheel,&wheelMsg,nullptr);
            telemError("wheel_debug",rc,errWheel,lastPublishLog);
        }
        if(now-lastFf>=500000ULL){
            lastFf=now;
            robot_drive_ff_status(ffStatusData);
            rc=rcl_publish(&pubFf,&ffStatus,nullptr);
            telemError("ff_status",rc,errFf,lastPublishLog);
        }
        if(now-lastPid>=500000){
            lastPid=now;
            robot_drive_pid_status(pidStatusData);
            rc=rcl_publish(&pubPid,&pidStatus,nullptr);
            telemError("pid_status",rc,errPid,lastPublishLog);
        }
        if(now-lastOdom>=uint64_t(RobotConfig::ODOM_MS)*1000){
            lastOdom=now;
            int64_t epoch=rmw_uros_epoch_nanos();
            if(epoch < 1600000000000000000LL){
                if(!warnedClock){
                    printf("WAIT time sync before publishing timestamped /odom\n");
                    warnedClock=true;
                }
            }else{
                WheelReport r;robot_drive_get_report(r);
                // Timestamp is taken at publication, with <50-ms report snapshot lag.
                copyStamp(odom.header.stamp,epoch);
                odom.pose.pose.position.x=r.x;
                odom.pose.pose.position.y=r.y;
                odom.pose.pose.position.z=0.0;
                odom.pose.pose.orientation.x=0;
                odom.pose.pose.orientation.y=0;
                odom.pose.pose.orientation.z=sin(r.yaw*0.5);
                odom.pose.pose.orientation.w=cos(r.yaw*0.5);
                odom.twist.twist.linear.x=r.linear;
                odom.twist.twist.linear.y=0;
                odom.twist.twist.linear.z=0;
                odom.twist.twist.angular.z=r.angular;
                rc=rcl_publish(&pubOdom,&odom,nullptr);
                telemError("odom",rc,errOdom,lastPublishLog);
            }
        }
        if(now-lastControl>=200000ULL){
            lastControl=now;
            WheelReport r;robot_drive_get_report(r);
            controlData[0]=r.refL;controlData[1]=r.refR;
            controlData[2]=r.ffL;controlData[3]=r.ffR;
            controlData[4]=r.pL;controlData[5]=r.iL;controlData[6]=r.dL;
            controlData[7]=r.pR;controlData[8]=r.iR;controlData[9]=r.dR;
            controlData[10]=r.demandL;controlData[11]=r.demandR;
            controlData[12]=r.pwmL;controlData[13]=r.pwmR;
            rc=rcl_publish(&pubControl,&controlMsg,nullptr);
            telemError("control_debug",rc,errControl,lastPublishLog);
        }
        if(now-lastLink>=uint64_t(RobotConfig::LINK_DEBUG_MS)*1000ULL){
            lastLink=now;
            robot_drive_link_debug(linkData);
            rc=rcl_publish(&pubLink,&linkMsg,nullptr);
            telemError("link_debug",rc,errLink,lastPublishLog);
        }
        vTaskDelay(pdMS_TO_TICKS(2));
    }
    // Release every entity on disconnect to avoid ghost DDS publishers.
    robot_drive_bench_request(false);
    (void)nav_msgs__msg__Odometry__fini(&odom);
    (void)sensor_msgs__msg__Imu__fini(&imuMsg);
    (void)sensor_msgs__msg__Range__fini(&rangeMsg);
    cleanup();
    printf("MICROROS_SESSION_CLOSED: restart Agent/ESP32; motors stopped by watchdog\n");
    vTaskDelete(nullptr);
#undef CHECK
}
} // namespace
extern "C" void app_main(){
    robot_sensors_start();
    robot_drive_start();
#if defined(CONFIG_MICRO_ROS_ESP_NETIF_WLAN) || defined(CONFIG_MICRO_ROS_ESP_NETIF_ENET)
    ESP_ERROR_CHECK(uros_network_interface_initialize());
#endif
    // Do not depend on undefined project-local Kconfig variables.
#ifdef CONFIG_MICRO_ROS_APP_STACK
    constexpr uint32_t taskStack=CONFIG_MICRO_ROS_APP_STACK;
#else
    constexpr uint32_t taskStack=20000;
#endif
#ifdef CONFIG_MICRO_ROS_APP_TASK_PRIO
    constexpr UBaseType_t taskPriority=CONFIG_MICRO_ROS_APP_TASK_PRIO;
#else
    constexpr UBaseType_t taskPriority=5;
#endif
    if(xTaskCreate(microRosTask,"uros_robot",taskStack,
                   nullptr,taskPriority,nullptr)!=pdPASS){
        printf("ERROR starting micro-ROS task, motors stop after watchdog\n");
    }
}
