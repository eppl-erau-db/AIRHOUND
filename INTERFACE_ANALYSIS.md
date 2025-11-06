# AIRHOUND Interface Contract Analysis
## Who's Right vs Scope Document?

### ROLE 2 → ROLE 3 Interface (THE CRITICAL QUESTION)

#### What the Scope Says (Choice Given):
```
ROLE 2 Output (choose one):
  - /target_yaw (std_msgs/Float32, radians) OR
  - /cmd_yaw_rate (geometry_msgs/Vector3 with z used)

ROLE 3 Input:
  - /target_yaw or /cmd_yaw_rate
```

#### What's Actually Implemented:

**Role 2 (Tracking-Geometry) - ACTUAL CODE:**
```python
self.pub_yaw = self.create_publisher(Float32, '/target_yaw_rate', 10)
```
❌ **WRONG** - Publishing `/target_yaw_rate` (not in spec!)
- Should be `/target_yaw` OR `/cmd_yaw_rate`
- Is Float32 (correct type for /target_yaw option)
- **Verdict: Role 2 used a HYBRID name not in the spec**

**Role 3 (middleware-test) - ACTUAL CODE:**
```cpp
subscription_ = this->create_subscription<std_msgs::msg::Float64>(
    "/yaw_command", 10, ...);
```
❌ **WRONG** - Subscribing to `/yaw_command` (not in spec!)
- Should be `/target_yaw` OR `/cmd_yaw_rate`
- Using Float64 (wrong - spec says Float32 or Vector3)
- **Verdict: Role 3 made up their own interface**

---

## CORRECT INTERFACE PER SCOPE

### Option 1: Absolute Yaw (Simpler)
```
Role 2 publishes: /target_yaw (std_msgs/Float32)
Role 3 subscribes: /target_yaw (std_msgs/Float32)
Role 3 maps to: TrajectorySetpoint.yaw (radians, FRD)
```

### Option 2: Yaw Rate
```
Role 2 publishes: /cmd_yaw_rate (geometry_msgs/Vector3)
Role 3 subscribes: /cmd_yaw_rate (geometry_msgs/Vector3)
Role 3 maps to: VehicleRatesSetpoint.yaw
```

---

## WHO NEEDS TO CHANGE?

### 🔧 Quick Fix (Minimal Changes):

**Both are wrong, but Role 2 is closer.** 

**Fix Role 2:**
```python
# Change line:
self.pub_yaw = self.create_publisher(Float32, '/target_yaw', 10)  # Remove '_rate'
```

**Fix Role 3:**
```cpp
// Change line:
subscription_ = this->create_subscription<std_msgs::msg::Float32>(
    "/target_yaw", 10, ...);
```

This gives you **Option 1** from the scope (absolute yaw).

---

## SEMANTICS MATTER

**Role 2's current name (`/target_yaw_rate`) is confusing because:**
- Publishing a rate (derivative)
- But it's actually computing yaw ERROR, not yaw rate
- The P controller converts error → rate implicitly
- Scope says pick ONE: absolute yaw OR yaw rate

**Scope's intent:**
- `/target_yaw` = where you want to point (absolute)
- `/cmd_yaw_rate` = how fast to spin (rate command)

**What Role 2 is actually doing:**
```python
yaw_rate = np.clip(yaw_err, -self.max_rate, self.max_rate)  # P control
```
This is computing a **rate command from error**, which is closer to `/cmd_yaw_rate` semantics.

---

## RECOMMENDED DECISION

### Choice A: Use Absolute Yaw (Cleaner)
- Role 2: Publish raw yaw error to `/target_yaw` 
- Role 3: Do the P control (rate limiting)
- **Pro:** Separation of concerns
- **Con:** Role 2 already has P control implemented

### Choice B: Use Yaw Rate (Current Code)
- Role 2: Keep P control, publish to `/cmd_yaw_rate` (fix name)
- Role 3: Just pass through to VehicleRatesSetpoint
- **Pro:** Matches current implementation
- **Con:** Wrong message type (Float32 vs Vector3)

### 🎯 EASIEST FIX: Choice A
```python
# Role 2 - Remove P control, publish raw error:
yaw_err = (u - self.cx) / self.fx
self.pub_yaw.publish(Float32(data=yaw_err))  # to /target_yaw

# Role 3 - Add P control:
float yaw_err = msg->data;
float yaw_rate = std::clamp(yaw_err, -max_rate, max_rate);
// ... publish to TrajectorySetpoint.yaw
```

---

## px4-converter-demo Branch

Checking what this branch actually contains...

## ACTUAL FINDINGS FROM px4-converter-demo BRANCH

**Role 3 (DDS_to_PX4_middleware package):**
```cpp
// middleware/DDS_to_PX4_middleware/src/px4_converter_node.cpp
yaw_command_sub_ = this->create_subscription<std_msgs::msg::Float64>(
    "/yaw_command", 10, ...);
```

✅ **This is a PROPER ROS 2 package** (has package.xml, CMakeLists.txt, launch files)
❌ **Still using wrong interface:** `/yaw_command` (Float64)

---

## FINAL VERDICT

### Who's Wrong?

**BOTH Role 2 AND Role 3 are wrong!**

| Role | Scope Says | Actually Did | Correct? |
|------|------------|--------------|----------|
| **Role 2** | `/target_yaw` (Float32) OR `/cmd_yaw_rate` (Vector3) | `/target_yaw_rate` (Float32) | ❌ Wrong name |
| **Role 3** | Subscribe to `/target_yaw` OR `/cmd_yaw_rate` | `/yaw_command` (Float64) | ❌ Wrong name & type |

### Why This Happened
**They worked in isolation without agreeing on the interface first!**
- Role 2 made up a hybrid name (`/target_yaw_rate`)
- Role 3 made up their own name (`/yaw_command`)
- Neither checked the scope document

---

## THE FIX (Team Decision Needed)

### Option 1: Follow Scope Exactly - `/target_yaw` (RECOMMENDED)
```python
# Role 2 - ONE LINE CHANGE:
self.pub_yaw = self.create_publisher(Float32, '/target_yaw', 10)
```
```cpp
// Role 3 - TWO LINE CHANGES:
yaw_command_sub_ = this->create_subscription<std_msgs::msg::Float32>(
    "/target_yaw", 10, ...);
```

### Option 2: Follow Scope - `/cmd_yaw_rate` (More Work)
```python
# Role 2 - Change message type:
from geometry_msgs.msg import Vector3
self.pub_yaw = self.create_publisher(Vector3, '/cmd_yaw_rate', 10)
msg = Vector3()
msg.z = yaw_rate  # Only z component used
```
```cpp
// Role 3 - Change subscriber:
#include <geometry_msgs/msg/vector3.hpp>
yaw_command_sub_ = this->create_subscription<geometry_msgs::msg::Vector3>(
    "/cmd_yaw_rate", 10, ...);
// Use msg->z
```

---

## CRITICAL: What About Role 4?

You said: *"role 4 is created on the local machine -- just not on the git"*

### This is RISKY because:
1. ❌ Not version controlled = can't reproduce
2. ❌ Team can't see/review the integration
3. ❌ If sim machine dies, work is lost
4. ❌ Can't test integration on other machines

### What SHOULD exist (even if sim-specific):
```
ws_ros2/src/airhound_sim/
  ├── launch/
  │   ├── sitl_base.launch.py       # SITL + Micro XRCE-DDS Agent
  │   ├── full_system.launch.py     # All nodes together
  │   └── README.md                 # "Run this on sim machine"
  └── config/
      └── bridge.yaml               # ros_gz_bridge config
```

**Even if Gazebo world is sim-specific, the LAUNCH FILES should be in git!**

---

## BOTTOM LINE

### To Get Working Demo:

1. **Immediate (30 min):** Team meeting to agree on ONE interface
   - Vote: `/target_yaw` (Float32) - easiest fix
   
2. **Role 2 (5 min fix):**
   ```bash
   # Fix import
   sed -i '1s/#import rclpy/import rclpy/' ws_ros2/src/Tracking-Geometry/Tracking-Geometry/tracking.py
   
   # Fix topic name
   sed -i "s|'/target_yaw_rate'|'/target_yaw'|" ws_ros2/src/Tracking-Geometry/Tracking-Geometry/tracking.py
   
   # Add entry point to setup.py
   # (needs manual edit - add 'tracking_node = ...')
   ```

3. **Role 3 (10 min fix):**
   ```bash
   # Update px4_converter_node.cpp:
   # Line 86-87: Change Float64 → Float32, /yaw_command → /target_yaw
   ```

4. **Role 4 (1 hour):**
   - Create minimal launch file in git
   - Document what needs to run on sim machine
   - Create README with exact commands

5. **Integration Test (2 hours):**
   - Launch all nodes
   - Verify topics connect
   - Record success video

### Current Status (Honest):
- ✅ Components exist
- ❌ Don't talk to each other (wrong interfaces)
- ❌ No integration tested
- ⚠️ Could fix in < 1 day if team coordinates

**You have the parts. You just need to make them fit together per the spec.**

