# AIRHOUND - Critical Fixes Needed (8 Days to Demo)

## 🔥 BLOCKERS (Must Fix to Get ANY Integration Working)

### 1. Role 2 (Tracking-Geometry) - THREE CRITICAL BUGS

**Location:** `ws_ros2/src/Tracking-Geometry/Tracking-Geometry/tracking.py`

#### Bug 1.1: Import Error (Line 1)
```python
# CURRENT (BROKEN):
#import rclpy           # ← COMMENTED OUT!
from rclpy.node import Node

# FIX:
import rclpy
from rclpy.node import Node
```
**Impact:** Code won't run at all  
**Time to fix:** 10 seconds

#### Bug 1.2: Wrong Topic Name (Line ~30)
```python
# CURRENT (WRONG - not in spec):
self.pub_yaw = self.create_publisher(Float32, '/target_yaw_rate', 10)

# FIX (match scope document):
self.pub_yaw = self.create_publisher(Float32, '/target_yaw', 10)
```
**Impact:** Won't connect to Role 3  
**Time to fix:** 10 seconds

#### Bug 1.3: Missing Entry Point
**Location:** `ws_ros2/src/Tracking-Geometry/setup.py`

```python
# CURRENT (BROKEN):
entry_points={
    'console_scripts': [
        # EMPTY!
    ],
},

# FIX:
entry_points={
    'console_scripts': [
        'tracking_node = Tracking-Geometry.tracking:main',
    ],
},
```
**Impact:** Can't launch with `ros2 run`  
**Time to fix:** 30 seconds

---

### 2. Role 3 (DDS_to_PX4_middleware) - TWO INTERFACE BUGS

**Location:** `middleware/DDS_to_PX4_middleware/src/px4_converter_node.cpp`

#### Bug 2.1: Wrong Topic Name + Type (Lines 86-88)
```cpp
// CURRENT (WRONG):
yaw_command_sub_ = this->create_subscription<std_msgs::msg::Float64>(
    "/yaw_command", 10,
    std::bind(&PX4ConverterNode::yaw_command_callback, this, std::placeholders::_1));

// FIX:
yaw_command_sub_ = this->create_subscription<std_msgs::msg::Float32>(
    "/target_yaw", 10,
    std::bind(&PX4ConverterNode::yaw_command_callback, this, std::placeholders::_1));
```
**Impact:** Won't receive commands from Role 2  
**Time to fix:** 1 minute

#### Bug 2.2: Update Callback Signature (Line 108)
```cpp
// CURRENT:
void yaw_command_callback(const std_msgs::msg::Float64::SharedPtr msg) {

// FIX:
void yaw_command_callback(const std_msgs::msg::Float32::SharedPtr msg) {
```
**Impact:** Won't compile  
**Time to fix:** 20 seconds

#### Bug 2.3: Package Not in Main Workspace
**Current:** `middleware/DDS_to_PX4_middleware/` (separate workspace)
**Should be:** `ws_ros2/src/airhound_offboard/` (unified workspace)

**Impact:** Can't build all packages together  
**Time to fix:** 5 minutes (copy + update paths)

---

### 3. Role 4 (Simulation) - MISSING COMPONENTS

#### Missing 3.1: Integration Launch File
**Needed:** `ws_ros2/src/airhound_sim/launch/full_system.launch.py`

Should launch:
- ✅ Mock detector (exists)
- ✅ Tracking node (exists, needs fixes)
- ✅ Offboard controller (exists, needs packaging)
- ❌ Micro XRCE-DDS Agent (missing launch)
- ❌ PX4 SITL (missing launch)

**Time to create:** 2 hours (if using stock PX4 SITL world)

#### Missing 3.2: Documentation
Need `ws_ros2/src/airhound_sim/README.md` with:
- Prerequisites (PX4 installed, Gazebo version, etc.)
- Launch sequence
- Expected topics
- How to verify it's working

**Time to create:** 30 minutes

---

## ⚠️ MEDIUM PRIORITY (Should Fix for Clean Demo)

### 4. Role 2 - Missing Launch File
**Needed:** `ws_ros2/src/Tracking-Geometry/launch/tracking.launch.py`

```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='Tracking-Geometry',
            executable='tracking_node',
            name='tracking_node',
            output='screen',
            parameters=[{
                'max_rate': 1.0,
                'deadband': 0.01,
            }]
        ),
    ])
```
**Time to create:** 5 minutes

---

### 5. Missing Dependencies in package.xml

#### Role 2 - Missing ROS Dependencies
**Location:** `ws_ros2/src/Tracking-Geometry/package.xml`

```xml
<!-- CURRENT: Only has test dependencies -->

<!-- NEED TO ADD: -->
<exec_depend>rclpy</exec_depend>
<exec_depend>std_msgs</exec_depend>
<exec_depend>sensor_msgs</exec_depend>
<exec_depend>vision_msgs</exec_depend>
<exec_depend>geometry_msgs</exec_depend>
<exec_depend>tf2_ros</exec_depend>
```
**Impact:** Won't install dependencies properly  
**Time to fix:** 2 minutes

---

## 📋 INTEGRATION CHECKLIST

### Minimal Working Demo (SITL Only)

**Required Steps:**
```bash
# 1. Fix Role 2 bugs (5 min)
# 2. Fix Role 3 interface (2 min)
# 3. Rebuild packages (5 min)
colcon build --packages-select Tracking-Geometry airhound_perception
# 4. Test Role 1 → Role 2 connection (10 min)
# 5. Package Role 3 properly (10 min)
# 6. Test Role 2 → Role 3 connection (10 min)
# 7. Create minimal system launch (30 min)
# 8. Test full pipeline (1 hour)
```

**Total Estimated Time (if nothing goes wrong):** ~3 hours

**Realistic Time (with debugging):** ~6-8 hours

---

## 🎯 WHAT YOU CAN DO RIGHT NOW

### Option A: Document & Delegate (Recommended)
1. ✅ Send this document to team
2. ✅ Assign bugs to role owners
3. ✅ Set deadline: "Fixed by tomorrow EOD"
4. ✅ Offer to help if stuck
5. Schedule integration test for Day 3

### Option B: Emergency Fix (If Team Unresponsive)
1. Create branch: `fix/critical-interface-bugs`
2. Fix Role 2 bugs (5 min)
3. Submit PR with detailed explanation
4. Message team: "Critical bugs blocking integration, please review ASAP"
5. If no response in 24h, merge and notify

### Option C: Pair Programming Session
1. Schedule 2-hour session with Role 2 & 3
2. Share screen, walk through bugs together
3. They make the changes while you guide
4. Builds understanding + gets it done

---

## 📊 HONEST TIMELINE ASSESSMENT

### If Fixes Start TODAY:
- **Day 1 (Today):** Fix bugs, rebuild  
- **Day 2:** Test individual connections  
- **Day 3:** Integration testing  
- **Day 4:** Debug issues, create launch  
- **Day 5:** Full system test  
- **Day 6:** Record demo video  
- **Day 7:** Create poster content  
- **Day 8:** Poster finalization  

✅ **FEASIBLE**

### If Fixes Start in 2+ Days:
- **Days 1-2:** Lost to coordination  
- **Day 3:** Rushed fixes  
- **Day 4:** Integration panic  
- **Day 5-7:** Debug hell  
- **Day 8:** "Components developed, integration in progress" poster  

⚠️ **RISKY**

---

## 🚨 ABSOLUTE MINIMUM FOR POSTER

To honestly claim "working system":

**Must Have:**
1. ✅ Role 1 publishes `/detections` (YOU HAVE THIS)
2. ✅ Role 2 subscribes `/detections`, publishes `/target_yaw` (NEEDS 3 FIXES)
3. ✅ Role 3 subscribes `/target_yaw`, publishes PX4 commands (NEEDS 2 FIXES)
4. ✅ One successful end-to-end topic flow recording (NEEDS INTEGRATION)
5. ✅ Screen recording of `ros2 topic list` showing all connected (5 MIN)

**Can Skip (Nice to Have):**
- Real Gazebo visualization (can use command line only)
- Actual quad movement (can show topic messages)
- Perfect control tuning (can show it "tries" to track)

**Minimum Honest Claim:**
"Perception, tracking, and offboard control modules developed and interface-validated in simulation environment"

---

## SUMMARY

**Total Critical Bugs:** 8  
**Total Fix Time (optimistic):** 30 minutes  
**Total Fix Time (realistic):** 2-3 hours with testing  
**Integration Time:** 4-6 hours  
**Documentation Time:** 2 hours  

**Total to Working Demo:** ~10-15 hours of focused team effort

**You have 8 days. This is 100% doable IF:**
- Team coordinates in next 24 hours
- Bugs fixed by end of Day 2
- Integration starts Day 3
- No one goes AWOL

**Your role:** Integration PM / Bug Detective (which you're already doing great at)

---

