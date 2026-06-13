import os
import shutil

moves = {
    "FC_Core/Modules/AttitudeController/AttitudeControl.cpp": "FC_Core/Controllers/mc_att_control/AttitudeControl.cpp",
    "FC_Core/Modules/AttitudeController/AttitudeControl.hpp": "FC_Core/Controllers/mc_att_control/AttitudeControl.hpp",
    "FC_Core/Modules/AttitudeController/AttitudeController.cpp": "FC_Core/Controllers/mc_att_control/AttitudeController.cpp",
    "FC_Core/Modules/AttitudeController/AttitudeController.h": "FC_Core/Controllers/mc_att_control/AttitudeController.h",
    "FC_Core/Modules/AttitudeController/RateControl.cpp": "FC_Core/Controllers/mc_rate_control/RateControl.cpp",
    "FC_Core/Modules/AttitudeController/RateControl.hpp": "FC_Core/Controllers/mc_rate_control/RateControl.hpp",
    "FC_Core/Modules/AttitudeController/PID.cpp": "FC_Core/Controllers/mc_rate_control/PID.cpp",
    "FC_Core/Modules/AttitudeController/PID.h": "FC_Core/Controllers/mc_rate_control/PID.h",
    "FC_Core/Modules/Mixer/ControlAllocation.cpp": "FC_Core/Controllers/control_allocator/ControlAllocation.cpp",
    "FC_Core/Modules/Mixer/ControlAllocation.hpp": "FC_Core/Controllers/control_allocator/ControlAllocation.hpp",
    "FC_Core/Modules/Mixer/Mixer.cpp": "FC_Core/Controllers/control_allocator/Mixer.cpp",
    "FC_Core/Modules/Mixer/Mixer.h": "FC_Core/Controllers/control_allocator/Mixer.h",
    "FC_Core/Modules/Commander/Commander.cpp": "FC_Core/Commander/Commander.cpp",
    "FC_Core/Modules/Commander/Commander.h": "FC_Core/Commander/Commander.h",
    "FC_Core/Modules/FlightModeManager/FlightModeManager.cpp": "FC_Core/Commander/flight_mode_manager/FlightModeManager.cpp",
    "FC_Core/Modules/FlightModeManager/FlightModeManager.h": "FC_Core/Commander/flight_mode_manager/FlightModeManager.h",
    "FC_Core/Modules/Sensors/SensorHub.cpp": "FC_Core/Sensors/SensorHub.cpp",
    "FC_Core/Modules/Sensors/SensorHub.h": "FC_Core/Sensors/SensorHub.h",
    "FC_Core/Modules/Math/AlphaFilter.hpp": "FC_Core/Math/AlphaFilter.hpp",
    "FC_Core/Estimator/AttitudeEstimator.cpp": "FC_Core/Estimator/ahrs/AttitudeEstimator.cpp",
    "FC_Core/Estimator/AttitudeEstimator.h": "FC_Core/Estimator/ahrs/AttitudeEstimator.h",
    "FC_Core/Estimator/MahonyAHRS.cpp": "FC_Core/Estimator/ahrs/MahonyAHRS.cpp",
    "FC_Core/Estimator/MahonyAHRS.h": "FC_Core/Estimator/ahrs/MahonyAHRS.h",
}

# Ensure destination dirs exist and move files
os.makedirs("FC_Core/Math", exist_ok=True)
for src, dst in moves.items():
    if os.path.exists(src):
        os.makedirs(os.path.dirname(dst), exist_ok=True)
        shutil.move(src, dst)

# Delete empty directories
for root, dirs, files in os.walk("FC_Core/Modules", topdown=False):
    for name in dirs:
        try:
            os.rmdir(os.path.join(root, name))
        except:
            pass
try:
    os.rmdir("FC_Core/Modules")
except:
    pass

# We also need to fix the Makefile
# Let's read the Makefile and replace the paths
makefile_path = "Debug/Makefile"
if os.path.exists(makefile_path):
    with open(makefile_path, "r") as f:
        content = f.read()
    for src, dst in moves.items():
        if src.endswith(".cpp") or src.endswith(".c"):
            content = content.replace(src, dst)
    with open(makefile_path, "w") as f:
        f.write(content)

print("Files moved successfully.")
