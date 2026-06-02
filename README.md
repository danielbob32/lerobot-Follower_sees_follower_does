# lerobot-Follower_sees_follower_does

This a setup for mimicking movement of SO-101 Leader arm (moved by hand) by the corresponding Follower arm.

Equipment:
 - SO-101 follower arm with a control board
 - SO-101 leader arm with a control board
 - 2 * power suppliers (5V)
 - 2 * USB-C cables

Prequisites:
- Python
- pyserial module

The main script is follower_sees_follower_does.py. It should run as expected without special adjustments, but a few things must be set up in the code, while the two arms are connected to power and to computer:
- Ports - check the port names of the two arms, and update them in the script
- Motor IDs - in the script, the servo motors IDs are oredered the way we set them up in the lab. For any other order, they should be ordered in the Python list by the order of joints in the [official assembly guide]([url](https://huggingface.co/docs/lerobot/so101)). Note that in our setup the motors were ordered from 1 to 12 (follower first), apart from motors 5 and 6, which are swapped.

### What it does
After running follower_sees_follower_does.py while the two arms are connected, you will be asked to move the two to the exact same position. It is required as the follower's moves will be in relation to the leader's, regardless of the starting point. After positioning, hit enter and the two arms will be in sync.

### Telemetry dashboard & 3D sim

While `follower_sees_follower_does.py` is running, a dashboard is served at `http://127.0.0.1:8080`. It shows:

- A 3D view of the SO-101 (ghost arm, semi-transparent) that mirrors the live follower pose.
- Per-motor tables: position, goal, temperature, voltage, load — with 60-second trend sparklines for temperature and load.

Thresholds (color-coded):
- Temperature: amber ≥ 50 °C, red ≥ 65 °C
- Voltage: amber outside 10.5–13.5 V
- Load: amber ≥ 70%, red ≥ 90%

The dashboard uses only the Python standard library on the server; three.js and the SO-101 URDF + meshes are vendored into the repo, so no network is needed at runtime. If port 8080 is already in use, the script logs a warning and continues without a dashboard.

#### Browser control mode

Instead of the physical leader arm, you can drive the follower by dragging the end-effector in the 3D view.

1. Start teleop normally — sync the two arms as usual and press ENTER.
2. In the browser, click **Engage**.
3. Click-drag the blue sphere at the end-effector to move the arm. IK solves in the browser; the follower tracks with ~200 ms of lag (intentional safety clamp).
4. Use the side sliders for wrist pitch, wrist roll, and gripper — these bypass IK.
5. Click **Release Control** to hand control back to the physical leader.

Safety layers active during browser control:
- **Heartbeat watchdog** — if the browser stops pinging for 1.5 s (tab closed, laptop sleep, network drop), the follower returns to physical-leader control automatically, with a torque blip to prevent drift.
- **Per-joint velocity clamp** — each sync-write advances a joint by at most 100 ticks (~9°).
- **URDF joint limits** — IK output is clipped to the servo's safe working range.
- **Release button** — always visible; one click returns control.
- **Two-stage Ctrl+C** — press once to freeze motion and release control (arm holds position). Press again within 5 seconds for full shutdown.

#### Per-rig calibration

The vendored SO-101 URDF defines joint angles in radians; servos speak ticks. `kinematics_calibration.json` owns the per-joint `tick_zero` (the tick value that corresponds to the URDF's home pose), `tick_sign`, and `tick_min`/`tick_max`.

One-time calibration:

1. Stop `follower_sees_follower_does.py`.
2. Manually move the follower arm into the URDF home pose: shoulder straight, elbow straight, wrist level, arm pointing forward along the world X axis, gripper half-open.
3. Run `python calibrate_home.py`. It records the current positions as `tick_zero`.
4. Restart the main script. The ghost arm should match the real arm very closely. If a joint moves the wrong way (ghost rotates left when real moves right), edit `kinematics_calibration.json` and flip that joint's `tick_sign` between `+1` and `-1`.

Defaults shipped with the repo assume a rig assembled per the [SO-101 assembly guide](https://huggingface.co/docs/lerobot/so101); calibration is strictly a refinement.

### Modifications & Troubleshooting
###### Motor ID
If you wish to change a motor ID, there is a provided script for that. To run the script, set the old ID constant in the code (the default is 1, if you don't know there a script provided), and to run from the terminal, type python set_motor_id.py new_id. Note that only a single motor can be connected at a time, and remember to change the port name.

###### Scan motors
To check which motors are connected, run the script scan_motors.py. It will print all IDs of discovered motors. If two motors share the same ID, the script won't differentiate between them.
