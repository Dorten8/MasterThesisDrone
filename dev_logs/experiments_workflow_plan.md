Here is the restored chronological, step-by-step execution logic. It removes the thematic groupings and aligns every task exactly in the order you need to perform it, incorporating the rooted phone, the visual standard diagrams, and the unaddressed technical gaps into the timeline.

# ---

**📋 Chronological Experiment Execution Blueprint**

**File Path Designation:** /home/ws/dev\_logs/experiment\_execution\_blueprint.md

## **Phase 0: Pre-Flight Code & Safety Hardening**

*This must be completed at the desk before the drone is powered on.*

* [x] **Audit Parameter Hacks:** Verified EKF2 parameters (EKF2_HGT_REF=3 for Vision, EKF2_EV_CTRL=11 for No Velocity, EKF2_EVP_NOISE=0.05 for MoCap trust). Kept CBRK_IO_SAFETY=22027 since no physical button is wired. Confirmed EKF2_EV_GATE remains at standard (5.0).  
* \[ \] **Code the Battery Failsafe:** Update the flight logic to actively monitor system voltage. Program a hard abort (immediate land/disarm) if capacity hits 40% to prevent ESC torque loss or Pi 5 compute failure.  
* \[ \] **Align Timestamps:** Configure PX4 to begin SD card logging (.ulg) precisely upon arming. Ensure the ROS 2 bag (.mcap) recording script registers the exact same Unix arming timestamp to sync high-frequency IMU data with MoCap truth.  
* \[ \] **Define Diagram Visual Standard:** Write the Python/Matplotlib template to generate top-down, orthogonal diagrams of the drone and column. This code must be locked in now so all future experiment graphics share an identical, thesis-ready visual standard.  
* \[ \] **Update Obstacle SSoT:** Register the cardboard column's dimensions and OptiTrack streaming ID inside config/drone\_config.json.  
* \[ \] **Code Geofencing:** Implement mathematical bounds in the Python trajectory script that refuse to send setpoints outside the safe MoCap area.

## **Phase 1: Physical Space & Recording Setup**

*Transition to the flight area to establish the physical constants.*

* \[ \] **Clear the Flight Area:** Remove all non-essential items to allow for clean video documentation.  
* \[ \] **Establish Camera Station:** Set up the tripod with the rooted Android phone at an orthogonal viewpoint. Tape the tripod legs to the floor so the framing is 100% repeatable across all experiments.  
* \[ \] **Establish Cloud Drop-Zone:** Create the local Google Drive folder structure (e.g., raw\_flight\_bags, high\_speed\_video, analytical\_plots) to streamline data offloading.

## **Phase 2: Calibration & Baseline Flights**

*Establishing the "Ground Truth" before intentional collisions.*

* \[ \] **Execute the "Ghost Flight":** Power the drone, initialize MoCap tracking, and physically carry the drone by hand through the intended obstacle-free path.  
* \[ \] **Normalize the Path:** Export the Ghost Flight data. Pass it to an LLM to extract a clean, normalized XYZ orthogonal flight path.  
* \[ \] **Fly the Baseline:** Run the drone autonomously along the normalized XYZ path. Ensure it completely clears the column.  
* \[ \] **Test the Data Pipeline:** Manually start the phone video, record the ROS 2 bag, fly the baseline, and drop all files into the Google Drive folder. Load them into Google Colab to verify the data parses correctly and matches the visual standard diagrams.

## **Phase 3: The Collision Experiments**

*The core loop for data collection.*

* \[ \] **Plan Vector 1:** Calculate the first intentional collision path with a very shallow impact vector against the column. Generate the visual standard diagram for this specific pass.  
* \[ \] **Execute Vector 1:** \- Start manual recording on the rooted Android phone.  
  * Initiate the modular Python recording script.  
  * Run the flight script.  
* \[ \] **Enforce Safety Pause:** The script must pause execution after the pass and display a terminal prompt (\[Proceed to next pass? Y/N\]).  
* \[ \] **Physical Inspection:** Visually inspect the protective cage clearance and propeller gaps to ensure structural integrity.  
* \[ \] **Acknowledge Prompt & Iterate:** If safe, hit 'Y' and increment the impact vector severity for the next pass. Repeat this loop until the required data limit is reached.

### ---

**❓ Technical Decisions Required (To unblock Phase 0):**

1. **Battery Monitoring:** Will you subscribe to /fmu/out/vehicle\_status in your offboard\_control.py, or run a separate node?  
2. **Geofencing:** Will this be mathematical clipping inside your Python code, or a PX4 internal geofence?  
3. **Recording Script:** Will the modular ROS 2 recording script be launched as a background subprocess within the main flight script, or run manually in a separate terminal?