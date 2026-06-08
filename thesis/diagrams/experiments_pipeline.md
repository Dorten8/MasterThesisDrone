```mermaid
graph TD
    %% ==========================================================
    %% 1. Node Definitions & Layout
    %% ==========================================================

    %% External Infrastructure & Hardware
    MOCAP["OptiTrack MOCAP System<br>(Spatial Ground Truth)"]:::classHardware
    PX4["Drone Flight Controller<br>(PX4 on NuttX Board)"]:::classHardware

    %% Onboard Hardware Environment
    subgraph Pi5 ["Hardware: Onboard Computer (Raspberry Pi 5)"]
        subgraph Ubuntu ["OS: Ubuntu 22.04 LTS"]
            subgraph ROS2 ["Software Stack: ROS 2 Workspace Layer"]
                
                %% Source Submodules (Under src/)
                MSGS["px4_msgs<br>(Interface Message Definitions)"]:::classSubmodule
                COM["px4_ros_com<br>(Frame Transforms & Utility Libs)"]:::classSubmodule
                MCT["motion_capture_tracking<br>(MOCAP Receiver Node)"]:::classSubmodule
                MPB["mocap_px4_bridge<br>(Odometry Transformer)"]:::classSubmodule
                FD["flight_director.py<br>(Mission Execution & Geofence)"]:::classFlightControl
                FM["flight_missions.py<br>(Discrete Flight Routines)"]:::classFlightControl
                FR["flight_recorder<br>(Telemetry Logger Node)"]:::classFlightControl
                
                %% Flight & Drone Control Bases / OS Layers
                Agent["Micro-XRCE-DDS Agent<br>(Middleware Bridge)"]:::classSubmodule
            end
            
            %% Data Blocks
            LogDir["Directory: dev_logs/flights/<br>(Local .mcap Storage)"]:::classData
        end
    end

    %% Container Styles
    style Pi5 fill:#7f8c8d,stroke:#333,stroke-width:1px,color:#fff
    style Ubuntu fill:#bdc3c7,stroke:#333,stroke-width:1px,color:#000
    style ROS2 fill:#bdc3c7,stroke:#333,stroke-width:1px,color:#000
    %% ==========================================================
    %% 2. Connections & Pipelines (Strict Arrow Ordering for linkStyle)
    %% ==========================================================
    
    %% [Link 0 to 4] Dependency Data Arrows (Orange)
    MSGS -.->|Provides Struct Layouts| COM
    MSGS -.->|Imported for Serialization| MPB
    MSGS -.->|Imported for Topic Pub/Sub| FD
    MSGS -.->|Imported for Setpoint Generation| FM
    MSGS -.->|Imported for Parsing Data| FR
    
    %% [Link 5] Transformation Logic (Orange)
    COM -.->|Handles ENU ↔ NED Conversions| MPB

    %% [Link 6 & 7] Real-time Localization Pipeline (Magenta / MOCAP)
    MOCAP -.->|Network Streams / Spatial Vectors| MCT
    MCT -->|Raw Spatial Coordinates| MPB
    
    %% [Link 8] Bridge to Middleware (Orange Data)
    MPB -->|/fmu/in/vehicle_visual_odometry| Agent
    
    %% [Link 9 & 10] Mission Logic & Control Pipeline (Green / Control)
    FD -->|Supervises State & Triggers| FM
    FM -->|/fmu/in/trajectory_setpoint| Agent
    
    %% [Link 11] Status Feedback Loop (Orange Data)
    Agent -->|/fmu/out/vehicle_status| FD
    
    %% [Link 12] Middleware Hardware Link (Green Control / Bidirectional)
    Agent <-->|Serial / UDP Transport Link| PX4
    
    %% [Link 13 & 14] Empirical Logging Pipeline (Orange Data)
    Agent -->|Streamed Topics /fmu/*| FR
    FR -->|Writes Segmented Data Streams| LogDir

    %% ==========================================================
    %% 3. Theme Classes (CSS)
    %% ==========================================================
    classDef classSubmodule fill:#3498db,stroke:#2980b9,stroke-width:2px,color:#fff;
    classDef classFlightControl fill:#2ecc71,stroke:#27ae60,stroke-width:2px,color:#fff;
    classDef classHardware fill:#7f8c8d,stroke:#34495e,stroke-width:2px,color:#fff;
    classDef classOS fill:#bdc3c7,stroke:#7f8c8d,stroke-width:2px,color:#000;
    classDef classData fill:#e67e22,stroke:#d35400,stroke-width:2px,color:#fff;

    %% ==========================================================
    %% 4. Connection Link Styling
    %% ==========================================================
    linkStyle 0,1,2,3,4,5 stroke:#e67e22,stroke-width:2px,stroke-dasharray: 5 5;
    linkStyle 6,7 stroke:#e024c3,stroke-width:2.5px;
    linkStyle 8 stroke:#e67e22,stroke-width:2px;
    linkStyle 9,10 stroke:#2ecc71,stroke-width:2.5px;
    linkStyle 11 stroke:#e67e22,stroke-width:2px;
    linkStyle 12 stroke:#2ecc71,stroke-width:2.5px;
    linkStyle 13,14 stroke:#e67e22,stroke-width:2px;