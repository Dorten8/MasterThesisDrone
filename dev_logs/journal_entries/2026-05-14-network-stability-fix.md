# Session Journal: 2026-05-14 — Network Stability & Wi-Fi Power Management

## Where Are We in the Project
We are refining the drone's reliability and infrastructure. Intermittent network issues on the Raspberry Pi were hindering remote access and telemetry streams, especially during headless boots.

## What We Worked On and Why

### 1. Wi-Fi Power Management Fix
- **What**: Disabled Wi-Fi power saving modes via NetworkManager configuration.
- **Why**: The Pi's Wi-Fi was failing to connect or stay active after booting up in headless mode. Interestingly, connecting a monitor seemed to "wake up" the interface, suggesting an aggressive power-saving state was preventing proper initialization.
- **Hurdle**: The `iw` utility was missing from the minimal OS image, and temporary fixes via CLI don't survive reboots.
- **Outcome**: Modified `/etc/NetworkManager/conf.d/default-wifi-powersave-on.conf` to explicitly disable power saving (`wifi.powersave = 2`).

## Technical Overview of Changes

### [Network Configuration]
#### [MODIFY] `/etc/NetworkManager/conf.d/default-wifi-powersave-on.conf`
- Changed `wifi.powersave` value from `3` (enabled) to `2` (disabled).
- Restarted `NetworkManager` service to apply changes.

## Outcome
- ✅ **Connectivity**: Wi-Fi now initializes reliably on boot without needing a physical monitor attached.
- ✅ **Stability**: Reduced risk of SSH/ROS2 log stream drops during flight prep.

## Learning Summary
1. **Minimal Image Omissions**: Minimal Linux distributions often exclude older tools like `iw`, requiring modern `nmcli` or configuration files for network tweaks.
2. **Headless Boot Quirks**: Display-related triggers (like plugging in a monitor) can sometimes affect peripheral power states on SBCs like the Raspberry Pi.
3. **NetworkManager Persistence**: Direct `iw` commands are often overwritten by NetworkManager on restart; configuration files are the standard for permanent changes.

## Next Steps
1. Monitor connection stability during the next field test.
2. Continue with the [Bottleneck Diagnosis Roadmap](file:///home/dorten/pi_drone_sshfs/dev_logs/journal_entries/2026-05-11-bottleneck-diagnosis-roadmap.md).
