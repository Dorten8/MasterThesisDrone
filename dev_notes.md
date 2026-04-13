# MAC addresses:
eth0 (Ethernet): 88:a2:9e:65:43:b4

wlan0 (WiFi): 88:a2:9e:65:43:b5

docker0 (Docker bridge): 36:b5:a7:3c:10:02

## SSHFS
~/pi_drone_sshfs

### Make sure the target folder exists first
mkdir -p ~/pi_drone_sshfs

### Connect the Pi's workspace to your laptop
sshfs dorten@dorten-pi5drone.local:/home/dorten/ws ~/pi_drone_sshfs -o auto_cache,reconnect,follow_symlinks
### unmount
umount ~/pi_drone_sshfs

### The -u is unmount, -z is 'lazy' (cleans up the mess even if busy)
fusermount -u -z ~/pi_drone_sshfs

## MAVLINK-ROUTER
Running it with specific config (not default in root)
mavlink-routerd -c /home/ws/config/mavlink-router/main.conf