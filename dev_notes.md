~/pi_drone_sshfs

# Make sure the target folder exists first
mkdir -p ~/pi_drone_sshfs

# Connect the Pi's workspace to your laptop
sshfs dorten@dorten-pi5drone.local:/home/dorten/ws ~/pi_drone_sshfs -o auto_cache,reconnect,follow_symlinks

# unmount
umount ~/pi_drone_sshfs

# The -u is unmount, -z is 'lazy' (cleans up the mess even if busy)
fusermount -u -z ~/pi_drone_sshfs