#!/bin/bash
set -e

# Give the unprivileged user access to GPU render nodes (/dev/dri/*).
# The base entrypoint drops to ${USERNAME} with gosu, which only reads /etc/group,
# so --group-add from "docker run" has no effect. Mirror the device groups here.
for dev in /dev/dri/*; do
    [ -c "$dev" ] || continue
    gid=$(stat -c '%g' "$dev")
    [ "$gid" = "0" ] && continue
    grp=$(getent group "$gid" | cut -d: -f1)
    if [ -z "$grp" ]; then
        grp="hostgpu${gid}"
        groupadd -g "$gid" "$grp" 2>/dev/null || true
    fi
    usermod -aG "$grp" "${USERNAME:-aw}" 2>/dev/null || true
done

exec /docker-entrypoint.sh "$@"
