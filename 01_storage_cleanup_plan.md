# Plan 1: Storage cleanup and system inventory
Purpose: inventory; free >= 20 GB (floor 15) without harming ROS 2 Humble, dual-boot, user data.
Inputs: none. Outputs: ~/drdo_setup/reports/01_storage_report.md, STATE.md.
Orchestrator turn budget: none on Gemini CLI (1,500 requests/day); 8 if on a 50/day orchestrator. Dispatch one card at a time.

## Orchestrator
Rules
1. Never run commands. Every {{PLACEHOLDER}} is filled non-empty and absolute; independent cards go out as one batch.
2. Update STATE.md after every report. free_gb = smallest free_gb in the latest batch (disk-changing cards end with df); T1 only at N0.
3. FAILED/BLOCKED/NEEDS_USER: show the user the raw block, ask; re-dispatch only after the user's fix.
4. Tier-2 (T8-T13) only after the user's yes per card; batch the questions using the N6 text.
5. apt cards T5 T9 T10: one per turn, never together (dpkg lock).
6. Never DISTRO=humble; WS_PATH never ~, ~/training_pool, ~/px4_ros_ws, ~/drdo_ws, ~/PX4-Autopilot, ~/ardupilot* or under /opt.
7. Never resize/move partitions or delete /swapfile. Fortress ros-humble-ros-gz*: notes only, never a candidate.
8. By turn 8 dispatch T14 regardless.

State keys: the machine-facts keys of the Report template, plus before_gb, deleted, declined.

Decision tree
```
N0  batch [T1 T2 T3]; before_gb = free_gb -> N1
N1  all DONE? no -> [ASK USER] raw block; re-dispatch that card after the fix -> N1 ; yes -> N2
N2  free_gb >= 20 -> N9 ; else -> N3
N3  lvm = yes and vg_free_gb >= 1 -> T4 {{LV_PATH}}=lv_path -> N2 ; else -> N4
N4  batch [T5 T6 T7 {{TRASH_LIMIT}}=500M] -> N5
N5  free_gb >= 20 -> N9 ; else -> N6
N6  candidates (fact -> card | what a yes deletes; state the GB):
      docker_present = yes, docker_gb >= 1 -> T8 | stopped containers, images no running container uses
      ros_distros entry != humble -> T9 {{DISTRO}}=entry, one card each | ros-<entry>-*, /opt/ros/<entry>
      ros_gz_packages has gazebo, libgazebo or ros-humble-gazebo- -> T10 | Gazebo Classic 11 + bindings
      px4_build_gb >= 1 -> T11 | build/ + submodule checkouts; Plan 2 rebuilds ~30 min, pointless if px4_tag is v1.16.*
      ardupilot_present = yes -> T12 | both repos incl. local commits, SITL logs; MAVProxy
      workspaces entry allowed by Rule 6 -> T13 {{WS_PATH}}=entry | build/ install/ log/
    none -> N8 ; else [ASK USER] one message, yes/no per candidate -> record declined -> N7
N7  batch approved non-apt cards + first apt card; other apt cards one per turn; record deleted -> N8
N8  free_gb >= 15 -> N9 ; else [ASK USER] "/data route: in Windows Disk Management shrink C:, leave it unallocated
    (BitLocker, Fast Startup off first); in Ubuntu GParted make a new ext4 ONLY in that space; mount /data via fstab" -> N9
N9  T14 with all facts -> done
```

Ask the user when
- A card returns NEEDS_USER, BLOCKED or FAILED.
- Before every Tier-2 card (N6). T7 Trash NEEDS_USER: after a yes re-dispatch T7 with {{TRASH_LIMIT}}=999G.
- N8 below 15 GB.

Done when: T14 DONE, STATE.md node: N9, free_gb >= 15 or the /data route was given at N8.

## Task cards

### T1 sense-disk  |  tier: flash  |  needs: sudo
Goal: free space, LVM, RAM, GPU.
Inputs: none
Run:
```bash
df -h /
findmnt -no SOURCE,FSTYPE /
lsblk -f
sudo vgs -o vg_name,vg_free --units g || true
sudo lvs -o lv_path,lv_dm_path,lv_size --units g || true
free -h
lspci | grep -Ei 'vga|3d|display'
nvidia-smi --query-gpu=name,memory.total --format=csv || echo nvidia-smi: none
```
Expect: df prints one / row; vgs/lvs empty or "command not found" = no LVM.
Stop if: df, findmnt or lsblk errors.
Return: free_gb (df Avail, integer), root_src (findmnt SOURCE), lvm (yes only if root_src equals an lvs lv_dm_path), lv_path (root_src when lvm=yes, else none), vg_free_gb (0 if none), ram_gb, gpu (all lspci and nvidia-smi lines).

### T2 sense-ros-gz-autopilots  |  tier: flash  |  needs: none
Goal: ROS distros, Gazebo versions, biggest packages, PX4, ArduPilot.
Inputs: none
Run:
```bash
ls /opt/ros
gz sim --versions || true
ign gazebo --versions || true
dpkg-query -Wf '${Package} ${Version}\n' | grep -Ei '^(gz-|libgz|ignition|libignition|gazebo|libgazebo|ros-humble-ros-gz|ros-humble-gz-|ros-humble-gazebo)' || true
dpkg-query -Wf '${Installed-Size}\t${Package}\n' | sort -n | tail -25
git -C ~/PX4-Autopilot describe --tags || echo px4: none
du -sh ~/PX4-Autopilot/build || echo px4_build: none
ls -d ~/ardupilot ~/ardupilot_gazebo || echo ardupilot: none
which sim_vehicle.py || echo sim_vehicle: none
python3 -m pip show MAVProxy || echo mavproxy: none
```
Expect: /opt/ros lists humble; "command not found", "No such file" and "none" lines acceptable.
Stop if: ls /opt/ros errors.
Return: ros_distros, gz_versions (gz and ign lines; none if both not found), ros_gz_packages, px4_path (or none), px4_tag, px4_build_gb (0 if absent), ardupilot_present (yes if either dir listed), mavproxy.

### T3 sense-caches  |  tier: flash  |  needs: sudo
Goal: snaps, journal, docker, caches, colcon workspaces, big files.
Inputs: none
Run:
```bash
snap list --all | awk '/disabled/{print "/var/lib/snapd/snaps/"$1"_"$3".snap"}' | xargs -r du -ch | tail -1
journalctl --disk-usage
sudo docker system df || echo docker: none
sudo du -xh -d1 / | sort -h | tail -12
du -xh -d1 ~ | sort -h | tail -12
du -sh ~/.cache/* 2>/dev/null | sort -h | tail -8
find ~ -maxdepth 5 -path '*/build/.built_by' -not -path '*/.*/*' -exec dirname {} \; | sed 's|/build$||' | xargs -r du -sh
sudo find / -xdev -type f -size +500M -not -path /swapfile -exec du -h {} + 2>/dev/null | sort -h
```
Expect: sizes; "docker: none" acceptable; blank first line = no disabled snaps.
Stop if: journalctl or du on / errors.
Return: snap_disabled_gb (0 if blank), journal_gb, docker_present, docker_gb (RECLAIMABLE Images + Build Cache, 0 if absent), workspaces (find line: path GB), big_files (path GB).

### T4 lvm-extend  |  tier: flash  |  needs: sudo
Goal: grow the root LV into free VG space, online.
Inputs: {{LV_PATH}}
Run:
```bash
findmnt -no FSTYPE,TARGET "{{LV_PATH}}"
sudo lvextend -t -l +100%FREE "{{LV_PATH}}"
sudo lvextend -l +100%FREE "{{LV_PATH}}"
sudo resize2fs "{{LV_PATH}}"
df -h /
```
Expect: "changed" from lvextend; resize2fs prints a new size.
Stop if: findmnt does not print exactly "ext4 /", or the -t line errors.
Return: free_gb.

### T5 tier1-apt  |  tier: flash  |  needs: sudo
Goal: purge orphans and apt cache.
Inputs: none
Run:
```bash
sudo apt-get autoremove --purge --dry-run
sudo apt-get autoremove --purge -y
sudo apt-get clean
df -h /
```
Expect: "N to remove"; then df.
Stop if: the dry-run REMOVED list has any package starting with ros-humble-, nvidia-, gz-, libgz or ignition, or any line starts with E:.
Return: removed_count, free_gb.

### T6 tier1-journal-snap  |  tier: flash  |  needs: sudo
Goal: cap journald to 200 MB; retain 2 snap revisions, remove disabled ones.
Inputs: none
Run:
```bash
journalctl --disk-usage
sudo journalctl --rotate --vacuum-size=200M
journalctl --disk-usage
snap list --all | awk '/disabled/{print $1, $3}'
sudo snap set system refresh.retain=2
snap list --all | awk '/disabled/{print $1, $3}' | while read n r; do sudo snap remove "$n" --revision="$r"; done
snap list --all | grep -c disabled || true
```
Expect: second disk-usage line well below the first (about 200-300M); "removed" per revision; final count 0.
Stop if: journalctl errors, or any line starting with error:.
Return: journal_gb, snap_removed (count), snap_disabled_gb (0 on success).

### T7 tier1-user-caches  |  tier: flash  |  needs: sudo
Goal: clear pip cache, thumbnails, Trash, ccache, /var/crash.
Inputs: {{TRASH_LIMIT}}
Run:
```bash
du -sh ~/.cache/pip ~/.cache/thumbnails ~/.local/share/Trash /var/crash 2>/dev/null || true
rm -rf ~/.cache/pip ~/.cache/thumbnails
rm -rf ~/.local/share/Trash/files ~/.local/share/Trash/info
ccache -C || true
sudo find /var/crash -mindepth 1 -delete || true
df -h /
```
Expect: du sizes, then df; "ccache: command not found" acceptable.
Stop if: du shows Trash above {{TRASH_LIMIT}} -> NEEDS_USER "Trash holds N, empty it?".
Return: trash_gb, free_gb.

### T8 tier2-docker-prune  |  tier: flash  |  needs: sudo
Goal: docker system prune -a. Never add --volumes (deletes data).
Inputs: none
Run:
```bash
sudo docker ps -a
sudo docker system df
sudo docker system prune -a -f
df -h /
```
Expect: "Total reclaimed space: N GB".
Stop if: docker errors.
Return: docker_gb (after), free_gb.

### T9 tier2-remove-ros-distro  |  tier: flash  |  needs: sudo
Goal: purge one non-Humble ROS distro.
Inputs: {{DISTRO}}
Run:
```bash
test -n "{{DISTRO}}" -a "{{DISTRO}}" != humble -a -d "/opt/ros/{{DISTRO}}" && echo DISTRO_OK || { echo DISTRO_REFUSED; exit 1; }
dpkg-query -Wf '${Package}\n' | grep '^ros-{{DISTRO}}-' > /tmp/ros_rm.txt || true
wc -l /tmp/ros_rm.txt
xargs -a /tmp/ros_rm.txt sudo apt-get remove --purge --dry-run
xargs -a /tmp/ros_rm.txt sudo apt-get remove --purge -y
sudo apt-get autoremove --purge --dry-run
sudo apt-get autoremove --purge -y
sudo rm -rf "/opt/ros/{{DISTRO}}"
df -h /
```
Expect: DISTRO_OK; only ros-{{DISTRO}}-* in the remove REMOVED list; /opt/ros/{{DISTRO}} gone.
Stop if: DISTRO_REFUSED printed, wc prints 0, the remove dry-run lists a package not starting with ros-{{DISTRO}}-, either dry-run lists any ros-humble-* (e.g. ros-humble-desktop, ros-humble-ros-base), or any line starts with E:.
Return: removed_count, free_gb.

### T10 tier2-remove-gazebo-classic  |  tier: flash  |  needs: sudo
Goal: purge Gazebo Classic 11 and its ROS bindings.
Inputs: none
Run:
```bash
dpkg-query -Wf '${Package}\n' | grep -E '^(gazebo|libgazebo|ros-humble-gazebo-)' > /tmp/gz_rm.txt || true
cat /tmp/gz_rm.txt
xargs -a /tmp/gz_rm.txt sudo apt-get remove --purge --dry-run
xargs -a /tmp/gz_rm.txt sudo apt-get remove --purge -y
sudo apt-get autoremove --purge --dry-run
sudo apt-get autoremove --purge -y
df -h /
```
Expect: only gazebo*/libgazebo*/ros-humble-gazebo-* in the remove REMOVED list.
Stop if: cat prints nothing, the remove dry-run lists a package not starting with gazebo, libgazebo or ros-humble-gazebo-, either dry-run lists any other ros-humble-* (e.g. ros-humble-desktop, ros-humble-ros-base), or any line starts with E:.
Return: removed_count, free_gb.

### T11 tier2-px4-distclean  |  tier: flash  |  needs: sudo
Goal: PX4 make distclean; tracked source stays, submodules deinitialised.
Inputs: none
Run:
```bash
du -sh ~/PX4-Autopilot/build
git -C ~/PX4-Autopilot status --short
git -C ~/PX4-Autopilot submodule foreach --quiet --recursive git status --short
git -C ~/PX4-Autopilot clean -n -X msg platforms posix-configs ROMFS src test Tools
make -C ~/PX4-Autopilot distclean
df -h /
```
Expect: both status lines print nothing; build/ gone.
Stop if: either git status prints any line.
Return: px4_build_gb (0), free_gb, note: "Plan 2 re-runs git submodule update --init --recursive".

### T12 tier2-remove-ardupilot  |  tier: flash  |  needs: sudo
Goal: remove ArduPilot, ardupilot_gazebo and MAVProxy.
Inputs: none
Run:
```bash
du -sh ~/ardupilot ~/ardupilot_gazebo 2>/dev/null || true
for d in ~/ardupilot ~/ardupilot_gazebo; do test -d "$d/.git" || continue; git -C "$d" status --short; git -C "$d" log --branches --not --remotes --oneline; git -C "$d" stash list; done
rm -rf ~/ardupilot ~/ardupilot_gazebo
python3 -m pip uninstall -y MAVProxy || true
df -h /
```
Expect: the for line prints nothing; dirs gone; MAVProxy uninstalled or "not installed".
Stop if: the for line prints any line (uncommitted, unpushed or stashed work).
Return: ardupilot_present (no), free_gb.

### T13 tier2-delete-workspace-build  |  tier: flash  |  needs: sudo
Goal: delete build/ install/ log/ of one colcon workspace; src/ stays.
Inputs: {{WS_PATH}}
Run:
```bash
test -n "{{WS_PATH}}" -a -d "{{WS_PATH}}/src" || { echo WS_PATH_REFUSED; exit 1; }
cat "{{WS_PATH}}/build/.built_by"
ls "{{WS_PATH}}"
du -sh "{{WS_PATH}}/build" "{{WS_PATH}}/install" "{{WS_PATH}}/log" 2>/dev/null
rm -rf "{{WS_PATH}}/build" "{{WS_PATH}}/install" "{{WS_PATH}}/log"
ls "{{WS_PATH}}"
df -h /
```
Expect: cat prints colcon; second ls shows src, no build/install/log.
Stop if: {{WS_PATH}} is empty, is ~, or does not contain a src/ directory (WS_PATH_REFUSED), cat does not print colcon, or first ls shows no src.
Return: ws_path, free_gb.

### T14 write-report  |  tier: flash  |  needs: none
Goal: write the storage report file.
Inputs: {{REPORT}} (filled report template)
Run:
~~~bash
mkdir -p ~/drdo_setup/reports
cat > ~/drdo_setup/reports/01_storage_report.md << 'EOF'
{{REPORT}}
EOF
grep -c '' ~/drdo_setup/reports/01_storage_report.md
~~~
Expect: line count above 20.
Stop if: cat errors.
Return: report_path.

## Report template
# 01 Storage report

| metric | before | after |
|---|---|---|
| free_gb | {{BEFORE_GB}} | {{AFTER_GB}} |

```machine-facts
free_gb:
lvm:
lv_path:
vg_free_gb:
ram_gb:
gpu:
ros_distros:
gz_versions:
ros_gz_packages:
px4_path:
px4_tag:
px4_build_gb:
ardupilot_present:
docker_present:
docker_gb:
snap_disabled_gb:
journal_gb:
workspaces:
big_files:
deleted:
declined:
notes:
```

## Facts
- https://snapcraft.io/docs/managing-updates
- https://docs.docker.com/reference/cli/docker/system/prune/
- https://ubuntu.com/server/docs/about-lvm
