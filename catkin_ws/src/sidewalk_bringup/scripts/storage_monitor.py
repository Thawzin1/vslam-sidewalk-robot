#!/usr/bin/env python3
"""storage_monitor.py - the storage page: every disk this project saves to, live.

Viewed in a browser on the PC or a phone (never on the Jetson's own screen - it
runs with no monitor, ENGINEERING_NOTES.md section 0.3 rule 3):

    the storage page (STORAGE_PAGE_URL)

VERSION 2 (2026-09-24). Version 1 showed the Jetson's internal disk and the
microSD card. Version 2 shows five, in plain words, each with total / used /
free, a bar, how fast it is filling right now, how long until it is full at
that rate, and one status sentence:

  on the Jetson   1 the internal disk "/"          (red under 10,547 MB: a fusion drive refuses)
                  2 the 32 GB USB stick CAM_REC     (for backups; found by its ID, never by /dev/sda)
                  3 the RETIRED microSD SIDEWALK128 (discards every write; red if it is ever mounted)
  on the robot    4 the robot's own disk "/"        (a colleague's disk; amber under 3 GB)
                  5 the robot's card "USB Drive"    (the LiDAR recordings; red under 8 GB)

The robot's two come from robot_storage_agent.py on the robot (port 8113,
read-only), asked every 5 s with a 1 s time limit from a background thread, so
a switched-off robot never slows this page. When it does not answer, the page
says "robot offline - last seen HH:MM" and keeps the last figures, greyed.

VERSION 3 (2026-09-25), after an independent check of version 2 found these:
  - A silent robot never reads ALL CLEAR or shows the green pip: the top line says
    "Jetson disks have room; robot not reporting since HH:MM", and a robot disk that
    was in trouble when last seen KEEPS its alarm level (drawn greyed, dashed frame).
  - Every robot disk entry is checked before the answer is accepted; a damaged answer
    is refused and can never blank the page. The Jetson's own disks always show.
  - The robot counts as offline only after 3 missed answers in a row (WiFi drops
    single answers every few minutes), and its filling rate survives a single miss.
  - A USB stick pulled out while still attached is amber with the fix; the mount
    command shown is the kernel-driver one (-i); a stick attached through the FUSE
    add-on driver (fstype fuseblk) is amber - the driver this project lost data through.
  - The browser side gives each request 4 s, never overlaps requests, and checks the
    age of the last good answer on every tick, so a hung Jetson shows NO DATA.
  - Plain words on the page for everything a user sees (no ROS/exFAT/kernel/driver/df).

VERSION 3.1 (2026-09-25), after a second check:
  - Fits one phone screen in every robot state: while the robot is not reporting, its
    greyed disks drop their description line and keep one "When last seen: ..." line
    only if they were in trouble; the last-seen time is said once, in the robot header;
    the "reporter not running" texts are shorter; on a phone each box has a little
    less space around it.
  - "offline" is said only of a silent robot. A robot that answers but has not measured
    its card for 30 s says "not measured for N s"; one whose reporter is not running
    says "not updating".
  - Rare faults are told in plain words: disks by their titles, figures by ordinary
    names, and program error names go to the log only.
  - An answer too deeply nested to read is "not understood", not a missed answer.
  - The Jetson's three disks are measured in three threads, each with its own age, so a
    hung USB stick cannot freeze the internal disk's figures; a disk whose check has
    not finished for 10 s says it may be stuck, by name.

HOW IT IS STARTED (found 2026-09-24, unchanged by version 2): the Jetson's
crontab runs `@reboot ~/s3/s3_boot.sh`, whose start_page starts
`python3 ~/storage_monitor.py` with NO ARGUMENTS if no process has
that exact file name. So the defaults below are the production settings.

The look is the project's approved one (station_panel.py): colourless while all
is well, one small green pip, amber for caution, and a slowly breathing red
frame round a disk that is in trouble. One screen, readable on a phone.

Standard library only (Python 3.8 on the Jetson). Every figure comes from
statvfs (the operating system's own "how big, how full" call - what df prints)
and /proc/mounts (what is attached where). NOTHING IS EVER WRITTEN TO ANY DISK
by this page, apart from its own log lines on stderr.

    python3 storage_monitor.py                         # production: port 8092
    python3 storage_monitor.py --port 8192 --robot 127.0.0.1:8193   # a test copy
    python3 storage_monitor.py --once                  # print one reading and exit

Robot address: --robot (comma list, tried in order, the last one that answered
first), default <robot-wifi-address>. It comes from DHCP and can change, so the page
also reads ~/storage_robot_addrs.txt on every cycle if that file exists - to
point it at a new address without restarting the page:
    echo <robot-wifi-address> > ~/storage_robot_addrs.txt
"""
import argparse
import errno
import glob
import http.client
import json
import math
import os
import re
import socket
import stat
import sys
import threading
import time
import urllib.error
import urllib.request
from collections import deque
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

VERSION = "3.1"
MB = 1024 ** 2            # df's units: 1 MB = 1,048,576 bytes, 1 GB = 1,024 MB,
GB = 1024 ** 3            # so this page and start_drive.sh agree to the megabyte

# ---------------------------------------------------------------- the limits ----
# Each one with where it comes from. Red lines are compared the way df rounds
# (UP to the next MB), because that is how start_drive.sh reads them.
FUSION_FLOOR = 10547 * MB   # start_drive.sh:92 - FUSION=1 refuses under `df -BM` 10547
DRIVE_FLOOR = 10240 * MB    # 10 GB, the drive-3 floor given by the lead. (start_drive.sh:78
                            # checks `df -BG` >= 10, which df's rounding-up passes above 9 GB)
JETSON_CAUTION = 12 * GB    # amber: under 12 GB there is little slack above the red lines
TRANSFER_FLOOR = int(1.2 * GB)   # ENGINEERING_NOTES.md rule 15: a copy need only leave 1.2 GB free
ROBOT_ROOT_CAUTION = 3 * GB      # the robot's own disk, ~95 % full, is a colleague's
ROBOT_ROOT_RED = 1 * GB
ROBOT_CARD_RED = 8 * GB          # one drive's LiDAR recording is ~6.3 GB; 8 GB leaves margin
DRIVE_SIZE = int(6.5 * GB)       # "about N more drives fit" counts drives of this size
IDLE = 0.05 * MB                 # under 0.05 MB/s either way counts as idle

USB_UUID, USB_LABEL = "EFFD-FE82", "CAM_REC"
USB_MOUNTPOINT = "/media/sidewalk/CAM_REC"
# -i makes mount use the kernel's own exFAT driver instead of /sbin/mount.exfat, which on
# this Jetson is the FUSE add-on driver (mount.exfat-fuse) this project lost data through
USB_MOUNT_CMD = "sudo mount -i -t exfat -o uid=1000,gid=1000 %s " + USB_MOUNTPOINT
STICK_TYPES = ("exfat", "fuseblk", "vfat", "ntfs", "ntfs3")
# The retired card, by its CID - the serial record burned into every SD card at the
# factory, so it still identifies the card after any format, and a replacement card
# given the same name and disk ID (to keep the fstab line working) is not mistaken
# serial 0x7bb4cebe, made 12/2024. The disk ID is the fallback (e.g. in a USB reader).
RETIRED_CID = "035344534e313238857bb4cebe018c00"
RETIRED_UUID = "21ea9290-47dc-4bf3-8b79-be2ebecdd8bc"
RETIRED_LABEL = "SIDEWALK128"
ROBOT_CARD_PATH = "/media/administrator/USB Drive"
ROBOT_PORT = 8113
ROBOT_HOST = "cpr-a200-0985"     # the robot's own name, as its reporter gives it
ROBOT_START_CMD = 'bash "/media/administrator/USB Drive/slam_series2/tools/robot_side.sh" storage -'
ROBOT_MISSES = 3                 # offline only after this many missed answers IN A ROW
ROBOT_STUCK_S = 30               # a robot disk not measured for this long may be hung
ROBOT_KINDS = ("robot_root", "robot_card")

os.environ["TZ"] = "America/Toronto"      # every time shown or logged is Hamilton time
time.tzset()


def hhmm(t):
    return time.strftime("%H:%M", time.localtime(t))


def log(msg):
    sys.stderr.write("%s storage_monitor: %s\n" % (time.strftime("%m-%d %H:%M:%S"), msg))
    sys.stderr.flush()


def mb_df(b):
    """Whole MB rounded UP, exactly as `df -BM` prints it."""
    return -(-int(b) // MB)


def fmt_mb(b):
    return "{:,} MB".format(int(b // MB))


def fmt_gb(b):
    return "%.1f GB" % (b / GB)


def fmt_size(b):
    """GB with one decimal from 1 GB up, whole MB below it."""
    return fmt_gb(b) if b >= GB else "{:,} MB".format(int(b // MB))


_PLAIN_ERRNO = {errno.EIO: "it stopped answering (a read/write error)",
                errno.ENOTCONN: "the program reading it has stopped",
                errno.ENOENT: "the folder is gone",
                errno.EACCES: "no permission to look at it",
                errno.ESTALE: "the attachment has gone stale",
                errno.ENODEV: "the device is gone"}


def plain_os_error(exc):
    """An OSError in ordinary words; the raw text goes to the log only (the caller logs it)."""
    return _PLAIN_ERRNO.get(getattr(exc, "errno", None), "it could not be read (%s)"
                            % (getattr(exc, "strerror", None) or "an unexpected error").lower())


# ----------------------------------------------------------- mounts & devices ----
_OCTAL = re.compile(r"\\([0-7]{3})")


def decode_mount_field(s):
    """/proc/mounts writes a space as \\040 (the robot's card is 'USB Drive')."""
    return _OCTAL.sub(lambda m: chr(int(m.group(1), 8)), s)


def read_mounts(path):
    out = []
    try:
        with open(path) as f:
            for line in f:
                p = line.split()
                if len(p) >= 4:
                    out.append({"device": decode_mount_field(p[0]),
                                "mountpoint": decode_mount_field(p[1]),
                                "fstype": p[2], "options": p[3].split(",")})
    except OSError as exc:
        log("cannot read %s: %s" % (path, exc))
    return out


def mount_at(mountpoint, mounts):
    """The mount entry for exactly this folder, or None. The last entry wins."""
    real = os.path.realpath(mountpoint)
    hit = None
    for m in mounts:
        if m["mountpoint"] == real:
            hit = m
    return hit


def same_block_device(a, b):
    if os.path.realpath(a) == os.path.realpath(b):
        return True
    try:
        sa, sb = os.stat(a), os.stat(b)
    except OSError:
        return False
    return stat.S_ISBLK(sa.st_mode) and stat.S_ISBLK(sb.st_mode) and sa.st_rdev == sb.st_rdev


def mounts_of(devices, mounts):
    """Every mount whose source is one of these block devices (/dev/...)."""
    return [m for m in mounts if m["device"].startswith("/dev/")
            and any(same_block_device(m["device"], d) for d in devices)]


def figures(mountpoint):
    """Size figures, or {'error': ...} (e.g. a stick pulled out while still mounted)."""
    try:
        st = os.statvfs(mountpoint)
    except OSError as exc:
        log("statvfs %s: %r" % (mountpoint, exc))
        return {"error": plain_os_error(exc)}
    return {"total": st.f_blocks * st.f_frsize,
            "used": (st.f_blocks - st.f_bfree) * st.f_frsize,
            "avail": st.f_bavail * st.f_frsize}   # what an ordinary user can still write (df "Avail")


def dev_gone(m):
    """A mount whose /dev node no longer exists: the disk was pulled out while attached."""
    return m["device"].startswith("/dev/") and not os.path.exists(m["device"])


def by_id(kind, value):
    p = "/dev/disk/by-%s/%s" % (kind, value)
    return os.path.realpath(p) if os.path.exists(p) else None


def sys_size(devpath):
    try:
        with open("/sys/class/block/%s/size" % os.path.basename(devpath)) as f:
            return int(f.read().strip()) * 512
    except (OSError, ValueError):
        return None


# ------------------------------------------------------------------ the rate ----
class Rate:
    """How fast the used space is changing, over the last `window` seconds of
    samples, on THIS machine's monotonic clock (never the robot's: its clock
    battery is dead and a jump would invent a huge rate)."""

    def __init__(self, window, min_span):
        self.window, self.min_span = window, min_span
        self.h = deque()
        self.ident = None

    def add(self, t, used, ident):
        if ident != self.ident:           # a different disk is now behind this name
            self.h.clear()
            self.ident = ident
        self.h.append((t, used))
        while self.h and t - self.h[0][0] > self.window:
            self.h.popleft()

    def clear(self):
        self.h.clear()
        self.ident = None

    def value(self):
        if len(self.h) < 2:
            return None
        (t0, u0), (t1, u1) = self.h[0], self.h[-1]
        return (u1 - u0) / (t1 - t0) if t1 - t0 >= self.min_span else None


def add_rate(d, rate):
    """Attach rate (bytes/s), time to full, and time to the red line (s)."""
    d["rate"] = rate
    d["full_in"] = d["floor_in"] = None
    if rate is not None and rate > IDLE and d.get("avail") is not None:
        d["full_in"] = d["avail"] / rate
        if d.get("floor") and d["avail"] > d["floor"]:
            d["floor_in"] = (d["avail"] - d["floor"]) / rate
    return d


# ----------------------------------------------------------- the Jetson disks ----
def jetson_root():
    d = {"key": "jetson_root", "machine": "jetson", "title": "Jetson internal disk",
         "what": "the Jetson's disk: system, robot software, maps, logs",
         "state": "mounted", "mountpoint": "/", "level": 0, "notes": [], "commands": [],
         "floor": FUSION_FLOOR,
         "floor_label": "10,547 MB: the least a fusion drive needs to start",
         "floor_short": "at the 10,547 MB line in"}
    d.update(figures("/"))
    if "error" in d:
        d.update(level=2, headline="no reading", status="Cannot read the disk: " + d["error"])
        return d
    a = d["avail"]
    free_mb = mb_df(a)
    d["headline"] = "{:,} MB free".format(free_mb)       # rounded up, as df prints it
    spare = free_mb - FUSION_FLOOR // MB
    if free_mb < DRIVE_FLOOR // MB:
        d.update(level=2, short="under 10 GB, not enough for a drive",
                 status="Under 10 GB (10,240 MB) free: not enough for a drive. "
                        "Free up space before the next one.")
    elif spare < 0:
        d.update(level=2, short="{:,} MB short for a fusion drive".format(-spare),
                 status="{:,} MB short of the 10,547 MB a fusion drive needs: "
                        "start_drive.sh with FUSION=1 will refuse.".format(-spare))
    elif a < JETSON_CAUTION:
        d.update(level=1, short="only {:,} MB above the fusion-drive line".format(spare),
                 status="Enough for a fusion drive, with only {:,} MB to spare "
                        "(amber under 12 GB).".format(spare))
    else:
        d["status"] = "Room for a fusion drive, with {:,} MB to spare.".format(spare)
    # Two answers to "how big a copy can I put here?": the one that still lets the next
    # drive start (the 10,547 MB line), and the bare 1.2 GB copy floor of ENGINEERING_NOTES.md rule 15.
    # Following the second alone would stop the next drive from starting.
    room_drive, room_any = a - FUSION_FLOOR, a - TRANSFER_FLOOR
    d["notes"].append("Largest copy that still leaves room for a drive: %s (%s if only "
                      "the 1.2 GB floor is kept - then no drive can start)."
                      % (fmt_size(room_drive) if room_drive > 0 else "none",
                         fmt_size(room_any) if room_any > 0 else "none"))
    return d


def usb_stick(cfg, mounts):
    d = {"key": "usb", "machine": "jetson", "title": "USB stick (CAM_REC)",
         "what": "the 32 GB memory stick in the Jetson, for backups",
         "level": 0, "notes": [], "commands": []}
    dev = by_id("uuid", cfg.usb_uuid) or by_id("label", cfg.usb_label)
    held = mount_at(USB_MOUNTPOINT, mounts)       # whatever is attached at the stick's folder
    if dev is None:
        # Pulled out while still attached: its ID link has gone, but the Jetson still lists
        # the old attachment. Version 2 said "not plugged in" here, with no caution.
        dead = held if (held and dev_gone(held)) else next(
            (m for m in mounts if dev_gone(m) and m["fstype"] in STICK_TYPES
             and m["mountpoint"].startswith("/media/")), None)
        if dead:
            d.update(state="pulled_out", level=1, headline="pulled out",
                     short="pulled out while still attached",
                     status="Pulled out while still attached: the Jetson still lists it as the "
                            "folder %s, so anything saved there fails. Before plugging it back "
                            "in, detach it - on the Jetson:" % dead["mountpoint"])
            d["commands"] = ["sudo umount %s" % dead["mountpoint"]]
            return d
        if held:
            d.update(state="absent", level=1, headline="not plugged in",
                     short="another disk is at its folder",
                     status="Not plugged in, but another disk (%s) is attached at its folder %s: "
                            "check what that is before saving backups there."
                            % (held["device"], USB_MOUNTPOINT))
            return d
        d.update(state="absent", headline="not plugged in",
                 status="Not plugged in. When it is, this shows whether it is ready.")
        return d
    d["device"] = dev
    size = sys_size(dev)
    ms = mounts_of([dev], mounts)
    if not ms:
        if held and dev_gone(held):               # re-plugged after a pull-out: old entry left
            d.update(state="replugged", level=1, headline="old attachment left",
                     short="plugged back in, old attachment still listed",
                     status="Plugged back in as %s, but its old attachment is still listed at %s "
                            "(it was pulled out while attached). Detach the old one, then "
                            "attach it again - on the Jetson:" % (dev, USB_MOUNTPOINT))
            d["commands"] = ["sudo umount %s" % USB_MOUNTPOINT, USB_MOUNT_CMD % dev]
            return d
        if held:
            d.update(state="folder_taken", level=1, headline="folder taken",
                     short="plugged in, but its folder is taken",
                     status="Plugged in as %s, but another disk (%s) is attached at its folder "
                            "%s. Detach that one first if it should not be there."
                            % (dev, held["device"], USB_MOUNTPOINT))
            return d
        d.update(state="unmounted", level=1, headline="not attached",
                 short="plugged in, not attached as a folder",
                 status="Plugged in (%sfound as %s by its ID) but not attached as a folder the "
                        "Jetson can save into. To attach it, on the Jetson:"
                        % ((fmt_gb(size) + ", ") if size else "", dev))
        d["commands"] = ["sudo mkdir -p %s" % USB_MOUNTPOINT, USB_MOUNT_CMD % dev]
        d["notes"].append("The -i makes the Jetson use its own built-in reader for this "
                          "stick's format, not the add-on one this project lost files through "
                          "in August.")
        return d
    m = next((x for x in ms if x["mountpoint"] == USB_MOUNTPOINT), ms[0])
    d.update(state="mounted", mountpoint=m["mountpoint"], fstype=m["fstype"],
             read_only="ro" in m["options"])
    d.update(figures(m["mountpoint"]))
    if "error" in d:
        d.update(level=2, headline="not answering", short="attached but not answering",
                 status="Listed as attached at %s but not answering (%s) - pulled out without "
                        "detaching? Detach it before plugging it back in - on the Jetson:"
                        % (m["mountpoint"], d["error"]))
        d["commands"] = ["sudo umount %s" % m["mountpoint"]]
        return d
    d["headline"] = fmt_gb(d["avail"]) + " free"
    where = "Attached as the folder %s" % m["mountpoint"]
    if m["fstype"] == "fuseblk":
        # The FUSE add-on driver: behind the August losses. Amber, with the way out.
        d.update(level=1, short="attached through the risky add-on reader",
                 status=where + ", but through the add-on reader program (FUSE) this project "
                        "lost files through in August. Detach it and attach it again with the "
                        "Jetson's built-in reader - on the Jetson:")
        d["commands"] = ["sudo umount %s" % m["mountpoint"], USB_MOUNT_CMD % dev]
    elif d["read_only"]:
        d.update(level=1, short="attached read-only",
                 status=where + ", READ-ONLY (it can be read but not saved to): backups to it "
                        "will fail.")
    elif d["avail"] < TRANSFER_FLOOR:
        d.update(level=1, short="nearly full",
                 status=where + ": nearly full, under the 1.2 GB a copy should leave free.")
    else:
        d["status"] = "Ready for backups, at %s." % m["mountpoint"]
    if m["fstype"] == "exfat":
        d["notes"].append("Read by the Jetson's built-in reader (the safe one).")
    return d


def retired_card(cfg, mounts):
    d = {"key": "retired_card", "machine": "jetson", "title": "Retired microSD card (SIDEWALK128)",
         "what": "the Jetson's old 128 GB memory card - it silently loses everything saved to it",
         "level": 0, "notes": [], "commands": []}
    root = os.stat("/").st_dev
    root_mm = "%d:%d" % (os.major(root), os.minor(root))
    found = None                                  # (kind, devices, cid)
    for blk in sorted(glob.glob(cfg.card_glob)):
        name = os.path.basename(blk)
        if not re.match(r"^mmcblk\d+$", name):    # skip mmcblk0boot0, mmcblk0rpmb, ...
            continue
        parts = [os.path.basename(p) for p in glob.glob(os.path.join(blk, name + "p*"))]
        mm = set()                                # "major:minor" of the disk and its partitions
        for devfile in [os.path.join(blk, "dev")] + [os.path.join(blk, n, "dev") for n in parts]:
            try:
                with open(devfile) as f:
                    mm.add(f.read().strip())
            except OSError:
                pass
        if root_mm in mm:                         # the Jetson's own boot disk
            continue
        try:
            with open(os.path.join(blk, "device", "cid")) as f:
                cid = f.read().strip().lower()
        except OSError:
            cid = ""
        devs = ["/dev/" + n for n in [name] + parts]
        found = ("retired" if cid == cfg.retired_cid.lower() else "other", devs, cid)
        if found[0] == "retired":
            break
    if found is None or found[0] == "other":      # not in the slot: in a USB reader, by disk ID?
        dev = by_id("uuid", cfg.retired_uuid)
        if dev and (found is None or dev not in found[1]):
            found = ("retired_by_id", [dev], "")
    if found is None:
        d.update(state="retired_absent", compact=True,
                 status="Retired microSD card (SIDEWALK128): not in the Jetson - good, "
                        "it loses everything saved to it.")
        return d
    kind, devs, cid = found
    ms = mounts_of(devs, mounts)
    if kind == "other":
        d.update(state="other_card", title="microSD card (not the retired one)",
                 what="a different card is in the Jetson's card slot (factory serial %s)"
                      % (cid[18:26] if len(cid) >= 26 else "unknown"),
                 headline="new card",
                 status="Test it with card_pattern_test.py (it checks the card gives back exactly "
                        "what was saved) before trusting it with anything.")
        if ms:
            d.update(mountpoint=ms[0]["mountpoint"], fstype=ms[0]["fstype"])
            d.update(figures(ms[0]["mountpoint"]))
            if "avail" in d:
                d["headline"] = fmt_gb(d["avail"]) + " free"
        return d
    how = "" if kind == "retired" else " (recognised by its disk ID)"
    if ms:
        mp = ms[0]["mountpoint"]
        d.update(state="retired_mounted", level=2, headline="ATTACHED", mountpoint=mp, what="",
                 short="the RETIRED card is attached - anything saved there is lost",
                 status="The RETIRED card%s is attached at %s: anything saved there is silently "
                        "lost. Detach it and take it out - on the Jetson:" % (how, mp))
        d["commands"] = ["sudo umount %s" % mp]
        return d
    d.update(state="retired_present", grey=True, headline="RETIRED", what="",
             status="RETIRED - loses what is saved to it%s. Take it out: the start-up disk "
                    "list (/etc/fstab) attaches it at restart." % how)
    return d


# ----------------------------------------------------------------- the robot ----
_HOST_RE = re.compile(r"^[A-Za-z0-9.-]+$")
_IPV6_RE = re.compile(r"^[0-9A-Fa-f:.]+$")


def parse_addr(a):
    """'host', 'host:port' or '[ipv6]:port' -> (host, port). ValueError if it is not one."""
    a = a.strip()
    if a.startswith("["):                         # [ipv6]:port
        host, sep, rest = a[1:].partition("]")
        if not sep or (rest and not rest.startswith(":")):
            raise ValueError(a)
        port = int(rest[1:]) if rest else ROBOT_PORT
    elif a.count(":") == 1:
        host, port = a.split(":")
        port = int(port)
    else:
        host, port = a, ROBOT_PORT
    ok = _IPV6_RE.match(host) if ":" in host else _HOST_RE.match(host)
    if not host or not ok or not 0 < port < 65536:
        raise ValueError(a)
    return host, port


def fmt_addr(host, port):
    h = "[%s]" % host if ":" in host else host
    return h if port == ROBOT_PORT else "%s:%d" % (h, port)


def _no_constants(name):
    raise ValueError("%s is not a number" % name)      # json would otherwise accept NaN


def _whole(v):
    return isinstance(v, int) and not isinstance(v, bool) and 0 <= v < 2 ** 62


# A fault in the robot's answer is told with the page's own names for the disks and
# ordinary words for the figures - never the report's key and field names (version 3
# showed "disk robot_card: its avail is not a whole number of bytes").
_DISK_WORDS = {"robot_root": "the robot's own disk", "robot_card": "the robot's card"}
_FIELD_WORDS = {"total": "size", "used": "used space", "avail": "free space",
                "mounted": "whether it is attached", "read_only": "whether it is read-only",
                "exists": "whether it exists", "measuring": "whether it is being measured",
                "path": "folder name", "device": "device name", "fstype": "format name",
                "error": "error message"}


def check_payload(data):
    """None if the robot's answer is a disk report this page can show; otherwise, in plain
    words, what is wrong with it. EVERY entry is checked before the answer is accepted, so a
    damaged or different-version answer can never reach the page (version 2 stored it first,
    and one bad entry then blanked the whole page, the Jetson's own disks included)."""
    if not isinstance(data, dict):
        return "the answer is not a disk report"
    disks = data.get("disks")
    if not isinstance(disks, list):
        return "it has no list of disks"
    if len(disks) > 16:
        return "it lists %d disks" % len(disks)
    if "host" in data and not isinstance(data["host"], str):
        return "its computer name is not text"
    seen = set()
    for i, x in enumerate(disks):
        if not isinstance(x, dict):
            return "disk %d is not a disk entry" % (i + 1)
        k = x.get("key")
        if not isinstance(k, str) or not k:
            return "disk %d has no name" % (i + 1)
        name = _DISK_WORDS.get(k, "disk %d" % (i + 1))
        if k in seen:
            return "%s is listed twice" % name
        seen.add(k)
        for f in ("total", "used", "avail"):
            if x.get(f) is not None and not _whole(x[f]):
                return "%s: its %s is not a whole number of bytes" % (name, _FIELD_WORDS[f])
        if _whole(x.get("total")):
            for f in ("used", "avail"):
                if _whole(x.get(f)) and x[f] > x["total"]:
                    return "%s: its %s is larger than the whole disk" % (name, _FIELD_WORDS[f])
        for f in ("mounted", "read_only", "exists", "measuring"):
            if f in x and not isinstance(x[f], bool):
                return "%s: %s is not given as yes or no" % (name, _FIELD_WORDS[f])
        for f in ("path", "device", "fstype", "error"):
            if x.get(f) is not None and not isinstance(x[f], str):
                return "%s: its %s is not text" % (name, _FIELD_WORDS[f])
        if "age_s" in x:
            v = x["age_s"]
            if isinstance(v, bool) or not isinstance(v, (int, float)) or not math.isfinite(v) \
                    or v < 0:
                return "%s: the age of its figures is not a number of seconds" % name
    return None


# How to say each way of not getting an answer. The first ones are "misses" (the
# robot may simply be off, or WiFi dropped one answer); "bad" and "noaddr" are not.
FAIL_ORDER = ("bad", "refused", "lookup", "unreachable", "cut", "timeout", "error")
# The ways of not answering that mean the robot itself is silent - the only ones called
# "offline". The others say what they are in the robot header instead.
SILENT_KINDS = ("timeout", "unreachable", "lookup", "cut")
HEADER_WORD = {"refused": "reporter not running", "bad": "report not understood",
               "noaddr": "no address to ask", "error": "could not ask"}


class Robot:
    def __init__(self, cfg):
        self.cfg = cfg
        self.lock = threading.Lock()
        self.payload = None          # the last GOOD (checked) answer
        self.ok_wall = None          # when it was received (this machine's clock)
        self.addr = None             # which address answered
        self.kind = "never"          # never / ok / timeout / refused / unreachable / lookup /
        self.reason = ""             #   cut / bad / noaddr / error
        self.tried = []
        self.try_wall = None
        self.try_seconds = None
        self.misses = 0              # failed asks in a row
        self.invalid = []            # address entries that are not addresses, named on the page
        self.hosts = {}              # address -> computer name that answered from it
        self.last_miss = None        # (kind, reason) of the latest miss while still online
        self.rates = {k: Rate(16.0, 7.0) for k in ROBOT_KINDS}
        self.logged = set()          # error names already logged (each once; the page
                                     # never shows them)
        self.opener = urllib.request.build_opener(urllib.request.ProxyHandler({}))
        self.started = time.time()

    def log_once(self, name, msg):
        if name not in self.logged and len(self.logged) < 50:
            self.logged.add(name)
            log(msg + " (logged once per kind)")

    def addresses(self):
        raw = []
        try:
            with open(self.cfg.robot_file) as f:
                for line in f:
                    raw += [x for x in re.split(r"[,\s]+", line.split("#")[0]) if x]
        except OSError:
            pass
        raw += [x.strip() for x in self.cfg.robot.split(",") if x.strip()]
        seen, ordered, invalid = set(), [], []
        for x in raw:
            try:
                ha = fmt_addr(*parse_addr(x))
            except ValueError:
                if x not in invalid:
                    invalid.append(x[:60])
                continue
            if ha not in seen:
                seen.add(ha)
                ordered.append(ha)
        if self.addr in seen:                     # the one that answered last, first
            ordered.remove(self.addr)
            ordered.insert(0, self.addr)
        return ordered, invalid

    def classify(self, exc, host):
        r = getattr(exc, "reason", exc)
        if isinstance(exc, urllib.error.HTTPError):
            return "bad", "it answered with an error page (code %d) instead of a disk report" % exc.code
        if isinstance(exc, socket.timeout) or isinstance(r, socket.timeout) \
                or "timed out" in str(r):
            return "timeout", "no answer within %g s" % self.cfg.robot_timeout
        if isinstance(r, ConnectionRefusedError):
            return "refused", "something answered, but no storage reporter is running there"
        if isinstance(r, socket.gaierror):
            return "lookup", 'the name "%s" could not be found on the network' % host
        if isinstance(r, OSError) and r.errno in (errno.EHOSTUNREACH, errno.ENETUNREACH,
                                                  errno.EHOSTDOWN):
            return "unreachable", "no route to it on the network"
        if isinstance(r, (ConnectionResetError, BrokenPipeError, http.client.IncompleteRead)):
            return "cut", "the connection was cut off before a full answer arrived"
        if isinstance(exc, http.client.HTTPException):
            return "bad", "something answered, but not in the web format a disk report uses"
        # Anything else: its program name goes to the log (once per kind), never to the page.
        self.log_once(type(r).__name__, "asking the robot at %s failed unexpectedly: %r"
                      % (host, r))
        return "error", "it could not be asked (an unexpected error; see the page's log)"

    def poll(self):
        t0 = time.monotonic()
        addrs, invalid = self.addresses()
        tried = []
        for a in addrs:
            host, port = parse_addr(a)
            url = "http://%s:%d/storage.json" % ("[%s]" % host if ":" in host else host, port)
            try:
                with self.opener.open(url, timeout=self.cfg.robot_timeout) as r:
                    raw = r.read(65537)
                if len(raw) > 65536:
                    raise ValueError("far too long")
                data = json.loads(raw.decode("utf-8"), parse_constant=_no_constants)
            except (ValueError, RecursionError) as exc:  # not JSON / not text / NaN /
                # too long / nested too deeply to read (RecursionError is not a ValueError,
                # so version 3 counted a 30,000-deep answer as a missed answer)
                self.log_once(type(exc).__name__, "the robot's answer at %s could not be "
                              "read: %s: %s" % (a, type(exc).__name__, str(exc)[:120]))
                tried.append((a, "bad", "something answered, but not with a disk report"))
                continue
            except Exception as exc:              # network, HTTP, protocol: never escapes
                tried.append((a,) + self.classify(exc, host))
                continue
            problem = check_payload(data)
            if problem:
                tried.append((a, "bad", "it answered, but its disk report is damaged (%s)"
                              % problem))
                continue
            now_m, now_w = time.monotonic(), time.time()
            with self.lock:
                if self.kind != "ok":
                    log("robot answering at %s (host %s)" % (a, data.get("host")))
                self.payload, self.ok_wall, self.addr = data, now_w, a
                self.kind, self.reason, self.misses = "ok", "", 0
                self.tried, self.invalid = tried + [(a, "ok", "")], invalid
                self.try_wall, self.try_seconds = now_w, now_m - t0
                if isinstance(data.get("host"), str):
                    self.hosts[a] = data["host"][:64]
                for dk in data["disks"]:
                    rt = self.rates.get(dk.get("key"))
                    if rt is None:
                        continue
                    age = dk.get("age_s") or 0.0     # version 2 reporters say how old
                    if dk.get("mounted") and dk.get("used") is not None \
                            and not dk.get("measuring") and age <= ROBOT_STUCK_S:
                        rt.add(now_m - age, dk["used"], (dk.get("device"), dk.get("total")))
                    else:
                        rt.clear()
            return
        kinds = [k for _, k, _ in tried]
        best = next((k for k in FAIL_ORDER if k in kinds), "noaddr")
        reason = next((t for _, k, t in tried if k == best),
                      "no valid address to ask" if invalid else "no address to ask")
        self._failed(best, reason, tried, invalid, t0)

    def _failed(self, best, reason, tried, invalid, t0):
        with self.lock:
            self.misses += 1
            self.tried, self.invalid = tried, invalid
            self.try_wall, self.try_seconds = time.time(), time.monotonic() - t0
            # One or two missed answers while it was answering: WiFi drops single answers
            # keep the rate history; only a damaged answer ends it at once.
            if self.kind == "ok" and self.misses < ROBOT_MISSES and best not in ("bad", "noaddr"):
                if self.misses == 1:
                    log("robot missed one answer (%s); still counted as answering"
                        % "; ".join("%s: %s" % (a, k) for a, k, _ in tried))
                self.last_miss = (best, reason)
                return
            if self.kind != best:
                log("robot not answering (%s)" % ("; ".join("%s: %s" % (a, k) for a, k, _ in tried)
                                                  or best))
            self.kind, self.reason = best, reason
            for rt in self.rates.values():
                rt.clear()

    def run(self):
        while True:
            try:
                self.poll()
            except Exception as exc:          # never let the robot thread die silently
                log("robot poll failed: %r" % exc)
                try:
                    self._failed("error", "the page's own robot check hit an unexpected error "
                                 "(see the page's log)", [], [], time.monotonic())
                except Exception:
                    pass
            time.sleep(self.cfg.robot_period)

    def silent_text(self, ok_wall, kind):
        if kind == "never":
            return "still asking the robot for the first time"
        if ok_wall:
            return "robot not reporting since %s" % hhmm(ok_wall)
        return "robot not seen since this page started at %s" % hhmm(self.started)

    def block(self):
        """The robot's two disks and its header, as the page shows them."""
        with self.lock:
            payload, ok_wall, kind = self.payload, self.ok_wall, self.kind
            addr, reason, tried = self.addr, self.reason, list(self.tried)
            try_wall, try_s, misses = self.try_wall, self.try_seconds, self.misses
            invalid, hosts = list(self.invalid), dict(self.hosts)
            rates = {k: r.value() for k, r in self.rates.items()}
        online = kind == "ok"
        tried_txt = ", ".join(a for a, _, _ in tried) or self.cfg.robot
        rb = {"online": online, "kind": kind, "reason": reason, "answered": addr if online else None,
              "last_seen": ok_wall, "tried": [{"addr": a, "kind": k, "text": t} for a, k, t in tried],
              "last_try": try_wall, "try_seconds": try_s, "level": 0, "misses": misses,
              "host": (payload or {}).get("host"), "commands": [], "short": "", "status": "",
              "silent": "" if online else self.silent_text(ok_wall, kind)}
        if online:
            rb["header"] = "answered at %s · %s" % (addr, hhmm(ok_wall))
            if misses:
                rb["header"] += " · %d missed since" % misses
        else:
            # "offline" only for a robot that has gone quiet (version 3.1). Something that
            # answers without a usable report is not offline, and the header says what it is.
            # The last-seen time is said HERE, once; the disk boxes below do not repeat it.
            word = HEADER_WORD.get(kind, "offline")
            if kind == "refused" and hosts.get(next((a for a, k, _ in tried if k == "refused"),
                                                    "")) != ROBOT_HOST:
                word = "no reporter there"          # it may not be the robot at all
            if ok_wall:
                rb["header"] = "%s · %s %s" % (word, "last seen" if word == "offline" else
                                              "last good report" if kind == "bad" else
                                              "last report", hhmm(ok_wall))
            else:
                rb["header"] = "%s · not seen since this page started at %s" % (
                    word, hhmm(self.started))
            if kind == "refused":
                a = next((a for a, k, _ in tried if k == "refused"), tried_txt)
                rb["level"] = 1
                rb["commands"] = [ROBOT_START_CMD]
                if hosts.get(a) == ROBOT_HOST:
                    rb["short"] = "Robot's storage reporter: not running"
                    rb["status"] = ("The robot (%s) answers at %s. Start its storage reporter - "
                                    "on the robot:" % (ROBOT_HOST, a))
                else:
                    rb["short"] = "Robot's address: something answered, but no storage reporter"
                    rb["status"] = ("Something at %s answered, but no storage reporter (the "
                                    "program that reports disk space) is running there. If it is "
                                    "the robot, on the robot:" % a)
            elif kind == "bad":
                a = next((a for a, k, _ in tried if k == "bad"), tried_txt)
                rb["level"] = 1
                rb["short"] = "Robot's storage report: not understood"
                rb["status"] = ("Asked %s: %s. The robot's reporter may be a different version "
                                "from this page." % (a, reason))
            elif kind == "never":
                rb["header"] = "asking..."
                rb["status"] = "Asking the robot for the first time..."
            elif kind == "noaddr":
                rb["level"] = 1
                rb["short"] = "Robot: no valid address to ask"
                rb["status"] = "No valid robot address to ask."
            elif kind == "timeout":
                rb["status"] = "No answer from %s: probably off or charging." % tried_txt
            elif kind == "error":
                rb["status"] = "Tried %s: %s." % (tried_txt, reason)
            else:
                rb["status"] = "Tried %s: %s. Probably off or charging." % (tried_txt, reason)
        if invalid:
            rb["level"] = max(rb["level"], 1)
            if not rb["short"]:
                rb["short"] = "Robot's address list: %s not valid" % ", ".join('"%s"' % x for x in invalid)
            rb["status"] = (rb["status"] + " " if rb["status"] else "") + (
                "Ignored %s: not a valid address (it should look like <robot-wifi-address> or "
                "<robot-wifi-address>:8113; set in ~/storage_robot_addrs.txt on the Jetson, or --robot)."
                % ", ".join('"%s"' % x for x in invalid))
        disks = {x["key"]: x for x in (payload or {}).get("disks", [])}
        have = payload is not None
        rb["devices"] = [self._root(disks.get("robot_root"), online, have, ok_wall, rates["robot_root"]),
                         self._card(disks.get("robot_card"), online, have, ok_wall, rates["robot_card"])]
        for d in rb["devices"]:          # what the figure line says instead of a filling rate
            if d.get("stale") and not d.get("stale_text"):
                d["stale_text"] = "offline" if kind in SILENT_KINDS else "not updating"
        return rb

    @staticmethod
    def _common(d, x, online, have, ok_wall):
        """False when there are no figures to judge (the caller returns d as it is)."""
        if x is None:
            if online:                  # answered, but without this disk: a caution, not grey
                d.update(state="missing", level=1, headline="not reported",
                         short="missing from the robot's report",
                         status="The robot's reporter answered without this disk - it may be a "
                                "different version from this page.")
            else:
                d.update(state="offline", stale=True, grey=True, headline="no reading",
                         status="No reading from the robot." if have else
                                "No reading from the robot yet.")
            add_rate(d, None)
            return False
        for k in ("device", "fstype", "total", "used", "avail", "error", "read_only"):
            if k in x:
                d[k] = x[k]
        d["mountpoint"] = x.get("path")
        age = x.get("age_s")
        if online and age is not None and age > ROBOT_STUCK_S:
            # The robot IS answering: never "offline" here (version 3 said it on the figure line)
            d.update(state="stuck", level=2, grey=True, stale=True, headline="stuck?",
                     stale_text="not measured for %d s" % age,
                     short="not measured for %d s, it may be stuck" % age,
                     status="The robot has not managed to measure this disk for %d s: it may be "
                            "stuck (a card that stopped answering?).%s"
                            % (age, "" if x.get("measuring") else " The figures are from before."))
            add_rate(d, None)
            return False
        if x.get("measuring"):
            d.update(state="measuring", grey=True, headline="measuring",
                     status="The robot is measuring this disk for the first time...")
            add_rate(d, None)
            return False
        if not online:
            d.update(stale=True, grey=True, state="offline")
        return True

    def _root(self, x, online, have, ok_wall, rate):
        d = {"key": "robot_root", "machine": "robot", "title": "Robot's own disk",
             "what": "a colleague's disk; only small logs of ours",
             "level": 0, "notes": [], "commands": [], "floor": ROBOT_ROOT_RED,
             "floor_label": "1 GB: the red line", "floor_short": "at 1 GB in"}
        if not self._common(d, x, online, have, ok_wall):
            return d
        if d.get("avail") is None:
            lvl, st = 2, "Cannot read the robot's own disk: %s" % d.get("error", "no figures")
        elif d["avail"] < ROBOT_ROOT_RED:
            lvl, st = 2, ("Under 1 GB free: the robot software's own logs alone could fill it "
                          "and stop the robot's programs.")
        elif d["avail"] < ROBOT_ROOT_CAUTION:
            lvl, st = 1, "Under 3 GB free (amber under 3 GB, red under 1 GB)."
        else:
            lvl, st = 0, "Room to spare."
        if d.get("avail") is not None:
            d["headline"] = fmt_gb(d["avail"]) + " free"
        return self._finish(d, lvl, st, online, ok_wall, rate)

    def _card(self, x, online, have, ok_wall, rate):
        d = {"key": "robot_card", "machine": "robot", "title": "Robot's memory card (USB Drive)",
             "what": "each drive's LiDAR (laser scanner) recording, about 6.3 GB",
             "level": 0, "notes": [], "commands": [], "floor": ROBOT_CARD_RED,
             "floor_label": "8 GB: the least to start a drive's recording",
             "floor_short": "at 8 GB in"}
        if not self._common(d, x, online, have, ok_wall):
            return d
        if not x.get("mounted"):
            if x.get("error"):
                lvl, st = 2, "The robot could not check its card: %s." % x["error"]
            else:
                lvl, st = 2, ("Not attached at %s (the card's folder): a LiDAR recording has "
                              "nowhere to go." % (x.get("path") or ROBOT_CARD_PATH))
            d["headline"] = "not attached"
        elif d.get("avail") is None:
            lvl, st = 2, "Attached but not answering: %s" % d.get("error", "no figures")
            d["headline"] = "not answering"
        elif d.get("read_only"):
            lvl, st = 2, ("Attached READ-ONLY (it can be read but not saved to): a LiDAR "
                          "recording would fail.")
        elif d["avail"] < ROBOT_CARD_RED:
            lvl, st = 2, ("Under 8 GB free: another drive's LiDAR recording (about 6.3 GB) "
                          "will not fit with margin.")
        else:
            n = int((d["avail"] - ROBOT_CARD_RED) // DRIVE_SIZE) + 1
            d["drives_fit"] = n
            lvl, st = 0, ("About %d more %s (6.5 GB each, never starting one under 8 GB)."
                          % (n, "drive fits" if n == 1 else "drives fit"))
        if d.get("avail") is not None and x.get("mounted"):
            d["headline"] = fmt_gb(d["avail"]) + " free"
        return self._finish(d, lvl, st, online, ok_wall, rate)

    SHORT = {"Cannot": "cannot be read", "Under 1 GB": "under 1 GB free",
             "Under 3 GB": "under 3 GB free", "Not attached": "not attached",
             "The robot could not": "could not be checked",
             "Attached but": "attached but not answering", "Attached READ": "attached read-only",
             "Under 8 GB": "under 8 GB free, not enough for a drive"}

    @classmethod
    def _finish(cls, d, lvl, st, online, ok_wall, rate):
        short = next((v for k, v in cls.SHORT.items() if st.startswith(k)), st) if lvl else ""
        if online:
            d.update(level=lvl, status=st, short=short)
            add_rate(d, rate)
        else:
            # The last figures, greyed - and the last ALARM LEVEL KEPT. Version 2 dropped it
            # to 0 here, so a red card read "Every disk has room" once the robot went quiet.
            # Version 3.1: no description line and no time (the robot header says when it
            # was last seen), and a status line only for a disk that was in trouble - the
            # figure line already says "offline" / "not updating" - so the page still fits a
            # phone (version 3's "Last known at HH:MM: ..." boxes took it to 879-928 px of 812).
            seen = hhmm(ok_wall) if ok_wall else "?"
            d.update(level=lvl, last_level=lvl, what="",
                     status=("When last seen: %s." % (short[:1].lower() + short[1:])) if lvl
                     else "")
            if lvl:
                d["short"] = "%s (when last seen at %s)" % (short, seen)
            add_rate(d, None)
        return d


# -------------------------------------------------------------- the sampler ----
def guarded(fn, key, title, *args):
    """One disk's check; if it fails, that disk says so and the others still show."""
    try:
        return fn(*args)
    except Exception as exc:
        log("%s check failed: %r" % (key, exc))       # the error's name: the log only
        return {"key": key, "machine": "jetson", "title": title, "level": 2,
                "headline": "no reading", "short": "the page could not check it",
                "status": "The page could not check this disk (an unexpected error; see the "
                          "page's log).",
                "notes": [], "commands": []}


LOCAL_PARTS = (("jetson_root", "Jetson internal disk"), ("usb", "USB stick (CAM_REC)"),
               ("retired_card", "Retired microSD card (SIDEWALK128)"))
LOCAL_NAMES = {"jetson_root": "its own internal disk", "usb": "the USB stick",
               "retired_card": "the retired microSD card"}
LOCAL_STUCK_MIN = 10.0       # a Jetson disk whose check has not finished for this long may be stuck


def stuck_local(key, title, last, age):
    """A Jetson disk whose check has not finished for `age` s (e.g. a USB stick that stopped
    answering): it says so by name, level 2, with its last figures kept, greyed."""
    d = {"key": key, "machine": "jetson", "title": title, "level": 2, "grey": True,
         "stale": True, "state": "stuck", "headline": "stuck?", "age_s": round(age, 1),
         "stale_text": "not checked for %d s" % age,
         "short": "not checked for %d s, it may be stuck" % age, "notes": [], "commands": [],
         "rate": None, "full_in": None, "floor_in": None}
    for k in ("what", "total", "used", "avail", "mountpoint", "device", "fstype",
              "floor", "floor_label", "floor_short"):
        if last and k in last:
            d[k] = last[k]
    d["status"] = ("The Jetson has not managed to check %s for %d s: it may be stuck%s.%s"
                   % (LOCAL_NAMES[key], age,
                      " (a stick that stopped answering? saving to it may hang too)"
                      if key == "usb" else "",
                      " The figures are from before." if d.get("avail") is not None else ""))
    return d


class Local:
    """The Jetson's three disks, each measured in ITS OWN thread (version 3.1), each with
    its own age. Version 3 measured all three in one loop, so a USB stick check that hung
    froze the internal disk's figures too - the number that decides whether a drive can
    start - and the page could only say "measuring has stopped", not which disk."""

    def __init__(self, cfg):
        self.cfg = cfg
        self.lock = threading.Lock()
        self.latest = {}             # key -> (device dict, wall time, monotonic time)
        self.control = {}            # key -> the attachment check's own control, per thread
        self.rates = {k: Rate(10.0, 3.0) for k, _ in LOCAL_PARTS}
        self.born = time.monotonic()

    def measure(self, key):
        """Check one disk and keep the result. Nothing is locked WHILE checking, so a check
        that hangs holds up nobody else."""
        title = dict(LOCAL_PARTS)[key]
        control = None
        if key == "jetson_root":
            d = guarded(jetson_root, key, title)
        else:
            mounts = read_mounts(self.cfg.mounts_file)
            # The page's own check, checked: "/" MUST be found mounted and /etc must NOT be.
            # If either fails, the stick and card states cannot be trusted either.
            control = mount_at("/", mounts) is not None and mount_at("/etc", mounts) is None
            d = guarded(usb_stick if key == "usb" else retired_card, key, title, self.cfg, mounts)
        now_m = time.monotonic()
        with self.lock:
            rt = self.rates[key]
            if d.get("used") is not None and "error" not in d:
                rt.add(now_m, d["used"], (d.get("device"), d.get("mountpoint"), d.get("total")))
                add_rate(d, rt.value())
            else:
                rt.clear()
                add_rate(d, None)
            self.latest[key] = (d, time.time(), now_m)
            if control is not None:
                self.control[key] = control

    def sample(self):
        """All three, one after the other (for --once and the tests)."""
        for key, _ in LOCAL_PARTS:
            self.measure(key)

    def _loop(self, key):
        while True:
            try:
                self.measure(key)
            except Exception as exc:
                log("%s measuring failed: %r" % (key, exc))
            time.sleep(self.cfg.local_period)

    def run(self):
        for key, _ in LOCAL_PARTS:
            threading.Thread(target=self._loop, args=(key,), name="local-" + key,
                             daemon=True).start()

    def snapshot(self):
        """(devices, time of the oldest figure shown, control) - each disk with its age; one
        whose figures are older than max(10 s, 5 periods) says it may be stuck, by name."""
        limit = max(LOCAL_STUCK_MIN, 5 * self.cfg.local_period)
        now_m = time.monotonic()
        with self.lock:
            latest, control = dict(self.latest), dict(self.control)
        devs, walls = [], []
        for key, title in LOCAL_PARTS:
            got = latest.get(key)
            last, age = (dict(got[0]), now_m - got[2]) if got else (None, now_m - self.born)
            if got:
                walls.append(got[1])
            if age > limit:
                d = stuck_local(key, title, last, age)
            elif last is None:
                d = {"key": key, "machine": "jetson", "title": title, "level": 0, "grey": True,
                     "headline": "measuring", "status": "Measuring it for the first time...",
                     "notes": [], "commands": [], "rate": None, "full_in": None, "floor_in": None}
            else:
                d = last
                d["age_s"] = round(age, 1)
            devs.append(d)
        ok = None if not control else (False not in control.values())
        return devs, (min(walls) if walls else 0.0), ok

    @property
    def devices(self):               # version 3's name, kept for the tests that read it
        return self.snapshot()[0]


def robot_fault_block(exc):
    """If putting the robot's part together ever fails, it says so - and the Jetson's
    own disks, which do not depend on it, still show. The error's name is logged by the
    caller and never shown."""
    placeholder = lambda key, title: {
        "key": key, "machine": "robot", "title": title, "level": 1, "grey": True, "stale": True,
        "headline": "not shown", "short": "not shown (page fault)",
        "status": "The page could not show this (an unexpected error; see the page's log).",
        "notes": [], "commands": [], "rate": None, "full_in": None, "floor_in": None}
    return {"online": False, "kind": "error", "level": 1, "header": "not shown (page fault)",
            "short": "Robot part of the page: fault", "commands": [], "silent": "robot figures not shown",
            "status": "The page could not put the robot's part together (an unexpected error; "
                      "see the page's log).",
            "devices": [placeholder("robot_root", "Robot's own disk"),
                        placeholder("robot_card", "Robot's memory card (USB Drive)")]}


def compose(local, robot):
    devs, t, control = local.snapshot()
    try:
        rb = robot.block()
    except Exception as exc:
        log("robot block failed: %r" % exc)
        rb = robot_fault_block(exc)
    devices = devs + rb["devices"]
    now = time.time()
    alerts = []
    # (Version 3 raised one page-wide "measuring has stopped" alarm here. Each Jetson disk
    # now carries its own age and says by name when its check may be stuck - snapshot().)
    if control is False:
        alerts.append((2, "The attachment check failed its own control (it could not find '/' "
                          "or found /etc): do not trust the stick and card states."))
    for d in devices:
        if d.get("level", 0) > 0:
            alerts.append((d["level"], "%s: %s" % (d["title"].split(" (")[0],
                                                   d.get("short") or d.get("status", ""))))
    if rb["level"] > 0:
        alerts.append((rb["level"], rb.get("short") or rb["status"]))
    alerts.sort(key=lambda a: -a[0])                 # the worst first, in full
    worst = alerts[0][0] if alerts else 0
    unknown = False
    word = ["ALL CLEAR", "CAUTION", "PROBLEM"][worst]
    silent = rb.get("silent") or ""
    if worst:
        sentence = alerts[0][1].rstrip(".") + "."
        if len(alerts) > 1:                           # the rest by name only - details are below
            sentence += " Also: " + ", ".join(a[1].split(":")[0] for a in alerts[1:]) + "."
        if not rb["online"] and rb["level"] == 0 and silent:
            sentence += " " + silent[0].upper() + silent[1:] + "."
    elif rb["online"]:
        sentence = "Every disk has room."
    else:
        # NEVER "all clear" while the robot is silent: nothing is known about its disks.
        unknown, word = True, "ROBOT SILENT"
        sentence = "Jetson disks have room; %s." % silent
    return {"version": VERSION, "now": now, "sampled": t, "control_ok": control,
            "local_period": local.cfg.local_period, "robot_period": local.cfg.robot_period,
            "verdict": {"level": worst, "word": word, "unknown": unknown,
                        "sentence": sentence, "count": len(alerts)},
            "robot": {k: v for k, v in rb.items() if k != "devices"},
            "devices": devices}


def api_body(local, robot):
    """The /api/storage answer - ALWAYS valid JSON (no NaN), whatever else went wrong."""
    try:
        return json.dumps(compose(local, robot), allow_nan=False).encode()
    except Exception as exc:
        log("compose failed: %r" % exc)                # the error's name: the log only
        try:
            devs, t, _ = local.snapshot()
            devs = json.loads(json.dumps(devs, allow_nan=False))
        except Exception:
            devs, t = [], 0.0
        return json.dumps({
            "version": VERSION, "now": time.time(), "sampled": t, "control_ok": None,
            "local_period": local.cfg.local_period, "robot_period": local.cfg.robot_period,
            "verdict": {"level": 2, "word": "PROBLEM", "unknown": False, "count": 1,
                        "sentence": "The page hit an unexpected fault putting its figures "
                                    "together (see the page's log); the Jetson's own disks "
                                    "are shown."},
            "robot": {"online": False, "level": 2, "header": "not shown (page fault)",
                      "status": "", "commands": []},
            "devices": devs}).encode()


# ------------------------------------------------------------------- the page ----
PAGE = r"""<!doctype html><html lang="en"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1,viewport-fit=cover">
<title>Storage Monitor</title>
<style>
:root{
  --ground:#0B0C0E; --panel:#131519; --raised:#1C1F24; --hair:#2A2E35;
  --ink:#F2F4F7; --ink2:#AAB2BD; --lab:#6E7681; --ghost:#4A5058; --fill:#B9C0CB;
  --amber:#F0A22E; --red:#FF4438; --redA:rgba(255,68,56,.3); --pip:#4FD08A;
  --mono:"JetBrains Mono","Roboto Mono",ui-monospace,"DejaVu Sans Mono",monospace;
  --ui:Inter,system-ui,-apple-system,"Segoe UI",sans-serif;
}
@media (prefers-color-scheme:light){:root:not([data-theme="dark"]){
  --ground:#F5F4F1; --panel:#FFFFFF; --raised:#EAE8E3; --hair:#D6D3CC;
  --ink:#14161A; --ink2:#4A5058; --lab:#7A8089; --ghost:#B0B5BC; --fill:#3C4550;
  --amber:#8A5A00; --red:#B3241A; --redA:rgba(179,36,26,.3); --pip:#1F9D5B;
}}
*{box-sizing:border-box;margin:0;padding:0}
body{background:var(--ground);color:var(--ink);font:14px/1.35 var(--ui);-webkit-font-smoothing:antialiased}
#wrap{max-width:1180px;margin:0 auto;padding:10px 12px;display:flex;flex-direction:column;gap:8px;min-height:100dvh}
@media (max-width:500px){#wrap{padding:6px 8px;gap:6px}}
.panel{background:var(--panel);border:1px solid var(--hair)}
.mono{font-family:var(--mono);font-variant-numeric:tabular-nums slashed-zero}
.lab{font-size:11px;font-weight:700;text-transform:uppercase;letter-spacing:.09em;color:var(--lab)}
#verdict{display:flex;align-items:center;gap:10px;padding:7px 12px;min-height:42px}
#pip{width:10px;height:10px;border-radius:50%;background:var(--pip);flex:none}
#pip.off{visibility:hidden} #pip.stale{visibility:visible;background:var(--ghost)}
#vstate{font-family:var(--mono);font-weight:700;font-size:clamp(16px,2.6vw,24px);white-space:nowrap}
#vsent{flex:1;color:var(--ink2);font-size:13px;min-width:0}
#clock{font-family:var(--mono);color:var(--lab);font-size:13px}
#cols{display:grid;grid-template-columns:1fr 1fr;gap:10px;align-items:start}
@media (max-width:860px){#cols{grid-template-columns:1fr}}
.grp{display:flex;flex-direction:column;gap:6px;min-width:0}
.gh{display:flex;justify-content:space-between;align-items:baseline;gap:8px;padding:0 2px}
.gh .r{font-family:var(--mono);font-size:11.5px;color:var(--lab);text-align:right}
.gs{font-size:12.5px;color:var(--ink2);padding:0 2px}
.gs.l1{color:var(--amber)} .gs.l2{color:var(--red)}
#rc{display:flex;flex-direction:column;gap:4px}
#rc:empty{display:none}
#age{font-family:var(--mono);color:var(--amber);font-size:12px;white-space:nowrap}
.dev{padding:6px 10px;display:flex;flex-direction:column;gap:3px;border-left:3px solid transparent;min-width:0}
.dh{display:flex;justify-content:space-between;align-items:baseline;gap:8px}
.dn{font-weight:700;font-size:14px}
.dv{font-family:var(--mono);font-variant-numeric:tabular-nums slashed-zero;font-size:14.5px;font-weight:700;white-space:nowrap}
.dw{font-size:11.5px;color:var(--lab)}
.bar{height:11px;background:var(--raised);border:1px solid var(--hair);position:relative;overflow:hidden;margin-top:1px}
.bar>i{position:absolute;inset:0 auto 0 0;background:var(--fill)}
.bar>s{position:absolute;top:0;bottom:0;width:2px;background:var(--ink);opacity:.75}
.dm{font-family:var(--mono);font-variant-numeric:tabular-nums slashed-zero;font-size:11.5px;color:var(--ink2)}
.ds{font-size:13px}
.dnote{font-size:11.5px;color:var(--lab)}
.cmd{font-family:var(--mono);font-size:11.5px;background:var(--raised);border:1px solid var(--hair);
     padding:5px 8px;overflow-wrap:anywhere;user-select:all;-webkit-user-select:all}
.dev.l1{border-left-color:var(--amber)} .dev.l1 .ds,.dev.l1 .dv{color:var(--amber)} .dev.l1 .bar>i{background:var(--amber)}
.dev.l2{outline:3px solid var(--red);outline-offset:-3px;animation:breathe 2s ease-in-out infinite}
.dev.l2 .ds,.dev.l2 .dv{color:var(--red)} .dev.l2 .bar>i{background:var(--red)}
@keyframes breathe{0%,100%{outline-color:var(--redA)}50%{outline-color:var(--red)}}
@media (prefers-reduced-motion:reduce){.dev.l2{animation:none}}
.dev.grey .dn,.dev.grey .dv,.dev.grey .dm,.dev.grey .ds,.dev.grey .dw{color:var(--ghost)}
.dev.grey .bar>i{background:var(--ghost)}
/* a robot disk that was in trouble when last seen: greyed figures, alarm KEPT, but a still
   dashed frame instead of the breathing one (that one means "in trouble right now") */
.dev.grey.l1{border-left:3px dashed var(--amber)}
.dev.grey.l2{animation:none;outline:2px dashed var(--red);outline-offset:-2px}
.dev.grey.l1 .ds{color:var(--amber)} .dev.grey.l2 .ds{color:var(--red)}
.dev.line{background:none;border:none;padding:0 2px;font-size:12px;color:var(--lab)}
#foot{font-size:11px;line-height:1.3;color:var(--lab);padding:0 2px}
@media (max-width:500px){.ds{font-size:12.5px}.dn{font-size:13.5px}.dv{font-size:14px}
  #vsent{font-size:12.5px}.gh .r{font-size:11px}
  /* 3.1: a little less space around each box on a phone, so every robot state fits */
  .dev{padding:4px 8px;gap:2px}.grp{gap:5px}#cols{gap:8px}}
body.stale .dv,body.stale .dm{color:var(--ghost)}
</style></head><body>
<div id="wrap">
  <div class="panel" id="verdict"><div id="pip" class="off"></div><div id="vstate">...</div>
    <div id="vsent">connecting to the Jetson...</div><span id="age"></span><div id="clock"></div></div>
  <div id="cols">
    <div class="grp"><div class="gh"><span class="lab">Jetson</span><span class="r" id="jh"></span></div>
      <div id="gj" class="grp"></div></div>
    <div class="grp"><div class="gh"><span class="lab">Robot</span><span class="r" id="rh"></span></div>
      <div class="gs" id="rs"></div><div id="rc"></div><div id="gr" class="grp"></div></div>
  </div>
  <div id="foot"><span id="upd"></span> · filling = how fast it is filling now · mark on a bar =
    that disk's red line · fusion drive = a mapping drive blending wheels, gyroscope and camera ·
    1 GB = 1,024 MB, as the drive scripts count</div>
</div>
<script>
const $=id=>document.getElementById(id);
const TZ='America/Toronto', MB=1048576, GB=1073741824;
const t2s=(t,sec)=>new Date(t*1000).toLocaleTimeString('en-CA',{timeZone:TZ,hour:'2-digit',
  minute:'2-digit',second:sec?'2-digit':undefined,hour12:false});
const esc=s=>String(s==null?'':s).replace(/[&<>"]/g,c=>({'&':'&amp;','<':'&lt;','>':'&gt;','"':'&quot;'}[c]));
const gb=b=>(b/GB).toFixed(1)+' GB';
function dur(s){s=Math.max(0,Math.round(s));if(s<90)return s+' s';const m=Math.round(s/60);
  if(m<90)return m+' min';const h=Math.floor(m/60);if(h<48)return h+' h '+String(m%60).padStart(2,'0')+' min';
  return Math.round(h/24)+' days';}
function rateText(d){
  if(d.stale)return d.stale_text||'offline';   // 3.1: 'offline' only for a silent robot
  if(d.rate==null)return 'measuring...';
  const r=d.rate/MB;
  if(Math.abs(r)<0.05)return 'not filling';
  if(r<0)return 'freeing '+(-r).toFixed(1)+' MB/s';
  let t='filling '+r.toFixed(1)+' MB/s · full in '+dur(d.full_in);
  if(d.floor_in!=null)t+=' · '+d.floor_short+' '+dur(d.floor_in);
  return t;
}
function inner(d){
  if(d.compact)return esc(d.status);
  let h=`<div class="dh"><span class="dn">${esc(d.title)}</span><span class="dv">${esc(d.headline)}</span></div>`;
  if(d.what)h+=`<div class="dw">${esc(d.what)}</div>`;
  if(d.total!=null&&d.avail!=null){
    const cap=d.used+d.avail, pct=cap?100*d.used/cap:0;
    let tick='';
    if(d.floor&&cap){const tp=100*(cap-d.floor)/cap;if(tp>0&&tp<100)tick=`<s style="left:${tp.toFixed(1)}%" title="${esc(d.floor_label)}"></s>`;}
    h+=`<div class="bar"><i style="width:${pct.toFixed(1)}%"></i>${tick}</div>`;
    h+=`<div class="dm">${gb(d.used)} of ${gb(d.total)} used · ${Math.round(pct)} % · ${esc(rateText(d))}</div>`;
  }
  if(d.status)h+=`<div class="ds">${esc(d.status)}</div>`;
  for(const c of d.commands||[])h+=`<div class="cmd">${esc(c)}</div>`;
  for(const n of d.notes||[])h+=`<div class="dnote">${esc(n)}</div>`;
  return h;
}
// One element per disk, kept between refreshes, so a breathing frame is not restarted every 2 s.
const els={};
function put(d){
  let el=els[d.key];
  if(!el){el=document.createElement('section');els[d.key]=el;$(d.machine==='robot'?'gr':'gj').appendChild(el);}
  const cls=d.compact?'dev line':['dev panel','l'+(d.level||0),d.grey?'grey':''].join(' ').trim();
  if(el.className!==cls)el.className=cls;
  const h=inner(d); if(el._h!==h){el.innerHTML=h;el._h=h;}
}
// Version 3: each request gets LIMIT_MS (then it is cancelled), a new one is never started
// while one is still waiting, and the age of the last good answer is checked on EVERY tick,
// whatever the request did. Version 2 only noticed a FAILED request, so a hung Jetson kept
// the green pip and a ticking clock for as long as the browser's own network timeout.
const LIMIT_MS=4000, HINT_MS=5000, STALE_MS=10000;
let lastOk=0, inflight=false; const startT=Date.now();
function render(j){
  document.body.classList.remove('stale');
  for(const d of j.devices||[]){try{put(d);}catch(e){}}
  const v=j.verdict||{};
  $('vstate').textContent=v.word||'?';
  $('vstate').style.color=v.level===2?'var(--red)':v.level===1?'var(--amber)':v.unknown?'var(--ink2)':'';
  $('vsent').textContent=v.sentence||'';
  $('pip').className=v.level===0?(v.unknown?'stale':''):'off';
  $('jh').textContent='measured every '+j.local_period+' s';
  const rb=j.robot||{};
  $('rh').textContent=rb.header||'';
  $('rs').textContent=rb.status||'';
  $('rs').className='gs'+(rb.level?' l'+rb.level:'');
  const rc=(rb.commands||[]).map(c=>`<div class="cmd">${esc(c)}</div>`).join('');
  if($('rc')._h!==rc){$('rc').innerHTML=rc;$('rc')._h=rc;}
  $('upd').textContent='updated '+t2s(j.now,true)+' (Hamilton time)';
}
function checkAge(){
  const age=Date.now()-(lastOk||startT);
  if(age>STALE_MS){
    document.body.classList.add('stale');
    $('pip').className='stale';$('vstate').textContent='NO DATA';$('vstate').style.color='';
    $('vsent').textContent=lastOk?('Lost contact with the Jetson since '+t2s(lastOk/1000,true)+
      ' - the figures below are old.'):'Cannot reach the Jetson.';
    $('age').textContent='';
  }else if(lastOk&&age>HINT_MS){
    $('age').textContent='figures '+Math.round(age/1000)+' s old';
  }else $('age').textContent='';
}
async function ask(){
  if(inflight)return;                       // never two at once
  inflight=true;
  const ac=new AbortController(), timer=setTimeout(()=>ac.abort(),LIMIT_MS);
  try{
    const r=await fetch('/api/storage',{cache:'no-store',signal:ac.signal});
    if(!r.ok)throw new Error('status '+r.status);
    const j=await r.json();
    lastOk=Date.now();
    try{render(j);}catch(e){$('vsent').textContent='This page could not draw the latest figures.';}
  }catch(e){}
  finally{clearTimeout(timer);inflight=false;checkAge();}
}
function tick(){
  $('clock').textContent=t2s(Date.now()/1000,false);
  checkAge();
  ask();
}
tick();setInterval(tick,2000);
</script>
</body></html>
"""

class QuietServer(ThreadingHTTPServer):
    """socketserver's default handle_error prints a full traceback for every broken
    connection (e.g. a phone that gave up) - logged here once per kind instead."""
    daemon_threads = True
    _seen = set()

    def handle_error(self, request, client_address):
        kind = type(sys.exc_info()[1]).__name__
        if kind not in self._seen and len(self._seen) < 50:
            self._seen.add(kind)
            log("a request from %s failed: %s (logged once per kind)" % (client_address[0], kind))


OLD_API = json.dumps({"timestamp": 0, "disks": [{
    "label": "This storage page was updated", "path": "", "mounted": False,
    "error": "reload the page to see the new version"}]}).encode()


def make_handler(local, robot):
    class Handler(BaseHTTPRequestHandler):
        timeout = 20                      # a connection that never sends a request is closed

        def log_message(self, fmt, *args):
            pass                          # polled every 2 s; never log each request

        def _send(self, code, ctype, body):
            self.send_response(code)
            self.send_header("Content-Type", ctype)
            self.send_header("Cache-Control", "no-store")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)

        def do_GET(self):
            p = self.path.split("?")[0]
            if p in ("/", "/index.html"):
                self._send(200, "text/html; charset=utf-8", PAGE.encode("utf-8"))
            elif p == "/api/storage":
                self._send(200, "application/json", api_body(local, robot))
            elif p == "/api/disks":       # a tab still showing version 1: tell it to reload
                self._send(200, "application/json", OLD_API)
            else:
                self._send(404, "text/plain", b"not found\n")

    return Handler


def main():
    ap = argparse.ArgumentParser(description="The storage page (version %s)." % VERSION)
    ap.add_argument("--port", type=int, default=8092)
    ap.add_argument("--robot", default=os.environ.get("STORAGE_ROBOT_ADDRS", os.environ.get("ROBOT_WIFI_ADDR", "")),
                    help="robot address(es), comma list, host or host:port (default port %d)"
                         % ROBOT_PORT)
    ap.add_argument("--robot-file", default=os.path.expanduser("~/storage_robot_addrs.txt"),
                    help="optional file of addresses, re-read every cycle, tried first")
    ap.add_argument("--robot-timeout", type=float, default=1.0)
    ap.add_argument("--robot-period", type=float, default=5.0)
    ap.add_argument("--local-period", type=float, default=2.0)
    # For tests only: simulate the other states of the stick and the card without
    # touching either (a different ID, a fake mount list, an empty card slot).
    ap.add_argument("--usb-uuid", default=USB_UUID, help=argparse.SUPPRESS)
    ap.add_argument("--usb-label", default=USB_LABEL, help=argparse.SUPPRESS)
    ap.add_argument("--retired-cid", default=RETIRED_CID, help=argparse.SUPPRESS)
    ap.add_argument("--retired-uuid", default=RETIRED_UUID, help=argparse.SUPPRESS)
    ap.add_argument("--card-glob", default="/sys/block/mmcblk*", help=argparse.SUPPRESS)
    ap.add_argument("--mounts-file", default="/proc/mounts", help=argparse.SUPPRESS)
    ap.add_argument("--once", action="store_true", help="print one reading as JSON and exit")
    cfg = ap.parse_args()

    local, robot = Local(cfg), Robot(cfg)
    if cfg.once:
        local.sample()
        robot.poll()
        print(json.dumps(json.loads(api_body(local, robot)), indent=1))
        return
    try:
        server = QuietServer(("0.0.0.0", cfg.port), make_handler(local, robot))
    except OSError as exc:
        log("cannot listen on port %d: %s - is another copy already running?" % (cfg.port, exc))
        sys.exit(1)
    local.run()                   # one thread per Jetson disk (version 3.1)
    threading.Thread(target=robot.run, name="robot", daemon=True).start()
    log("version %s serving on port %d (pid %d), robot at %s"
        % (VERSION, cfg.port, os.getpid(), cfg.robot))
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
