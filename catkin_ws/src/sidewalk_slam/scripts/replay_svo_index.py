#!/usr/bin/env python3
"""replay_svo_index.py - list the capture time of every frame in a ZED camera recording (SVO file).

WHY
    A replay must know WHEN the camera recording starts, to place the robot's bag at the same
    moment, and it must know each frame's true capture time to measure how far the replayed
    pictures drift from the wheel data. The ZED wrapper does not publish those times in
    playback (it stamps pictures with the current time), so they are read here, once, with the
    ZED SDK's Python module (pyzed), without computing depth.

HOW TO RUN
    replay_svo_index.py <recording.svo2> <out.csv>

INPUT   a .svo2 file
OUTPUT  <out.csv>       one line per frame:  frame,timestamp_ns   (nanoseconds since 1970)
        <out.csv>.json  a summary: frames, first_ns, last_ns, duration_s, mean_fps
        The summary is also printed as the last line of standard output.

Reading is limited by video decoding: about 65 frames a second on the Jetson, so a 20-minute
recording takes roughly 4-5 minutes. The output is kept beside the replay so this runs once.
"""
import collections
import json
import sys


def read_frame_times(svo_path):
    """Open the recording with depth switched off and return the capture time of every frame."""
    import pyzed.sl as sl  # imported here so --help works on a machine without the SDK
    params = sl.InitParameters()
    params.set_from_svo_file(svo_path)
    params.svo_real_time_mode = False      # step through every frame, as fast as it decodes
    params.depth_mode = sl.DEPTH_MODE.NONE  # we only want timestamps
    params.sdk_verbose = 0
    camera = sl.Camera()
    status = camera.open(params)
    if status != sl.ERROR_CODE.SUCCESS:
        raise RuntimeError("could not open %s: %s" % (svo_path, status))
    times_ns = []
    while camera.grab() == sl.ERROR_CODE.SUCCESS:
        times_ns.append(camera.get_timestamp(sl.TIME_REFERENCE.IMAGE).get_nanoseconds())
    camera.close()
    return times_ns


def main(argv):
    if len(argv) >= 2 and argv[1] in ("-h", "--help"):
        print(__doc__)
        return 0
    if len(argv) != 3:
        print(__doc__)
        return 2
    svo_path, out_csv = argv[1], argv[2]
    times_ns = read_frame_times(svo_path)
    if not times_ns:
        print("no frames could be read from %s" % svo_path, file=sys.stderr)
        return 1
    with open(out_csv, "w") as f:
        f.write("frame,timestamp_ns\n")
        for i, t in enumerate(times_ns):
            f.write("%d,%d\n" % (i, t))
    duration_s = (times_ns[-1] - times_ns[0]) / 1e9
    gaps = collections.Counter(round((b - a) / 1e9, 4) for a, b in zip(times_ns, times_ns[1:]))
    summary = {
        "svo": svo_path,
        "frames": len(times_ns),
        "first_ns": times_ns[0],
        "last_ns": times_ns[-1],
        "duration_s": round(duration_s, 3),
        "mean_fps": round(len(times_ns) / duration_s, 2) if duration_s > 0 else None,
        "most_common_gaps_s": gaps.most_common(4),
    }
    with open(out_csv + ".json", "w") as f:
        json.dump(summary, f, indent=1)
    print(json.dumps(summary))
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv))
