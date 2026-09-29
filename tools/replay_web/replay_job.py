#!/usr/bin/env python3
"""replay_job.py - run ONE job for the replay web page: a mapping replay or a camera bench test -
and keep its state file up to date while it runs.

WHAT IT DOES
    replay_control_server.py writes a job description (JSON) and starts this program in a session
    of its own (setsid), so the job carries on even if the web backend restarts. This program:
      - runs the job's real work as child processes:
          slam      run/replay.sh <run> --bag ... --svo ... --out ...  (the tested replay tool), plus
                    map_view_local.py so the page can show the map while it builds
          bench     svo_replay_n.py once per chosen depth mode (the camera bench test tool)
      - rewrites $JOBS_DIR/replay_web/current.json every ~2 s (state, progress, counts, results)
        and a one-line $JOBS_DIR/replay_web.progress for the jobs page
      - on SIGTERM (the page's Stop button) shuts the work down in the right order:
          slam      if the replay is already playing, it drops the marker file svo_end.json that
                    replay.sh's own watch loop stops on - replay.sh then closes RTAB-Map FIRST and
                    waits for its database, then the rest, and still writes its summary.
                    If it is still starting up, this program stops the processes itself in the
                    same order: RTAB-Map first (waiting for it), then the inputs, then the rest.
          bench     stops the depth tool's process group (nothing there holds a database)
      - when the work ends: pictures and numbers (replay_figures.py), a copy of the final state in
        the history folder, and the lock file removed so the next job may start.

HOW TO RUN
    Started by replay_control_server.py:  setsid nice -n 19 python3 replay_job.py <job.json>
    By hand only for debugging, with the same argument.

INPUTS    the job description file (see replay_control_server.py, make_job())
OUTPUTS   the job's result folder ($REPLAY_RESULTS_DIR/<job id>/): console.log plus the tool's own
          outputs; the state file; the history copy; the progress line.
"""
import json
import os
import signal
import sqlite3
import subprocess
import sys
import time

import replay_common as rc
import replay_figures as figures

HERE = os.path.dirname(os.path.abspath(__file__))
SIDEWALK_ENV = rc.setting("SIDEWALK_ENV", "~/.sidewalk_env.sh")
REPLAY_SH = os.path.join(rc.REPO_ROOT, "run", "replay.sh")
COMPARE_PY = os.path.join(rc.REPO_ROOT, "catkin_ws", "src", "sidewalk_slam", "scripts", "replay_compare.py")
BENCH_PY = os.path.join(rc.REPO_ROOT, "catkin_ws", "src", "sidewalk_evaluation", "scripts",
                        "depth_benchmark", "svo_replay_n.py")
STATE_EVERY_S = 2.0
STOP_MARKER_WHY = "stopped by the Stop button on the replay page"


def db_tools_dir():
    """replay.sh needs db_to_tum.py. On the project Jetson it sits in $WORK_DIR/tools; in the
    public repository it is in tools/db."""
    explicit = os.environ.get("TOOLS_DIR")
    if explicit:
        return explicit
    work_tools = os.path.join(rc.WORK_DIR, "tools")
    if os.path.exists(os.path.join(work_tools, "db_to_tum.py")):
        return work_tools
    return os.path.join(rc.REPO_ROOT, "tools", "db")


class Job:
    def __init__(self, spec_path):
        self.spec = rc.read_json(spec_path)
        if not self.spec:
            sys.exit("cannot read the job description %s" % spec_path)
        self.id = self.spec["id"]
        self.kind = self.spec["kind"]
        self.params = self.spec["params"]
        self.out = self.spec["out_dir"]
        os.makedirs(self.out, exist_ok=True)
        self.log_path = os.path.join(self.out, "console.log")
        self.started = rc.now_s()
        self.state = dict(self.spec, state="starting", reason="", runner_pid=os.getpid(),
                          started_at=self.started, log=self.log_path, progress={}, result={})
        self.last_saved = 0.0
        self.stop_requested = False
        signal.signal(signal.SIGTERM, self.on_stop_signal)
        signal.signal(signal.SIGINT, self.on_stop_signal)

    # ------------------------------------------------------------ bookkeeping
    def on_stop_signal(self, *_):
        self.stop_requested = True

    def set(self, force=True, **fields):
        self.state.update(fields)
        if force or rc.now_s() - self.last_saved >= STATE_EVERY_S:
            self.last_saved = rc.now_s()
            rc.save_state(self.state)

    def say(self, text):
        with open(self.log_path, "a") as f:
            f.write("%s [replay page] %s\n" % (time.strftime("%F %T"), text))

    def progress_line(self, text):
        rc.write_line_atomic(rc.PROGRESS_FILE, "REPLAY_WEB %s %s" % (self.id, text))

    def elapsed(self):
        return int(rc.now_s() - self.started)

    def open_log(self):
        return open(self.log_path, "a")

    # ------------------------------------------------------------ stopping processes, by number
    def interrupt_and_wait(self, pid, sig=signal.SIGINT, timeout_s=None, label="process"):
        """Send `sig` to one process and wait until it has gone (timeout None = no deadline).
        Returns the seconds waited."""
        if not rc.is_alive(pid):
            return 0
        try:
            os.kill(pid, sig)
        except OSError:
            return 0
        waited = 0
        while rc.is_alive(pid) and (timeout_s is None or waited < timeout_s):
            time.sleep(1)
            waited += 1
            if waited % 30 == 0:
                self.say("still waiting for %s %d to close (%d s)" % (label, pid, waited))
        return waited

    def stop_replay_tree(self, root_pid):
        """Stop a replay that has not started playing yet, in the order replay.sh itself uses.
        The process list is taken BEFORE anything is stopped, because a stopped parent's
        children would otherwise be handed to another parent and be harder to find."""
        tree = rc.descendants_of(root_pid)
        names = {pid: rc.process_name(pid) for pid in tree}
        self.say("stopping before playback: %d processes under replay.sh" % len(tree))
        # 1. replay.sh itself, so it does not react to its children stopping
        self.interrupt_and_wait(root_pid, signal.SIGTERM, 10, "replay.sh")
        # 2. RTAB-Map first, with no deadline: its database must close properly
        for pid, name in names.items():
            if name == "rtabmap":
                waited = self.interrupt_and_wait(pid, signal.SIGINT, None, "RTAB-Map")
                self.say("RTAB-Map closed its database after %ds" % waited)
        # 3. the inputs (bag player, camera), then the launch files, then helpers, then the master
        order = [("play", "zed_wrapper_nod"), ("roslaunch",), ("python3", "python"), ("roscore", "rosmaster", "rosout")]
        for group in order:
            for pid, name in names.items():
                if name in group:
                    self.interrupt_and_wait(pid, signal.SIGINT, 90, name)
        # 4. anything still left gets a plain termination request
        for pid in tree:
            if rc.is_alive(pid):
                self.interrupt_and_wait(pid, signal.SIGTERM, 10, names.get(pid) or "process")

    # ------------------------------------------------------------ the live map picture
    def start_map_view(self):
        script = ('if [ -r "$1" ]; then . "$1"; else . /opt/ros/noetic/setup.bash; fi; '
                  'export ROS_MASTER_URI="http://localhost:$3"; exec python3 "$2" _port:="$4"')
        env = dict(os.environ, REPO_ROOT=rc.REPO_ROOT)
        log = open(os.path.join(self.out, "map_view.log"), "a")
        proc = subprocess.Popen(["bash", "-c", script, "map_view", SIDEWALK_ENV,
                                 os.path.join(HERE, "map_view_local.py"),
                                 str(rc.REPLAY_ROS_PORT), str(rc.MAP_VIEW_PORT)],
                                stdout=log, stderr=subprocess.STDOUT, stdin=subprocess.DEVNULL,
                                env=env, start_new_session=True)
        self.say("live map picture server started (process %d, port %d)" % (proc.pid, rc.MAP_VIEW_PORT))
        return proc

    def stop_map_view(self, proc):
        if proc and proc.poll() is None:
            self.interrupt_and_wait(proc.pid, signal.SIGINT, 10, "map picture server")
            if proc.poll() is None:
                proc.terminate()
            proc.wait()

    # ------------------------------------------------------------ kind 1: mapping replay
    def run_slam(self):
        p = self.params
        cmd = ["bash", REPLAY_SH, p["run"], "--bag", p["bag"], "--svo", p["svo"], "--out", self.out,
               "--depth-mode", p["depth_mode"]]
        if int(p.get("max_s") or 0) > 0:
            cmd += ["--max-s", str(int(p["max_s"]))]
        env = dict(os.environ, REPO_ROOT=rc.REPO_ROOT, RECORDS_DIR=rc.RECORDS_DIR, WORK_DIR=rc.WORK_DIR,
                   JOBS_DIR=rc.JOBS_DIR, REPLAY_ROS_PORT=str(rc.REPLAY_ROS_PORT), TOOLS_DIR=db_tools_dir())
        self.say("command: " + " ".join(cmd))
        proc = subprocess.Popen(cmd, stdout=self.open_log(), stderr=subprocess.STDOUT,
                                stdin=subprocess.DEVNULL, env=env)
        self.set(state="starting", child_pid=proc.pid,
                 reason="reading the camera recording's frame times, then opening it "
                        "(the depth model takes 10-30 s to load)")
        replay_progress = os.path.join(rc.JOBS_DIR, "%s_replay.progress" % p["run"])
        pids_file = os.path.join(self.out, "pids.txt")
        map_view = None
        stop_sent = False
        playing = False
        while proc.poll() is None:
            time.sleep(1)
            playing = os.path.exists(pids_file)
            if playing and self.state["state"] == "starting":
                self.set(state="running", reason="playing the recordings")
            if playing and map_view is None:
                map_view = self.start_map_view()
                self.set(map_view_pid=map_view.pid)
            line = self.read_fresh_line(replay_progress)
            played, total, _ = rc.progress_numbers(line)
            self.state["progress"] = {"line": line, "done": played, "total": total, "elapsed_s": self.elapsed()}
            self.progress_line("%s/%s s  %ds %s" % (int(played or 0), int(total or 0), self.elapsed(),
                                                    self.state["state"]))
            if self.stop_requested and not stop_sent:
                stop_sent = True
                self.set(state="stopping", reason="Stop pressed: closing RTAB-Map first, then the rest")
                if playing:
                    # replay.sh's watch loop checks for this file every 5 s and then stops in order
                    rc.write_json_atomic(os.path.join(self.out, "svo_end.json"), {"why": STOP_MARKER_WHY})
                    self.say("stop marker written; replay.sh will close RTAB-Map first")
                else:
                    self.stop_replay_tree(proc.pid)
            self.set(force=False)
        self.stop_map_view(map_view)
        code = proc.returncode
        self.say("replay.sh ended with exit code %s" % code)
        summary = rc.read_json(os.path.join(self.out, "replay_summary.json"))
        if summary:
            self.finish_slam(summary, stopped=stop_sent)
        elif stop_sent:
            self.set(state="done", reason="stopped before playback began - there are no results")
        else:
            self.set(state="failed", reason=self.failure_reason(code))

    def read_fresh_line(self, path):
        """A progress line, but only if it was written during this job (an old one is stale)."""
        try:
            if os.path.getmtime(path) < self.started:
                return ""
            with open(path) as f:
                return f.read().strip()
        except OSError:
            return ""

    def failure_reason(self, code):
        for line in reversed(rc.tail_lines(self.log_path, 60)):
            if "ERROR" in line or "missing:" in line or "Error" in line:
                return line.strip()
        return "replay.sh ended with exit code %s - see the log" % code

    def check_database(self, db_path):
        """Did RTAB-Map close its database cleanly? Asks SQLite to check the file (read-only) and
        looks for a leftover journal (a journal with content means an unfinished write)."""
        check = {"database": os.path.basename(db_path), "exists": os.path.exists(db_path)}
        if not check["exists"]:
            return check
        check["size_mb"] = round(os.path.getsize(db_path) / 1e6, 1)
        journal = db_path + "-journal"
        check["leftover_journal_bytes"] = os.path.getsize(journal) if os.path.exists(journal) else 0
        try:
            con = sqlite3.connect("file:%s?mode=ro" % db_path, uri=True)
            check["sqlite_quick_check"] = con.execute("PRAGMA quick_check").fetchone()[0]
            con.close()
        except sqlite3.Error as err:
            check["sqlite_quick_check"] = "could not check: %s" % err
        closed_lines = [l for l in rc.tail_lines(self.log_path, 200) if "closed its database" in l]
        check["log_says"] = closed_lines[-1].strip() if closed_lines else "no 'closed its database' line"
        check["clean"] = (check["sqlite_quick_check"] == "ok" and check["leftover_journal_bytes"] == 0)
        return check

    def original_dir(self):
        """The original drive's path files (camera.tum, camera_corrected.tum), if any are kept
        next to the recordings: <run folder>/original/ or one folder up."""
        run_dir = os.path.dirname(self.params["bag"])
        for folder in (os.path.join(run_dir, "original"), os.path.join(os.path.dirname(run_dir), "original")):
            if os.path.exists(os.path.join(folder, "camera.tum")):
                return folder
        return None

    def finish_slam(self, summary, stopped):
        result = {"summary": summary}
        result["database_check"] = self.check_database(os.path.join(self.out, "%s_replay.db" % self.params["run"]))
        original = self.original_dir()
        result["original_dir"] = original
        try:
            result["figure_note"] = figures.slam_trajectory_figure(self.out, original,
                                                                   os.path.join(self.out, "trajectory.png"))
            result["figure"] = "trajectory.png" if os.path.exists(os.path.join(self.out, "trajectory.png")) else None
        except Exception as err:          # a picture must never turn a good replay into a failed one
            result["figure_note"] = "picture failed: %s" % err
        if original and os.path.exists(COMPARE_PY):
            compare_out = os.path.join(self.out, "compare.json")
            env = dict(os.environ, PATH=os.path.expanduser("~/.local/bin") + ":" + os.environ.get("PATH", ""))
            subprocess.call(["nice", "-n", "19", "python3", COMPARE_PY, "--original", original,
                             "--replay", self.out, "--out", compare_out],
                            stdout=self.open_log(), stderr=subprocess.STDOUT, env=env)
            result["compare"] = rc.read_json(compare_out)
        why = summary.get("stopped_because", "")
        reason = ("stopped by the Stop button after %s s of playback" % summary.get("played_wall_s")
                  if stopped or STOP_MARKER_WHY in why else "finished: %s" % why)
        self.set(state="done", reason=reason, result=result)

    # ------------------------------------------------------------ kind 2: camera bench test
    @staticmethod
    def read_bench_counters(folder):
        """Add up the per-worker counter files svo_replay_n.py writes: 'shard done span accepted
        sparse nofit' (sparse = the target box came back nearly empty; nofit = points there, but
        no three spheres of the right size and layout)."""
        total = {"done": 0, "accepted": 0, "sparse": 0, "nofit": 0}
        try:
            names = [n for n in os.listdir(folder) if n.startswith("shard_") and not n.endswith(".tmp")]
        except OSError:
            return total
        for name in names:
            try:
                with open(os.path.join(folder, name)) as f:
                    _, done, _, accepted, sparse, nofit = [int(v) for v in f.read().split()[:6]]
            except (OSError, ValueError):
                continue
            total["done"] += done
            total["accepted"] += accepted
            total["sparse"] += sparse
            total["nofit"] += nofit
        return total

    def run_bench(self):
        p = self.params
        modes = p["modes"]
        planned_total = int(p["frames"]) * len(modes)
        counts, results, failures = {}, {}, []
        self.set(state="running", reason="replaying the camera recording", bench_counts=counts, result={})
        for index, mode in enumerate(modes, 1):
            if self.stop_requested:
                break
            counter_dir = os.path.join(self.out, "progress_%s" % mode)
            csv_path = os.path.join(self.out, "centres_%s.csv" % mode)
            cmd = ["python3", BENCH_PY, "--svo", p["svo"], "--mode", mode, "--roi-x", str(p["roi_x"]),
                   "--sensor-h", str(p["sensor_h"]), "--workers", str(p["workers"]), "--limit", str(p["frames"]),
                   "--progress-dir", counter_dir, "--depth-stabilization", str(p["stabilization"]),
                   "--out", csv_path]
            self.say("depth mode %s (%d of %d): %s" % (mode, index, len(modes), " ".join(cmd)))
            proc = subprocess.Popen(cmd, stdout=self.open_log(), stderr=subprocess.STDOUT,
                                    stdin=subprocess.DEVNULL, start_new_session=True)
            self.set(child_pid=proc.pid, reason="depth mode %s (%d of %d)" % (mode, index, len(modes)))
            stop_sent = False
            while proc.poll() is None:
                time.sleep(1)
                counts[mode] = dict(self.read_bench_counters(counter_dir), planned=int(p["frames"]))
                done_all = sum(c["done"] for c in counts.values())
                self.state["progress"] = {"done": done_all, "total": planned_total, "elapsed_s": self.elapsed(),
                                          "line": "%s: %d of %d frames" % (mode, counts[mode]["done"], int(p["frames"]))}
                self.progress_line("frames %d/%d (%s, mode %d of %d)  %ds"
                                   % (done_all, planned_total, mode, index, len(modes), self.elapsed()))
                if self.stop_requested and not stop_sent:
                    stop_sent = True
                    self.set(state="stopping", reason="Stop pressed: stopping the depth tool")
                    # its own process group: the tool and its worker processes, nothing else
                    os.killpg(proc.pid, signal.SIGTERM)
                self.set(force=False, bench_counts=counts)
            counts[mode] = dict(self.read_bench_counters(counter_dir), planned=int(p["frames"]))
            if proc.returncode != 0 and not self.stop_requested:
                failures.append("%s: the depth tool ended with exit code %s" % (mode, proc.returncode))
            summary = figures.bench_mode_summary(csv_path, counts[mode]["done"])
            summary.update(sparse=counts[mode]["sparse"], nofit=counts[mode]["nofit"])
            scatter = "scatter_%s.png" % mode
            figures.bench_scatter_figure(csv_path, mode, counts[mode]["done"], os.path.join(self.out, scatter))
            summary["figure"] = scatter
            results[mode] = summary
            rc.write_json_atomic(os.path.join(self.out, "bench_summary.json"), results)
            self.set(bench_counts=counts, result={"modes": results})
        if failures:
            self.set(state="failed", reason="; ".join(failures))
        elif self.stop_requested:
            self.set(state="done", reason="stopped by the Stop button; the modes finished so far are shown")
        else:
            self.set(state="done", reason="all %d depth modes replayed" % len(modes))

    # ------------------------------------------------------------ the whole job
    def run(self):
        self.say("job %s (%s) started, runner process %d" % (self.id, self.kind, os.getpid()))
        self.set(state="starting")
        try:
            {"slam": self.run_slam, "bench": self.run_bench}[self.kind]()
        except Exception as err:          # record it, never leave the page saying "running"
            self.say("the runner hit an error: %r" % err)
            self.set(state="failed", reason="the job runner hit an error: %s" % err)
        finally:
            self.state["ended_at"] = rc.now_s()
            if self.state.get("state") not in rc.FINAL_STATES:
                self.state["state"] = "failed"
            self.set()
            rc.write_json_atomic(os.path.join(rc.HISTORY_DIR, "%s.json" % self.id), self.state)
            self.progress_line("%s %s  %ds" % (self.state["state"].upper(), self.state.get("reason", ""),
                                               self.elapsed()))
            if rc.lock_holder() == self.id:
                os.remove(rc.LOCK_FILE)
            self.say("job ended: %s - %s" % (self.state["state"], self.state.get("reason", "")))


if __name__ == "__main__":
    if len(sys.argv) != 2:
        sys.exit(__doc__)
    Job(sys.argv[1]).run()
