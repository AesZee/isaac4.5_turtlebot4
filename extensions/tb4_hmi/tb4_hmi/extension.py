"""TurtleBot4 HMI — Isaac Sim extension (omni.ext.IExt).

A dockable omni.ui panel that mimics the Create 3 HMI: a 6-segment light ring, a
battery readout, dock/undock action buttons, teleop nudge buttons, and an E-STOP.
All robot I/O goes through :class:`Tb4HmiRos` (rclpy on the sim's domain-0 DDS).

Threading: the rclpy executor spins on its own daemon thread; this module only
touches omni.ui from the Kit update-event callback (main thread) and reads a
lock-guarded snapshot each frame. See ros_node.py for the contract.

Light-ring mapping (Create 3 parity):
  E-STOP active ............ red, fast pulse
  no data / connecting ..... dim blue, solid
  low battery (<15%), not charging .. amber, pulse
  docked + charging ........ green, rotating "comet"
  docked + full ............ green, solid
  undocked / idle .......... white, solid
"""
import math
import time

import omni.ext
import omni.ui as ui
import omni.kit.app

from .ros_node import Tb4HmiRos

# ── tuning ───────────────────────────────────────────────────────────────────
NUDGE_LIN = 0.15      # m/s   (well under the 0.31 cap)
NUDGE_ANG = 0.8       # rad/s (well under the 1.9 cap)
NUDGE_SECS = 0.6      # how long one tap keeps publishing /cmd_vel (watchdog = 0.5 s)
LOW_BATT = 0.15       # fraction below which the ring warns
RING_PUB_PERIOD = 0.5 # s between /cmd_lightring publishes
NUM_LEDS = 6

# BatteryState.POWER_SUPPLY_STATUS_* (avoid importing the msg here)
_BATT_CHARGING, _BATT_FULL = 1, 4


def _abgr(r, g, b, bright=1.0):
    """Pack to omni.ui's 0xAABBGGRR int, scaling RGB by ``bright`` (0..1)."""
    r = max(0, min(255, int(r * bright)))
    g = max(0, min(255, int(g * bright)))
    b = max(0, min(255, int(b * bright)))
    return 0xFF000000 | (b << 16) | (g << 8) | r


class Tb4HmiExtension(omni.ext.IExt):
    WINDOW_TITLE = "TurtleBot4 HMI"

    # ── lifecycle ────────────────────────────────────────────────────────────
    def on_startup(self, ext_id):
        self._estop = False
        self._nudge = (0.0, 0.0)
        self._nudge_until = 0.0
        self._t0 = time.monotonic()
        self._last_ring_pub = 0.0
        self._window = None
        self._leds = []

        self._ros = None
        try:
            self._ros = Tb4HmiRos()
        except Exception as e:  # noqa: BLE001
            print(f"[tb4_hmi] ROS bridge failed to start: {e!r} — panel runs UI-only.")

        self._build_window()
        self._update_sub = omni.kit.app.get_app().get_update_event_stream(
        ).create_subscription_to_pop(self._on_update, name="tb4_hmi.update")

    def on_shutdown(self):
        self._update_sub = None
        if self._ros is not None:
            try:
                self._ros.stop()
            except Exception:  # noqa: BLE001
                pass
            self._ros.shutdown()
            self._ros = None
        if self._window is not None:
            self._window.destroy()
            self._window = None
        self._leds = []

    # ── UI ───────────────────────────────────────────────────────────────────
    def _build_window(self):
        self._window = ui.Window(self.WINDOW_TITLE, width=320, height=440)
        with self._window.frame:
            with ui.VStack(spacing=8, height=0):
                ui.Label("TurtleBot4 HMI", height=22,
                         style={"font_size": 18, "color": 0xFFDDDDDD})

                # ── light ring (6 segments) ──
                with ui.HStack(height=44, spacing=6):
                    ui.Spacer()
                    for _ in range(NUM_LEDS):
                        self._leds.append(ui.Rectangle(
                            width=32, height=32,
                            style={"background_color": _abgr(20, 20, 20),
                                   "border_radius": 16,
                                   "border_color": 0xFF555555, "border_width": 1}))
                    ui.Spacer()
                self._ring_label = ui.Label("ring: connecting…", height=18,
                                            style={"color": 0xFF999999})

                ui.Spacer(height=6)
                self._batt_label = ui.Label("Battery:  —", height=20,
                                            style={"font_size": 15})
                self._dock_label = ui.Label("Dock:     —", height=20)
                self._action_label = ui.Label("Action:   idle", height=20,
                                              style={"color": 0xFF99CCFF})

                ui.Spacer(height=6)
                with ui.HStack(height=32, spacing=6):
                    ui.Button("Undock", clicked_fn=self._on_undock)
                    ui.Button("Dock", clicked_fn=self._on_dock)

                ui.Label("Teleop  (tap = brief nudge)", height=18,
                         style={"color": 0xFF999999})
                with ui.HStack(height=30, spacing=6):
                    ui.Spacer()
                    ui.Button("▲ Fwd", width=90, clicked_fn=lambda: self._do_nudge(NUDGE_LIN, 0.0))
                    ui.Spacer()
                with ui.HStack(height=30, spacing=6):
                    ui.Button("◄ Left", width=90, clicked_fn=lambda: self._do_nudge(0.0, NUDGE_ANG))
                    ui.Button("■ Stop", width=70, clicked_fn=self._do_stop)
                    ui.Button("Right ►", width=90, clicked_fn=lambda: self._do_nudge(0.0, -NUDGE_ANG))
                with ui.HStack(height=30, spacing=6):
                    ui.Spacer()
                    ui.Button("▼ Back", width=90, clicked_fn=lambda: self._do_nudge(-NUDGE_LIN, 0.0))
                    ui.Spacer()

                ui.Spacer(height=6)
                self._estop_btn = ui.Button(
                    "E-STOP", height=40, clicked_fn=self._toggle_estop,
                    style={"background_color": 0xFF2222CC, "font_size": 16})

    # ── button handlers (main thread) ────────────────────────────────────────
    def _on_dock(self):
        if self._ros:
            self._ros.send_dock()

    def _on_undock(self):
        if self._ros:
            self._ros.send_undock()

    def _do_nudge(self, lin, ang):
        if self._estop:
            return
        self._nudge = (lin, ang)
        self._nudge_until = time.monotonic() + NUDGE_SECS

    def _do_stop(self):
        self._nudge_until = 0.0
        if self._ros:
            self._ros.stop()

    def _toggle_estop(self):
        self._estop = not self._estop
        self._nudge_until = 0.0
        if self._ros:
            self._ros.stop()
        if self._estop_btn:
            self._estop_btn.text = "E-STOP ENGAGED — click to release" if self._estop else "E-STOP"
            self._estop_btn.style = {
                "background_color": 0xFF0000FF if self._estop else 0xFF2222CC,
                "font_size": 16}

    # ── per-frame update (main thread) ───────────────────────────────────────
    def _on_update(self, _e):
        now = time.monotonic()

        # 1) motion: E-STOP wins, then an active nudge keeps the watchdog fed
        if self._ros:
            if self._estop:
                self._ros.stop()
            elif now < self._nudge_until:
                self._ros.drive(*self._nudge)

        # 2) light ring from current state
        snap = self._ros.snapshot() if self._ros else {}
        base, mode, text = self._ring_state(snap)
        self._paint_ring(base, mode, now - self._t0)

        # 3) periodic /cmd_lightring publish (representative solid base color)
        if self._ros and now - self._last_ring_pub >= RING_PUB_PERIOD:
            self._last_ring_pub = now
            try:
                self._ros.publish_lightring([base] * NUM_LEDS, override=True)
            except Exception:  # noqa: BLE001
                pass

        # 4) text readouts
        self._ring_label.text = f"ring: {text}"
        pct = snap.get("battery_pct")
        st = snap.get("battery_status")
        st_txt = {1: "charging", 2: "discharging", 3: "not charging",
                  4: "full"}.get(st, "—")
        self._batt_label.text = (f"Battery:  {pct * 100:5.1f}%   ({st_txt})"
                                 if pct is not None else "Battery:  —")
        dk = snap.get("is_docked")
        self._dock_label.text = ("Dock:     docked" if dk else
                                 "Dock:     undocked" if dk is False else "Dock:     —")
        self._action_label.text = f"Action:   {snap.get('action', 'idle')}"

    # ── ring helpers ─────────────────────────────────────────────────────────
    def _ring_state(self, snap):
        """Return (base_rgb, mode, text). mode ∈ {solid, pulse, spin}."""
        if self._estop:
            return (255, 0, 0), "pulse", "E-STOP"
        if not snap or snap.get("is_docked") is None and snap.get("battery_pct") is None:
            return (0, 40, 90), "solid", "connecting…"
        pct = snap.get("battery_pct")
        st = snap.get("battery_status")
        charging = st in (_BATT_CHARGING, _BATT_FULL)
        if pct is not None and pct < LOW_BATT and not charging:
            return (255, 90, 0), "pulse", "low battery"
        if snap.get("is_docked"):
            if st == _BATT_CHARGING:
                return (0, 200, 40), "spin", "docked · charging"
            return (0, 200, 40), "solid", "docked"
        return (210, 210, 210), "solid", "undocked · idle"

    def _paint_ring(self, base, mode, t):
        r, g, b = base
        if mode == "pulse":
            br = 0.35 + 0.65 * (0.5 + 0.5 * math.sin(2 * math.pi * 0.9 * t))
            for led in self._leds:
                led.style = self._led_style(r, g, b, br)
        elif mode == "spin":
            head = int(t * 3.0) % NUM_LEDS
            for i, led in enumerate(self._leds):
                d = min((i - head) % NUM_LEDS, (head - i) % NUM_LEDS)
                br = {0: 1.0, 1: 0.5}.get(d, 0.22)
                led.style = self._led_style(r, g, b, br)
        else:  # solid
            for led in self._leds:
                led.style = self._led_style(r, g, b, 1.0)

    @staticmethod
    def _led_style(r, g, b, br):
        return {"background_color": _abgr(r, g, b, br), "border_radius": 16,
                "border_color": 0xFF555555, "border_width": 1}
