# BMCU Klipper "extras" module (raw-serial JSON transport)
#
# Drop this file into: ~/klipper/klippy/extras/bmcu.py
#
# Minimal printer.cfg:
#
#   [bmcu]
#   serial: /dev/serial/by-id/usb-1a86_USB_Serial-if00-port0
#   baud: 250000
#
# Optional tuning keys:
#   timeout, poll_interval, read_interval, debug,
#   line_ending, tx_rx_mode,
#   connect_flush, connect_flush_delay, connect_drain_s,
#   connect_set_dtr, connect_dtr_low, connect_dtr_settle,
#   connect_set_rts, connect_rts_low,
#   default_speed, max_move_mm, max_speed,
#   default_lane_feed_mm, default_lane_retract_mm,
#   supported_cmds
#
# Firmware surface (per KlipperCLI.cpp):
#   PING, STATUS, GET_SENSORS, MOVE, STOP, SELECT_LANE,
#   SET_AUTO_FEED, GET_FILAMENT_INFO, SET_FILAMENT_INFO

import logging
import json
import time

import serial  # pyserial


class BMCU:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.reactor = self.printer.get_reactor()
        self.gcode = self.printer.lookup_object('gcode')

        # -----------------------------
        # Config
        # -----------------------------
        self.serial_port = config.get('serial')
        self.baud = config.getint('baud', 115200)  # Reduced from 250000
        self.timeout = config.getfloat('timeout', 0.1)

        # Gentler defaults
        self.poll_interval = config.getfloat('poll_interval', 10.0)
        self.read_interval = config.getfloat('read_interval', 0.05)

        # Quiet by default; turn on in cfg when needed
        self.debug = config.getboolean('debug', False)

        # Line ending for JSON packets
        self.line_ending = self._normalize_line_ending(config.get('line_ending', 'LF'))

        # TX/RX mode: "full" or "halfduplex"
        self.tx_rx_mode = (config.get("tx_rx_mode", "halfduplex") or "halfduplex").lower()

        # Connect hygiene
        self.connect_flush = config.getboolean('connect_flush', True)
        self.connect_flush_delay = config.getfloat('connect_flush_delay', 0.2)
        self.connect_drain_s = config.getfloat('connect_drain_s', 0.2)

        # DTR/RTS behavior (baked to your working defaults)
        self.connect_set_dtr = config.getboolean('connect_set_dtr', False)
        self.connect_dtr_low = config.getboolean('connect_dtr_low', True)
        self.connect_dtr_settle = config.getfloat('connect_dtr_settle', 0.10)

        self.connect_set_rts = config.getboolean('connect_set_rts', True)
        self.connect_rts_low = config.getboolean('connect_rts_low', True)

        # Motion limits
        self.default_speed = config.getfloat('default_speed', 20.0)
        self.max_move_mm = config.getfloat('max_move_mm', 1200.0)
        self.max_speed = config.getfloat('max_speed', 200.0)

        self.default_lane_feed_mm = config.getfloat('default_lane_feed_mm', 60.0)
        self.default_lane_retract_mm = config.getfloat('default_lane_retract_mm', 60.0)

        # Optional allowlist of commands
        supported = config.get('supported_cmds', '').strip()
        if supported:
            self.supported_cmds = {c.strip() for c in supported.split(',') if c.strip()}
        else:
            self.supported_cmds = set()

        # -----------------------------
        # State
        # -----------------------------
        self.ser = None
        self.is_connected = False
        self._buf = ""
        self._last_connect_attempt = 0.0
        self._connect_backoff = 1.0

        self.lanes = None
        self.last_rx = None
        self.last_rx_by_id = {}

        # Have we already kicked a STATUS after STARTUP?
        self._did_startup_status = False

        # Events
        self.printer.register_event_handler("klippy:shutdown", self._handle_shutdown)

        # Expose BMCU lane data to Moonraker and Klipper clients via
        # status callbacks.  We attempt to register a callback with
        # the Moonraker webhooks API, but also implement a
        # get_status() method for Klipper's native status polling.
        # Some installs may not have a webhooks component or the
        # register_status_callback method; in those cases get_status()
        # will still expose lane data.
        try:
            self.webhooks = self.printer.lookup_object('webhooks')
            if hasattr(self.webhooks, 'register_status_callback'):
                # Wrap get_status() so that the callback returns a
                # dictionary keyed by the module name.  Moonraker will
                # merge this into the printer status under 'bmcu'.
                def _bmcu_status_callback(eventtime):
                    return {'bmcu': self.get_status(eventtime)}
                self.webhooks.register_status_callback(_bmcu_status_callback)
        except Exception:
            # Gracefully ignore if webhooks or its status API is unavailable.
            pass
        # G-code commands
        self._register_gcode()

        # Timers
        now = self.reactor.NOW
        self._read_timer = self.reactor.register_timer(self._handle_read, now + self.read_interval)
        self._poll_timer = self.reactor.register_timer(self._handle_poll, now + self.poll_interval)

        if self.debug:
            logging.info("BMCU: initialized (deferred connect) serial=%s baud=%d",
                         self.serial_port, self.baud)

    # -----------------------------
    # G-code registration
    # -----------------------------
    def _register_gcode(self):
        gc = self.gcode
        gc.register_command("BMCU_CAPS", self.cmd_BMCU_CAPS)
        gc.register_command("BMCU_PING", self.cmd_BMCU_PING)
        gc.register_command("BMCU_STATUS", self.cmd_BMCU_STATUS)
        gc.register_command("BMCU_GET_SENSORS", self.cmd_BMCU_GET_SENSORS)
        gc.register_command("BMCU_STOP", self.cmd_BMCU_STOP)
        gc.register_command("BMCU_SELECT_LANE", self.cmd_BMCU_SELECT_LANE)
        gc.register_command("BMCU_SET_AUTO_FEED", self.cmd_BMCU_SET_AUTO_FEED)
        gc.register_command("BMCU_MOVE", self.cmd_BMCU_MOVE)
        gc.register_command("BMCU_FEED", self.cmd_BMCU_FEED)
        gc.register_command("BMCU_SELECTOR", self.cmd_BMCU_SELECTOR)
        gc.register_command("BMCU_SPOOL", self.cmd_BMCU_SPOOL)
        gc.register_command("BMCU_GET_FILAMENT_INFO", self.cmd_BMCU_GET_FILAMENT_INFO)
        gc.register_command("BMCU_SET_FILAMENT_INFO", self.cmd_BMCU_SET_FILAMENT_INFO)
        gc.register_command("BMCU_CALL", self.cmd_BMCU_CALL)
        gc.register_command("BMCU_LANE_FEED", self.cmd_BMCU_LANE_FEED)
        gc.register_command("BMCU_LANE_RETRACT", self.cmd_BMCU_LANE_RETRACT)

    # -----------------------------
    # Timers
    # -----------------------------
    def _handle_poll(self, eventtime):
        if not self.is_connected:
            self._maybe_connect(eventtime)
            return eventtime + max(self.poll_interval, 0.5)

        # Lightweight health check: PING only
        self._send_pkt("PING", {}, note="poll")

        if self.tx_rx_mode == 'halfduplex':
            self._pump_rx(0.05)

        return eventtime + self.poll_interval

    def _handle_read(self, eventtime):
        if not self.is_connected or self.ser is None:
            return eventtime + max(self.read_interval, 0.1)
        try:
            data = self.ser.read(4096)
            if data:
                if self.debug:
                    logging.info("BMCU RX chunk: len=%d hex=%s...",
                                 len(data), data[:16].hex())
                self._buf += data.decode('utf-8', errors='ignore')

                while '\n' in self._buf or '\r' in self._buf:
                    idx_n = self._buf.find('\n')
                    idx_r = self._buf.find('\r')
                    idxs = [i for i in (idx_n, idx_r) if i != -1]
                    if not idxs:
                        break
                    idx = min(idxs)
                    line = self._buf[:idx]
                    self._buf = self._buf[idx+1:]
                    line = line.strip()
                    if line:
                        self._process_line(line)

                if len(self._buf) > 4096:
                    logging.error("BMCU: Buffer overflow (>4k), clearing")
                    self._buf = ""
        except Exception as e:
            logging.error("BMCU: read error: %s", e)
            self._disconnect()
        return eventtime + self.read_interval

    # -----------------------------
    # Connect / disconnect
    # -----------------------------
    def _maybe_connect(self, eventtime):
        if (eventtime - self._last_connect_attempt) < self._connect_backoff:
            return
        self._last_connect_attempt = eventtime
        try:
            self._connect()
            self._connect_backoff = 1.0
        except Exception as e:
            if self.debug:
                logging.warning("BMCU: connect failed: %s", e)
            self._connect_backoff = min(self._connect_backoff * 2.0, 30.0)

    def _normalize_line_ending(self, val):
        if val is None:
            return "\n"
        v = str(val).strip().upper()
        if v in ("LF", "\\N", "N", "NEWLINE"):
            return "\n"
        if v in ("CRLF", "\\R\\N", "RN", "WINDOWS"):
            return "\r\n"
        if v in ("CR", "\\R"):
            return "\r"
        raw = str(val)
        if "\r\n" in raw:
            return "\r\n"
        if "\n" in raw:
            return "\n"
        if "\r" in raw:
            return "\r"
        return "\n"

    def _connect(self):
        self.ser = serial.Serial(self.serial_port, self.baud, timeout=self.timeout)
        try:
            if self.connect_set_dtr:
                self.ser.dtr = (False if self.connect_dtr_low else True)
        except Exception:
            pass
        try:
            if self.connect_set_rts:
                self.ser.rts = (False if self.connect_rts_low else True)
        except Exception:
            pass
        try:
            time.sleep(max(0.0, float(self.connect_dtr_settle)))
        except Exception:
            pass

        if self.connect_flush:
            try:
                self.ser.reset_input_buffer()
            except Exception:
                pass
            try:
                self.ser.reset_output_buffer()
            except Exception:
                pass
            try:
                time.sleep(max(0.0, float(self.connect_flush_delay)))
            except Exception:
                pass
            try:
                old_to = self.ser.timeout
                self.ser.timeout = 0.0
                t_end = time.time() + max(0.0, float(self.connect_drain_s))
                while time.time() < t_end:
                    data = self.ser.read(4096)
                    if not data:
                        break
                self.ser.timeout = old_to
            except Exception:
                pass

        try:
            self.reactor.pause(self.reactor.monotonic() + 0.12)
        except Exception:
            pass
        self.is_connected = True
        self._did_startup_status = False
        if self.debug:
            logging.info("BMCU: connected on %s @ %d", self.serial_port, self.baud)

    def _disconnect(self):
        self.is_connected = False
        if self.ser is not None:
            try:
                self.ser.close()
            except Exception:
                pass
        self.ser = None

    def _handle_shutdown(self):
        self._disconnect()

    # -----------------------------
    # Protocol helpers
    # -----------------------------
    def _pump_rx(self, budget_s: float = 0.05) -> None:
        if not self.is_connected or self.ser is None:
            return
        try:
            old_to = self.ser.timeout
            self.ser.timeout = 0.0
        except Exception:
            old_to = None
        t_end = time.time() + max(0.0, float(budget_s))
        try:
            while time.time() < t_end:
                data = self.ser.read(4096)
                if not data:
                    break
                self._buf += data.decode('utf-8', errors='ignore')
                while '\n' in self._buf or '\r' in self._buf:
                    idx_n = self._buf.find('\n')
                    idx_r = self._buf.find('\r')
                    idxs = [i for i in (idx_n, idx_r) if i != -1]
                    if not idxs:
                        break
                    idx = min(idxs)
                    line = self._buf[:idx]
                    self._buf = self._buf[idx+1:]
                    line = line.strip()
                    if line:
                        self._process_line(line)
        except Exception:
            pass
        finally:
            try:
                if old_to is not None:
                    self.ser.timeout = old_to
            except Exception:
                pass

    def _next_id(self) -> int:
        # Match your UI behavior (0..9999)
        return int(time.time() * 1000) % 10000

    def _cmd_allowed(self, cmd):
        return (not self.supported_cmds) or (cmd in self.supported_cmds)

    def _send_pkt(self, cmd, args, note=None):
        if not self.is_connected or self.ser is None:
            return False, None
        if not self._cmd_allowed(cmd):
            return False, None
        pkt_id = self._next_id()
        pkt = {"id": pkt_id, "cmd": cmd, "args": args}
        try:
            msg = json.dumps(pkt) + self.line_ending
            if self.debug:
                logging.info("BMCU TX(%s): %s", note or "pkt", msg.strip())
            self.ser.write(msg.encode('utf-8'))
            return True, pkt_id
        except Exception as e:
            logging.error("BMCU: send error: %s", e)
            self._disconnect()
            return False, None

    def _process_line(self, line):
        # Junk-tolerant: handle 'UN{...}' and similar noise
        if not isinstance(line, str):
            line = str(line)
        raw_line = line

        # Strip leading junk before first { or [
        idx_obj = line.find("{")
        idx_arr = line.find("[")
        idxs = [i for i in (idx_obj, idx_arr) if i != -1]
        if idxs:
            start = min(idxs)
            if start > 0:
                line = line[start:]

        # Trim trailing junk after last } or ]
        last_obj = line.rfind("}")
        last_arr = line.rfind("]")
        last_idxs = [i for i in (last_obj, last_arr) if i != -1]
        if last_idxs:
            end = max(last_idxs) + 1
            if end < len(line):
                line = line[:end]

        if self.debug:
            logging.info("BMCU RX: %s", line[:300])

        try:
            pkt = json.loads(line)
            self.last_rx = pkt

            # Special: firmware startup event
            if isinstance(pkt, dict) and pkt.get("event") == "STARTUP":
                if self.debug:
                    logging.info("BMCU: got STARTUP event, requesting STATUS once")
                if not self._did_startup_status:
                    self._did_startup_status = True
                    self._send_pkt("STATUS", {}, note="startup_status")
                return

            if isinstance(pkt, dict) and "id" in pkt:
                try:
                    rx_id = int(pkt["id"])
                    self.last_rx_by_id[rx_id] = pkt
                    if len(self.last_rx_by_id) > 50:
                        for k in list(self.last_rx_by_id.keys())[:10]:
                            self.last_rx_by_id.pop(k, None)
                except Exception:
                    pass

            if isinstance(pkt, dict) and "lanes" in pkt:
                self.lanes = pkt.get("lanes")

        except json.JSONDecodeError:
            if self.debug:
                logging.warning("BMCU: non-json line: %r", raw_line[:240])
        except Exception as e:
            logging.error("BMCU: parse error: %s", e)

    def _clamp(self, v, lo, hi):
        return max(lo, min(hi, v))

    # -----------------------------
    # Wait for reply helper
    # -----------------------------
    def _wait_for_reply(self, gcmd, pkt_id, wait_s):
        if not wait_s or wait_s <= 0:
            return
        wait_s = max(0.0, min(wait_s, 5.0))
        end = self.reactor.monotonic() + wait_s
        while self.reactor.monotonic() < end:
            self.reactor.pause(self.reactor.monotonic() + 0.05)
            if pkt_id is not None and pkt_id in self.last_rx_by_id:
                gcmd.respond_info("BMCU: reply: " + json.dumps(self.last_rx_by_id[pkt_id])[:1200])
                return
        gcmd.respond_info(f"BMCU: no reply within {wait_s:.2f}s (id={pkt_id})")

    # -----------------------------
    # Status callback / reporting
    # -----------------------------
    def get_status(self, eventtime=None):
        """
        Provide current lane state for status queries.

        This method returns a dictionary of fields that will be
        merged into printer.bmcu by Klipper's status machinery.
        It should contain only module‑specific values; Klipper will
        handle prefixing with the module name.  If the BMCU has not
        yet reported any lanes (STATUS frame not received), return an
        empty list.

        The eventtime parameter is ignored but preserved for API
        compatibility with Moonraker and Klipper.
        """
        lanes = self.lanes if isinstance(self.lanes, list) else []
        return {
            'lanes': lanes
        }

    # Preserve the old _get_status for backward compatibility; delegate to get_status.
    def _get_status(self, eventtime=None):
        """
        Backward‑compatibility wrapper that returns a dict keyed by
        'bmcu'.  This is retained for older versions of Moonraker
        which might still invoke the old callback signature.
        """
        return {'bmcu': self.get_status(eventtime)}

    # -----------------------------
    # GCODE COMMANDS
    # -----------------------------
    def cmd_BMCU_CAPS(self, gcmd):
        if not self.supported_cmds:
            gcmd.respond_info("BMCU_CAPS: all commands allowed (no allowlist).")
        else:
            gcmd.respond_info("Allowed commands: " + ", ".join(sorted(self.supported_cmds)))

    def cmd_BMCU_PING(self, gcmd):
        wait_s = gcmd.get_float("WAIT", 0.0)
        ok, pkt_id = self._send_pkt("PING", {}, note="ping")
        gcmd.respond_info(f"Pinging... id={pkt_id}")
        if ok:
            self._wait_for_reply(gcmd, pkt_id, wait_s)

    def cmd_BMCU_STATUS(self, gcmd):
        if self.lanes is None:
            gcmd.respond_info("No cached STATUS yet.")
            return
        out = ["Cached STATUS:"]
        for i, lane in enumerate(self.lanes):
            out.append(f"  Lane {i}: {lane}")
        gcmd.respond_info("\n".join(out))

    def cmd_BMCU_GET_SENSORS(self, gcmd):
        wait_s = gcmd.get_float("WAIT", 0.5)
        ok, pkt_id = self._send_pkt("GET_SENSORS", {}, note="sensors")
        gcmd.respond_info(f"GET_SENSORS sent id={pkt_id}")
        if ok:
            self._wait_for_reply(gcmd, pkt_id, wait_s)

    def cmd_BMCU_STOP(self, gcmd):
        ok, pkt_id = self._send_pkt("STOP", {}, note="stop")
        gcmd.respond_info(f"STOP sent id={pkt_id}")

    def cmd_BMCU_SELECT_LANE(self, gcmd):
        lane = gcmd.get_int("LANE", 0)
        wait_s = gcmd.get_float("WAIT", 0.3)
        ok, pkt_id = self._send_pkt("SELECT_LANE", {"lane": lane}, note="select")
        gcmd.respond_info(f"SELECT_LANE lane={lane} id={pkt_id}")
        if ok:
            self._wait_for_reply(gcmd, pkt_id, wait_s)

    def cmd_BMCU_SET_AUTO_FEED(self, gcmd):
        lane = gcmd.get_int("LANE", 0)
        enable = bool(gcmd.get_int("ENABLE", 1))
        ok, pkt_id = self._send_pkt("SET_AUTO_FEED", {"lane": lane, "enable": enable}, note="auto_feed")
        gcmd.respond_info(f"AUTO_FEED lane={lane} -> {enable}")

    def cmd_BMCU_MOVE(self, gcmd):
        axis = gcmd.get("AXIS", "0")
        dist = gcmd.get_float("DIST", 0.0)
        speed = gcmd.get_float("SPEED", self.default_speed)
        wait_s = gcmd.get_float("WAIT", 0.0)
        dist = self._clamp(dist, -self.max_move_mm, self.max_move_mm)
        speed = self._clamp(speed, -self.max_speed, self.max_speed)
        ok, pkt_id = self._send_pkt(
            "MOVE",
            {"axis": str(axis), "dist_mm": float(dist), "speed": float(speed)},
            note="move"
        )
        gcmd.respond_info(f"MOVE axis={axis} dist={dist} speed={speed}")
        if ok:
            self._wait_for_reply(gcmd, pkt_id, wait_s)

    def cmd_BMCU_FEED(self, gcmd):
        mm = gcmd.get_float("MM", self.default_lane_feed_mm)
        speed = gcmd.get_float("SPEED", self.default_speed)
        ok, pkt_id = self._send_pkt(
            "MOVE",
            {"axis": "FEED", "dist_mm": float(mm), "speed": float(speed)}, note="feed")
        gcmd.respond_info(f"FEED {mm}mm @ {speed}")

    def cmd_BMCU_SELECTOR(self, gcmd):
        mm = gcmd.get_float("MM", 10.0)
        speed = gcmd.get_float("SPEED", self.default_speed)
        ok, pkt_id = self._send_pkt(
            "MOVE",
            {"axis": "SELECTOR", "dist_mm": float(mm), "speed": float(speed)}, note="selector")
        gcmd.respond_info(f"SELECTOR: {mm}mm")

    def cmd_BMCU_SPOOL(self, gcmd):
        lane = gcmd.get_int("LANE", 0)
        mm = gcmd.get_float("MM", 20.0)
        speed = gcmd.get_float("SPEED", self.default_speed)
        axis = f"SPOOL{lane}"
        ok, pkt_id = self._send_pkt(
            "MOVE",
            {"axis": axis, "dist_mm": float(mm), "speed": float(speed)}, note="spool")
        gcmd.respond_info(f"SPOOL lane={lane} mm={mm}")

    def cmd_BMCU_GET_FILAMENT_INFO(self, gcmd):
        lane = gcmd.get_int("LANE", 0)
        ok, pkt_id = self._send_pkt("GET_FILAMENT_INFO", {"lane": lane}, note="get_fil_info")
        gcmd.respond_info(f"GET_FILAMENT_INFO lane={lane}")

    def cmd_BMCU_SET_FILAMENT_INFO(self, gcmd):
        lane = gcmd.get_int("LANE", 0)
        name = gcmd.get("NAME", None)
        tmin_raw = gcmd.get("TEMP_MIN", None)
        tmax_raw = gcmd.get("TEMP_MAX", None)
        rfid = gcmd.get("ID", None) or gcmd.get("RFID", None)
        meters_raw = gcmd.get("METERS", None)
        color_hex = gcmd.get("COLOR", None)
        color_hex_plain = gcmd.get("COLOR_HEX", None)
        color_r = gcmd.get("COLOR_R", None)
        color_g = gcmd.get("COLOR_G", None)
        color_b = gcmd.get("COLOR_B", None)

        args = {"lane": lane}

        if name is not None:
            args["name"] = name

        if tmin_raw is not None:
            args["temp_min"] = int(float(tmin_raw))

        if tmax_raw is not None:
            args["temp_max"] = int(float(tmax_raw))

        if rfid:
            s = str(rfid)[:8]
            args["rfid"] = s
            args["id_str"] = s   # keep both, firmware-safe

        if meters_raw is not None:
            meters_val = float(meters_raw)
            if meters_val >= 0:
                args["meters"] = meters_val

        color = None
        # Prefer explicit component params if provided
        try:
            if color_r is not None or color_g is not None or color_b is not None:
                r = self._clamp(int(float(color_r or 0)), 0, 255)
                g = self._clamp(int(float(color_g or 0)), 0, 255)
                b = self._clamp(int(float(color_b or 0)), 0, 255)
                color = [r, g, b, 255]
        except Exception:
            pass

        # Otherwise fall back to hex (with or without a leading #)
        if color is None and isinstance(color_hex, str):
            hex_str = color_hex.strip()
            if hex_str.startswith("#"):
                hex_str = hex_str[1:]
            if len(hex_str) == 6:
                try:
                    r = int(hex_str[0:2], 16)
                    g = int(hex_str[2:4], 16)
                    b = int(hex_str[4:6], 16)
                    color = [r, g, b, 255]
                except Exception:
                    color = None

        if color is None and isinstance(color_hex_plain, str):
            hex_str = color_hex_plain.strip().lstrip("#")
            if len(hex_str) == 6:
                try:
                    r = int(hex_str[0:2], 16)
                    g = int(hex_str[2:4], 16)
                    b = int(hex_str[4:6], 16)
                    color = [r, g, b, 255]
                except Exception:
                    color = None

        # Or keep the existing lane color if known
        if color is None and self.lanes:
            for ln in self.lanes:
                if ln.get("id") == lane and "color" in ln:
                    color = ln["color"]
                    break

        if color is not None:
            args["color"] = color

        ok, pkt_id = self._send_pkt("SET_FILAMENT_INFO", args, note="set_filament")
        gcmd.respond_info(f"SET_FILAMENT_INFO lane={lane} ok={ok} id={pkt_id}")


    def cmd_BMCU_CALL(self, gcmd):
        cmd = gcmd.get("CMD")
        args_json = gcmd.get("ARGS", "{}")
        wait_s = gcmd.get_float("WAIT", 0.0)
        try:
            args = json.loads(args_json)
        except Exception:
            gcmd.respond_info("Invalid ARGS JSON")
            return
        ok, pkt_id = self._send_pkt(cmd, args, note="call")
        gcmd.respond_info(f"CALL {cmd} id={pkt_id}")
        if ok:
            self._wait_for_reply(gcmd, pkt_id, wait_s)

    def cmd_BMCU_LANE_FEED(self, gcmd):
        lane = gcmd.get_int("LANE", 0)
        mm = gcmd.get_float("MM", self.default_lane_feed_mm)
        speed = gcmd.get_float("SPEED", self.default_speed)
        ok, _ = self._send_pkt(
            "MOVE",
            {"axis": str(lane), "dist_mm": float(mm), "speed": float(speed)}, note="lane_feed")
        gcmd.respond_info(f"Lane {lane} FEED {mm}mm")

    def cmd_BMCU_LANE_RETRACT(self, gcmd):
        lane = gcmd.get_int("LANE", 0)
        mm = gcmd.get_float("MM", self.default_lane_retract_mm)
        speed = gcmd.get_float("SPEED", self.default_speed)
        ok, _ = self._send_pkt(
            "MOVE",
            {"axis": str(lane), "dist_mm": float(-mm), "speed": float(speed)}, note="lane_retract")
        gcmd.respond_info(f"Lane {lane} RETRACT {mm}mm")


def load_config(config):
    return BMCU(config)
