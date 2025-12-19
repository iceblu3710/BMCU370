# Klipper–Happy Hare Friendly MMS Command Set

This document defines a **recommended command interface** between a custom MMS (Multi‑Material System) and **Klipper + Happy Hare (HH)**.

The goal is a **thin, stable abstraction layer** that:
- Matches Happy Hare’s state‑machine philosophy
- Is sensor‑driven, not distance‑only
- Avoids hard‑coding workflows like "LOAD" or "UNLOAD"
- Works over serial, USB, CAN, or other transports

---

## Design Principles

1. **Primitive, not semantic**
   - HH orchestrates behavior
   - MMS exposes mechanical capabilities

2. **Move‑until over move‑X**
   - Most MMU reliability comes from sensor‑gated motion

3. **Structured responses**
   - Every command returns a reason code and telemetry

4. **Timeouts everywhere**
   - No command should block indefinitely

5. **Transport‑agnostic**
   - JSON‑lines shown for clarity, binary is fine

---

## Message Envelope

### Request
```json
{
  "id": 42,
  "cmd": "MOVE_UNTIL",
  "args": { ... }
}
```

### Response
```json
{
  "id": 42,
  "ok": true,
  "code": "STOP_CONDITION",
  "msg": "toolhead present",
  "telemetry": { ... }
}
```

### Async Event (optional)
```json
{
  "evt": "SENSORS",
  "t_ms": 123456,
  "s": { ... }
}
```

---

## Axes Naming Convention

Use logical axes HH can reason about:

- `FEED` – main filament path toward toolhead
- `SELECTOR` – lane selector axis (if continuous)
- `SPOOL[n]` – per‑lane feeder motors (optional)

Distances are **signed millimeters**.

---

## Core Commands

### 1. `PING`
Health check + version discovery.

```json
{"id":1,"cmd":"PING","args":{}}
```

Response telemetry should include:
- firmware version
- protocol version
- uptime

---

### 2. `CAPS`
Report MMS capabilities.

```json
{"id":2,"cmd":"CAPS","args":{}}
```

Example response:
```json
{
  "lanes": 4,
  "selector": true,
  "encoder": true,
  "cutter": false,
  "tension_sensor": false
}
```

---

### 3. `STATUS`
Current state snapshot.

```json
{"id":3,"cmd":"STATUS","args":{}}
```

---

## Lane Management

### 4. `SELECT_LANE`
Route filament from a given lane.

```json
{
  "id":10,
  "cmd":"SELECT_LANE",
  "args":{
    "lane":2,
    "timeout_ms":3000
  }
}
```

Return codes:
- `OK`
- `TIMEOUT`
- `FAULT_ACTIVE`

---

## Motion Primitives

### 5. `MOVE`
Open‑loop motion.

```json
{
  "id":20,
  "cmd":"MOVE",
  "args":{
    "axis":"FEED",
    "dist_mm":120,
    "speed":60,
    "accel":400
  }
}
```

Telemetry (if available):
- `moved_mm`
- `encoder_ticks`

---

### 6. `MOVE_UNTIL`
Move until a stop condition occurs.

```json
{
  "id":21,
  "cmd":"MOVE_UNTIL",
  "args":{
    "axis":"FEED",
    "dist_mm":500,
    "speed":80,
    "stop": {"toolhead":"PRESENT"},
    "timeout_ms":8000
  }
}
```

Supported stop conditions:

- `{"toolhead":"PRESENT" | "CLEAR"}`
- `{"buffer":"PRESENT" | "CLEAR"}`
- `{"lane_present":{"lane":1,"state":"PRESENT"}}`
- `{"encoder":{"min_ticks":1200}}`
- `{"stall":"DETECT"}`
- `{"slip":"DETECT"}`

Return codes:
- `STOP_CONDITION`
- `MAX_DISTANCE`
- `TIMEOUT`
- `SLIP_DETECTED`
- `JAM_DETECTED`

---

### 7. `JOG`
Short timed movement, typically for recovery.

```json
{
  "id":22,
  "cmd":"JOG",
  "args":{
    "axis":"FEED",
    "dir":-1,
    "speed":40,
    "duration_ms":250
  }
}
```

---

### 8. `STOP`
Immediate stop of all motion.

```json
{"id":23,"cmd":"STOP","args":{"reason":"HOST_STOP"}}
```

---

## Sensors & Telemetry

### 9. `GET_SENSORS`
Snapshot of all sensors.

```json
{"id":30,"cmd":"GET_SENSORS","args":{}}
```

Stable schema recommended:
```json
{
  "toolhead":0,
  "buffer":1,
  "lane":[1,0,1,1],
  "encoder_ticks":1234,
  "tension":0.0,
  "fault":"NONE"
}
```

---

### 10. `SUBSCRIBE`
Enable periodic async sensor events.

```json
{
  "id":31,
  "cmd":"SUBSCRIBE",
  "args":{
    "what":["SENSORS","FAULTS"],
    "period_ms":200
  }
}
```

---

## Actuators (Optional)

### 11. `CUT`
Filament cutter.

```json
{"id":40,"cmd":"CUT","args":{"timeout_ms":2000}}
```

---

### 12. `GATE`
Lane gate or idler pressure control.

```json
{
  "id":41,
  "cmd":"GATE",
  "args":{
    "lane":2,
    "state":"OPEN"
  }
}
```

---

### 13. `SERVO`
Generic servo control.

```json
{
  "id":42,
  "cmd":"SERVO",
  "args":{
    "name":"idler",
    "pos":0.72
  }
}
```

---

## Fault Handling

### 14. `CLEAR_FAULT`
Reset MMS fault state.

```json
{"id":50,"cmd":"CLEAR_FAULT","args":{}}
```

---

### 15. `RECOVER_HINT` (Optional)
Ask MMS for recovery suggestion.

```json
{"id":51,"cmd":"RECOVER_HINT","args":{}}
```

---

## Minimal Required Set

For first integration with Happy Hare, implement at least:

- `PING`
- `STATUS`
- `SELECT_LANE`
- `MOVE`
- `MOVE_UNTIL`
- `GET_SENSORS`
- `STOP`
- `CLEAR_FAULT`

This is sufficient for real multi‑material printing.

---

## Philosophy Reminder

Happy Hare is the **brain**.

Your MMS is the **muscle and nerves**.

Keep this interface mechanical, observable, and boring.

That’s how it becomes reliable.

