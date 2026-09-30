# Autonomy HUD Controls

The RViz panel uses the existing `sdc/HudPanel` plugin and topic names.
The redesigned drive console shows speed, front distance, action, mission stage
and position. AUTO/MANUAL are separate mode buttons. Start/Pause are above
Clear trail/Reset car. Narrow and short docks can scroll vertically.

Unread telemetry displays `--`; speed telemetry older than two seconds is
marked stale. UI updates from ROS are queued on the Qt GUI thread.

## Manual driving

Choose **MANUAL** and keep keyboard focus in the panel. Clicking a mode button
returns keyboard focus to the panel. The key indicators show the held keys.

| Keys | Command |
|---|---|
| W | Forward throttle |
| S | Reverse throttle |
| W + A | Forward and left steering |
| W + D | Forward and right steering |
| W + S / A + D | Opposite inputs cancel |
| Release keys | Zero corresponding throttle/steering request |

A/D steer the wheels; the stationary car needs W or S to move along the turn.
The existing velocity and steering-rate limits still smooth the vehicle motion,
so zero input is not an instantaneous physical stop.

Switching to AUTO, resetting, losing panel focus or closing the panel clears
held inputs. Auto-repeat release events do not clear a physically held key.
Manual mode requests and commands still publish to `sdc/set_mode` and
`sdc/manual_cmd`. The simulator consumes `Twist.linear.x` as throttle and
`Twist.angular.z` as normalized steering.

The previous PID/ramp velocity controllers clamped negative speed to zero,
preventing S from reversing the vehicle. `applyManual()` now explicitly enables
signed velocity. Autonomous velocity calls retain their default behavior.

## Verification

`sdc_hud_panel_smoke` runs the actual RViz panel offscreen, sends Qt key events,
observes its ROS mode/Twist publications, and checks forward/reverse and turn
direction with the existing simulated vehicle. It covers key release,
opposing keys, auto-repeat, focus loss and mode switches. It also renders real
Qt previews at wide/narrow widths and a short dock height.

`sdc_manual_drive_tests` independently checks signed velocity for PID, Ramp and
Bang-Bang, manual reverse movement and unchanged forward-only default clamps.
With RViz/Qt installed, CTest runs both checks. GitHub Actions explicitly builds
the panel and uploads screenshots as the `hud-preview` artifact.
