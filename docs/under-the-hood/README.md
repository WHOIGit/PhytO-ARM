# PhytO-ARM: Under the Hood

An 8-bit, music-synced animation of what PhytO-ARM's ROS nodes do during a deployment. Open `index.html` in a browser and press start. It has no dependencies; the chiptune is synthesized live with WebAudio.

Four arm configurations, each with its own scenes and key:

| Config | Launch | Shows |
| --- | --- | --- |
| **DOCK** | `arm_ifcb`, `ifcb_winch: true` | Arm loop, upcast/downcast, profiler peak, IFCB playlist, scheduled depth, wiz probe |
| **DUET** | `arm_ifcb` + `arm_chanos`, `max_moving_winches: 1` | Lock manager semaphore, shared `/arm_ifcb/profiler` peak, stepped downcast with clamping, continuous upcast |
| **DUTY** (PR #121, unmerged) | `arm_duty_cycle`, `duty_cycle_winch: true` | Scheduled sessions, Digital Logger power-on with retries, aux power for the CTD, peak + scheduled depths, rsync, SSH shutdown |
| **ROAMER** | `arm_seatrac`, `ifcb_winch: false` | `no_winch` loop, GPS metadata via `web` field_map, Starlink SOC thresholds, nag timer, 12:30 UTC check |

Keys: <kbd>Space</kbd> play/pause, <kbd>←</kbd>/<kbd>→</kbd> one bar, <kbd>1</kbd>–<kbd>4</kbd> configuration.

## Rendering video files

```bash
node render.cjs out all 30 4   # out/phyto-arm-{dock,duet,roam,duty}.mp4, 1280x720 @ 30 fps
```

Needs Playwright (Chromium) and `ffmpeg`. Frames and audio are rendered deterministically from the same timeline the page plays, so picture and music stay locked.

Times are compressed: a 60 s dwell lasts a couple of beats.
