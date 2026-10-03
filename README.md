# Bodycam Motion

Bodycam-style camera movement for Minecraft Bedrock (Android, LeviLaunchroid):
walking bob, turning and strafing roll, and idle sway.

## How it works
Every frame, before the game swaps buffers, the finished frame is copied to a
texture and drawn back through a fullscreen pass that rotates, shifts and zooms
it (`eglSwapBuffers` hook). The game's own camera is not touched, so no game
offsets are needed. Zoom is computed each frame so the image always covers the
screen (no empty borders); the cost is a slightly cropped picture during strong
roll.

Movement input comes from touch events:
- touch held on the left half of the screen = moving (drag = strafe / back)
- horizontal drag on the right half = turning

## Tuning
Edit `config/config.json` in the mod folder (or use the launcher's config
editor). Main values: `bobAmount`, `stepFrequency`, `turningRoll`,
`strafingRoll`, `swayIntensity`, `swayFrequency`, `moveSmoothing`,
`rollSmoothing`. Negative roll values flip direction.

## Known limits
- Touch also moves the effect while a menu is open.
- No jump/fall motion (no player state is read).
- Costs one fullscreen copy + draw per frame.

## Credits
Motion model ideas (forward/vertical pitch, turning/strafing roll, idle sway,
smoothing parameters) follow **CameraOverhaul by LENDS DZIN**, used with the
author's permission. This is an independent image-space implementation.
