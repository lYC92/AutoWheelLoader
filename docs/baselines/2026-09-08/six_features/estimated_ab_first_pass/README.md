# First complete estimated-pose A/B cycle

Actual frontend: static map point-to-plane ICP + 15-state IMU fusion (`lio_map`).
The original metrics reporter mistakenly labels all world-frame fusion as KISS-ICP; the numeric results are preserved unchanged. The launcher/reporter now records the selected algorithm explicitly.

Loaded/unloaded 0.700336 m3, 100% at B, full return. Independent world-frame RMSE 0.084900 m; maximum 0.390517 m. No truth alignment. Later initializer, feedback and recovery changes require their own regression.
