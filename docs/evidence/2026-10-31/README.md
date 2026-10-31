# Recorded evidence — 31 October 2026

These files preserve the supplied experiments; they are not fresh simulator runs
of this repository checkpoint. Detailed calculations, source hashes and limits
are recorded in [mission_metrics.json](mission_metrics.json).

| File | Source / purpose |
|---|---|
| `autonomous_20261031_124702_440857.zip` | Original supplied baseline mapping-only mission |
| `autonomous_20261031_131817_595257.zip` | Original supplied faster mapping-only mission |
| `localization_attempt_133408.zip` | Original supplied short saved-map attempt; confidence timeout, zero laps |
| `localization_attempt_134058.zip` | Bundle assembled from the latest supplied node logs, CSVs, JSONs and console; local guard stop, zero laps |
| `f1tenth_sim_1mps_exploration.patch.gz` | Losslessly compressed original local demo patch, already integrated into this checkpoint; do not apply it again |
| `mission_metrics.json` | Recalculated recorded stage timings, route lengths and all-sample relative localization errors |
| `checkpoint_validation.json` | Offline checks run when packaging this checkpoint; does not assert live ROS/Gym or hardware acceptance |

`SOURCE_FILES.json` inside the assembled latest-localization ZIP maps uploaded
filenames to normalized archive names. The two mapping ZIPs and first attempt
are preserved byte for byte.

Map/route profile values are generated plans. Nominal geometry acceptance does
not prove execution under uncertain localization. AMCL readiness is not an
independent accuracy check. Historical driving remains paused for the current
mapping/localization/planning milestone.

The [complete project log](../../F1TENTH_COMPLETE_PROJECT_LOG_2026-10-31.md) is a
dated handoff snapshot created before this packaging/push task; only Markdown trailing spaces were normalized for Git. Its statement
that no push occurred during log preparation remains a historical statement.
