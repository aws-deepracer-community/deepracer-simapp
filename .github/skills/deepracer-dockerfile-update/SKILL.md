---
name: deepracer-dockerfile-update
description: "Use when: updating Dockerfile.upstream or Dockerfile.upstream-adjusted to a new deepracer-on-aws-simapp version; syncing changes from a new upstream ECR image; inspecting docker image layers to find version bumps; reverse-engineering changes between simapp image versions."
---

# DeepRacer SimApp Dockerfile Update

## Overview

`Dockerfile.upstream` tracks `public.ecr.aws/aws-solutions/deepracer-on-aws-simapp` verbatim.  
`Dockerfile.upstream-adjusted` is a derived variant with local patches (e.g. removing `ros-$ROS_DISTRO-desktop`, adding custom `web-video-server`, different ROS repo config).

Apply changes found in the new upstream image to **both** files, but only apply changes to `Dockerfile.upstream-adjusted` where the relevant section exists and was not intentionally diverged.

---

## Workflow

### 1. Confirm the image is available locally

```bash
docker images | grep deepracer-on-aws-simapp
```

If not present, pull it:

```bash
docker pull public.ecr.aws/aws-solutions/deepracer-on-aws-simapp:<VERSION>
```

### 2. Inspect image metadata

Get environment variables, entrypoint, working directory, and labels:

```bash
docker inspect public.ecr.aws/aws-solutions/deepracer-on-aws-simapp:<VERSION> | python3 -c "
import json, sys
data = json.load(sys.stdin)
cfg = data[0].get('Config', {})
print('Created:', data[0].get('Created'))
print('ID:', data[0].get('Id')[:20])
print('Env:', json.dumps(cfg.get('Env', []), indent=2))
print('Entrypoint:', cfg.get('Entrypoint'))
print('WorkingDir:', cfg.get('WorkingDir'))
"
```

### 3. Extract and compare layer history

Save full layer history for both the previous and new version:

```bash
docker history public.ecr.aws/aws-solutions/deepracer-on-aws-simapp:<OLD> \
  --no-trunc --format "{{.CreatedBy}}" > /tmp/old_history.txt

docker history public.ecr.aws/aws-solutions/deepracer-on-aws-simapp:<NEW> \
  --no-trunc --format "{{.CreatedBy}}" > /tmp/new_history.txt

diff <(sort /tmp/old_history.txt) <(sort /tmp/new_history.txt)
```

The diff output shows exactly which layers changed. Each line is a Dockerfile `RUN`, `ENV`, `COPY`, etc. instruction.

### 4. Identify changes

For each differing layer, determine the category:

| Category | Example | What to update |
|----------|---------|----------------|
| Package version bump | `curl==8.16.0` → `conda-forge::curl==8.19.0` | Version pin in both files |
| New package added | new apt/pip install | Add to both files |
| Package removed | layer deleted | Remove from both files |
| New ENV variable | `ENV FOO=bar` | Add ENV to both files |
| New COPY/layer | new COPY instruction | Evaluate if applicable to adjusted |
| Structural change | reordered layers | Update both files if order matters |

### 5. Update the Dockerfile headers

Always update the comment block at the top of `Dockerfile.upstream`:

```dockerfile
# Optimized Dockerfile for public.ecr.aws/aws-solutions/deepracer-on-aws-simapp:<NEW_VERSION>
# Image ID: <first 12 chars of image ID>
# Base: Ubuntu 24.04 (Noble)
# Built: <Creation date from docker inspect>
# Optimized: <today's date>
```

For `Dockerfile.upstream-adjusted`, update the same header fields.

### 6. Apply changes

Use `multi_replace_string_in_file` to apply all edits in a single call across both files.

**Key divergences in `Dockerfile.upstream-adjusted`** — do NOT revert these intentional differences:
- Uses `ros-$ROS_DISTRO-web-video-server` instead of `ros-$ROS_DISTRO-desktop`
- Does NOT include the Gazebo stable apt repo (`gazebo-stable.list`)
- Adds `COPY ./bundle/src/deepracer_node_monitor/config/*.txt /opt/amazon/script/config/`
- Build/log cleanup after ROS packages build: `rm -rf /opt/amazon/log /opt/amazon/build`

### 7. Validate

After editing:

```bash
# Check both files parse (no syntax errors)
docker build --no-cache --dry-run -f docker/Dockerfile.upstream . 2>&1 | head -20
```

---

## Reference: Known Version History

| Image Version | Key Changes |
|---------------|-------------|
| v1.0.6 | Baseline for `Dockerfile.upstream-adjusted` |
| v1.0.8 | Baseline for `Dockerfile.upstream` |
| v1.0.9 | Added `conda-forge::curl==8.16.0`, `libcurl==8.16.0` (security fix) |
| v1.0.11 | Bumped `conda-forge::curl==8.19.0`, `conda-forge::libcurl==8.19.0` |
| v1.0.12 | Baseline (from v1.0.11 base) |
| v1.0.13 | Base Ubuntu layer refresh only; `LABEL org.opencontainers.image.ref.name=ubuntu` removed; no package changes |
