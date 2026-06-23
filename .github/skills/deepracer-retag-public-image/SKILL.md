---
name: deepracer-retag-public-image
description: 'Re-tag an image in a Public ECR repository without downloading it. Use when you need to add a new tag to an existing image in a public.ecr.aws repo, copy a tag, or alias a version tag (e.g. point v1.2.0 at the same image as v1.2.1).'
argument-hint: '<alias>/<repo> <existing-tag> <new-tag>'
---

# Re-tag a Public ECR Image (no download)

## When to Use
- Adding a new tag to an image in a Public ECR repo without pulling GB of layers
- Aliasing a version tag (e.g. create `v1.2.0` pointing at the same image as `v1.2.1`)
- `aws ecr-public batch-get-image` does **not** exist — use this workflow instead

## Prerequisites
- AWS CLI with `ecr-public:PutImage` permission on the target repository
- `python3` and `curl` available in the shell
- All `ecr-public` API calls must use `--region us-east-1` regardless of where the repo is

## Procedure

### 1. Gather inputs
- `ALIAS` — the short registry alias in the gallery URL (e.g. `deepracer-community` from `public.ecr.aws/deepracer-community/…`)
- `REPO` — repository name (e.g. `deepracer-on-aws-simapp`)
- `EXISTING_TAG` — the tag whose manifest you want to copy
- `NEW_TAG` — the new tag to create

### 2. Get an anonymous pull token

The token endpoint URL **requires a trailing slash** — without it you get a `301 Moved Permanently` redirect and an empty response.

```bash
TOKEN=$(curl -s "https://public.ecr.aws/token/?service=public.ecr.aws&scope=repository:${ALIAS}/${REPO}:pull" \
  | python3 -c "import sys,json; print(json.load(sys.stdin)['token'])")
```

### 3. Fetch the existing manifest

```bash
MANIFEST=$(curl -s \
  -H "Authorization: Bearer $TOKEN" \
  -H "Accept: application/vnd.docker.distribution.manifest.v2+json" \
  "https://public.ecr.aws/v2/${ALIAS}/${REPO}/manifests/${EXISTING_TAG}")
```

Verify `$MANIFEST` is valid JSON (not an error object) before continuing:

```bash
echo $MANIFEST | python3 -c "import sys,json; d=json.load(sys.stdin); print('OK:', d.get('schemaVersion'))"
```

### 4. Push the manifest under the new tag

```bash
aws ecr-public put-image \
  --repository-name "${REPO}" \
  --image-tag "${NEW_TAG}" \
  --image-manifest "$MANIFEST" \
  --region us-east-1
```

A successful response includes the new `imageTag` in the returned `image.imageId` object.

## DRoA Images

For a DeepRacer-on-AWS (DRoA) release there are typically three images to retag:

| Repository | Description |
|------------|-------------|
| `deepracer-on-aws-simapp` | Main simulation application |
| `deepracer-on-aws-model-validation` | Model validation container |
| `deepracer-on-aws-reward-function-validation` | Reward function validation container |

Repeat the procedure above for each repository, keeping `ALIAS` and the tag values consistent across all three.

## Common Errors

| Error | Cause | Fix |
|-------|-------|-----|
| `Invalid JSON syntax` at `PutImage` | `$MANIFEST` contains an error object (auth failed, wrong URL) | Check token step; print `$MANIFEST` before calling put-image |
| `301 Moved Permanently` from token endpoint | Missing trailing slash in token URL | Add `/` after `token` in the URL |
| `ParamValidation: Found invalid choice 'batch-get-image'` | Using `aws ecr` command instead of `aws ecr-public` | Use `aws ecr-public` — `batch-get-image` only exists for private ECR |
| `DENIED: Your Authorization Token is invalid` | Used `aws ecr-public get-authorization-token` output directly | Use the anonymous token flow above instead |
