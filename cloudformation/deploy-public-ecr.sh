#!/usr/bin/env bash
# Deploy (or update) the public-ECR pipeline stack.
#
# Prerequisites:
#   - AWS CLI v2 configured with credentials that have CloudFormation, CodeBuild,
#     CodePipeline, IAM, S3, and ECR Public permissions.
#   - A CodeStar / CodeConnections connection to GitHub already created and *Available*
#     in the target account.  Create one in the AWS Console under
#     Settings > Connections if needed.
#
# Usage:
#   ./deploy-public-ecr.sh [options]
#
# Options (all except --connection-arn have sensible defaults):
#   --stack-name         NAME       CloudFormation stack name          (default: deepracer-simapp-public)
#   --region             REGION     AWS region to deploy into          (default: us-east-1)
#   --connection-arn     ARN        CodeStar connection ARN            (required)
#   --repo-owner         OWNER      GitHub org or user                 (default: aws-deepracer-community)
#   --repo-name          NAME       GitHub repository name             (default: deepracer-simapp)
#   --branch             BRANCH     Branch that triggers the pipeline  (default: main)
#   --public-ecr-prefix  PREFIX     Namespace under public.ecr.aws     (default: deepracer-community)
#   --image-variants     VARIANTS   "cpu" or "cpu gpu"                 (default: cpu)
#   --create-repos       true|false Create public ECR repos in stack   (default: false)
#   --no-cache           true|false Disable Docker layer cache         (default: false; omitted when false)
#   --dry-run                       Print the command without running it

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
TEMPLATE="${SCRIPT_DIR}/simapp-public-ecr-pipeline.yml"

# ── defaults ────────────────────────────────────────────────────────────────────
STACK_NAME="deepracer-simapp-public"
REGION="us-east-1"
CONNECTION_ARN=""
REPO_OWNER="aws-deepracer-community"
REPO_NAME="deepracer-simapp"
BRANCH="main"
PUBLIC_ECR_PREFIX="deepracer-community"
IMAGE_VARIANTS="cpu"
CREATE_REPOS="false"
NO_CACHE=""
DRY_RUN=""

# ── parse args ──────────────────────────────────────────────────────────────────
while [[ $# -gt 0 ]]; do
    case "$1" in
    --stack-name)         STACK_NAME="$2";       shift 2 ;;
    --region)             REGION="$2";            shift 2 ;;
    --connection-arn)     CONNECTION_ARN="$2";    shift 2 ;;
    --repo-owner)         REPO_OWNER="$2";        shift 2 ;;
    --repo-name)          REPO_NAME="$2";         shift 2 ;;
    --branch)             BRANCH="$2";            shift 2 ;;
    --public-ecr-prefix)  PUBLIC_ECR_PREFIX="$2"; shift 2 ;;
    --image-variants)     IMAGE_VARIANTS="$2";    shift 2 ;;
    --create-repos)       CREATE_REPOS="$2";      shift 2 ;;
    --no-cache)           NO_CACHE="$2";          shift 2 ;;
    --dry-run)            DRY_RUN="true";         shift   ;;
    *)
        echo "Unknown option: $1" >&2
        exit 1
        ;;
    esac
done

if [[ -z "${CONNECTION_ARN}" ]]; then
    echo "Error: --connection-arn is required." >&2
    echo "Find your connection ARN:" >&2
    echo "  aws codestar-connections list-connections --region ${REGION} --query 'Connections[?ConnectionStatus==\`AVAILABLE\`].[ConnectionArn,ConnectionName]' --output table" >&2
    exit 1
fi

# ── build parameter overrides ────────────────────────────────────────────────────
PARAMS=(
    "GitHubConnectionArn=${CONNECTION_ARN}"
    "RepositoryOwner=${REPO_OWNER}"
    "RepositoryName=${REPO_NAME}"
    "RepositoryBranch=${BRANCH}"
    "PublicEcrPrefix=${PUBLIC_ECR_PREFIX}"
    "ImageVariants=${IMAGE_VARIANTS}"
    "CreatePublicEcrRepositories=${CREATE_REPOS}"
)

if [[ -n "${NO_CACHE}" && "${NO_CACHE}" == "true" ]]; then
    PARAMS+=("NoCache=true")
fi

# ── assemble command ─────────────────────────────────────────────────────────────
CMD=(
    aws cloudformation deploy
    --region "${REGION}"
    --stack-name "${STACK_NAME}"
    --template-file "${TEMPLATE}"
    --capabilities CAPABILITY_NAMED_IAM
    --parameter-overrides "${PARAMS[@]}"
)

echo "Deploying stack '${STACK_NAME}' to region '${REGION}'..."
echo "  Template : ${TEMPLATE}"
echo "  ECR prefix: public.ecr.aws/${PUBLIC_ECR_PREFIX}"
echo "  Variants  : ${IMAGE_VARIANTS}"
echo "  Branch    : ${BRANCH}"
echo ""

if [[ -n "${DRY_RUN}" ]]; then
    echo "[dry-run] Command that would be executed:"
    echo "  ${CMD[*]}"
else
    "${CMD[@]}"
    echo ""
    echo "Stack deployed. To check outputs:"
    echo "  aws cloudformation describe-stacks --region ${REGION} --stack-name ${STACK_NAME} --query 'Stacks[0].Outputs' --output table"
fi
