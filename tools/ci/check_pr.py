"""Validate PR metadata without interpolating user input into shell commands."""

import json
import os
import re
from pathlib import Path

TITLE = re.compile(r"^(feat|fix|docs|refactor|test|build|ci|chore)(\([a-z0-9_-]+\))?: \S.+$")
BRANCH = re.compile(r"^(feat|fix|docs|refactor|test|build|ci|chore|codex)/[a-z0-9][a-z0-9._/-]*$")


def validate(pr):
    errors = []
    if pr["base"]["ref"] != "main":
        errors.append("PR must target main.")
    if pr.get("user", {}).get("login") == "dependabot[bot]":
        return errors
    if not TITLE.fullmatch(pr["title"]):
        errors.append("Title: feat(stm32): description (or fix/docs/refactor/test/build/ci/chore).")
    if not BRANCH.fullmatch(pr["head"]["ref"]):
        errors.append("Branch: feat/12-description (or fix/docs/refactor/test/build/ci/chore/codex).")
    return errors


if __name__ == "__main__":
    event = json.loads(Path(os.environ["GITHUB_EVENT_PATH"]).read_text(encoding="utf-8"))
    errors = validate(event["pull_request"])
    for error in errors:
        print(error)
    raise SystemExit(bool(errors))
