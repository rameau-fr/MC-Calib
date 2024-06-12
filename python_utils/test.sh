#!/bin/bash

FILES_TO_FORMAT="*.py"

isort $FILES_TO_FORMAT --check-only
black $FILES_TO_FORMAT --check
mypy $FILES_TO_FORMAT --disable-error-code=import-untyped
pylint $FILES_TO_FORMAT