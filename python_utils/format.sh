#!/bin/bash

FILES_TO_FORMAT="*.py"

isort $FILES_TO_FORMAT
black $FILES_TO_FORMAT