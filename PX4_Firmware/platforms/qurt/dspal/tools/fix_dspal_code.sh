#!/bin/bash
TOOLSDIR=$( dirname "${BASH_SOURCE[0]}" )
cd ${TOOLSDIR}/.. && ${TOOLSDIR}/fix_code_style.sh -p ".git build"
