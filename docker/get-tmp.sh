#!/bin/bash
cd "$(dirname "$0")"
cd ..

DIR_NAME="${PWD##*/}"

if [ "$DIR_NAME" = "CARMAPlatform" ]; then
  echo "carma"
else
  echo "${PWD##*/}" | sed -r 's/CARMA/carma/' | sed -r 's/([A-Z])/-\L\1/g' | sed 's/^_//'
fi
