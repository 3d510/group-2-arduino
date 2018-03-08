#!/bin/bash - 
#===============================================================================
#
#          FILE: scp_src.sh
# 
#         USAGE: ./scp_src.sh 
# 
#   DESCRIPTION: 
# 
#       OPTIONS: ---
#  REQUIREMENTS: ---
#          BUGS: ---
#         NOTES: ---
#        AUTHOR: YOUR NAME (), 
#  ORGANIZATION: 
#       CREATED: 03/07/2018 18:32
#      REVISION:  ---
#===============================================================================

set -o nounset                              # Treat unset variables as an error

scp main/main.ino pi@192.168.2.1:~/mdp/arduino_code/src
