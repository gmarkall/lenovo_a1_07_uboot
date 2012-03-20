#!/bin/bash
#

CPU_JOB_NUM=$(grep processor /proc/cpuinfo | awk '{field=$NF};END{print field+1}')
MY_PANEL=`echo "$1" | tr '[:upper:]' '[:lower:]'`


case $MY_PANEL in
  "tft")
    MY_PANEL=$MY_PANEL
    ;;
  "3g")
    MY_PANEL=$MY_PANEL
    ;;
  *)
    MY_PANEL=tft
    ;;
esac

echo "=== PANEL = $MY_PANEL ==="

export PATH=$PATH:./tools
START_TIME=`date +%s`
make distclean
if [ "$MY_PANEL" == "edp1" ]; then
make omap3621_edp1_config 
fi
if [ "$MY_PANEL" == "tft" ]; then
make omap3621_edp1_tft_config 
fi
if [ "$MY_PANEL" == "3g" ]; then
make omap3621_edp1_tft_3g_config 
fi
make -j$CPU_JOB_NUM
END_TIME=`date +%s`
let "ELAPSED_TIME=$END_TIME-$START_TIME"

