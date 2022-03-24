#!/bin/bash

if [ -f anemometr.zip ]; then
	rm anemometr.zip
fi
mv Anemometr-B_Cu.gbr Bottom.gbr
mv Anemometr-B_Mask.gbr MaskBottom.gbr
mv Anemometr.drl NCData.drl
mv Anemometr-F_Cu.gbr Top.gbr
mv Anemometr-F_Mask.gbr MaskTop.gbr
mv Anemometr-Edge_Cuts.gbr Border.gbr
zip anemometr.zip Border.gbr Bottom.gbr MaskBottom.gbr MaskTop.gbr NCData.drl Top.gbr