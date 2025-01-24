----Flashing the Pi Image----
Download the newest photonvision image for the pi from https://github.com/PhotonVision/photonvision/releases (photonvision-v<MOST-RECENT-VERSION>-linuxarm64_RaspberryPi.img.xz)
Open the Pi's SD card on a computer
Open BalenaEtcher and flash the SD card using the newly downloaded file
Put the SD card back in the pi and connect it to the robot and turn the robot on

----Making the IP Static----
Connect to the robot wifi and navigate to photonvision.local:5800
In settings check the "Static" box under "IP Assignment Mode" and set the IP to 10.51.15.44
!NOTE: DO NOT SET THE IP IN CMDLINE.TXT!
  setting the ip in anyway other than through photonvision's ui could cause big problems.
set the "Team Number/NetworkTables Server Address" to 5115
save the settings

----Settings, Pipelines, Configs, and Calibrations----
reboot the pi and navigate to 10.51.15.44:5800
to import calibration files go to the camera tab and under calibration select the info icon next to the resolution you want
create a pipeline that uses AprilTags
set the pipeline's procesing mode to 3D
  if it is grayed out, you must make sure that the selected resolution has been calibrated
under "april tag" (in between "input" and "output") set the tag family to 36h11 (6.5 in)
under "input" enable auto exposure
set the resolution to a resolution that has been calibrated for
make sure the orientation makes the view right side up and then ensure that is properly reflected in VisionConstants.robotToCamTransform
