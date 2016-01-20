# SDP-2016-Team-F

## Configuring the SRF stick and the RF chip
Guard character are likely to be # or ~.

Simplest way is to use the graphical XCM (XRF Config Manager). This can be found in the tools directory. Sadly this only works on Windows. This tool can be used both for the stick and arduino RF chip.

The configuration is somewhat persistent.
Once the equipment is configured there is no need to use any code to reconfigure most of it.
It might be good (or necessary) to do some configuring nonetheless, namely, channel (ATCN): 

_Note that the frequency settings do not appear to be persistent (i.e. you’ll need to set them every time you power up)._

ATCN (Channel nubmer)

Group 11 0x60

Group 12 0x67

## Arduino Libraries
To use the arduino code you need to have the libraries used correctly linked. 
Library files can be found in the repo. If linking is broken check your IDE settings to make sure it can find the libs.

## Workflow
Please use the GitHub wiki a lot. If you find something out or tricky and get a solution chances are
that others will have hit the same problem. Please amend or add to the wiki as you see fit.

Issues and tasks will be managed through the GitHub issue tracker.

