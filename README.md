# LocalAir_development

This is the first version of the LocalAir microprocesser software, for use on the Teensy 4.0.

This version works, but is grim. A product version is currently under development.

To load this software onto Teensy use the arduino with the Teensy add-on.

# specFileDecrypt.py
This now have some extra options. Files are passed using the -i flag, and you can specify whether you want to handle the format tha comes off the server after it has been uploaded with the -u flag.
Errors, which are mostly when it has failed to decode something becasue it is guff at the end of a line or file, are sent to stderr, so you can pipe the ouptut from the script into a file.

Here is an example:

python3 specFileDecrypt.py -i '/home/lifegarb/Documents/Work/PhD_Notes/23/10/17/upload_3_231017.txt' -u > outputdata.json^
