TODO for tech spec:
 - Motivation for feature decisions
   - Move sideways: fast interception, fast blocking
   - Solonoid: Small, lightweight, powerful.
   - Grabbers: Fill the remaining space, tuck away, open by default
 - Document robot iterations, and why we changed them.
   [Note: These may not be accurate and are from (very bad) memory]
   - Very first design - why did we discard this one again?
   - STCv1. Basic geared kicker
   - STCv2. Added solonoid
   - STCv3. Improved stability
   - STCv4. Reconstructed after it fell apart
   - STCv5. Refit with new batteries, wheels moved to allow for better rotation.
 - Briefly touch on Euans original vision system, and why we ended up not using it.
 - Redo the opcodes stuff, go into brief but clear detail of the program structure, have opcodes as appendix.
   - Arduino main loop: Comms receive, sensors poll, instructions advance and motors get set.
   - Difference between comms instructions and arduino internal instructions.
     - Public-facing API, (mostly) constant API
   - Comms: base64, ACKs, target/source and checksum. Motivation, and goals for it.
   - Comms: Opcode instructions for our robot.
 - General architecture (how arduino, comms, vision, planning interact)
   - Already got the rough draft from ages ago, just needs adjusting because there are no sensors.
 - Add motivation for technologies used

