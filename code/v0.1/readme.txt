6526ESP32

v0.1 - Largely disabled wifi, device boots into Ready mode instead of Modem mode
     - Added 1541diag cartridge (to use, poke 56704,5:poke 56705,0:sys56777)
     - Added Tool-64 cartridge (to use, poke 56704,5:poke 56705,1:sys56777)
     - Intended to use a new version of BASIC with new commands to use the
     device easily without pokes or sys calls, and also replace math etc.

v0.0 - straight conversion from ESP8266, added a few things but very dirty
