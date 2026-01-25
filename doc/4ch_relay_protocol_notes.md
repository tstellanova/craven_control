

LC Technology Modbus Realy X4_V2.1
www.lctech-inc.com

Introduction to Modbus RTU Instructions Modbus devices execute operations by receiving Modbus RTU commands from external controllers (e.g., host computers/MCUs). A command frame typically comprises the device address, function code, register address, register data, and checksum. The frame length depends on the function code. The first byte of each frame data is the device address, which can be set within the range 1-255, with a default of 255 (0xFF). The final two bytes constitute the CRC checksum. Assuming a device address of 255, common Modbus RTU commands are as follows: 
1. Activate Relay 1 (Manual Mode) 
    - Send: FF 05 00 00 FF 00 99 E4 
    - Return unchanged: FF 05 00 00 FF 00 99 E4 
    - Notes: (1) Bytes 3-4 of the transmitted frame represent the relay address. Addresses for relays 1–8 are respectively: 0x0000, 0x0001, 0x0002, 0x0003, 0x0004, 0x0005, 0x0006 0x0000, 0x0001, 0x0002, 0x0003, 0x0004, 0x0005, 0x0006, 0x0007 (2) Bytes 5-6 of the transmission frame represent the data: 0xFF00 indicates open relay, 0x0000 indicates close relay.
2. Deactivate Relay 1 (Manual Mode) 
    - Send: FF 0500 00 00 00 D814 
    - Return unchanged: FF 05 00 00 00 00 D814 
3. Open Relay 2 (Manual Mode)
    - Send: FF 05 00 01 FF 00 C8 24 
    - Return unchanged: FF 05 00 01 FF 00 C8 24 
4. Close Relay 2 (Manual Mode) 
    - Send: FF 05 00 0100 00 89D4 
    - Return unchanged: FF 05 00 0100 00 89 D4 
5. Open all relays 
    - Send: FF 0F00 00 00 08 01FF301D 
    - Return: FF 0F00 00 00 08 41D3 6. 
    - Close all relays Send: FF 0F00 00 00 08 0100705D Return: FF0F00 00 00 0841D3
7. Set device address to 1 
    - Send: 001000 00 00010200 016A00 
    - Return unchanged: 00 10 0000 00010200016A00 
    - Note: The 9th byte 0x01 in the send frame represents the written device address 
8. Set device address to 255
    - Send: 0010 00 00 00010200FFEB80 
    - Return unchanged: 0010 00 00 000102 00FFEB80 
    - Note: The 9th byte of the sent frame, 0xFF, is the written device address 
9. Read device address 
    - Send: 00 0300 0000 0185DB 
    - Return: 00 030200FF C5C4 
    - Note: The fifth byte of the return frame, 0xFF, is the read device address 
10. Read relay status 
    - Send: FF0100 00 00 0828 12 
    - Return: FF010101A1A0 
    - Note: Bits 0–7 of the fourth byte (0x01) in the return frame represent relays 1–8 respectively. 0 indicates off, 1 indicates on.
11. Read optocoupler input status 
    - Send: FF02 00 00 00 08 6C12 
    - Return: FF02 01 01 51 A0 
    - Note: The fourth byte 0x01 in the return frame represents optocoupler inputs IN1 to IN8 respectively. 0 indicates a low level, 1 indicates a high level. 
12. Set baud rate to 4800 
    - Send: FF10 03E900 0102 00 02 4A0C 
    - Return: FF10 03E9 00 01C5A7 
    - Note: The 9th byte of the transmission frame is the baud rate setting value. 0x02, 0x03, 0x04 represent 4800, 9600, 19200 respectively. 
13. setting baud rate to 9600 
    - Send: FF10 03E9 00 010200 038BCC 
    - Return: FF10 03E9 00 01C5A7
14. Set baud rate to 19200 
    - Send: FF10 03E9 00 0102 00 04 CA 0E 
    - Return: FF10 03E9 00 01C5A7 
15. Read baud rate 
    - Send: FF03 03E8 00 0111A4 
    - Return: FF0302 00 04 90 53 
    - Note: The fifth byte of the return frame indicates the read baud rate. Values 0x02, 0x03, and 0x04 correspond to 4800, 9600, and 19200 baud respectively.
16. Activate Relay 1 (flash-off mode, 2 seconds) 
    - Send: FF10 00 03 00 02 0400040014C5 9F 
    - Return: FF10 00 03 0002A416 
    - Notes: (1) Bytes 3–4 of the transmitted frame represent the relay address. Addresses for relays 1–8 are respectively: 0x0003, 0x0008, 0x000D, 0x0012, 0x0017, 0x001C, 0x002 1, 0x0026 (2) The 10th–11th bytes of the transmission frame denote the delay setting value. The delay base unit is 0.1 seconds, thus the delay duration is 0x0014 × 0.1 = 20 × 0.1 seconds = 2 seconds. The relay automatically de-energises after 2 seconds. 
17. De-energise Relay 1 (3-second flash-off mode) Transmit: FF10 00 0300 020400 02001EA5 99 Return: FF10 000300 02A416 Notes: (1) Bytes 3–4 of the transmitted frame represent the relay address. The addresses for relays 1–8 are respectively: 8 are respectively 0x0003, 0x0008, 0x000D, 0x0012, 0x0017, 0x001C, 0x0021, 0x0026 (2) The 10th–11th bytes of the transmission frame denote the delay setting value. The delay base unit is 0.1 seconds, thus the delay duration is 0x001E × 0.1 = 30 × 0.1 seconds = 3 seconds. The relay automatically closes after remaining off for 3 seconds.