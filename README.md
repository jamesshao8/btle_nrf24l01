# btle_nrf24l01
send or receiver btle packet with nrf24l01

Connection:
nrf24l01 arduino
VCC 	3.3V
GND 	GND
CE 	10
CSN 	9
SCK 	13
MOSI 	11
MISO 	12
IRQ 	NC

Instruction:
You need 2 sets of nrf24l01 and arduino(uno). Connect your nrf24l01 and arduino(uno) as in the table above.
One of arduino runs send/send_simplified. The other one runs recv.
The NRF24 library file is already included in send/recv folder. No need to add them to arduino ide's library.

If you want to interact with HackRF, you can download this project:
https://github.com/jamesshao8/BTLE

You can either run send/send_simplified on arduino, and use a computer + hackrf to run btle_rx to receive the packets from arduino.
Or you can also run recv on arduino, then use the computer + hackrf to run btle_tx to send packets to arduino.

Receiving command with hackrf is simple:
./btle_rx
You will receive a lot or packet from all devices around you, check carefully of the mac address, you should be able to find the packets from arduino.

Sending command can be:
./btle_tx 39-ADV_IND-TxAdd-1-RxAdd-0-AdvA-685746352413-AdvData-0201060608534841524604ccaabbdd r1000
Which means it's sending on Channel 39 (2480MHz), sending mac address 13:24:35:46:57:68 (won't be shown on recv prgram on arduino, but will show on "hctool lescan").
AdvData is the information data before encoding on transmitter side. This is also the data after decoding on receiver side.
You can verify it on recv, its serial output should look like:
2 1 6 6 8 53 48 41 52 46 4 CC AA BB DD

You can also verify it in the code send_simplifed, it's exactly the same data before encoding. But in this little project, the comment and data structure is more clear.

Let's look into detail of the btle_tx command parameters. "0201060608534841524604ccaabbdd" is also known as INFO bit, which will then be encoded to PHY bit.
You can modify "5348415246" in this parameter to other value, and decode with different name at receiver, or modify ccaabbdd to different data. But please don't modify their length.
