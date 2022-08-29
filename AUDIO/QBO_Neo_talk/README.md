QBO_Neo_talk
===========


NEW  : May 29, 2015

***************************************************************************************************************

Actually, QBO_Neo_talk configured to work with SVOX (quite better voice, between mbrola and acapela voices)

********************************************
sudo apt-get install libttspico-utils
********************************************
To test, execute :

$ pico2wave -w /tmp/test.wav "Hello World"  #record sentence in spoken wav file
$ aplay /tmp/test.wav                       # play sentence
********************************************

