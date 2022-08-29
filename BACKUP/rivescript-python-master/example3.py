#!/usr/bin/python3

# elpimous 15 oct 2018

from rivescript import RiveScript
import re
rs = RiveScript(utf8=True)
rs.unicode_punctuation = re.compile(r'[.,!?;:]')

rs.load_directory("/home/nvidia/Documents/rivescript-python-master/eg/neo")
rs.sort_replies()

while True:
    msg = input("You> ")
    if msg == '/quit':
        quit()
    reply = rs.reply("localuser", msg)
    print("Bot>", reply)

