#!/usr/bin/python
# coding: utf-8

from rivescript import RiveScript
import re
rs = RiveScript(utf8=True)
rs.unicode_punctuation = re.compile(r'[.,!?;:]')

rs.load_directory("/home/nvidia/Documents/rivescript-python-master/eg/neo")
rs.sort_replies()

while True:
    msg = raw_input("utilisateur > ")
    msg = msg.replace('é', 'e').replace('è', 'e').replace('ç', 'c')
    if msg == '/quit':
        quit()
    reply = rs.reply("localuser", msg)
    print "Néo >", reply

# vim:expandtab
