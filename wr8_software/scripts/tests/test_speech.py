#!/usr/bin/env python
# -*- coding: utf-8 -*-

import pyttsx3

engine = pyttsx3.init()
# engine.say('Sally sells seashells by the seashore.')
# engine.say('The quick brown fox jumped over the lazy dog.')

rate = engine.getProperty('rate')
engine.setProperty('rate', 95)

# voices = engine.getProperty('voices')
# for voice in voices:
#     print("Voice:")
#     print(" - ID: %s" % voice.id)
#     print(" - Name: %s" % voice.name)
#     print(" - Languages: %s" % voice.languages)
#     print(" - Gender: %s" % voice.gender)
#     print(" - Age: %s" % voice.age)

# engine.setProperty('voice', 'russian')

engine.say(u'Lidar is ready')
engine.say(u'Lidar is ready')
engine.runAndWait()
