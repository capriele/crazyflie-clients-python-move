# Crazyflie PC client PSmove edition

A combination of ps3 controller and psmove thanks to psmoveapi.

Motivations
-----------

The main reason for this project is to add the ability to have the northbound always in front of you. So thanks to psmoveapi combining the ps3 controller and the psmove controller in this way:
![alt tag](https://raw.github.com/capriele/crazyflie-clients-python-move/images/ps3MoveController.jpg)

I've created a sort of pointer of the north.
The most important difference from the "crazyflie-clients-python" is in input.py file in fact here I've ported the C code from sensfusion6.c (of the firmware).


Installation
------------

See https://github.com/capriele/crazyflie-clients-python for the client
See https://github.com/thp/psmoveapi/ for psmoveapi (you have to bind python: install swig and pydev)

IMPORTAN: add path to psmoveapi in this way: export PYTHONPATH=/path/to/psmove/build/directory

