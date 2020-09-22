#!/bin/sh

sudo cp ./flyStereo.service /etc/systemd/system/

sudo systemctl daemon-reload
sudo systemctl enable flyStereo

