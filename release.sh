#!/usr/bin/env bash

compote component pack --name sx127x
compote component upload --namespace dernasherbrezon --name sx127x

pio account login
pio pkg publish