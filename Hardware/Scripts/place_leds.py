#!/usr/bin/env python3

import math

NUM_LEDS = 16
LED_RADIUS = 14
R_RADIUS = LED_RADIUS - 3
START_ANGLE = 245
END_ANGLE = 315
LED_ANGLE = 270
R_ANGLE = 0

for led in range(1, NUM_LEDS + 1):
	angle = (START_ANGLE - led * (((START_ANGLE - END_ANGLE) % 360) / NUM_LEDS)) % 360
	theta = (math.pi * 2) * (angle / 360)
	led_x, led_y = LED_RADIUS * math.cos(theta), LED_RADIUS * math.sin(theta)
	r_x, r_y = R_RADIUS * math.cos(theta), R_RADIUS * math.sin(theta)

	print(f'ROTATE =R{(angle+LED_ANGLE)%360:06.3f} LED{led}')
	print(f'MOVE LED{led} ({led_x:06.4f} {led_y:06.4f})')

	print(f'ROTATE =R{(angle+R_ANGLE) % 360:06.3f} \'R{led}\'')
	print(f'MOVE R{led} ({r_x:06.4f} {r_y:06.4f})')

