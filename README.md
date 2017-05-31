# weather_board_driver
odroid's weather board driver for ROS

This uses Adafruit TCA9548A 1-to-8 I2C Multiplexer Breakout: https://learn.adafruit.com/adafruit-tca9548a-1-to-8-i2c-multiplexer-breakout?view=all
for reading multiple weather boards from hard kernel: http://www.hardkernel.com/main/products/prdt_info.php?g_code=G144533067183.

Make sure to have wiringPi installed:
git clone hard kernel's wiringPi, by following these instructions: https://github.com/hardkernel/wiringPi/blob/master/INSTALL.
