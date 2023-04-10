# Simple demo of reading and writing the time for the DS3231 real-time clock.
########## Setup ##########
import time
import board
import adafruit_ds3231

# Lookup table for names of days (nicer printing).
days = ("Monday", "Tuesday", "Wednesday", "Thursday", "Friday", "Saturday", "Sunday")

# Create sensor object, communicating over the board's default I2C bus
i2c = board.I2C()                                               # creating the i2c bus
i = 0                                                           # counter

########## Setting the Time ##########
while True:
    try:
        if (i==0):                                              # only let if statement happen once
            rtc = adafruit_ds3231.DS3231(i2c)                   # extract rtc from i2c
#                        year, mon, date, hour, min, sec, wday, yday, isdst
            t = time.struct_time((2023, 3, 2, 15, 0, 0, 3, -1, 0))  # Configure Time:
#                                                                you must set year, mon, date, hour, min, sec and weekday
#                                                                yearday (day within the year: 1-366) is not supported
#                                                                isdst (daylight savings): 0 if not in daylight savings
#                                                                                          1 if in daylight savings
            print("Setting time to:", t)                        # uncomment for debugging
            rtc.datetime = t                                    # setting updated time
            print()                                             # seperator
            i += 1                                              # increment counter


########## Main Loop ##########
        while True:
            rtc = adafruit_ds3231.DS3231(i2c)               # extract rtc from i2c
            t = rtc.datetime                                # read time from rtc
            print(t)                                        # uncomment for debugging
            print("The date is {} {}/{}/{}".format(days[int(t.tm_wday)], t.tm_mday, t.tm_mon, t.tm_year))
            print("The time is {}:{:02}:{:02}".format(t.tm_hour, t.tm_min, t.tm_sec))
            time.sleep(1)                                   # wait a second
    except:
        print('DS3231 DISCONNECTED')
        time.sleep(1)
