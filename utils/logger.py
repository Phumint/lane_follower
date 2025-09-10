import csv
import time

class Logger:
    def __init__(self, filename="telemetry.csv"):
        self.file = open(filename, "w", newline="")
        self.writer = csv.writer(self.file)
        self.writer.writerow(["time","offset","heading","steer_angle","motor_speed","confidence"])

    def log(self, offset, heading, steer, speed, conf):
        self.writer.writerow([time.time(), offset, heading, steer, speed, conf])
        self.file.flush()
