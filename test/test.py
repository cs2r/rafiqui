import xarm

arm = xarm.Controller('USB')
battery_voltage = arm.getBatteryVoltage()

print('Battery voltage (V)', battery_voltage)

arm.setPosition(1, 100, 100, True)
