import sys
import os

sys.path.insert(0, os.path.abspath("."))

from dobot_python import Dobot

bot = Dobot("/dev/ttyUSB0")


try:
    print("Bot status:", "connected" if bot.connected() else "not connected")
    pose = bot.get_pose()
    print("Pose:", pose)

    print("Moving to absolute coordinate")
    bot.move_to(175, 0, 35, 0.5)
    print("Moving to relative coordinate")
    bot.move_to_relative(15, 15, 15, 0.5)
    print("Moving back from relative coordinate")
    bot.move_to_relative(-15, -15, -15, -0.5)

    print("Sliding to absolute coordinate")
    bot.slide_to(160, -40, 40, 0.5)
    print("Sliding to relative coordinate")
    bot.slide_to_relative(15, 15, 15, 0.5)
    print("Sliding back from relative coordinate")
    bot.slide_to_relative(-15, -15, -15, -0.5)

    print("Jumping to absolute coordinate")
    bot.jump_to(175, 0, 40, 0.5)
    print("Jumping to relative coordinate")
    bot.jump_to_relative(-5, -5, -5, -0.5)
    print("Jumping back from relative coordinate")
    bot.jump_to_relative(5, 5, 5, 0.5)

    print("Following a path")
    bot.follow_path_relative([[5, 5, 5], [10, 0, 0], [0, 5, 0], [0, 0, 5]])
except KeyboardInterrupt:
    print("Interrupted by user")
finally:
    print("Closing connection")
    bot.close(force_close=True)
