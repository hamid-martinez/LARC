# import sys
# sys.path.append("/home/pi/Desktop/git-larc/LARC/program/env")
# from env.lib import gspread

import gspread

sa = gspread.service_account()

sheet = sa.open("Inventory")