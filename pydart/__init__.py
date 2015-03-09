from pydart import *
import misc
import glutgui

import imp
try:
    imp.find_module('PyQt4')
    import qtgui
except ImportError:
    print ' [pydart init] PyQt4 does not exist'
# from pydart_api import *
