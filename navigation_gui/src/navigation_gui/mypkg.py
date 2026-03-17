from qt_gui.plugin import Plugin
from navigation_gui.mypkg_widget import MyWidget
from PyQt5.QtWidgets import QWidget, QVBoxLayout, QLabel, QPushButton
from PyQt5.QtGui import QPixmap, QImage
from python_qt_binding.QtCore import Qt, QTimer, Signal, Slot
from PyQt5.QtWidgets import QApplication
from PyQt5.QtCore import QT_VERSION_STR
import PyQt5.QtCore as QtCore

import roslaunch
 
class MyPlugin(Plugin):
    def __init__(self, context):
        super(MyPlugin, self).__init__(context)
        self.setObjectName('MyPlugin')
        self._widget = MyWidget()
        
        if context.serial_number() > 1:
            self._widget.setWindowTitle(self._widget.windowTitle() +
                                        (' (%d)' % context.serial_number()))


        context.add_widget(self._widget)


    def shutdown_plugin(self):
        self._widget.close_plugin()
        # Just make sure to stop timers and publishers, unsubscribe from Topics etc in the shutdown_plugin method.
        pass
 
    def save_settings(self, plugin_settings, instance_settings):
        pass
 
    def restore_settings(self, plugin_settings, instance_settings):
        pass