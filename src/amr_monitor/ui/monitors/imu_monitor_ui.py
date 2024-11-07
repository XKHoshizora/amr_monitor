# -*- coding: utf-8 -*-

################################################################################
## Form generated from reading UI file 'imu_monitor.ui'
##
## Created by: Qt User Interface Compiler version 6.8.0
##
## WARNING! All changes made in this file will be lost when recompiling UI file!
################################################################################

from PySide6.QtCore import (QCoreApplication, QDate, QDateTime, QLocale,
    QMetaObject, QObject, QPoint, QRect,
    QSize, QTime, QUrl, Qt)
from PySide6.QtGui import (QBrush, QColor, QConicalGradient, QCursor,
    QFont, QFontDatabase, QGradient, QIcon,
    QImage, QKeySequence, QLinearGradient, QPainter,
    QPalette, QPixmap, QRadialGradient, QTransform)
from PySide6.QtWidgets import (QApplication, QCheckBox, QComboBox, QFrame,
    QGridLayout, QGroupBox, QHBoxLayout, QLabel,
    QPushButton, QSizePolicy, QSpacerItem, QSpinBox,
    QSplitter, QStatusBar, QVBoxLayout, QWidget)

class Ui_IMUMonitor(object):
    def setupUi(self, IMUMonitor):
        if not IMUMonitor.objectName():
            IMUMonitor.setObjectName(u"IMUMonitor")
        IMUMonitor.resize(800, 600)
        self.verticalLayout = QVBoxLayout(IMUMonitor)
        self.verticalLayout.setObjectName(u"verticalLayout")
        self.controlFrame = QFrame(IMUMonitor)
        self.controlFrame.setObjectName(u"controlFrame")
        self.horizontalLayout = QHBoxLayout(self.controlFrame)
        self.horizontalLayout.setObjectName(u"horizontalLayout")
        self.topicLabel = QLabel(self.controlFrame)
        self.topicLabel.setObjectName(u"topicLabel")

        self.horizontalLayout.addWidget(self.topicLabel)

        self.topicComboBox = QComboBox(self.controlFrame)
        self.topicComboBox.setObjectName(u"topicComboBox")
        self.topicComboBox.setMinimumWidth(200)

        self.horizontalLayout.addWidget(self.topicComboBox)

        self.refreshButton = QPushButton(self.controlFrame)
        self.refreshButton.setObjectName(u"refreshButton")

        self.horizontalLayout.addWidget(self.refreshButton)

        self.bufferLabel = QLabel(self.controlFrame)
        self.bufferLabel.setObjectName(u"bufferLabel")

        self.horizontalLayout.addWidget(self.bufferLabel)

        self.bufferSizeSpinBox = QSpinBox(self.controlFrame)
        self.bufferSizeSpinBox.setObjectName(u"bufferSizeSpinBox")
        self.bufferSizeSpinBox.setMinimum(100)
        self.bufferSizeSpinBox.setMaximum(10000)
        self.bufferSizeSpinBox.setValue(500)

        self.horizontalLayout.addWidget(self.bufferSizeSpinBox)

        self.line = QFrame(self.controlFrame)
        self.line.setObjectName(u"line")
        self.line.setFrameShape(QFrame.Shape.VLine)
        self.line.setFrameShadow(QFrame.Shadow.Sunken)

        self.horizontalLayout.addWidget(self.line)

        self.filteringCheckBox = QCheckBox(self.controlFrame)
        self.filteringCheckBox.setObjectName(u"filteringCheckBox")
        self.filteringCheckBox.setChecked(True)

        self.horizontalLayout.addWidget(self.filteringCheckBox)

        self.filterSettingsButton = QPushButton(self.controlFrame)
        self.filterSettingsButton.setObjectName(u"filterSettingsButton")

        self.horizontalLayout.addWidget(self.filterSettingsButton)

        self.horizontalSpacer = QSpacerItem(0, 0, QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Minimum)

        self.horizontalLayout.addItem(self.horizontalSpacer)

        self.statusLabel = QLabel(self.controlFrame)
        self.statusLabel.setObjectName(u"statusLabel")

        self.horizontalLayout.addWidget(self.statusLabel)


        self.verticalLayout.addWidget(self.controlFrame)

        self.splitter = QSplitter(IMUMonitor)
        self.splitter.setObjectName(u"splitter")
        self.splitter.setOrientation(Qt.Vertical)
        self.angularVelGroup = QGroupBox(self.splitter)
        self.angularVelGroup.setObjectName(u"angularVelGroup")
        self.angularVelLayout = QVBoxLayout(self.angularVelGroup)
        self.angularVelLayout.setObjectName(u"angularVelLayout")
        self.splitter.addWidget(self.angularVelGroup)
        self.linearAccGroup = QGroupBox(self.splitter)
        self.linearAccGroup.setObjectName(u"linearAccGroup")
        self.linearAccLayout = QVBoxLayout(self.linearAccGroup)
        self.linearAccLayout.setObjectName(u"linearAccLayout")
        self.splitter.addWidget(self.linearAccGroup)
        self.orientationGroup = QGroupBox(self.splitter)
        self.orientationGroup.setObjectName(u"orientationGroup")
        self.orientationLayout = QVBoxLayout(self.orientationGroup)
        self.orientationLayout.setObjectName(u"orientationLayout")
        self.splitter.addWidget(self.orientationGroup)

        self.verticalLayout.addWidget(self.splitter)

        self.infoFrame = QFrame(IMUMonitor)
        self.infoFrame.setObjectName(u"infoFrame")
        self.gridLayout = QGridLayout(self.infoFrame)
        self.gridLayout.setObjectName(u"gridLayout")
        self.angularVelLabel = QLabel(self.infoFrame)
        self.angularVelLabel.setObjectName(u"angularVelLabel")

        self.gridLayout.addWidget(self.angularVelLabel, 0, 0, 1, 1)

        self.linearAccLabel = QLabel(self.infoFrame)
        self.linearAccLabel.setObjectName(u"linearAccLabel")

        self.gridLayout.addWidget(self.linearAccLabel, 0, 1, 1, 1)

        self.orientationLabel = QLabel(self.infoFrame)
        self.orientationLabel.setObjectName(u"orientationLabel")

        self.gridLayout.addWidget(self.orientationLabel, 1, 0, 1, 1)

        self.performanceLabel = QLabel(self.infoFrame)
        self.performanceLabel.setObjectName(u"performanceLabel")

        self.gridLayout.addWidget(self.performanceLabel, 1, 1, 1, 1)


        self.verticalLayout.addWidget(self.infoFrame)

        self.statusBar = QStatusBar(IMUMonitor)
        self.statusBar.setObjectName(u"statusBar")

        self.verticalLayout.addWidget(self.statusBar)


        self.retranslateUi(IMUMonitor)

        QMetaObject.connectSlotsByName(IMUMonitor)
    # setupUi

    def retranslateUi(self, IMUMonitor):
        self.topicLabel.setText(QCoreApplication.translate("IMUMonitor", u"Topic:", None))
        self.refreshButton.setText(QCoreApplication.translate("IMUMonitor", u"Refresh", None))
        self.bufferLabel.setText(QCoreApplication.translate("IMUMonitor", u"Buffer:", None))
        self.filteringCheckBox.setText(QCoreApplication.translate("IMUMonitor", u"Enable Filtering", None))
        self.filterSettingsButton.setText(QCoreApplication.translate("IMUMonitor", u"Filter Settings", None))
        self.statusLabel.setText(QCoreApplication.translate("IMUMonitor", u"Status: Not Connected", None))
        self.angularVelGroup.setTitle(QCoreApplication.translate("IMUMonitor", u"Angular Velocity (rad/s)", None))
        self.linearAccGroup.setTitle(QCoreApplication.translate("IMUMonitor", u"Linear Acceleration (m/s\u00b2)", None))
        self.orientationGroup.setTitle(QCoreApplication.translate("IMUMonitor", u"Orientation (degrees)", None))
        self.angularVelLabel.setText(QCoreApplication.translate("IMUMonitor", u"Angular Velocity: x=0.00, y=0.00, z=0.00", None))
        self.linearAccLabel.setText(QCoreApplication.translate("IMUMonitor", u"Linear Acceleration: x=0.00, y=0.00, z=0.00", None))
        self.orientationLabel.setText(QCoreApplication.translate("IMUMonitor", u"Orientation: r=0.00, p=0.00, y=0.00", None))
        self.performanceLabel.setText(QCoreApplication.translate("IMUMonitor", u"Performance: CPU=0%, Memory=0%", None))
        pass
    # retranslateUi

