# -*- coding: utf-8 -*-

################################################################################
## Form generated from reading UI file 'odom_monitor.ui'
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
    QProgressBar, QPushButton, QSizePolicy, QSpacerItem,
    QSplitter, QStatusBar, QTabWidget, QVBoxLayout,
    QWidget)

class Ui_OdomMonitor(object):
    def setupUi(self, OdomMonitor):
        if not OdomMonitor.objectName():
            OdomMonitor.setObjectName(u"OdomMonitor")
        OdomMonitor.resize(1000, 800)
        self.verticalLayout = QVBoxLayout(OdomMonitor)
        self.verticalLayout.setObjectName(u"verticalLayout")
        self.controlFrame = QFrame(OdomMonitor)
        self.controlFrame.setObjectName(u"controlFrame")
        self.horizontalLayout = QHBoxLayout(self.controlFrame)
        self.horizontalLayout.setObjectName(u"horizontalLayout")
        self.topicLabel = QLabel(self.controlFrame)
        self.topicLabel.setObjectName(u"topicLabel")

        self.horizontalLayout.addWidget(self.topicLabel)

        self.topicComboBox = QComboBox(self.controlFrame)
        self.topicComboBox.setObjectName(u"topicComboBox")

        self.horizontalLayout.addWidget(self.topicComboBox)

        self.refreshButton = QPushButton(self.controlFrame)
        self.refreshButton.setObjectName(u"refreshButton")

        self.horizontalLayout.addWidget(self.refreshButton)

        self.showTrajectoryCheckBox = QCheckBox(self.controlFrame)
        self.showTrajectoryCheckBox.setObjectName(u"showTrajectoryCheckBox")
        self.showTrajectoryCheckBox.setChecked(False)

        self.horizontalLayout.addWidget(self.showTrajectoryCheckBox)

        self.clearTrajectoryButton = QPushButton(self.controlFrame)
        self.clearTrajectoryButton.setObjectName(u"clearTrajectoryButton")

        self.horizontalLayout.addWidget(self.clearTrajectoryButton)

        self.horizontalSpacer = QSpacerItem(0, 0, QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Minimum)

        self.horizontalLayout.addItem(self.horizontalSpacer)

        self.statusLabel = QLabel(self.controlFrame)
        self.statusLabel.setObjectName(u"statusLabel")

        self.horizontalLayout.addWidget(self.statusLabel)


        self.verticalLayout.addWidget(self.controlFrame)

        self.mainSplitter = QSplitter(OdomMonitor)
        self.mainSplitter.setObjectName(u"mainSplitter")
        self.mainSplitter.setOrientation(Qt.Orientation.Horizontal)
        self.trajectoryGroup = QGroupBox(self.mainSplitter)
        self.trajectoryGroup.setObjectName(u"trajectoryGroup")
        self.trajectoryLayout = QVBoxLayout(self.trajectoryGroup)
        self.trajectoryLayout.setObjectName(u"trajectoryLayout")
        self.mainSplitter.addWidget(self.trajectoryGroup)
        self.rightFrame = QFrame(self.mainSplitter)
        self.rightFrame.setObjectName(u"rightFrame")
        self.rightLayout = QVBoxLayout(self.rightFrame)
        self.rightLayout.setObjectName(u"rightLayout")
        self.currentVelGroup = QGroupBox(self.rightFrame)
        self.currentVelGroup.setObjectName(u"currentVelGroup")
        self.currentVelLayout = QGridLayout(self.currentVelGroup)
        self.currentVelLayout.setObjectName(u"currentVelLayout")
        self.linearSpeedLabel = QLabel(self.currentVelGroup)
        self.linearSpeedLabel.setObjectName(u"linearSpeedLabel")

        self.currentVelLayout.addWidget(self.linearSpeedLabel, 0, 0, 1, 1)

        self.linearSpeedBar = QProgressBar(self.currentVelGroup)
        self.linearSpeedBar.setObjectName(u"linearSpeedBar")
        self.linearSpeedBar.setMaximum(100)

        self.currentVelLayout.addWidget(self.linearSpeedBar, 0, 1, 1, 1)

        self.angularSpeedLabel = QLabel(self.currentVelGroup)
        self.angularSpeedLabel.setObjectName(u"angularSpeedLabel")

        self.currentVelLayout.addWidget(self.angularSpeedLabel, 1, 0, 1, 1)

        self.angularSpeedBar = QProgressBar(self.currentVelGroup)
        self.angularSpeedBar.setObjectName(u"angularSpeedBar")
        self.angularSpeedBar.setMaximum(100)

        self.currentVelLayout.addWidget(self.angularSpeedBar, 1, 1, 1, 1)


        self.rightLayout.addWidget(self.currentVelGroup)

        self.velocityTabWidget = QTabWidget(self.rightFrame)
        self.velocityTabWidget.setObjectName(u"velocityTabWidget")
        self.linearVelTab = QWidget()
        self.linearVelTab.setObjectName(u"linearVelTab")
        self.linearVelTabLayout = QVBoxLayout(self.linearVelTab)
        self.linearVelTabLayout.setObjectName(u"linearVelTabLayout")
        self.velocityTabWidget.addTab(self.linearVelTab, "")
        self.angularVelTab = QWidget()
        self.angularVelTab.setObjectName(u"angularVelTab")
        self.angularVelTabLayout = QVBoxLayout(self.angularVelTab)
        self.angularVelTabLayout.setObjectName(u"angularVelTabLayout")
        self.velocityTabWidget.addTab(self.angularVelTab, "")

        self.rightLayout.addWidget(self.velocityTabWidget)

        self.mainSplitter.addWidget(self.rightFrame)

        self.verticalLayout.addWidget(self.mainSplitter)

        self.infoFrame = QFrame(OdomMonitor)
        self.infoFrame.setObjectName(u"infoFrame")
        self.gridLayout = QGridLayout(self.infoFrame)
        self.gridLayout.setObjectName(u"gridLayout")
        self.positionLabel = QLabel(self.infoFrame)
        self.positionLabel.setObjectName(u"positionLabel")

        self.gridLayout.addWidget(self.positionLabel, 0, 0, 1, 1)

        self.orientationLabel = QLabel(self.infoFrame)
        self.orientationLabel.setObjectName(u"orientationLabel")

        self.gridLayout.addWidget(self.orientationLabel, 0, 1, 1, 1)

        self.velocityLabel = QLabel(self.infoFrame)
        self.velocityLabel.setObjectName(u"velocityLabel")

        self.gridLayout.addWidget(self.velocityLabel, 1, 0, 1, 1)

        self.distanceLabel = QLabel(self.infoFrame)
        self.distanceLabel.setObjectName(u"distanceLabel")

        self.gridLayout.addWidget(self.distanceLabel, 1, 1, 1, 1)


        self.verticalLayout.addWidget(self.infoFrame)

        self.statusBar = QStatusBar(OdomMonitor)
        self.statusBar.setObjectName(u"statusBar")

        self.verticalLayout.addWidget(self.statusBar)


        self.retranslateUi(OdomMonitor)

        self.velocityTabWidget.setCurrentIndex(1)


        QMetaObject.connectSlotsByName(OdomMonitor)
    # setupUi

    def retranslateUi(self, OdomMonitor):
        self.topicLabel.setText(QCoreApplication.translate("OdomMonitor", u"Topic:", None))
        self.refreshButton.setText(QCoreApplication.translate("OdomMonitor", u"Refresh", None))
        self.showTrajectoryCheckBox.setText(QCoreApplication.translate("OdomMonitor", u"Show Trajectory", None))
        self.clearTrajectoryButton.setText(QCoreApplication.translate("OdomMonitor", u"Clear Trajectory", None))
        self.statusLabel.setText(QCoreApplication.translate("OdomMonitor", u"Status: Not Connected", None))
        self.trajectoryGroup.setTitle(QCoreApplication.translate("OdomMonitor", u"Robot Trajectory", None))
        self.currentVelGroup.setTitle(QCoreApplication.translate("OdomMonitor", u"Current Velocity", None))
        self.linearSpeedLabel.setText(QCoreApplication.translate("OdomMonitor", u"Linear Speed:", None))
        self.linearSpeedBar.setFormat(QCoreApplication.translate("OdomMonitor", u"%v m/s", None))
        self.angularSpeedLabel.setText(QCoreApplication.translate("OdomMonitor", u"Angular Speed:", None))
        self.angularSpeedBar.setFormat(QCoreApplication.translate("OdomMonitor", u"%v rad/s", None))
        self.velocityTabWidget.setTabText(self.velocityTabWidget.indexOf(self.linearVelTab), QCoreApplication.translate("OdomMonitor", u"Linear Velocity", None))
        self.velocityTabWidget.setTabText(self.velocityTabWidget.indexOf(self.angularVelTab), QCoreApplication.translate("OdomMonitor", u"Angular Velocity", None))
        self.positionLabel.setText(QCoreApplication.translate("OdomMonitor", u"Position (x, y, z): 0.00, 0.00, 0.00", None))
        self.orientationLabel.setText(QCoreApplication.translate("OdomMonitor", u"Orientation (r, p, y): 0.00, 0.00, 0.00", None))
        self.velocityLabel.setText(QCoreApplication.translate("OdomMonitor", u"Velocity (lin_x, lin_y, ang_z): 0.00, 0.00, 0.00", None))
        self.distanceLabel.setText(QCoreApplication.translate("OdomMonitor", u"Total Distance: 0.00 m", None))
        pass
    # retranslateUi

