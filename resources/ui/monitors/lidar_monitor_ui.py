# -*- coding: utf-8 -*-

################################################################################
## Form generated from reading UI file 'lidar_monitor.ui'
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
from PySide6.QtWidgets import (QApplication, QCheckBox, QComboBox, QDoubleSpinBox,
    QFrame, QGridLayout, QGroupBox, QHBoxLayout,
    QHeaderView, QLabel, QPushButton, QSizePolicy,
    QSpacerItem, QSplitter, QStatusBar, QTableWidget,
    QTableWidgetItem, QVBoxLayout, QWidget)

class Ui_LidarMonitor(object):
    def setupUi(self, LidarMonitor):
        if not LidarMonitor.objectName():
            LidarMonitor.setObjectName(u"LidarMonitor")
        LidarMonitor.resize(800, 600)
        self.verticalLayout = QVBoxLayout(LidarMonitor)
        self.verticalLayout.setObjectName(u"verticalLayout")
        self.controlFrame = QFrame(LidarMonitor)
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

        self.rangeLabel = QLabel(self.controlFrame)
        self.rangeLabel.setObjectName(u"rangeLabel")

        self.horizontalLayout.addWidget(self.rangeLabel)

        self.rangeSpinBox = QDoubleSpinBox(self.controlFrame)
        self.rangeSpinBox.setObjectName(u"rangeSpinBox")
        self.rangeSpinBox.setMinimum(1.000000000000000)
        self.rangeSpinBox.setMaximum(50.000000000000000)
        self.rangeSpinBox.setValue(10.000000000000000)

        self.horizontalLayout.addWidget(self.rangeSpinBox)

        self.line1 = QFrame(self.controlFrame)
        self.line1.setObjectName(u"line1")
        self.line1.setFrameShape(QFrame.Shape.VLine)
        self.line1.setFrameShadow(QFrame.Shadow.Sunken)

        self.horizontalLayout.addWidget(self.line1)

        self.filteringCheckBox = QCheckBox(self.controlFrame)
        self.filteringCheckBox.setObjectName(u"filteringCheckBox")
        self.filteringCheckBox.setChecked(True)

        self.horizontalLayout.addWidget(self.filteringCheckBox)

        self.filterSettingsButton = QPushButton(self.controlFrame)
        self.filterSettingsButton.setObjectName(u"filterSettingsButton")

        self.horizontalLayout.addWidget(self.filterSettingsButton)

        self.line2 = QFrame(self.controlFrame)
        self.line2.setObjectName(u"line2")
        self.line2.setFrameShape(QFrame.Shape.VLine)
        self.line2.setFrameShadow(QFrame.Shadow.Sunken)

        self.horizontalLayout.addWidget(self.line2)

        self.intensityCheckBox = QCheckBox(self.controlFrame)
        self.intensityCheckBox.setObjectName(u"intensityCheckBox")

        self.horizontalLayout.addWidget(self.intensityCheckBox)

        self.clustersCheckBox = QCheckBox(self.controlFrame)
        self.clustersCheckBox.setObjectName(u"clustersCheckBox")

        self.horizontalLayout.addWidget(self.clustersCheckBox)

        self.polarCheckBox = QCheckBox(self.controlFrame)
        self.polarCheckBox.setObjectName(u"polarCheckBox")
        self.polarCheckBox.setChecked(True)

        self.horizontalLayout.addWidget(self.polarCheckBox)

        self.horizontalSpacer = QSpacerItem(0, 0, QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Minimum)

        self.horizontalLayout.addItem(self.horizontalSpacer)

        self.statusLabel = QLabel(self.controlFrame)
        self.statusLabel.setObjectName(u"statusLabel")

        self.horizontalLayout.addWidget(self.statusLabel)


        self.verticalLayout.addWidget(self.controlFrame)

        self.splitter = QSplitter(LidarMonitor)
        self.splitter.setObjectName(u"splitter")
        self.splitter.setOrientation(Qt.Horizontal)
        self.scanGroup = QGroupBox(self.splitter)
        self.scanGroup.setObjectName(u"scanGroup")
        self.scanLayout = QVBoxLayout(self.scanGroup)
        self.scanLayout.setObjectName(u"scanLayout")
        self.plotWidget = QWidget(self.scanGroup)
        self.plotWidget.setObjectName(u"plotWidget")

        self.scanLayout.addWidget(self.plotWidget)

        self.plotControlFrame = QFrame(self.scanGroup)
        self.plotControlFrame.setObjectName(u"plotControlFrame")
        self.plotControlLayout = QHBoxLayout(self.plotControlFrame)
        self.plotControlLayout.setObjectName(u"plotControlLayout")
        self.minRangeLabel = QLabel(self.plotControlFrame)
        self.minRangeLabel.setObjectName(u"minRangeLabel")

        self.plotControlLayout.addWidget(self.minRangeLabel)

        self.minRangeSpinBox = QDoubleSpinBox(self.plotControlFrame)
        self.minRangeSpinBox.setObjectName(u"minRangeSpinBox")
        self.minRangeSpinBox.setMaximum(10.000000000000000)
        self.minRangeSpinBox.setValue(0.100000000000000)

        self.plotControlLayout.addWidget(self.minRangeSpinBox)

        self.maxRangeLabel = QLabel(self.plotControlFrame)
        self.maxRangeLabel.setObjectName(u"maxRangeLabel")

        self.plotControlLayout.addWidget(self.maxRangeLabel)

        self.maxRangeSpinBox = QDoubleSpinBox(self.plotControlFrame)
        self.maxRangeSpinBox.setObjectName(u"maxRangeSpinBox")
        self.maxRangeSpinBox.setMaximum(50.000000000000000)
        self.maxRangeSpinBox.setValue(30.000000000000000)

        self.plotControlLayout.addWidget(self.maxRangeSpinBox)

        self.resetViewButton = QPushButton(self.plotControlFrame)
        self.resetViewButton.setObjectName(u"resetViewButton")

        self.plotControlLayout.addWidget(self.resetViewButton)


        self.scanLayout.addWidget(self.plotControlFrame)

        self.splitter.addWidget(self.scanGroup)
        self.statsGroup = QGroupBox(self.splitter)
        self.statsGroup.setObjectName(u"statsGroup")
        self.statsLayout = QVBoxLayout(self.statsGroup)
        self.statsLayout.setObjectName(u"statsLayout")
        self.statsTable = QTableWidget(self.statsGroup)
        self.statsTable.setObjectName(u"statsTable")
        self.statsTable.setMinimumWidth(250)

        self.statsLayout.addWidget(self.statsTable)

        self.clusterGroup = QGroupBox(self.statsGroup)
        self.clusterGroup.setObjectName(u"clusterGroup")
        self.clusterLayout = QVBoxLayout(self.clusterGroup)
        self.clusterLayout.setObjectName(u"clusterLayout")
        self.clusterTable = QTableWidget(self.clusterGroup)
        self.clusterTable.setObjectName(u"clusterTable")

        self.clusterLayout.addWidget(self.clusterTable)


        self.statsLayout.addWidget(self.clusterGroup)

        self.splitter.addWidget(self.statsGroup)

        self.verticalLayout.addWidget(self.splitter)

        self.infoFrame = QFrame(LidarMonitor)
        self.infoFrame.setObjectName(u"infoFrame")
        self.infoLayout = QGridLayout(self.infoFrame)
        self.infoLayout.setObjectName(u"infoLayout")
        self.scanInfoLabel = QLabel(self.infoFrame)
        self.scanInfoLabel.setObjectName(u"scanInfoLabel")

        self.infoLayout.addWidget(self.scanInfoLabel, 0, 0, 1, 1)

        self.rangeInfoLabel = QLabel(self.infoFrame)
        self.rangeInfoLabel.setObjectName(u"rangeInfoLabel")

        self.infoLayout.addWidget(self.rangeInfoLabel, 0, 1, 1, 1)

        self.pointsLabel = QLabel(self.infoFrame)
        self.pointsLabel.setObjectName(u"pointsLabel")

        self.infoLayout.addWidget(self.pointsLabel, 1, 0, 1, 1)

        self.performanceLabel = QLabel(self.infoFrame)
        self.performanceLabel.setObjectName(u"performanceLabel")

        self.infoLayout.addWidget(self.performanceLabel, 1, 1, 1, 1)


        self.verticalLayout.addWidget(self.infoFrame)

        self.statusBar = QStatusBar(LidarMonitor)
        self.statusBar.setObjectName(u"statusBar")

        self.verticalLayout.addWidget(self.statusBar)


        self.retranslateUi(LidarMonitor)

        QMetaObject.connectSlotsByName(LidarMonitor)
    # setupUi

    def retranslateUi(self, LidarMonitor):
        self.topicLabel.setText(QCoreApplication.translate("LidarMonitor", u"Topic:", None))
        self.refreshButton.setText(QCoreApplication.translate("LidarMonitor", u"Refresh", None))
        self.rangeLabel.setText(QCoreApplication.translate("LidarMonitor", u"Range (m):", None))
        self.filteringCheckBox.setText(QCoreApplication.translate("LidarMonitor", u"Enable Filtering", None))
        self.filterSettingsButton.setText(QCoreApplication.translate("LidarMonitor", u"Filter Settings", None))
        self.intensityCheckBox.setText(QCoreApplication.translate("LidarMonitor", u"Show Intensity", None))
        self.clustersCheckBox.setText(QCoreApplication.translate("LidarMonitor", u"Show Clusters", None))
        self.polarCheckBox.setText(QCoreApplication.translate("LidarMonitor", u"Polar View", None))
        self.statusLabel.setText(QCoreApplication.translate("LidarMonitor", u"Status: Not Connected", None))
        self.scanGroup.setTitle(QCoreApplication.translate("LidarMonitor", u"Scan Data", None))
        self.minRangeLabel.setText(QCoreApplication.translate("LidarMonitor", u"Min Range:", None))
        self.maxRangeLabel.setText(QCoreApplication.translate("LidarMonitor", u"Max Range:", None))
        self.resetViewButton.setText(QCoreApplication.translate("LidarMonitor", u"Reset View", None))
        self.statsGroup.setTitle(QCoreApplication.translate("LidarMonitor", u"Statistics", None))
        self.clusterGroup.setTitle(QCoreApplication.translate("LidarMonitor", u"Cluster Information", None))
        self.scanInfoLabel.setText(QCoreApplication.translate("LidarMonitor", u"Scan Info: angle_min=0.0, angle_max=0.0, angle_increment=0.0", None))
        self.rangeInfoLabel.setText(QCoreApplication.translate("LidarMonitor", u"Range: min=0.0, max=0.0, avg=0.0", None))
        self.pointsLabel.setText(QCoreApplication.translate("LidarMonitor", u"Points: valid=0/total=0", None))
        self.performanceLabel.setText(QCoreApplication.translate("LidarMonitor", u"Performance: freq=0.0Hz, proc_time=0.0ms", None))
        pass
    # retranslateUi

