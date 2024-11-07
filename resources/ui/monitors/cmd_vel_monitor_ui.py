# -*- coding: utf-8 -*-

################################################################################
## Form generated from reading UI file 'cmd_vel_monitor.ui'
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
    QGridLayout, QGroupBox, QHBoxLayout, QHeaderView,
    QLabel, QProgressBar, QPushButton, QSizePolicy,
    QSpacerItem, QSplitter, QStatusBar, QTableWidget,
    QTableWidgetItem, QVBoxLayout, QWidget)

class Ui_CmdVelMonitor(object):
    def setupUi(self, CmdVelMonitor):
        if not CmdVelMonitor.objectName():
            CmdVelMonitor.setObjectName(u"CmdVelMonitor")
        CmdVelMonitor.resize(800, 600)
        self.verticalLayout = QVBoxLayout(CmdVelMonitor)
        self.verticalLayout.setObjectName(u"verticalLayout")
        self.controlFrame = QFrame(CmdVelMonitor)
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

        self.filteringCheckBox = QCheckBox(self.controlFrame)
        self.filteringCheckBox.setObjectName(u"filteringCheckBox")
        self.filteringCheckBox.setChecked(True)

        self.horizontalLayout.addWidget(self.filteringCheckBox)

        self.filterSettingsButton = QPushButton(self.controlFrame)
        self.filterSettingsButton.setObjectName(u"filterSettingsButton")

        self.horizontalLayout.addWidget(self.filterSettingsButton)

        self.line = QFrame(self.controlFrame)
        self.line.setObjectName(u"line")
        self.line.setFrameShape(QFrame.Shape.VLine)
        self.line.setFrameShadow(QFrame.Shadow.Sunken)

        self.horizontalLayout.addWidget(self.line)

        self.analysisCheckBox = QCheckBox(self.controlFrame)
        self.analysisCheckBox.setObjectName(u"analysisCheckBox")
        self.analysisCheckBox.setChecked(True)

        self.horizontalLayout.addWidget(self.analysisCheckBox)

        self.horizontalSpacer = QSpacerItem(0, 0, QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Minimum)

        self.horizontalLayout.addItem(self.horizontalSpacer)

        self.statusLabel = QLabel(self.controlFrame)
        self.statusLabel.setObjectName(u"statusLabel")

        self.horizontalLayout.addWidget(self.statusLabel)


        self.verticalLayout.addWidget(self.controlFrame)

        self.splitter = QSplitter(CmdVelMonitor)
        self.splitter.setObjectName(u"splitter")
        self.splitter.setOrientation(Qt.Horizontal)
        self.plotFrame = QFrame(self.splitter)
        self.plotFrame.setObjectName(u"plotFrame")
        self.plotLayout = QVBoxLayout(self.plotFrame)
        self.plotLayout.setObjectName(u"plotLayout")
        self.linearVelGroup = QGroupBox(self.plotFrame)
        self.linearVelGroup.setObjectName(u"linearVelGroup")
        self.linearVelLayout = QVBoxLayout(self.linearVelGroup)
        self.linearVelLayout.setObjectName(u"linearVelLayout")

        self.plotLayout.addWidget(self.linearVelGroup)

        self.angularVelGroup = QGroupBox(self.plotFrame)
        self.angularVelGroup.setObjectName(u"angularVelGroup")
        self.angularVelLayout = QVBoxLayout(self.angularVelGroup)
        self.angularVelLayout.setObjectName(u"angularVelLayout")

        self.plotLayout.addWidget(self.angularVelGroup)

        self.splitter.addWidget(self.plotFrame)
        self.analysisFrame = QFrame(self.splitter)
        self.analysisFrame.setObjectName(u"analysisFrame")
        self.analysisLayout = QVBoxLayout(self.analysisFrame)
        self.analysisLayout.setObjectName(u"analysisLayout")
        self.currentStateGroup = QGroupBox(self.analysisFrame)
        self.currentStateGroup.setObjectName(u"currentStateGroup")
        self.currentStateLayout = QGridLayout(self.currentStateGroup)
        self.currentStateLayout.setObjectName(u"currentStateLayout")
        self.linearSpeedLabel = QLabel(self.currentStateGroup)
        self.linearSpeedLabel.setObjectName(u"linearSpeedLabel")

        self.currentStateLayout.addWidget(self.linearSpeedLabel, 0, 0, 1, 1)

        self.linearSpeedBar = QProgressBar(self.currentStateGroup)
        self.linearSpeedBar.setObjectName(u"linearSpeedBar")
        self.linearSpeedBar.setMaximum(100)

        self.currentStateLayout.addWidget(self.linearSpeedBar, 0, 1, 1, 1)

        self.angularSpeedLabel = QLabel(self.currentStateGroup)
        self.angularSpeedLabel.setObjectName(u"angularSpeedLabel")

        self.currentStateLayout.addWidget(self.angularSpeedLabel, 1, 0, 1, 1)

        self.angularSpeedBar = QProgressBar(self.currentStateGroup)
        self.angularSpeedBar.setObjectName(u"angularSpeedBar")
        self.angularSpeedBar.setMaximum(100)

        self.currentStateLayout.addWidget(self.angularSpeedBar, 1, 1, 1, 1)


        self.analysisLayout.addWidget(self.currentStateGroup)

        self.motionProfileGroup = QGroupBox(self.analysisFrame)
        self.motionProfileGroup.setObjectName(u"motionProfileGroup")
        self.motionProfileLayout = QVBoxLayout(self.motionProfileGroup)
        self.motionProfileLayout.setObjectName(u"motionProfileLayout")
        self.profileTable = QTableWidget(self.motionProfileGroup)
        if (self.profileTable.columnCount() < 2):
            self.profileTable.setColumnCount(2)
        __qtablewidgetitem = QTableWidgetItem()
        self.profileTable.setHorizontalHeaderItem(0, __qtablewidgetitem)
        __qtablewidgetitem1 = QTableWidgetItem()
        self.profileTable.setHorizontalHeaderItem(1, __qtablewidgetitem1)
        self.profileTable.setObjectName(u"profileTable")
        self.profileTable.setMinimumHeight(200)
        self.profileTable.setColumnCount(2)

        self.motionProfileLayout.addWidget(self.profileTable)

        self.exportDataButton = QPushButton(self.motionProfileGroup)
        self.exportDataButton.setObjectName(u"exportDataButton")

        self.motionProfileLayout.addWidget(self.exportDataButton)


        self.analysisLayout.addWidget(self.motionProfileGroup)

        self.splitter.addWidget(self.analysisFrame)

        self.verticalLayout.addWidget(self.splitter)

        self.infoFrame = QFrame(CmdVelMonitor)
        self.infoFrame.setObjectName(u"infoFrame")
        self.gridLayout = QGridLayout(self.infoFrame)
        self.gridLayout.setObjectName(u"gridLayout")
        self.linearVelLabel = QLabel(self.infoFrame)
        self.linearVelLabel.setObjectName(u"linearVelLabel")

        self.gridLayout.addWidget(self.linearVelLabel, 0, 0, 1, 1)

        self.angularVelLabel = QLabel(self.infoFrame)
        self.angularVelLabel.setObjectName(u"angularVelLabel")

        self.gridLayout.addWidget(self.angularVelLabel, 0, 1, 1, 1)

        self.limitsLabel = QLabel(self.infoFrame)
        self.limitsLabel.setObjectName(u"limitsLabel")

        self.gridLayout.addWidget(self.limitsLabel, 1, 0, 1, 1)

        self.performanceLabel = QLabel(self.infoFrame)
        self.performanceLabel.setObjectName(u"performanceLabel")

        self.gridLayout.addWidget(self.performanceLabel, 1, 1, 1, 1)


        self.verticalLayout.addWidget(self.infoFrame)

        self.statusBar = QStatusBar(CmdVelMonitor)
        self.statusBar.setObjectName(u"statusBar")

        self.verticalLayout.addWidget(self.statusBar)


        self.retranslateUi(CmdVelMonitor)

        QMetaObject.connectSlotsByName(CmdVelMonitor)
    # setupUi

    def retranslateUi(self, CmdVelMonitor):
        self.topicLabel.setText(QCoreApplication.translate("CmdVelMonitor", u"Topic:", None))
        self.refreshButton.setText(QCoreApplication.translate("CmdVelMonitor", u"Refresh", None))
        self.filteringCheckBox.setText(QCoreApplication.translate("CmdVelMonitor", u"Enable Filtering", None))
        self.filterSettingsButton.setText(QCoreApplication.translate("CmdVelMonitor", u"Filter Settings", None))
        self.analysisCheckBox.setText(QCoreApplication.translate("CmdVelMonitor", u"Motion Analysis", None))
        self.statusLabel.setText(QCoreApplication.translate("CmdVelMonitor", u"Status: Not Connected", None))
        self.linearVelGroup.setTitle(QCoreApplication.translate("CmdVelMonitor", u"Linear Velocity (m/s)", None))
        self.angularVelGroup.setTitle(QCoreApplication.translate("CmdVelMonitor", u"Angular Velocity (rad/s)", None))
        self.currentStateGroup.setTitle(QCoreApplication.translate("CmdVelMonitor", u"Current State", None))
        self.linearSpeedLabel.setText(QCoreApplication.translate("CmdVelMonitor", u"Linear Speed:", None))
        self.linearSpeedBar.setFormat(QCoreApplication.translate("CmdVelMonitor", u"%v m/s", None))
        self.angularSpeedLabel.setText(QCoreApplication.translate("CmdVelMonitor", u"Angular Speed:", None))
        self.angularSpeedBar.setFormat(QCoreApplication.translate("CmdVelMonitor", u"%v rad/s", None))
        self.motionProfileGroup.setTitle(QCoreApplication.translate("CmdVelMonitor", u"Motion Profile", None))
        ___qtablewidgetitem = self.profileTable.horizontalHeaderItem(0)
        ___qtablewidgetitem.setText(QCoreApplication.translate("CmdVelMonitor", u"Metric", None));
        ___qtablewidgetitem1 = self.profileTable.horizontalHeaderItem(1)
        ___qtablewidgetitem1.setText(QCoreApplication.translate("CmdVelMonitor", u"Value", None));
        self.exportDataButton.setText(QCoreApplication.translate("CmdVelMonitor", u"Export Data", None))
        self.linearVelLabel.setText(QCoreApplication.translate("CmdVelMonitor", u"Linear Velocity (m/s): x=0.00, y=0.00, z=0.00", None))
        self.angularVelLabel.setText(QCoreApplication.translate("CmdVelMonitor", u"Angular Velocity (rad/s): x=0.00, y=0.00, z=0.00", None))
        self.limitsLabel.setText(QCoreApplication.translate("CmdVelMonitor", u"Limits: lin=0.00 m/s, ang=0.00 rad/s", None))
        self.performanceLabel.setText(QCoreApplication.translate("CmdVelMonitor", u"Performance: freq=0.0Hz, cpu=0%", None))
        pass
    # retranslateUi

