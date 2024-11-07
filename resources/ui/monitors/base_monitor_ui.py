# -*- coding: utf-8 -*-

################################################################################
## Form generated from reading UI file 'base_monitor.ui'
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
from PySide6.QtWidgets import (QApplication, QComboBox, QFrame, QHBoxLayout,
    QLabel, QPushButton, QSizePolicy, QSpacerItem,
    QVBoxLayout, QWidget)

class Ui_BaseMonitor(object):
    def setupUi(self, BaseMonitor):
        if not BaseMonitor.objectName():
            BaseMonitor.setObjectName(u"BaseMonitor")
        BaseMonitor.resize(600, 400)
        self.verticalLayout = QVBoxLayout(BaseMonitor)
        self.verticalLayout.setObjectName(u"verticalLayout")
        self.controlFrame = QFrame(BaseMonitor)
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

        self.statusLabel = QLabel(self.controlFrame)
        self.statusLabel.setObjectName(u"statusLabel")

        self.horizontalLayout.addWidget(self.statusLabel)

        self.horizontalSpacer = QSpacerItem(0, 0, QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Minimum)

        self.horizontalLayout.addItem(self.horizontalSpacer)


        self.verticalLayout.addWidget(self.controlFrame)

        self.plotFrame = QFrame(BaseMonitor)
        self.plotFrame.setObjectName(u"plotFrame")
        self.plotLayout = QVBoxLayout(self.plotFrame)
        self.plotLayout.setObjectName(u"plotLayout")

        self.verticalLayout.addWidget(self.plotFrame)

        self.infoFrame = QFrame(BaseMonitor)
        self.infoFrame.setObjectName(u"infoFrame")
        self.infoLayout = QHBoxLayout(self.infoFrame)
        self.infoLayout.setObjectName(u"infoLayout")
        self.dataInfoLabel = QLabel(self.infoFrame)
        self.dataInfoLabel.setObjectName(u"dataInfoLabel")

        self.infoLayout.addWidget(self.dataInfoLabel)


        self.verticalLayout.addWidget(self.infoFrame)


        self.retranslateUi(BaseMonitor)

        QMetaObject.connectSlotsByName(BaseMonitor)
    # setupUi

    def retranslateUi(self, BaseMonitor):
        self.topicLabel.setText(QCoreApplication.translate("BaseMonitor", u"Topic:", None))
        self.refreshButton.setText(QCoreApplication.translate("BaseMonitor", u"Refresh", None))
        self.statusLabel.setText(QCoreApplication.translate("BaseMonitor", u"Status: Not Connected", None))
        self.dataInfoLabel.setText(QCoreApplication.translate("BaseMonitor", u"No data received", None))
        pass
    # retranslateUi

