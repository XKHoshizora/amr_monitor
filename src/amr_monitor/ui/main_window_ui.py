# -*- coding: utf-8 -*-

################################################################################
## Form generated from reading UI file 'main_window.ui'
##
## Created by: Qt User Interface Compiler version 6.8.0
##
## WARNING! All changes made in this file will be lost when recompiling UI file!
################################################################################

from PySide6.QtCore import (QCoreApplication, QDate, QDateTime, QLocale,
    QMetaObject, QObject, QPoint, QRect,
    QSize, QTime, QUrl, Qt)
from PySide6.QtGui import (QAction, QBrush, QColor, QConicalGradient,
    QCursor, QFont, QFontDatabase, QGradient,
    QIcon, QImage, QKeySequence, QLinearGradient,
    QPainter, QPalette, QPixmap, QRadialGradient,
    QTransform)
from PySide6.QtWidgets import (QApplication, QMainWindow, QMenu, QMenuBar,
    QSizePolicy, QStatusBar, QTabWidget, QVBoxLayout,
    QWidget)

class Ui_MainWindow(object):
    def setupUi(self, MainWindow):
        if not MainWindow.objectName():
            MainWindow.setObjectName(u"MainWindow")
        MainWindow.resize(1200, 800)
        self.actionExit = QAction(MainWindow)
        self.actionExit.setObjectName(u"actionExit")
        self.actionRefreshTopics = QAction(MainWindow)
        self.actionRefreshTopics.setObjectName(u"actionRefreshTopics")
        self.actionResetLayout = QAction(MainWindow)
        self.actionResetLayout.setObjectName(u"actionResetLayout")
        self.centralwidget = QWidget(MainWindow)
        self.centralwidget.setObjectName(u"centralwidget")
        self.verticalLayout = QVBoxLayout(self.centralwidget)
        self.verticalLayout.setObjectName(u"verticalLayout")
        self.monitorTab = QTabWidget(self.centralwidget)
        self.monitorTab.setObjectName(u"monitorTab")
        self.odomTab = QWidget()
        self.odomTab.setObjectName(u"odomTab")
        self.odomLayout = QVBoxLayout(self.odomTab)
        self.odomLayout.setObjectName(u"odomLayout")
        self.monitorTab.addTab(self.odomTab, "")
        self.imuTab = QWidget()
        self.imuTab.setObjectName(u"imuTab")
        self.imuLayout = QVBoxLayout(self.imuTab)
        self.imuLayout.setObjectName(u"imuLayout")
        self.monitorTab.addTab(self.imuTab, "")
        self.lidarTab = QWidget()
        self.lidarTab.setObjectName(u"lidarTab")
        self.lidarLayout = QVBoxLayout(self.lidarTab)
        self.lidarLayout.setObjectName(u"lidarLayout")
        self.monitorTab.addTab(self.lidarTab, "")
        self.cmdvelTab = QWidget()
        self.cmdvelTab.setObjectName(u"cmdvelTab")
        self.cmdVelLayout = QVBoxLayout(self.cmdvelTab)
        self.cmdVelLayout.setObjectName(u"cmdVelLayout")
        self.monitorTab.addTab(self.cmdvelTab, "")

        self.verticalLayout.addWidget(self.monitorTab)

        MainWindow.setCentralWidget(self.centralwidget)
        self.menubar = QMenuBar(MainWindow)
        self.menubar.setObjectName(u"menubar")
        self.menubar.setGeometry(QRect(0, 0, 1200, 33))
        self.menuFile = QMenu(self.menubar)
        self.menuFile.setObjectName(u"menuFile")
        self.menuView = QMenu(self.menubar)
        self.menuView.setObjectName(u"menuView")
        MainWindow.setMenuBar(self.menubar)
        self.statusbar = QStatusBar(MainWindow)
        self.statusbar.setObjectName(u"statusbar")
        MainWindow.setStatusBar(self.statusbar)

        self.menubar.addAction(self.menuFile.menuAction())
        self.menubar.addAction(self.menuView.menuAction())
        self.menuFile.addAction(self.actionExit)
        self.menuView.addAction(self.actionRefreshTopics)
        self.menuView.addSeparator()
        self.menuView.addAction(self.actionResetLayout)

        self.retranslateUi(MainWindow)

        self.monitorTab.setCurrentIndex(0)


        QMetaObject.connectSlotsByName(MainWindow)
    # setupUi

    def retranslateUi(self, MainWindow):
        MainWindow.setWindowTitle(QCoreApplication.translate("MainWindow", u"AMR Monitor", None))
        self.actionExit.setText(QCoreApplication.translate("MainWindow", u"Exit", None))
        self.actionRefreshTopics.setText(QCoreApplication.translate("MainWindow", u"Refresh Topics", None))
        self.actionResetLayout.setText(QCoreApplication.translate("MainWindow", u"Reset Layout", None))
        self.monitorTab.setTabText(self.monitorTab.indexOf(self.odomTab), QCoreApplication.translate("MainWindow", u"Odometry Monitor", None))
        self.monitorTab.setTabText(self.monitorTab.indexOf(self.imuTab), QCoreApplication.translate("MainWindow", u"IMU Monitor", None))
        self.monitorTab.setTabText(self.monitorTab.indexOf(self.lidarTab), QCoreApplication.translate("MainWindow", u"LiDAR Monitor", None))
        self.monitorTab.setTabText(self.monitorTab.indexOf(self.cmdvelTab), QCoreApplication.translate("MainWindow", u"CmdVel Monitor", None))
        self.menuFile.setTitle(QCoreApplication.translate("MainWindow", u"File", None))
        self.menuView.setTitle(QCoreApplication.translate("MainWindow", u"View", None))
    # retranslateUi

