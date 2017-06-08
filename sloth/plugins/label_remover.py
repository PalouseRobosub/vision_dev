#!/bin/env python

from PyQt4.QtGui import *
from PyQt4.QtCore import *

from sloth.core.commands import register_command, get_commands
from sloth.core.cli import BaseCommand, CommandError
from sloth.conf import config

class LabelRemoverPlugin(QObject):
    def __init__(self, labeltool):
        QObject.__init__(self)
        self._labeltool = labeltool
        self._wnd = labeltool.mainWindow()
        self._sc = QAction("Remove Labels", self._wnd)
        self._sc.triggered.connect(self.doit)
        self.setupDialog()

    def setupDialog(self):
        # Setup dialog
        self.dialog = QDialog()
        self.dialog.setWindowTitle("Remove Label")
        self.dialog.resize(625, 250)
        self.acceptButton = QPushButton("Remove", self.dialog)
        self.cancelButton = QPushButton("Cancel", self.dialog)
        self.annList = QListWidget(self.dialog)
        self.removeList = QListWidget(self.dialog)
        self.moveRightButton = QPushButton(">>", self.dialog)
        self.moveLeftButton = QPushButton("<<", self.dialog)
        self.toRemoveLabel = QLabel(self.dialog)
        self.labelsLabel = QLabel(self.dialog)

        self.acceptButton.move(450,220)
        self.acceptButton.clicked.connect(self.remove)
        self.cancelButton.move(90,220)
        self.cancelButton.clicked.connect(self.cancel)
        self.moveRightButton.move(270,70)
        self.moveRightButton.clicked.connect(self.moveToRemove)
        self.moveLeftButton.move(270,120)
        self.moveLeftButton.clicked.connect(self.moveFromRemove)
        self.annList.move(10, 20)
        self.removeList.move(360, 20)
        for i in config.LABELS:
           print(i['attributes']['class'])
           self.annList.addItem(i['attributes']['class'] + " : " + i['text'])
        self.toRemoveLabel.setText("To Remove")
        self.labelsLabel.setText("Labels")
        self.labelsLabel.move(110, 0)
        self.toRemoveLabel.move(460, 0)

    def remove(self):
        print("Remove stuff")
        items = []
        for i in xrange(self.removeList.count()):
            items.append(self.removeList.item(i))
        annotations = [str(i.text().split(" : ")[0]) for i in items]

        print(annotations)

        print(self._labeltool.annotations())

        for ann in self._labeltool.annotations():
            for i in ann['annotations']:
                if i['class'] in annotations:
                    print("Removing: {}".format(i))
                    ann['annotations'].remove(i)

        print(self._labeltool.annotations())

        self.dialog.close()

    def moveToRemove(self):
        selection = self.annList.currentRow()
        self.removeList.addItem(self.annList.currentItem().text())
        self.annList.takeItem(selection)

    def moveFromRemove(self):
        selection = self.removeList.currentRow()
        self.annList.addItem(self.removeList.currentItem().text())
        self.removeList.takeItem(selection)

    def cancel(self):
        self.dialog.close()

    def doit(self):
        self.setupDialog()
        self.dialog.exec_()

    def action(self):
        return self._sc;
