from PyQt4.Qt import *
from sloth.items.items import RectItem

showLabels = True

class LabeledRectItem(RectItem):
    def __init__(self, model_item=None, prefix="", parent=None):
        RectItem.__init__(self, model_item, prefix, parent)
        self.label = QString(model_item['class'])
        self.text_pen = QPen()
        self.text_pen.setColor(Qt.green)

    def paint(self, painter, option, widget=None):
        RectItem.paint(self, painter, option, widget)
        global showLabels
        if not showLabels:
            return
        painter.setPen(self.text_pen)
        #topLeft = self.boundingRect().
        painter.drawText(QPoint(0, -8),self.label)
        painter.setPen(self.pen())

def toggleShowLabels():
    global showLabels
    showLabels = not showLabels
