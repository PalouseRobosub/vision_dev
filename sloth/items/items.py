from PyQt4.Qt import *
from sloth.items.items import RectItem

class LabeledRectItem(RectItem):
    def __init__(self, model_item=None, prefix="", parent=None):
        RectItem.__init__(self, model_item, prefix, parent)
        self.label = QString(model_item['class'])
        self.text_pen = QPen()
        self.text_pen.setColor(Qt.green)

    def paint(self, painter, option, widget=None):
        RectItem.paint(self, painter, option, widget)
        painter.setPen(self.text_pen)
        #topLeft = self.boundingRect().
        painter.drawText(QPoint(0, -8),self.label)
        painter.setPen(self.pen())
