from PySide.QtCore import QPointF
from PySide.QtGui import QGraphicsEllipseItem

class GraphicsItemInfo(object):
    # __slots__ = ['item', 'uid', 'active', 'label', 'clicked']
    def __init__(self, item, uid, label='', active=False):
        self.item = item
        self.uid = uid
        self.active = active
        self.label = label
        self.clicked = False

        if label:
            intern(label) 


class TreePoint(QPointF):
    def __len__(self):
        return self.toTuple().__len__()

    def __getslice__(self, i, j):
        return self.toTuple().__getslice__(i, j)

    def __getitem__(self, i):
        return self.toTuple().__getitem__(i)

class TreeCircle(QGraphicsEllipseItem):
    def __init__(self, rect, pen, point3d):
        super(TreeCircle, self).__init__(rect)
        self.setPen(pen)
        self.point3d = point3d

    def __len__(self):
        return self.point3d.__len__()

    def __getslice__(self, i, j):
        return self.point3d.__getslice__(i, j)

    def __getitem__(self, i):
        return self.point3d.__getitem__(i)

class TreeCircleInfo(TreeCircle, GraphicsItemInfo):
    def __init__(self, rect, pen, point3d):
        # super(TreeCircleInfo, self).__init__(rect, pen, point3d)
        TreeCircle.__init__(self, rect, pen, point3d)
        GraphicsItemInfo.__init__(self, self, None, None)

    def __repr__(self):
        return 'TreeCircleInfo: (%s, %s, %s) / (%s, %s)' % (tuple(self.point3d) + self.rect().center().toTuple())

if __name__ == '__main__':
    import kdtree
    tree = kdtree.create(dimensions=2)
    tree.add(TreePoint(1,2))
    tree.add(TreePoint(3,4))
    tree.add(TreePoint(10,20))

    q = TreePoint(1,1)
    nodes = tree.search_nn_dist(q, 100)
    for n in nodes:
        print n.dist(q) 