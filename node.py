class Node:

    def __init__(self, point):
        self.parent = None
        self.children = []
        self.point = point

    def get_point(self):
        return self.point

    def set_parent(self, obj):
        self.parent = obj

    def get_children(self):
        return self.children

    def get_parent(self):
        return self.parent

    def add_child(self, point):
        for p in self.children:
            if p.get_point() == point:
                return
        new = Node(point)
        new.set_parent(self)
        self.children.append(new)
        return new

    def del_child(self, point):
        for p in self.children:
            if p.get_point() == point:
                self.children.remove(p)
                return

    def find_point(self, point):
        if point == self.point:
            return self
        for n in self.children:
            tmp = n.find_point(point)
            if tmp is not None:
                return tmp
        return None
