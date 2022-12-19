class Quadrant():

    def __init__(self, p: tuple, d: tuple) -> None:
        x, y = p
        w, h = d
        self.x, self.y = x, y
        self.w, self.h = w, h
        self.left = x-w
        self.right = x+w
        self.top = y-h
        self.bottom = y+h

    def contains(self, p: tuple) -> bool:
        x, y = p.U
        return self.left <= x and x <= self.right and self.top <= y and y <= self.bottom

    def intesects_quadrant(self, quadrant) -> bool:
        return not (self.right < quadrant.left or quadrant.right < self.left or
                    self.bottom < quadrant.top or quadrant.bottom < self.top)

    def subdivide(self) -> list:
        divs = []
        nw = self.w/2
        nh = self.h/2
        x, y = self.x, self.y

        divs.append(Quadrant((x+nw, y+nw), (nw, nh)))
        divs.append(Quadrant((x+nw, y-nw), (nw, nh)))
        divs.append(Quadrant((x-nw, y+nw), (nw, nh)))
        divs.append(Quadrant((x-nw, y-nw), (nw, nh)))
        return divs

    def draw(self, region):

        region.create_line((self.x, self.top),
                           (self.x, self.bottom), color="gray", width=2)
        region.create_line((self.left, self.y),
                           (self.right, self.y), color="gray", width=2)

    def draw_range(self, region):
        region.create_line((self.left, self.top),
                           (self.left, self.bottom), color="yellow", width=2)
        region.create_line((self.right, self.top),
                           (self.right, self.bottom), color="yellow", width=2)
        region.create_line((self.left, self.top),
                           (self.right, self.top), color="yellow", width=2)
        region.create_line((self.left, self.bottom),
                           (self.right, self.bottom), color="yellow", width=2)


class QuadTree():

    def __init__(self, boundary: Quadrant, n: int = 1, depth: int = 1) -> None:
        self.boundary = boundary
        self.points = []
        self.n = n
        self.divided = False
        self.children = []
        self.depth = depth

    def draw(self, region):
        if self.divided:
            self.boundary.draw(region)
        for c in self.children:
            c.draw(region)

    def contains(self, p: tuple) -> bool:
        return self.boundary.contains(p)

    def subdivide(self) -> None:
        self.divided = True
        self.children = []

        divs = self.boundary.subdivide()
        for d in divs:
            self.children.append(QuadTree(d, self.n, self.depth+1))

    def add_point(self, p: tuple) -> bool:
        if not self.contains(p):
            return False
        if len(self.points) < self.n and not self.divided:
            self.points.append(p)
            return True
        if not self.divided:
            self.subdivide()
            for p2 in self.points[::-1]:
                for sq in self.children:
                    if sq.add_point(p2):
                        self.points.pop()
                        break
        for sq in self.children:
            if sq.add_point(p):
                return True
        raise Exception("This should never happen")

    def query_range(self, quadrant: Quadrant) -> bool:
        result = []
        if not self.boundary.intesects_quadrant(quadrant):
            return result
        for p in self.points:
            if quadrant.contains(p):
                result.append(p)
        if not self.divided:
            return result
        for sq in self.children:
            result += sq.query_range(quadrant)
        return result


if __name__ == '__main__':
    W = 300
    H = 300
    b = Quadrant((W/2, H/2), (W/2, H/2))
    O = QuadTree(b)
    print(O)
