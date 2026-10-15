"""(M5) Sprawdzanie kolizji: siatki STL + BVH (python-fcl). Interfejs docelowy:

class CollisionChecker:
    def set_pairs(self, pairs): ...          # mapa kolizji (które pary sprawdzać)
    def check(self, station) -> list[tuple]: # lista par w kolizji
"""


class CollisionChecker:
    def __init__(self):
        self.pairs = []

    def check(self, station):
        raise NotImplementedError("M5: zaimplementuj z python-fcl / trimesh")
