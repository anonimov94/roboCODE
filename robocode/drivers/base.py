"""(M8) Sterowniki online. Zasada: zanim cokolwiek ruszy prawdziwym robotem –
symulacja OK, kolizje OK, limity OK, prędkość ograniczona, jawne potwierdzenie operatora."""


class RobotDriver:
    def connect(self, ip: str, port: int) -> bool: raise NotImplementedError
    def ping(self) -> bool: raise NotImplementedError
    def joints(self) -> list[float]: raise NotImplementedError
    def move_j(self, joints, confirmed: bool = False): raise NotImplementedError
    def move_l(self, pose, confirmed: bool = False): raise NotImplementedError
    def disconnect(self): raise NotImplementedError


class SimulatedDriver(RobotDriver):
    """Sterownik testowy bez sprzętu."""
    def __init__(self, robot):
        self.robot, self.connected = robot, False
    def connect(self, ip="127.0.0.1", port=0): self.connected = True; return True
    def ping(self): return self.connected
    def joints(self): return list(self.robot.joints)
    def move_j(self, joints, confirmed=False):
        if not confirmed:
            raise PermissionError("Ruch wymaga potwierdzenia operatora")
        self.robot.joints = joints
    def move_l(self, pose, confirmed=False):
        q = self.robot.model.ik(pose, self.robot.joints)
        if q is None:
            raise ValueError("Poza nieosiągalna")
        self.move_j(q, confirmed)
    def disconnect(self): self.connected = False
