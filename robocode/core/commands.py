"""Komendy z undo/redo. GUI NIGDY nie mutuje sceny bezpośrednio."""
from __future__ import annotations

from dataclasses import dataclass, field
from typing import Callable


@dataclass
class Command:
    label: str
    do: Callable[[], None]
    undo: Callable[[], None]
    merge_key: str | None = None  # komendy o tym samym kluczu łączone w jedną (np. jog)


@dataclass
class UndoStack:
    bundle_similar: bool = True
    _done: list[Command] = field(default_factory=list)
    _undone: list[Command] = field(default_factory=list)

    def push(self, cmd: Command) -> None:
        cmd.do()
        if (self.bundle_similar and cmd.merge_key and self._done
                and self._done[-1].merge_key == cmd.merge_key):
            prev = self._done[-1]
            self._done[-1] = Command(cmd.label, cmd.do, prev.undo, cmd.merge_key)
        else:
            self._done.append(cmd)
        self._undone.clear()

    def undo(self) -> bool:
        if not self._done:
            return False
        c = self._done.pop()
        c.undo()
        self._undone.append(c)
        return True

    def redo(self) -> bool:
        if not self._undone:
            return False
        c = self._undone.pop()
        c.do()
        self._done.append(c)
        return True

    def history(self) -> list[str]:
        return [c.label for c in self._done]


# --- gotowe komendy ---

def set_attr(obj, attr: str, value, label: str | None = None, merge_key: str | None = None) -> Command:
    old = getattr(obj, attr)
    old = old.copy() if hasattr(old, "copy") else old
    new = value.copy() if hasattr(value, "copy") else value
    return Command(label or f"Zmień {attr}", lambda: setattr(obj, attr, new),
                   lambda: setattr(obj, attr, old), merge_key)


def add_item(parent, child, label: str = "Dodaj element") -> Command:
    def do():
        parent.add(child)

    def undo():
        parent.children.remove(child)
        child.parent = None

    return Command(label, do, undo)


def rename(item, name: str) -> Command:
    return set_attr(item, "name", name, f"Zmień nazwę na {name}")
