"""Validation report: structured issues with location and a fix hint (docs/SPECYFIKACJA.md §7)."""
from __future__ import annotations

from dataclasses import asdict, dataclass, field
from enum import Enum


class Severity(str, Enum):
    ERROR = "error"      # blocks export
    WARNING = "warning"  # export allowed, operator must read it
    INFO = "info"


@dataclass
class Issue:
    severity: Severity
    code: str                      # stable machine id, e.g. "JOINT_LIMIT", "UNREACHABLE"
    message: str                   # operator text (PL)
    stage: str = "trajectory"      # parse | static | trajectory | post
    instruction: int | None = None  # index in Program.instructions
    source_line: int | None = None  # line in the NC/CSV input
    hint: str | None = None

    def text(self) -> str:
        where = []
        if self.instruction is not None:
            where.append(f"instr. {self.instruction}")
        if self.source_line is not None:
            where.append(f"linia NC {self.source_line}")
        loc = f" ({', '.join(where)})" if where else ""
        h = f" → {self.hint}" if self.hint else ""
        icon = {"error": "✗", "warning": "⚠", "info": "ℹ"}[self.severity.value]
        return f"{icon} [{self.stage}/{self.code}]{loc} {self.message}{h}"

    def to_dict(self) -> dict:
        d = asdict(self)
        d["severity"] = self.severity.value
        return d


@dataclass
class ValidationReport:
    issues: list[Issue] = field(default_factory=list)
    cycle_time_s: float | None = None
    samples_checked: int = 0

    def add(self, *a, **kw) -> Issue:
        it = Issue(*a, **kw)
        self.issues.append(it)
        return it

    def extend(self, other: "ValidationReport") -> None:
        self.issues.extend(other.issues)
        self.samples_checked += other.samples_checked
        if other.cycle_time_s is not None:
            self.cycle_time_s = other.cycle_time_s

    @property
    def errors(self) -> list[Issue]:
        return [i for i in self.issues if i.severity == Severity.ERROR]

    @property
    def warnings(self) -> list[Issue]:
        return [i for i in self.issues if i.severity == Severity.WARNING]

    @property
    def ok(self) -> bool:
        return not self.errors

    def summary(self, max_items: int = 20) -> str:
        info = [i for i in self.issues if i.severity == Severity.INFO]
        head = (f"Walidacja: {'OK' if self.ok else 'BŁĘDY'} | błędy: {len(self.errors)} | "
                f"ostrzeżenia: {len(self.warnings)} | informacje: {len(info)} | "
                f"próbek trajektorii: {self.samples_checked}")
        if self.cycle_time_s is not None:
            head += f" | czas cyklu ≈ {self.cycle_time_s:.1f} s"
        displayed = self.errors + self.warnings + info
        lines = [head] + ["  " + issue.text() for issue in displayed[:max_items]]
        rest = len(displayed) - max_items
        if rest > 0:
            lines.append(f"  … i {rest} więcej (pełna lista w raporcie JSON)")
        return "\n".join(lines)

    def to_dict(self) -> dict:
        return {"ok": self.ok, "errors": len(self.errors), "warnings": len(self.warnings),
                "cycle_time_s": self.cycle_time_s, "samples_checked": self.samples_checked,
                "issues": [i.to_dict() for i in self.issues]}
