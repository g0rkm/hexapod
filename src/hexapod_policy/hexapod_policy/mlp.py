"""Torch'suz politika çıkarımı: numpy ile küçük MLP (GOREVLER.md G8).

Eğitim SB3 + PyTorch ile PC'de; Raspberry Pi 4'te yalnızca aktör ağının
ileri geçişi gerekiyor (29 -> 128 -> 128 -> 18, ~20 bin çarpma). Torch'u
Pi'ye kurmak yerine ağırlıklar bir .npz dosyasına aktarılır
(`python -m hexapod_rl.export`) ve burada numpy ile çalıştırılır.

Dosya politikanın eğitildiği SÖZLEŞMEYİ de taşır (PolicyContract): eylem
ölçeği, eylemin eklendiği varsayılan duruş, adım saati ve eğitimde görülen
hız komutu aralıkları. Düğüm bunları eğitim kodundan değil dosyadan alır;
böylece eğitim ayarı değişse bile eski bir politika doğru çalışır.
"""

from __future__ import annotations

import json
from dataclasses import asdict, dataclass
from pathlib import Path

import numpy as np

FORMAT_VERSION = 1
_ACTIVATIONS = {
    "tanh": np.tanh,
    "relu": lambda x: np.maximum(x, 0.0),
}


@dataclass(frozen=True)
class PolicyContract:
    obs_size: int
    action_size: int
    action_scale: float                 # rad; eylem 1 -> varsayılandan bu kadar
    gait_hz: float                      # gözlemdeki adım saatinin frekansı
    control_hz: float                   # politikanın eğitildiği komut hızı
    default_rad: tuple[float, ...]      # eylem 0'ın karşılığı (ayakta duruş), interface sırası
    command_ranges: dict[str, tuple[float, float]]   # vx, vy (m/s), wz (rad/s)
    # "absolute": hedef = default_rad + action_scale x eylem
    # "residual": hedef = tripod(saat, komut) + residual_scale x eylem (tripod.PhaseTripod)
    action_mode: str = "absolute"
    residual_scale: float = 0.0
    base_gait: dict | None = None       # residual: groups, reach_mm, height_mm, lift_mm

    def __post_init__(self) -> None:
        if self.action_mode not in ("absolute", "residual"):
            raise ValueError(f"bilinmeyen eylem modu: {self.action_mode!r}")
        if self.action_mode == "residual":
            if not self.residual_scale > 0:
                raise ValueError("artık eylem modunda residual_scale pozitif olmalı")
            missing = {"groups", "reach_mm", "height_mm", "lift_mm"} - set(self.base_gait or {})
            if missing:
                raise ValueError(f"artık eylem modunda taban yürüyüş eksik: {sorted(missing)}")
        if len(self.default_rad) != self.action_size:
            raise ValueError(f"varsayılan duruş {len(self.default_rad)} değer, "
                             f"eylem boyutu {self.action_size}")
        for axis in ("vx", "vy", "wz"):
            lo, hi = self.command_ranges[axis]
            if lo > hi:
                raise ValueError(f"{axis} aralığı ters: {lo} > {hi}")


class MlpPolicy:
    """Gözlem -> eylem ortalaması (deterministik). Çıkış kırpılmaz; kırpma denetleyicide."""

    def __init__(self, layers: list[tuple[np.ndarray, np.ndarray]], activation: str,
                 contract: PolicyContract, source: str = "") -> None:
        if activation not in _ACTIVATIONS:
            raise ValueError(f"bilinmeyen aktivasyon: {activation}")
        if not layers:
            raise ValueError("en az bir katman gerekir")
        self.layers = [(np.asarray(w, dtype=np.float64), np.asarray(b, dtype=np.float64))
                       for w, b in layers]
        width = contract.obs_size
        for i, (w, b) in enumerate(self.layers):
            if w.shape[1] != width or b.shape != (w.shape[0],):
                raise ValueError(f"katman {i}: ağırlık {w.shape}, sapma {b.shape}, "
                                 f"giriş {width} bekleniyordu")
            width = w.shape[0]
        if width != contract.action_size:
            raise ValueError(f"çıkış {width}, eylem boyutu {contract.action_size}")
        self.activation = activation
        self.contract = contract
        self.source = source

    def __call__(self, obs) -> np.ndarray:
        x = np.asarray(obs, dtype=np.float64)
        if x.shape != (self.contract.obs_size,):
            raise ValueError(f"gözlem {x.shape}, ({self.contract.obs_size},) bekleniyordu")
        act = _ACTIVATIONS[self.activation]
        for w, b in self.layers[:-1]:
            x = act(w @ x + b)
        w, b = self.layers[-1]
        return w @ x + b

    # -- dosya ------------------------------------------------------------------

    def save(self, path: str | Path) -> Path:
        path = Path(path)
        meta = {"format_version": FORMAT_VERSION, "activation": self.activation,
                "source": self.source, "contract": asdict(self.contract)}
        arrays = {f"w{i}": w for i, (w, _) in enumerate(self.layers)}
        arrays.update({f"b{i}": b for i, (_, b) in enumerate(self.layers)})
        with path.open("wb") as f:
            np.savez(f, meta=np.array(json.dumps(meta, ensure_ascii=False)), **arrays)
        return path

    @classmethod
    def load(cls, path: str | Path) -> "MlpPolicy":
        with np.load(Path(path), allow_pickle=False) as data:
            meta = json.loads(str(data["meta"]))
            if meta.get("format_version") != FORMAT_VERSION:
                raise ValueError(f"{path}: biçim sürümü {meta.get('format_version')}, "
                                 f"{FORMAT_VERSION} bekleniyordu")
            n = sum(1 for k in data.files if k.startswith("w"))
            layers = [(data[f"w{i}"], data[f"b{i}"]) for i in range(n)]
        c = meta["contract"]
        contract = PolicyContract(
            obs_size=int(c["obs_size"]), action_size=int(c["action_size"]),
            action_scale=float(c["action_scale"]), gait_hz=float(c["gait_hz"]),
            control_hz=float(c["control_hz"]),
            default_rad=tuple(float(v) for v in c["default_rad"]),
            command_ranges={k: (float(v[0]), float(v[1])) for k, v in c["command_ranges"].items()},
            action_mode=c.get("action_mode", "absolute"),
            residual_scale=float(c.get("residual_scale", 0.0)),
            base_gait=c.get("base_gait"),
        )
        return cls(layers, meta["activation"], contract, meta.get("source", ""))


__all__ = ["FORMAT_VERSION", "MlpPolicy", "PolicyContract"]
