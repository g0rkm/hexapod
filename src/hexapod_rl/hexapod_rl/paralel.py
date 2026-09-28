"""Ölçüm araçlarının ortak süreç havuzu (olcum, robustness, reflex_probe).

Her iş kendi sürecinde koşar (maxtasksperchild=1): süreç içi Gazebo
dünyaları ve gz-transport kaynakları bir işten ötekine birikmesin.

Neden "forkserver", "fork" değil (2026-09-28): maxtasksperchild=1'de havuz
yeni işçileri kendi yardımcı iş parçacığından fork ediyor. Çok iş parçacıklı
bir süreçten fork, o an başka bir iş parçacığının tuttuğu kilidi çocuğa
kilitli kopyalayabilir; çocuk sonsuza kadar bekler, pool.map de hiç bitmez.
Ölçüldü: 60 işlik deneylerde 3 koşunun 3'ünde bir iş takıldı (her seferinde
farklı iş, bütün iş parçacıkları futex'te, %0 işlemci; yeniden deneyince
geçti). forkserver işçileri tek iş parçacıklı bir sunucudan açar (Python
3.14'te Linux'ta varsayılan da bu). Bedeli: her işçi modülleri yeniden yükler
(~2-3 s).
"""

from __future__ import annotations

import multiprocessing as mp


def job_pool(workers: int):
    """Her işi ayrı (forkserver) süreçte koşan havuz. İş fonksiyonu ve
    argümanları modül düzeyinde ve pickle'lanabilir olmalı."""
    return mp.get_context("forkserver").Pool(workers, maxtasksperchild=1)


__all__ = ["job_pool"]
