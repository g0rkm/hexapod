"""hexapod_rl — pekiştirmeli öğrenme için simülasyon ve ortam (GOREVLER.md G6, G7).

sim.py       süreç içi Gazebo (ROS'suz, hızlı); gz.sim Python bağları gerekir
state.py     simülasyon durumu (saf Python)
task.py      gözlem, eylem, ödül, devrilme (saf Python)
env.py       Gymnasium ortamı
demo.py      taklit için gösterim tripod'u (saf Python)
pretrain.py  taklit ile başlatma (behavior cloning)
train.py     PPO eğitimi
evaluate.py  eğitilmiş modeli ölçme
math3d.py    saf Python yardımcılar

Eğitilen politika gerçek robotta ROS arayüzü üzerinden çalışır (docs/ARAYUZ.md,
G8); bu paket yalnızca eğitim tarafı.
"""

__version__ = "0.1.0"
