"""hexapod_terrain — RL eğitimi ve ölçümü için Gazebo zeminleri (GOREVLER.md S5).

Saf Python: gz, ROS, numpy gerektirmez. Her zemin bir `Terrain`: `sdf` (Gazebo
modeli) + `height(x, y)` (yüzey yüksekliği, m). İkisi birebir tutarlıdır;
hexapod_rl ayak temasını, gövde yüksekliğini, devrilmeyi ve robotun doğduğu
yüksekliği `height`'a göre hesaplıyor.

    from hexapod_terrain import terrain, sets
    sdf, height = terrain.slope(-20.0)                  # 20° yokuş yukarı
    env = HexapodEnv(terrain_sdf=sdf, terrain_height=height)

    for z in sets.levels("engebe"):                     # kolaydan zora
        print(z.label)

terrain.py  zemin üreticileri (düz, eğim, basamak, merdiven, engebe, çukur, yayla)
sets.py     zorluk seviyeleri, müfredat basamakları, S6'nın ölçüm listesi
world.py    ROS'lu sim için dünya dosyası (sim.launch.py world:=...)
"""

from .sets import difficulty, evaluation_set, kinds, level, levels
from .terrain import Terrain, flat, pit, plateau, rough, slope, stairs, step
from .world import world_sdf, write_world

__all__ = [
    "Terrain",
    "difficulty", "evaluation_set", "kinds", "level", "levels",
    "flat", "pit", "plateau", "rough", "slope", "stairs", "step",
    "world_sdf", "write_world",
]

__version__ = "0.1.0"
