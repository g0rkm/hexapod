"""Zeminden ROS'lu sim dünyası (sim.launch.py world:=...) üret.

Fizik ayarları ve sistem eklentileri hexapod_gazebo/worlds/flat.sdf ile AYNI
olmak zorunda (IMU ve Contact eklentileri robotun sensörleri için; dünya
dosyası eklenti tanımlarsa Gazebo varsayılanları yüklemez). Yalnız zemin
modeli değişir.

DİKKAT: ROS'lu sim (sim.launch.py) robotu düz zemine göre doğurur, zemin
yüksekliğini bilmez. Bu yüzden orijini z=0'da olmayan zeminlerde (yayla) robot
havada ya da gömülü doğar. Süreç içi sim (hexapod_rl.sim) zeminin height
fonksiyonuna göre doğurduğu için (spawn_height) böyle bir sorunu yok; eğitim ve
ölçüm oradan yapılıyor. ROS'lu dünyalar gözle bakmak ve G8 denemeleri için.
"""

from __future__ import annotations

from pathlib import Path

from .terrain import Terrain

_HEADER = "<!-- ÜRETİLDİ: hexapod_terrain.world (GOREVLER.md S5). Elle düzenlemeyin. -->"


def world_sdf(item: Terrain | str, name: str = "zemin") -> str:
    """Zemini tam bir dünya dosyasına göm."""
    ground = item.sdf if isinstance(item, Terrain) else item
    label = item.label if isinstance(item, Terrain) else name
    return f"""<?xml version="1.0"?>
{_HEADER}
<!-- zemin: {label} -->
<sdf version="1.9">
  <world name="{name}">
    <physics name="1ms" type="ignored">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
    </physics>

    <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
    <plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"/>
    <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>
    <plugin filename="gz-sim-imu-system" name="gz::sim::systems::Imu"/>
    <plugin filename="gz-sim-contact-system" name="gz::sim::systems::Contact"/>

    <light type="directional" name="sun">
      <cast_shadows>true</cast_shadows>
      <pose>0 0 10 0 0 0</pose>
      <diffuse>0.8 0.8 0.8 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular>
      <direction>-0.5 0.1 -0.9</direction>
    </light>

    {ground}
  </world>
</sdf>
"""


def write_world(item: Terrain, path: str | Path, name: str = "zemin") -> Path:
    """Dünya dosyasını yaz. Satır sonu LF (robot Linux'ta; .gitattributes)."""
    p = Path(path)
    p.parent.mkdir(parents=True, exist_ok=True)
    p.write_text(world_sdf(item, name), encoding="utf-8", newline="\n")
    return p


__all__ = ["world_sdf", "write_world"]
