"""Sensör düğümü: mesafe ve IMU verisini ROS'a yayınlar (GOREVLER.md S7).

    ros2 run hexapod_sensors sensors
    ros2 run hexapod_sensors sensors --ros-args -p rate_hz:=30.0

Yayınladıkları:
  /range0../range2   sensor_msgs/Range   mesafe (m); görmüyorsa max_range
  /imu               sensor_msgs/Imu     yönelim + açısal hız, GÖVDE çerçevesinde

Neden bu konular: politika düğümü (hexapod_policy) `/imu`'yu base_link
yöneliminde bekliyor (docs/ARAYUZ.md madde 3) ve kaldırma refleksi mesafe
istiyor (`controller.on_ranges`). Range için standart `sensor_msgs/Range`
yeterli (Görkem'in notu, GOREVLER.md S7).

Asıl mantık sürücülerde (vl53l0x, rangefinders, bno055, mount); bu dosya
yalnızca onları ROS'a bağlar — hexapod_teleop/hexapod_hardware ile aynı desen.

DONANIMDA DENENMEDİ. Sürücüler taklit cihazlarla yazmaç düzeyinde
doğrulandı; gerçek sensörle ilk çalıştırma D8'in işi. Kablolama
(`sensors.range_finders.devices[*].xshut_gpio/address`) ve IMU montajı
(`sensors.imu.mount_rotation_deg`) robot.yaml'da girilmeden düğüm başlamaz
ve neyin eksik olduğunu söyler.
"""

from __future__ import annotations

import sys


def main(argv: list[str] | None = None) -> int:
    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import Imu, Range

    from hexapod_driver import DryRunGpio, LgpioBackend, RobotConfig, SMBusBackend
    from hexapod_driver.errors import HexapodError
    from hexapod_driver.config import Value

    from .bno055 import Bno055
    from .mount import mount_from_config, quaternion_to_base, rotate_to_base
    from .rangefinders import RangeFinders, specs_from_config

    class StartupError(Exception):
        """Düğüm kurulamadı; mesaj kullanıcıya gösterilir."""

    class SensorsNode(Node):
        def __init__(self) -> None:
            super().__init__("hexapod_sensors")
            self.declare_parameter("rate_hz", 25.0)     # refleksin istediği en az hız
            self.declare_parameter("dry_run", False)
            self.declare_parameter("config", "")
            rate = float(self.get_parameter("rate_hz").value)
            dry_run = bool(self.get_parameter("dry_run").value)
            config_path = str(self.get_parameter("config").value) or None

            self._finders, self._imu, self._mount, self._field = self._build(
                config_path, dry_run)

            self._range_pubs = [
                self.create_publisher(Range, f"/range{spec.id}", 10)
                for spec in self._finders.specs
            ]
            self._imu_pub = self.create_publisher(Imu, "/imu", 10)
            self._timer = self.create_timer(1.0 / rate, self._tick)
            mode = "DRY-RUN" if dry_run else "GERÇEK DONANIM"
            self.get_logger().info(
                f"hexapod_sensors hazır [{mode}]: {len(self._range_pubs)} mesafe + IMU, "
                f"{rate:g} Hz")

        @staticmethod
        def _build(config_path, dry_run):
            try:
                config = RobotConfig.load(config_path)
                specs = specs_from_config(config)
                imu_raw = ((config.raw.get("sensors") or {}).get("imu") or {})
                imu_address = Value.parse(imu_raw.get("address"),
                                          "sensors.imu.address").require()
                matrix = mount_from_config(config)
                bus = int(imu_raw.get("i2c_bus", 1))
                # "görmüyor" eşiği robot.yaml'dan: politika düğümünün refleksi de
                # aynı değeri kullanıyor (hexapod_policy.ranges).
                rf_raw = ((config.raw.get("sensors") or {}).get("range_finders") or {})
                max_range = float(Value.parse(rf_raw.get("max_range_m"),
                                              "sensors.range_finders.max_range_m").require())

                if dry_run:
                    from .fake import FakeBno055, FakeI2CBus, FakeVl53l0x, FakeXshutGpio
                    devices = {s.xshut_gpio: FakeVl53l0x(distance_mm=400) for s in specs}
                    i2c = FakeI2CBus({int(imu_address): FakeBno055(int(imu_address))})
                    gpio = FakeXshutGpio(i2c, devices)
                else:
                    i2c, gpio = SMBusBackend(bus), LgpioBackend()

                finders = RangeFinders(specs, i2c, gpio, imu_address=int(imu_address),
                                       max_range_m=max_range)
                finders.begin()
                imu = Bno055(i2c, int(imu_address))
                imu.begin()
                return finders, imu, matrix, "imu_link"
            except HexapodError as exc:
                raise StartupError(str(exc)) from exc

        def shutdown(self) -> None:
            try:
                self._finders.close()
            except Exception:
                pass

        def _tick(self) -> None:
            now = self.get_clock().now().to_msg()
            try:
                distances = self._finders.read()
            except HexapodError as exc:
                self.get_logger().warning(f"mesafe okunamadı: {exc}",
                                          throttle_duration_sec=2.0)
                distances = None
            if distances is not None:
                for pub, spec, distance in zip(self._range_pubs, self._finders.specs,
                                               distances):
                    msg = Range()
                    msg.header.stamp = now
                    msg.header.frame_id = f"range{spec.id}"
                    msg.radiation_type = Range.INFRARED
                    msg.field_of_view = 0.44          # VL53L0X ~25°, veri sayfası
                    msg.min_range = 0.03
                    msg.max_range = float(self._finders.max_range_m)
                    msg.range = float(distance)
                    pub.publish(msg)

            try:
                gravity = self._imu.gravity_direction()
                omega = self._imu.angular_velocity()
                quat = self._imu.quaternion()
            except HexapodError as exc:
                self.get_logger().warning(f"IMU okunamadı: {exc}", throttle_duration_sec=2.0)
                return

            # Sensör çerçevesinden gövde çerçevesine: politika base_link bekliyor.
            gx, gy, gz = rotate_to_base(self._mount, gravity)
            wx, wy, wz = rotate_to_base(self._mount, omega)
            msg = Imu()
            msg.header.stamp = now
            msg.header.frame_id = self._field
            # Quaternion da gövde çerçevesine: politika yönelimi BUNDAN
            # okuyor (controller.on_imu), ivmeden değil. Montaj dönüşü burada
            # atlanırsa IMU dönük takılıyken eğiklik yanlış okunur.
            qw, qx, qy, qz = quaternion_to_base(self._mount, quat)
            msg.orientation.w, msg.orientation.x = float(qw), float(qx)
            msg.orientation.y, msg.orientation.z = float(qy), float(qz)
            msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z = (
                float(wx), float(wy), float(wz))
            # Yerçekimi YÖNÜ (birim), ivme yazmacına m/s² olarak: politika
            # gözlemi bunu birim vektöre çeviriyor, ölçek önemli değil ama
            # işaret ve yön önemli.
            msg.linear_acceleration.x = float(-gx * 9.81)
            msg.linear_acceleration.y = float(-gy * 9.81)
            msg.linear_acceleration.z = float(-gz * 9.81)
            self._imu_pub.publish(msg)

    rclpy.init(args=argv)
    try:
        node = SensorsNode()
    except StartupError as exc:
        print(f"HATA: sensör düğümü başlatılamadı: {exc}", file=sys.stderr)
        rclpy.try_shutdown()
        return 2

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception:
        # Ctrl+C / SIGTERM bağlamı kapatır; spin Lyrical'da
        # ExternalShutdownException ya da RCLError ile çıkar (PROJE_DEVIR §12.19).
        if rclpy.ok():
            raise
    finally:
        node.shutdown()
        node.destroy_node()
        rclpy.try_shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
