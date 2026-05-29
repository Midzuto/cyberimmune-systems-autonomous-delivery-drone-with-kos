#!/usr/bin/env python3
"""
E2E тест: проверяет что при включённом ветре и выходе дрона
за коридор 10 метров модуль безопасности инициирует посадку.
"""

import sys
import time
import math

from pymavlink import mavutil

# ─── Настройки ───────────────────────────────────────────
DRONE_ADDR   = "tcp:172.28.0.2:5760"  # адрес симулятора
CORRIDOR_M   = 10.0                    # радиус коридора в метрах
TIMEOUT_S    = 180                     # макс. время теста в секундах
ARM_TIMEOUT  = 60                      # ожидание ARM

# Режим LAND в ArduCopter = 9
LAND_MODE    = 9
# ─────────────────────────────────────────────────────────


def dist_meters(lat1, lon1, lat2, lon2):
    """Расстояние между двумя точками в метрах (плоское приближение)."""
    dlat = (lat1 - lat2) / 1e7 * 111320.0
    dlon = (lon1 - lon2) / 1e7 * 111320.0 * \
           math.cos(math.radians(lat1 / 1e7))
    return math.sqrt(dlat**2 + dlon**2)


def wait_heartbeat(master, timeout=30):
    print("[TEST] Ожидаем heartbeat от дрона...")
    master.wait_heartbeat(timeout=timeout)
    print(f"[TEST] Подключились! system={master.target_system} "
          f"component={master.target_component}")


def arm_and_start_mission(master):
    """Армирует дрон и запускает выполнение миссии."""
    print("[TEST] Армируем дрон...")

    # Ждём ARM (модуль безопасности должен его разрешить)
    t0 = time.time()
    while time.time() - t0 < ARM_TIMEOUT:
        master.mav.command_long_send(
            master.target_system,
            master.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0, 1, 0, 0, 0, 0, 0, 0
        )
        msg = master.recv_match(type='COMMAND_ACK', blocking=True, timeout=3)
        if msg and msg.command == mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM:
            if msg.result == 0:
                print("[TEST] Дрон заармирован!")
                break
        time.sleep(2)
    else:
        print("[TEST] FAIL: не удалось заармировать дрон")
        sys.exit(1)

    # Запускаем миссию
    print("[TEST] Запускаем миссию...")
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_MISSION_START,
        0, 0, 0, 0, 0, 0, 0, 0
    )
    time.sleep(3)


def get_current_position(master):
    """Возвращает текущие lat, lon или None."""
    msg = master.recv_match(
        type='GLOBAL_POSITION_INT', blocking=True, timeout=5)
    if msg:
        return msg.lat, msg.lon
    return None, None


def main():
    print("=" * 50)
    print("[TEST] E2E тест: ветер + коридор безопасности")
    print("=" * 50)

    # Подключаемся к симулятору
    master = mavutil.mavlink_connection(DRONE_ADDR)
    wait_heartbeat(master)

    # Армируем и стартуем миссию
    arm_and_start_mission(master)

    # Запоминаем стартовую позицию
    print("[TEST] Получаем стартовую позицию...")
    start_lat, start_lon = None, None
    for _ in range(10):
        start_lat, start_lon = get_current_position(master)
        if start_lat is not None:
            break
        time.sleep(1)

    if start_lat is None:
        print("[TEST] FAIL: не удалось получить стартовую позицию")
        sys.exit(1)

    print(f"[TEST] Стартовая позиция: lat={start_lat} lon={start_lon}")
    print(f"[TEST] Ждём выхода дрона за {CORRIDOR_M} м и автопосадки...")
    print(f"[TEST] Таймаут: {TIMEOUT_S} сек")
    print("-" * 50)

    # Основной цикл: следим за позицией и режимом полёта
    t0 = time.time()
    max_dist = 0.0
    landed   = False

    while time.time() - t0 < TIMEOUT_S:

        # Текущие координаты
        cur_lat, cur_lon = get_current_position(master)
        if cur_lat is None:
            time.sleep(0.5)
            continue

        # Расстояние от старта
        dist = dist_meters(cur_lat, cur_lon, start_lat, start_lon)
        if dist > max_dist:
            max_dist = dist

        elapsed = time.time() - t0
        print(f"[TEST] t={elapsed:.0f}s  dist={dist:.1f}m  "
              f"(max={max_dist:.1f}m)", end="\r")

        # Проверяем режим полёта
        hb = master.recv_match(type='HEARTBEAT', blocking=False)
        if hb is not None:
            mode = hb.custom_mode
            if mode == LAND_MODE:
                print(f"\n[TEST] Дрон перешёл в режим LAND!")
                if dist > CORRIDOR_M:
                    print(f"[TEST] Расстояние при посадке: {dist:.1f} м "
                          f"(>{CORRIDOR_M} м) — корридор был нарушен ✓")
                    landed = True
                    break
                else:
                    print(f"[TEST] Посадка, но дрон ещё в коридоре "
                          f"({dist:.1f} м) — ждём дальше...")

        time.sleep(0.5)

    print()
    print("=" * 50)
    if landed:
        print("[TEST] PASS ✅  Модуль безопасности сработал корректно:")
        print(f"       - Ветер снёс дрон на {max_dist:.1f} м")
        print(f"       - При выходе за {CORRIDOR_M} м инициирована посадка")
        sys.exit(0)
    else:
        print("[TEST] FAIL ❌  За отведённое время посадка не была")
        print(f"       инициирована. Макс. отклонение: {max_dist:.1f} м")
        sys.exit(1)


if __name__ == "__main__":
    main()
