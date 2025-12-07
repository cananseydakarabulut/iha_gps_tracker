import math
import time
from collections import deque
from itertools import permutations
import numpy as np


class RivalBehaviorMemory:
    def __init__(self):
        self.rival_history = {}
        self.trap_attempts = {}
        self.blacklist = set()

    def record_interaction(self, tid, interaction_type, outcome):
        if tid not in self.rival_history:
            self.rival_history[tid] = []
            self.trap_attempts[tid] = 0

        self.rival_history[tid].append(
            {"time": time.time(), "type": interaction_type, "outcome": outcome}
        )

        if interaction_type == "BOUNDARY_TRAP":
            self.trap_attempts[tid] += 1
            if self.trap_attempts[tid] >= 3:
                self.blacklist.add(tid)
                print(f"Rival {tid} blacklist'e eklendi (3 tuzak denemesi)")

    def is_blacklisted(self, tid):
        return tid in self.blacklist

    def get_success_rate(self, tid):
        if tid not in self.rival_history:
            return 0.0
        history = self.rival_history[tid]
        total = len(history)
        success = sum(1 for h in history if h.get("outcome") == "SUCCESS")
        return success / total if total > 0 else 0.0

    def recommend_strategy(self, tid):
        if self.is_blacklisted(tid):
            return "IGNORE"
        success_rate = self.get_success_rate(tid)
        if success_rate > 0.7:
            return "AGGRESSIVE"
        if success_rate > 0.3:
            return "NORMAL"
        return "CAUTIOUS"


class MultiTargetOptimizer:
    def optimize_target_sequence(self, my_pos, candidates, time_budget_remaining):
        if not candidates:
            return None
        best_sequence = None
        best_value = -float("inf")
        max_len = min(len(candidates), 3)
        for perm in permutations(candidates, max_len):
            value, feasible = self._evaluate_sequence(my_pos, perm, time_budget_remaining)
            if feasible and value > best_value:
                best_value = value
                best_sequence = perm
        return best_sequence[0] if best_sequence else candidates[0]

    def _evaluate_sequence(self, my_pos, sequence, time_budget):
        total_value = 0
        total_time = 0
        current_pos = my_pos
        for target in sequence:
            dist = math.hypot(target["x"] - current_pos[0], target["y"] - current_pos[1])
            travel_time = dist / 35.0
            lock_time = 5.0
            target_outside = math.hypot(target["x"], target["y"]) > 500.0
            outside_time = lock_time if target_outside else 0
            total_time += travel_time + lock_time
            if outside_time > time_budget:
                return total_value, False
            value = 100 / (1 + dist / 50.0)
            if target.get("hiz", 50) < 20:
                value *= 1.5
            total_value += value
            time_budget -= outside_time
            current_pos = (target["x"], target["y"], target.get("z", 0))
        return total_value, total_time < 60.0


class AltitudeOptimizer:
    def compute_optimal_altitude(self, my_pos, target, situation):
        target_alt = target.get("z", 50) if target else 50
        if situation == "PURSUIT":
            optimal = min(100.0, max(30.0, target_alt + 15.0))
            return optimal
        if situation == "LOCK":
            return target_alt + 5.0
        if situation == "ESCAPE":
            return 80.0
        if situation == "BOUNDARY":
            return 60.0
        return 50.0

    def compute_vertical_speed(self, current_alt, target_alt, dt):
        alt_diff = target_alt - current_alt
        max_vz = 3.0
        desired_vz = alt_diff / max(dt, 1.0)
        return float(np.clip(desired_vz, -max_vz, max_vz))


class EnergyAwareController:
    def __init__(self):
        self.energy_consumption_model = {
            "hover": 1.0,
            "cruise": 1.2,
            "sprint": 2.5,
            "maneuver": 1.8,
        }

    def get_performance_mode(self, battery_percent, mission_phase):
        if battery_percent < 15 or mission_phase == "CRITICAL":
            return {"mode": "RTB_ONLY", "max_speed": 20.0, "max_accel": 3.0, "allow_pursuit": False}
        if battery_percent < 30:
            return {
                "mode": "CONSERVATIVE",
                "max_speed": 40.0,
                "max_accel": 5.0,
                "allow_pursuit": True,
                "pursue_only_easy_targets": True,
            }
        if battery_percent < 50:
            return {"mode": "BALANCED", "max_speed": 55.0, "max_accel": 7.0, "allow_pursuit": True}
        return {"mode": "FULL_PERFORMANCE", "max_speed": 65.0, "max_accel": 10.0, "allow_pursuit": True}

    def estimate_remaining_mission_time(self, battery_percent, current_mode):
        consumption_rate = self.energy_consumption_model.get(current_mode, 1.2)
        remaining_minutes = (battery_percent / 100.0) * 20.0 / consumption_rate
        return remaining_minutes


class WindEstimatorAndCompensator:
    def __init__(self):
        self.wind_vx = 0.0
        self.wind_vy = 0.0
        self.confidence = 0.0
        self.samples = deque(maxlen=50)

    def update(self, gps_velocity, body_velocity, heading_rad):
        cos_h = math.cos(heading_rad)
        sin_h = math.sin(heading_rad)
        vx_body_world = body_velocity[0] * cos_h - body_velocity[1] * sin_h
        vy_body_world = body_velocity[0] * sin_h + body_velocity[1] * cos_h
        wind_sample_x = gps_velocity[0] - vx_body_world
        wind_sample_y = gps_velocity[1] - vy_body_world
        self.samples.append((wind_sample_x, wind_sample_y))
        if len(self.samples) > 10:
            samples_array = np.array(self.samples)
            self.wind_vx = float(np.median(samples_array[:, 0]))
            self.wind_vy = float(np.median(samples_array[:, 1]))
            std = np.std(samples_array, axis=0)
            self.confidence = float(1.0 / (1.0 + np.linalg.norm(std)))

    def get_compensated_heading(self, desired_ground_heading_rad, airspeed):
        wind_speed = math.sqrt(self.wind_vx**2 + self.wind_vy**2)
        if wind_speed < 0.5 or self.confidence < 0.3:
            return desired_ground_heading_rad
        wind_heading = math.atan2(self.wind_vy, self.wind_vx)
        relative_wind = wind_heading - desired_ground_heading_rad

        # Rüzgar çok güçlü olsa bile kompanzasyon yap
        if airspeed > wind_speed * 0.3:  # Minimum %30 hız şartı
            # Etkili airspeed hesapla (rüzgar güçlüyse daha agresif kompanzasyon)
            effective_airspeed = max(airspeed, wind_speed * 1.2)
            sin_crab = wind_speed * math.sin(relative_wind) / effective_airspeed
            sin_crab = float(np.clip(sin_crab, -1, 1))
            crab_angle = math.asin(sin_crab)
            if wind_speed > airspeed:
                print(f"Ruzgar guclu (W:{wind_speed:.1f} > A:{airspeed:.1f}), agresif kompanzasyon")
        else:
            # Çok düşük hızda - basit yön düzeltmesi
            crab_angle = -relative_wind * 0.3  # Rüzgara doğru 30% düzeltme
            print("Cok dusuk hiz, basit kompanzasyon")

        compensated_heading = desired_ground_heading_rad + crab_angle
        return compensated_heading

    def get_wind_info(self):
        speed = math.sqrt(self.wind_vx**2 + self.wind_vy**2)
        direction = math.degrees(math.atan2(self.wind_vy, self.wind_vx)) % 360
        return {
            "speed": speed,
            "direction": direction,
            "vx": self.wind_vx,
            "vy": self.wind_vy,
            "confidence": self.confidence,
        }


class TargetLossRecovery:
    def __init__(self):
        self.lost_targets = {}
        self.search_patterns = {
            "SPIRAL": self._spiral_search,
            "GRID": self._grid_search,
            "PREDICT": self._predictive_search,
        }

    def on_target_lost(self, tid, last_known_state):
        self.lost_targets[tid] = {
            "pos": last_known_state["pos"],
            "velocity": last_known_state.get("velocity", (0, 0, 0)),
            "time": time.time(),
            "search_started": False,
        }

    def get_search_waypoint(self, my_pos, tid, search_type="PREDICT"):
        if tid not in self.lost_targets:
            return None
        lost_info = self.lost_targets[tid]
        time_since_loss = time.time() - lost_info["time"]
        if time_since_loss > 10.0:
            del self.lost_targets[tid]
            return None
        search_func = self.search_patterns.get(search_type, self._predictive_search)
        return search_func(my_pos, lost_info, time_since_loss)

    def _predictive_search(self, my_pos, lost_info, time_elapsed):
        last_pos = lost_info["pos"]
        last_vel = lost_info["velocity"]
        predicted_x = last_pos[0] + last_vel[0] * time_elapsed
        predicted_y = last_pos[1] + last_vel[1] * time_elapsed
        predicted_z = last_pos[2]
        return (predicted_x, predicted_y, predicted_z)

    def _spiral_search(self, my_pos, lost_info, time_elapsed):
        r = 20 + 5 * time_elapsed
        angle = time_elapsed
        x = lost_info["pos"][0] + r * math.cos(angle)
        y = lost_info["pos"][1] + r * math.sin(angle)
        z = lost_info["pos"][2]
        return (x, y, z)

    def _grid_search(self, my_pos, lost_info, time_elapsed):
        step = 30.0
        idx = int(time_elapsed // 2) % 4
        offsets = [(step, 0), (-step, 0), (0, step), (0, -step)]
        dx, dy = offsets[idx]
        x = lost_info["pos"][0] + dx
        y = lost_info["pos"][1] + dy
        z = lost_info["pos"][2]
        return (x, y, z)
