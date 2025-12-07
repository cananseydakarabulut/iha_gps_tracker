import math
import time


class BoundaryTimeBudgetManager:
    """
    Saha disi zaman butcesi ve hiz profili yonetimi.
    """

    def __init__(self):
        self.TOTAL_BUDGET_S = 25.0
        self.CRITICAL_THRESHOLD_S = 20.0
        self.WARNING_THRESHOLD_S = 15.0
        self.total_outside_time = 0.0
        self.current_outside_session_start = None
        self.outside_sessions = []
        self.remaining_budget = self.TOTAL_BUDGET_S
        self.is_outside = False
        self.current_session_reason = None
        self.speed_profiles = {
            "RELAXED": {"speed": 25.0, "turn_rate": 30.0},
            "NORMAL": {"speed": 35.0, "turn_rate": 45.0},
            "URGENT": {"speed": 50.0, "turn_rate": 60.0},
            "CRITICAL": {"speed": 65.0, "turn_rate": 90.0},
            "PANIC": {"speed": 70.0, "turn_rate": 120.0},
        }

    def start_outside_session(self, reason="UNKNOWN"):
        if not self.is_outside:
            self.current_outside_session_start = time.time()
            self.is_outside = True
            self.current_session_reason = reason
            print(f"\nSaha disi oturum basladi: {reason} (Kalan: {self.remaining_budget:.1f}s)")

    def end_outside_session(self):
        if self.is_outside and self.current_outside_session_start:
            duration = time.time() - self.current_outside_session_start
            self.outside_sessions.append(
                {
                    "start": self.current_outside_session_start,
                    "end": time.time(),
                    "duration": duration,
                    "reason": self.current_session_reason,
                }
            )
            self.total_outside_time += duration
            self.remaining_budget = max(0, self.TOTAL_BUDGET_S - self.total_outside_time)
            print(
                f"\nSaha icine girildi. Oturum: {duration:.1f}s | Toplam: {self.total_outside_time:.1f}s | Kalan: {self.remaining_budget:.1f}s"
            )
            self.is_outside = False
            self.current_outside_session_start = None
            self.current_session_reason = None

    def get_current_outside_duration(self):
        if self.is_outside and self.current_outside_session_start:
            return time.time() - self.current_outside_session_start
        return 0.0

    def _get_speed_multiplier(self, urgency):
        multipliers = {
            "PANIC": 1.3,
            "CRITICAL": 1.2,
            "URGENT": 1.1,
            "NORMAL": 1.0,
            "RELAXED": 0.9,
        }
        return multipliers.get(urgency, 1.0)

    def _get_lock_speed_profile(self, urgency):
        if urgency == "PANIC":
            lock_speed = 40.0
        elif urgency == "CRITICAL":
            lock_speed = 35.0
        elif urgency == "URGENT":
            lock_speed = 30.0
        else:
            lock_speed = 25.0
        return {"speed": lock_speed, "turn_rate": 45.0}

    def _select_speed_profile(self, urgency, lock_status):
        if lock_status and lock_status.get("is_locking"):
            return self._get_lock_speed_profile(urgency)
        return self.speed_profiles.get(urgency, self.speed_profiles["NORMAL"])

    def _calculate_urgency(self, current_duration):
        budget_ratio = self.remaining_budget / self.TOTAL_BUDGET_S
        session_ratio = current_duration / 8.0
        urgency_score = 0.0
        if budget_ratio < 0.1:
            urgency_score += 0.6
        elif budget_ratio < 0.2:
            urgency_score += 0.5
        elif budget_ratio < 0.4:
            urgency_score += 0.3
        elif budget_ratio < 0.6:
            urgency_score += 0.1
        if session_ratio > 0.9:
            urgency_score += 0.4
        elif session_ratio > 0.7:
            urgency_score += 0.3
        elif session_ratio > 0.5:
            urgency_score += 0.2
        elif session_ratio > 0.3:
            urgency_score += 0.1
        if urgency_score > 0.8:
            return "PANIC"
        if urgency_score > 0.6:
            return "CRITICAL"
        if urgency_score > 0.4:
            return "URGENT"
        if urgency_score > 0.2:
            return "NORMAL"
        return "RELAXED"

    def calculate_return_eta(self, my_pos, speed_profile):
        dist_to_center = math.sqrt(my_pos[0] ** 2 + my_pos[1] ** 2)
        return_speed = speed_profile["speed"]
        eta_seconds = dist_to_center / max(return_speed, 1.0)
        return eta_seconds

    def should_panic_return(self, my_pos):
        eta = self.calculate_return_eta(my_pos, self.speed_profiles["CRITICAL"])
        safe_eta = eta * 1.2
        if safe_eta > self.remaining_budget:
            return True, f"Donus: {eta:.1f}s butce: {self.remaining_budget:.1f}s"
        return False, None

    def get_decision(self, lock_status, target_info=None):
        current_duration = self.get_current_outside_duration()
        urgency = self._calculate_urgency(current_duration)
        speed_profile = self._select_speed_profile(urgency, lock_status)
        speed_multiplier = self._get_speed_multiplier(urgency)

        if self.remaining_budget < 2.0:
            return {
                "decision": "IMMEDIATE_RETURN",
                "priority": "CRITICAL",
                "allowed_time": 0.0,
                "reason": "BUTCE TUKENDI",
                "commanded_action": "rtl",
                "speed_profile": self.speed_profiles["PANIC"],
                "speed_multiplier": 1.0,
                "urgency_level": "PANIC",
            }
        if current_duration > 8.0:
            return {
                "decision": "IMMEDIATE_RETURN",
                "priority": "HIGH",
                "allowed_time": 0.0,
                "reason": "TEK OTURUM LIMITI",
                "commanded_action": "return_now",
                "speed_profile": self.speed_profiles["CRITICAL"],
                "speed_multiplier": 1.0,
                "urgency_level": "CRITICAL",
            }
        if lock_status and lock_status.get("is_locking"):
            progress = lock_status.get("lock_progress", 0)
            time_remaining = lock_status.get("time_remaining", 5.0)
            if progress > 0.8:
                return {
                    "decision": "STAY_FOR_LOCK",
                    "priority": "HIGH",
                    "allowed_time": min(time_remaining + 1.0, self.remaining_budget),
                    "reason": f"KILIT %{int(progress*100)}",
                    "commanded_action": "continue_lock",
                    "speed_profile": self._get_lock_speed_profile(urgency),
                    "speed_multiplier": speed_multiplier,
                    "urgency_level": urgency,
                }
            if progress > 0.5:
                return {
                    "decision": "CAUTIOUS_LOCK",
                    "priority": "MEDIUM",
                    "allowed_time": min(3.0, self.remaining_budget - current_duration),
                    "reason": f"KILIT %{int(progress*100)} TEMKINLI",
                    "commanded_action": "continue_cautious",
                    "speed_profile": self._get_lock_speed_profile(urgency),
                    "speed_multiplier": speed_multiplier,
                    "urgency_level": urgency,
                }
            return {
                "decision": "ABORT_LOCK",
                "priority": "LOW",
                "allowed_time": 1.0,
                "reason": "KILIT BASLANGIC - BUTCE KORU",
                "commanded_action": "abort_and_return",
                "speed_profile": self.speed_profiles["URGENT"],
                "speed_multiplier": 1.0,
                "urgency_level": urgency,
            }
        if current_duration > 2.0:
            return {
                "decision": "RETURN_NO_LOCK",
                "priority": "MEDIUM",
                "allowed_time": 0.5,
                "reason": "KILIT YOK - DON",
                "commanded_action": "return_now",
                "speed_profile": speed_profile,
                "speed_multiplier": speed_multiplier,
                "urgency_level": urgency,
            }
        if target_info:
            target_distance = target_info.get("distance", 999)
            target_speed = target_info.get("speed", 50)
            if target_distance < 30 and target_speed < 15:
                return {
                    "decision": "STAY_FOR_OPPORTUNITY",
                    "priority": "LOW",
                    "allowed_time": 3.0,
                    "reason": "HEDEF YAKIN - FIRSAT",
                    "commanded_action": "pursue",
                    "speed_profile": self.speed_profiles["NORMAL"],
                    "speed_multiplier": 0.8,
                    "urgency_level": urgency,
                }
        return {
            "decision": "MONITOR_BRIEF",
            "priority": "LOW",
            "allowed_time": 2.0,
            "reason": "IZLENIYOR",
            "commanded_action": "monitor",
            "speed_profile": self.speed_profiles["RELAXED"],
            "speed_multiplier": 1.0,
            "urgency_level": urgency,
        }

    def get_status_summary(self):
        return {
            "total_outside_time": self.total_outside_time,
            "remaining_budget": self.remaining_budget,
            "current_session_duration": self.get_current_outside_duration(),
            "is_outside": self.is_outside,
            "sessions_count": len(self.outside_sessions),
        }
