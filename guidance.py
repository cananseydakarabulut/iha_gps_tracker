import math
from dataclasses import dataclass
import numpy as np
from tactical_modules import AltitudeOptimizer, EnergyAwareController, WindEstimatorAndCompensator

# ==========================
# AYARLAR
# ==========================
# Saha parametreleri
SAHA_YARICAPI = 500.0  # metre
GUVENLI_IRTIFA = 50.0   # Saha disina cikinca donulecek irtifa
VISUAL_DIST_THRESH_M = 35.0  # Gorsel moda gecis mesafe esigi
VISUAL_SPEED_THRESH = 6.0    # Gorsel moda gecis hedef hiz esigi


# ==========================
# KOMUT VER YAPISI
# ==========================
@dataclass
class GuidanceCommand:
    desired_speed_ms: float
    desired_yaw_deg: float
    desired_altitude_m: float
    distance_to_target_m: float
    mode: str


# ==========================
# AI NORMALZASYONU
# ==========================
def _normalize_deg(angle: float) -> float:
    """Ay (-180, 180] aralna evirir."""
    while angle > 180.0:
        angle -= 360.0
    while angle <= -180.0:
        angle += 360.0
    return angle


# ==========================
# GPS TAKP KONTROLCS
# ==========================
class GpsPursuitController:
    def __init__(
        self,
        min_speed_ms: float = 12.0,
        max_speed_ms: float = 65.0,
        full_brake_speed_ms: float = 6.0,
        max_escape_speed_ms: float = 55.0,
        brake_distance_m: float = 30.0,
        hard_brake_distance_m: float = 12.0,
        cruise_distance_m: float = 120.0,
        catchup_gain: float = 0.25,
        max_yaw_step_deg: float = 6.0,
    ):
        self.min = min_speed_ms
        self.max = max_speed_ms
        self.full_brake = full_brake_speed_ms
        self.max_escape_speed = max_escape_speed_ms
        self.brake = brake_distance_m
        self.hard_brake = hard_brake_distance_m
        self.cruise = cruise_distance_m
        self.k = catchup_gain

        self.max_yaw_step = max_yaw_step_deg
        self._last_yaw_cmd = None
        # Geofence bilincli takip
        self.arena_radius = SAHA_YARICAPI
        self.safe_margin = 50.0
        self.safe_radius = max(0.0, self.arena_radius - self.safe_margin)
        # Opsiyonel optimizasyonlar
        self.altitude_opt = AltitudeOptimizer()
        self.energy_ctrl = EnergyAwareController()
        self.wind_comp = WindEstimatorAndCompensator()

    # -----------------------------------------
    # YAW YUMUATMA (SMOOTHING)
    # -----------------------------------------
    def _smooth_yaw(self, desired_yaw: float) -> float:
        if self._last_yaw_cmd is None:
            self._last_yaw_cmd = desired_yaw
            return desired_yaw

        diff = _normalize_deg(desired_yaw - self._last_yaw_cmd)

        if diff > self.max_yaw_step:
            diff = self.max_yaw_step
        elif diff < -self.max_yaw_step:
            diff = -self.max_yaw_step

        new_yaw = (self._last_yaw_cmd + diff) % 360.0
        self._last_yaw_cmd = new_yaw
        return new_yaw

    # ======================================================
    #             ARPIMA/KAI ANALZ
    # ======================================================
    def _escape_check(self, dist, target_speed, angle_diff, vertical_sep=None):
        """
        Çarpışma riski kontrolü (3D - Dikey ayrım dahil)
        dist: 3D mesafe (m)
        target_speed: Hedefin hızı (m/s)
        angle_diff: Yönelim farkı (derece)
        vertical_sep: Dikey ayrım |my_z - target_z| (m)
        """
        # Dikey ayrım varsa güvenlik mesafesi artırılabilir
        safe_vertical_sep = 15.0  # 15m dikey ayrım güvenli sayılır

        # 1) Çok yakın çarpışma riski (3D)
        if dist < 10.0:
            # Ama yeterli dikey ayrım varsa escape gerekmez
            if vertical_sep and vertical_sep > safe_vertical_sep:
                return False, ""
            return True, "KRITIK YAKINLIK"

        # 2) Rakip tam bize bakıyor (Kafa kafaya)
        if angle_diff > 130 and dist < 45.0:
            if vertical_sep and vertical_sep > safe_vertical_sep:
                return False, ""
            return True, "KAFA KAFA"

        # 3) Rakip çok hızlı yaklaşıyor
        if target_speed > 50.0 and dist < 35.0:
            if vertical_sep and vertical_sep > safe_vertical_sep:
                return False, ""
            return True, "HIZ TEHDIT"

        return False, ""

    # -----------------------------------------
    # NGRL TAKP (LEAD PURSUIT)
    # -----------------------------------------
    def _lead_pursuit(self, my_pos, target):
        """
        Hedefin ileride olaca noktay tahmin eder; dz hat + basit yaw_rate turn.
        """
        tgt_speed = float(target.get("hiz", 0.0))
        tgt_yaw_deg = float(target.get("yaw_deg", 0.0))
        tgt_yaw_rate = float(target.get("yaw_rate", 0.0))  # deg/s

        dx = target["x"] - my_pos[0]
        dy = target["y"] - my_pos[1]
        dist = math.hypot(dx, dy)

        my_max = max(self.max, 1e-3)
        look_ahead = dist / my_max

        tgt_vx = tgt_speed * math.cos(math.radians(tgt_yaw_deg))
        tgt_vy = tgt_speed * math.sin(math.radians(tgt_yaw_deg))

        # Dz uu tahmini
        pred_x = target["x"] + tgt_vx * look_ahead
        pred_y = target["y"] + tgt_vy * look_ahead

        # Dn tahmini
        if abs(tgt_yaw_rate) > 1e-3:
            yaw_rate_rad = math.radians(tgt_yaw_rate)
            turn_radius = tgt_speed / max(abs(yaw_rate_rad), 1e-3)
            ang_change = yaw_rate_rad * look_ahead
            pred_x = target["x"] + turn_radius * (
                math.sin(math.radians(tgt_yaw_deg) + ang_change) - math.sin(math.radians(tgt_yaw_deg))
            )
            pred_y = target["y"] + turn_radius * (
                math.cos(math.radians(tgt_yaw_deg)) - math.cos(math.radians(tgt_yaw_deg) + ang_change)
            )

        return pred_x, pred_y

    # -----------------------------------------
    # GEOFENCE YARDIMCI KOMUTLAR
    # -----------------------------------------
    def _emergency_return(self, my_pos):
        center_yaw = math.degrees(math.atan2(-my_pos[1], -my_pos[0])) % 360.0
        return GuidanceCommand(
            desired_speed_ms=min(self.max, 50.0),
            desired_yaw_deg=self._smooth_yaw(center_yaw),
            desired_altitude_m=60.0,
            distance_to_target_m=math.hypot(my_pos[0], my_pos[1]),
            mode="SAHA DISI - ACIL DON",
        )

    def _refuse_boundary_target(self, my_pos):
        center_yaw = math.degrees(math.atan2(-my_pos[1], -my_pos[0])) % 360.0
        return GuidanceCommand(
            desired_speed_ms=30.0,
            desired_yaw_deg=self._smooth_yaw(center_yaw),
            desired_altitude_m=max(my_pos[2], 50.0),
            distance_to_target_m=9999.9,
            mode="HEDEF SINIRDA - RED",
        )

    def _ignore_outside_target(self, my_pos):
        my_dist = math.hypot(my_pos[0], my_pos[1])
        if my_dist < 100.0:
            patrol_yaw = (math.degrees(math.atan2(my_pos[1], my_pos[0])) + 90) % 360.0
            speed = 25.0
            mode = "MERKEZ PATROL"
        else:
            center_yaw = math.degrees(math.atan2(-my_pos[1], -my_pos[0])) % 360.0
            patrol_yaw = center_yaw
            speed = 30.0
            mode = "MERKEZE YAKLAS"
        return GuidanceCommand(
            desired_speed_ms=speed,
            desired_yaw_deg=self._smooth_yaw(patrol_yaw),
            desired_altitude_m=80.0,
            distance_to_target_m=9999.9,
            mode=f"HEDEF DISARIDA - {mode}",
        )

    def _boundary_aware_pursuit(self, my_pos, target):
        dx = target["x"] - my_pos[0]
        dy = target["y"] - my_pos[1]
        dist = math.hypot(dx, dy)
        pursuit_yaw = math.degrees(math.atan2(dy, dx)) % 360.0
        center_yaw = math.degrees(math.atan2(-my_pos[1], -my_pos[0])) % 360.0
        angle_diff = abs((pursuit_yaw - center_yaw + 180) % 360 - 180)
        if angle_diff < 90:
            speed = min(self.max * 0.8, target.get("hiz", self.max) + 5.0)
            final_yaw = pursuit_yaw
            mode_suffix = "SINIR - GUVENLI TAKIP"
        else:
            blend_yaw = (pursuit_yaw * 0.3 + center_yaw * 0.7) % 360.0
            speed = min(self.max * 0.6, 35.0)
            final_yaw = blend_yaw
            mode_suffix = "SINIR - TEMKINLI"
        return GuidanceCommand(
            desired_speed_ms=speed,
            desired_yaw_deg=self._smooth_yaw(final_yaw),
            desired_altitude_m=target["z"],
            distance_to_target_m=dist,
            mode=mode_suffix,
        )

    def _trap_aware_pursuit(self, my_pos, target):
        dx = target["x"] - my_pos[0]
        dy = target["y"] - my_pos[1]
        dist = math.sqrt(dx * dx + dy * dy + (target["z"] - my_pos[2]) ** 2)
        target_heading = target.get("yaw_deg", 0.0)
        target_to_center = math.degrees(math.atan2(-target["y"], -target["x"])) % 360.0
        heading_diff = abs((target_heading - target_to_center + 180) % 360 - 180)
        if heading_diff < 90:
            center_yaw = math.degrees(math.atan2(-my_pos[1], -my_pos[0])) % 360.0
            return GuidanceCommand(
                desired_speed_ms=25.0,
                desired_yaw_deg=self._smooth_yaw(center_yaw),
                desired_altitude_m=70.0,
                distance_to_target_m=dist,
                mode="TUZAK - MERKEZE DON",
            )
        safe_follow = 60.0
        if dist < safe_follow:
            speed = self.min
            mode = "SINIR YAKINI - MESAFE KORU"
        else:
            speed = min(self.max * 0.7, target.get("hiz", self.max))
            mode = "SINIR YAKINI - TEMKINLI TAKIP"
        pursuit_yaw = math.degrees(math.atan2(dy, dx)) % 360.0
        return GuidanceCommand(
            desired_speed_ms=speed,
            desired_yaw_deg=self._smooth_yaw(pursuit_yaw),
            desired_altitude_m=target["z"],
            distance_to_target_m=dist,
            mode=mode,
        )

    # -----------------------------------------
    # ANA GUIDANCE HESABI
    # -----------------------------------------
    def compute_command(self, my_pos, my_speed, target):
        """
        my_pos   : (x, y, z)
        my_speed : float (HA'nn kendi hz)
        target   : RivalTracker'dan gelen dict
        """

        # 0. SAHA DII KONTROL
        field_radius = SAHA_YARICAPI + 50.0
        return_speed = min(self.max, max(self.min + 5.0, 22.0))
        dist_from_center = math.sqrt(my_pos[0] ** 2 + my_pos[1] ** 2)

        if dist_from_center > field_radius:
            return_yaw = math.degrees(math.atan2(-my_pos[1], -my_pos[0])) % 360.0
            return GuidanceCommand(
                desired_speed_ms=return_speed,
                desired_yaw_deg=self._smooth_yaw(return_yaw),
                desired_altitude_m=my_pos[2],
                distance_to_target_m=dist_from_center,
                mode="SAHA DISI - MERKEZE DON",
            )

        if dist_from_center > SAHA_YARICAPI:
            center_yaw = math.degrees(math.atan2(-my_pos[1], -my_pos[0])) % 360.0
            return GuidanceCommand(
                desired_speed_ms=self.max,
                desired_yaw_deg=self._smooth_yaw(center_yaw),
                desired_altitude_m=GUVENLI_IRTIFA,
                distance_to_target_m=9999.9,
                mode="SAHA DISI - DON",
            )

        # Geofence aware kontroller
        my_dist_center = math.hypot(my_pos[0], my_pos[1])
        tgt_dist_center = math.hypot(target["x"], target["y"])

        if my_dist_center > self.arena_radius:
            return self._emergency_return(my_pos)

        if my_dist_center > self.safe_radius and tgt_dist_center > self.safe_radius:
            return self._refuse_boundary_target(my_pos)

        if tgt_dist_center > self.arena_radius:
            return self._ignore_outside_target(my_pos)

        if my_dist_center > self.safe_radius:
            return self._boundary_aware_pursuit(my_pos, target)

        if tgt_dist_center > self.safe_radius:
            trap_cmd = self._trap_aware_pursuit(my_pos, target)
            if trap_cmd:
                return trap_cmd

        # 1. HEDEF MESAFE VE KONUM FARKI (ongorulu takip)
        lead_x, lead_y = self._lead_pursuit(my_pos, target)
        dx = lead_x - my_pos[0]
        dy = lead_y - my_pos[1]
        dz = target["z"] - my_pos[2]
        dist = math.sqrt(dx * dx + dy * dy + dz * dz)

        # 2. AILAR
        raw_yaw = math.degrees(math.atan2(dy, dx)) % 360.0
        # Ruzgar kompanzasyonu (ground yaw -> crab angle)
        compensated_yaw_rad = self.wind_comp.get_compensated_heading(math.radians(raw_yaw), my_speed)
        compensated_yaw = math.degrees(compensated_yaw_rad) % 360.0
        target_heading = target.get("yaw_deg", compensated_yaw)
        angle_diff = abs(_normalize_deg(compensated_yaw - target_heading))

        target_speed = float(target.get("hiz", 20.0))

        # 3. Dikey ayrım hesapla
        vertical_separation = abs(dz)

        # 4. ÇARPIŞMA KONTROLÜ (dikey ayrım dahil)
        escape, esc_reason = self._escape_check(dist, target_speed, angle_diff, vertical_separation)
        if escape:
            yaw1 = (target_heading + 90) % 360
            yaw2 = (target_heading - 90) % 360
            current_yaw = self._last_yaw_cmd if self._last_yaw_cmd is not None else raw_yaw

            if abs(_normalize_deg(yaw1 - current_yaw)) < abs(_normalize_deg(yaw2 - current_yaw)):
                best_escape_yaw = yaw1
            else:
                best_escape_yaw = yaw2

            desired_yaw = self._smooth_yaw(best_escape_yaw)
            desired_speed = min(self.max, max(self.min, target_speed + 10.0))

            if "KAFA" in esc_reason:
                desired_alt = max(10.0, target["z"] - 15.0)
                maneuver = "DALIS"
            else:
                desired_alt = target["z"] + 20.0
                maneuver = "TIRMANIS"

            desired_speed = min(self.max_escape_speed, max(self.min, desired_speed))

            return GuidanceCommand(
                desired_speed_ms=desired_speed,
                desired_yaw_deg=desired_yaw,
                desired_altitude_m=desired_alt,
                distance_to_target_m=dist,
                mode=f"{esc_reason} + {maneuver}",
            )

        # 4. GRSEL MOD GEÇ HAZIRLIĞI - KAMERA KLDU çN HASSAS HZALANMA
        # Kameraya geçiş için:
        # 1. Rakibin TAM ARKASINDA olmak (yaw hizalı)
        # 2. Rakiple AYNI irtifada olmak
        # 3. Optimal mesafede olmak (15-35m)
        # 4. Hedef yavaş hareket etmeli

        # Yaw hizalama kontrolü - Rakibin yönüne tam bakıyor muyuz?
        my_heading_to_target = math.degrees(math.atan2(dy, dx)) % 360.0
        yaw_alignment_error = abs(_normalize_deg(my_heading_to_target - target_heading))

        # İrtifa hizalama kontrolü
        altitude_error = abs(my_pos[2] - target["z"])

        # Görsel kilit hazırlık modu (kameraya yakın)
        visual_ready = (
            dist < 50.0 and  # 50m içinde
            dist > 10.0 and  # Çok yakın değil
            yaw_alignment_error < 20.0 and  # 20 derece içinde hizalı
            altitude_error < 10.0  # 10m irtifa farkı içinde
        )

        # Görsel kilit aktif (kamera devreye girdi)
        # Hedef hızlı veya yavaş olabilir - önemli olan POZİSYON HİZALANMASI
        visual_lock = (
            dist < VISUAL_DIST_THRESH_M and
            dist > 12.0 and  # Minimum güvenli mesafe
            yaw_alignment_error < 10.0 and  # 10 derece içinde TAM hizalı
            altitude_error < 5.0  # 5m irtifa farkı içinde
        )

        if visual_lock:
            # KAMERA AKTİF - Rakibin TAM arkasında tut
            desired_yaw = self._smooth_yaw(target_heading)  # Rakibin yönüne TAM bak
            desired_alt = target["z"]  # Rakibin TAM aynı irtifası

            # Hızı rakiple senkronize et - ÇARPIŞMA RİSKİNE DİKKAT
            # Hedef yavaşsa: Hızı kes, çarpma
            # Hedef hızlıysa: Eşitle, takip et

            if target_speed < 15.0:
                # YAVAŞ HEDEF - Çarpışma riski yüksek!
                if dist < 18.0:
                    spd = max(self.full_brake, target_speed * 0.6)  # Çok yavaş
                elif dist > 25.0:
                    spd = min(target_speed + 2.0, 18.0)  # Kontrollü yaklaş
                else:
                    spd = max(self.full_brake, target_speed * 0.8)  # Güvenli mesafe
            else:
                # HIZLI HEDEF - Normal takip
                if dist < 18.0:
                    spd = max(self.full_brake, min(target_speed * 0.9, self.max))
                elif dist > 28.0:
                    spd = min(target_speed + 3.0, self.max)
                else:
                    spd = min(target_speed + 1.0, self.max)

            return GuidanceCommand(
                desired_speed_ms=spd,
                desired_yaw_deg=desired_yaw,
                desired_altitude_m=desired_alt,
                distance_to_target_m=dist,
                mode="KAMERA KILIT",
            )

        if visual_ready:
            # KAMERAYA HAZIRLIK - Pozisyon al
            desired_yaw = self._smooth_yaw(target_heading)  # Rakibin yönüne hizalan
            desired_alt = target["z"]  # Rakiple aynı irtifaya gel

            # Mesafeyi ayarla - yavaş hedeflerde ÇARPIŞMA RİSKİ!
            if target_speed < 15.0:
                # YAVAŞ HEDEF - Dikkatli yaklaş
                if dist < 25.0:
                    spd = max(self.full_brake, target_speed * 0.7)  # Hız kes
                elif dist > 40.0:
                    spd = min(target_speed + 3.0, 20.0)  # Kontrollü yaklaş
                else:
                    spd = min(target_speed + 1.0, 18.0)  # Hafif hızlı
            else:
                # HIZLI HEDEF - Normal yaklaşım
                if dist < 20.0:
                    spd = min(target_speed * 0.95, self.max)
                elif dist > 35.0:
                    spd = min(target_speed + 5.0, self.max)
                else:
                    spd = min(target_speed + 2.0, self.max)

            return GuidanceCommand(
                desired_speed_ms=spd,
                desired_yaw_deg=desired_yaw,
                desired_altitude_m=desired_alt,
                distance_to_target_m=dist,
                mode="KAMERA HAZIR",
            )

        # 5. NORMAL TAKP - KAMERA KLDU çN ARKA HZALANMA
        # Kamera için rakibin TAM ARKASINDA olmalıyız
        # Hedefin yönelim açısının 180 derece arkası = bizim yönümüz

        # Rakibin arkasındaki ideal nokta
        behind_angle_rad = math.radians(target_heading)

        # Mesafeye göre takip stratejisi
        if dist < 50.0:  # Yakın mesafe - tam arka hizalanma
            # Rakibin TAM ARKASINA bak
            final_yaw_target = target_heading
            mode_yaw = " ARKA KILIT"
        elif dist < 100.0:  # Orta mesafe - lead pursuit ile yaklaş
            # Hedefin yönüne + hafif lead
            lead_angle = min(15.0, dist * 0.15)  # Mesafeye göre lead açısı
            final_yaw_target = (target_heading + lead_angle) % 360.0
            mode_yaw = " YAKINLASMA"
        else:  # Uzak mesafe - direkt yönel
            final_yaw_target = raw_yaw
            mode_yaw = " DIREKT"

        desired_yaw = self._smooth_yaw(final_yaw_target)

        # İRTİFA STRATEJİSİ - KAMERA GÖRÜŞ HATTI
        # Kameranın düz görmesi için AYNI irtifada + hafif üstte
        # Rakiple aynı seviyede olmak KRITIK
        desired_alt = target["z"]  # Rakiple tamamen aynı irtifa

        # HIZ STRATEJS - MESAFE + HEDEF HIZI KOMBINASYONU
        mode_speed = ""

        # UZAK MESAFE (>100m) - Hızlı yaklaş
        if dist > 100.0:
            if target_speed < 20.0:
                # Yavaş hedef + uzak → Orta hızda yaklaş (fazla hızlanma)
                spd = min(30.0, self.max)
                mode_speed = " YAVAS-UZAK GAZ"
            else:
                # Hızlı hedef + uzak → Çok hızlı yaklaş!
                spd = min(target_speed + 10.0, self.max)
                mode_speed = " HIZLI-UZAK MAX GAZ"

        # ORTA MESAFE (50-100m) - Pozisyon al
        elif dist > 50.0:
            if target_speed < 20.0:
                # Yavaş hedef + orta → Dikkatli yaklaş
                spd = min(target_speed + 5.0, 25.0)
                mode_speed = " YAVAS-ORTA YAKLAS"
            else:
                # Hızlı hedef + orta → Hızlı yaklaş
                spd = min(target_speed + 5.0, self.max)
                mode_speed = " HIZLI-ORTA GAZ"

        # YAKIN MESAFE (30-50m) - Hassas takip
        elif dist > 30.0:
            if target_speed < 15.0:
                # Yavaş hedef + yakın → Çok dikkatli!
                spd = max(self.full_brake, min(target_speed + 2.0, 20.0))
                mode_speed = " YAVAS-YAKIN DIKKAT"
            else:
                # Hızlı hedef + yakın → Hız eşitle
                spd = min(target_speed + 2.0, self.max)
                mode_speed = " HIZLI-YAKIN TAKIP"

        # ÇOK YAKIN (15-30m) - Kameraya hazırlık
        elif dist > 15.0:
            if target_speed < 15.0:
                # Yavaş hedef + çok yakın → HıZ KES!
                spd = max(self.full_brake, target_speed * 0.8)
                mode_speed = " YAVAS-COK YAKIN FREN"
            elif target_speed < 30.0:
                # Orta hız + çok yakın → Eşitle
                spd = min(target_speed, self.max)
                mode_speed = " ORTA-COK YAKIN ESIT"
            else:
                # Hızlı hedef + çok yakın → Eşitle
                spd = min(target_speed, self.max)
                mode_speed = " HIZLI-COK YAKIN ESIT"

        # KRİTİK YAKIN (<15m) - Acil fren / Kilit hazırlık
        else:
            if target_speed < 15.0:
                # Yavaş hedef + kritik → MİNİMUM HIZ!
                spd = self.full_brake
                mode_speed = " YAVAS-KRITIK FREN"
            else:
                # Hızlı hedef + kritik → Kontrollü yavaşla
                spd = max(self.full_brake, target_speed * 0.7)
                mode_speed = " HIZLI-KRITIK FREN"

        final_speed = max(self.min, min(self.max, spd))

        return GuidanceCommand(
            desired_speed_ms=final_speed,
            desired_yaw_deg=desired_yaw,
            desired_altitude_m=desired_alt,
            distance_to_target_m=dist,
            mode=f"{mode_speed} | {mode_yaw}",
        )
