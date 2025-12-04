import math
from dataclasses import dataclass

# ==========================
# AYARLAR
# ==========================
SAHA_YARICAPI = 400.0  # (Metre) Yarışma alanının yarıçapı
GUVENLI_IRTİFA = 40.0  # Saha dışına çıkınca dönülecek irtifa

# ==========================
# KOMUT VERİ YAPISI
# ==========================
@dataclass
class GuidanceCommand:
    desired_speed_ms: float
    desired_yaw_deg: float
    desired_altitude_m: float
    distance_to_target_m: float
    mode: str 

# ==========================
# AÇI NORMALİZASYONU
# ==========================
def _normalize_deg(angle: float) -> float:
    """Açıyı (-180, 180] aralığına çevirir."""
    while angle > 180.0:
        angle -= 360.0
    while angle <= -180.0:
        angle += 360.0
    return angle

# ==========================
# GPS TAKİP KONTROLCÜSÜ
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

    # -----------------------------------------
    # YAW YUMUŞATMA (SMOOTHING)
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
    #             🔥 KAÇIŞ ANALİZİ (GELİŞMİŞ)
    # ======================================================
    def _escape_check(self, dist, target_speed, angle_diff):
        # 1 — Çok yakın çarpışma riski
        if dist < 10.0:
            return True, " KRİTİK YAKINLIK"

        # 2 — Rakip tam bize bakıyor (Kafa kafaya)
        if angle_diff > 130 and dist < 45.0:
            return True, "KAFA KAFAYA"

        # 3 — Rakip aşırı hızlı yaklaşıyor
        if target_speed > 50.0 and dist < 35.0:
            return True, " HIZ TEHDİDİ"

        return False, ""

    # -----------------------------------------
    # ANA GUIDANCE HESABI
    # -----------------------------------------
    def compute_command(self, my_pos, my_speed, target):
        """
        my_pos   : (x, y, z)
        my_speed : float (İHA'nın kendi hızı)
        target   : RivalTracker’dan gelen dict
        """

        # ======================================================
        #      ⭐ SAHA DIŞINA ÇIKMA KONTROLÜ (EKLEME) ⭐
        # ======================================================
        FIELD_RADIUS = SAHA_YARICAPI + 50  # saha sınırından biraz fazlası
        RETURN_SPEED = min(self.max, max(self.min + 5.0, 22))

        dist_from_center = math.sqrt(my_pos[0]**2 + my_pos[1]**2)

        if dist_from_center > FIELD_RADIUS:
            return_yaw = math.degrees(math.atan2(-my_pos[1], -my_pos[0])) % 360.0
            return GuidanceCommand(
                desired_speed_ms=RETURN_SPEED,
                desired_yaw_deg=self._smooth_yaw(return_yaw),
                desired_altitude_m=my_pos[2],
                distance_to_target_m=dist_from_center,
                mode="🏁 SAHA DIŞI → MERKEZE DÖNÜŞ"
            )

        # 0. GEOFENCE (SAHA SINIRI) KONTROLÜ - ÖNCE GÜVENLİK!
        dist_from_center = math.sqrt(my_pos[0]**2 + my_pos[1]**2)
        
        if dist_from_center > SAHA_YARICAPI:
            # Merkeze (0,0) dönmemiz lazım
            center_yaw = math.degrees(math.atan2(-my_pos[1], -my_pos[0])) % 360.0
            
            return GuidanceCommand(
                desired_speed_ms=self.max, # Hızla içeri gir
                desired_yaw_deg=self._smooth_yaw(center_yaw),
                desired_altitude_m=GUVENLI_IRTİFA,
                distance_to_target_m=9999.9, # Hedefle ilgilenmiyoruz
                mode="🚧 SAHA DIŞI - DÖNÜŞ"
            )

        # -----------------------------------------------------

        # 1. HEDEF MESAFE VE KONUM FARKI
        dx = target["x"] - my_pos[0]
        dy = target["y"] - my_pos[1]
        dz = target["z"] - my_pos[2]

        dist = math.sqrt(dx*dx + dy*dy + dz*dz)

        # 2. AÇILAR
        raw_yaw = math.degrees(math.atan2(dy, dx)) % 360.0
        target_heading = target.get("yaw_deg", raw_yaw)
        angle_diff = abs(_normalize_deg(raw_yaw - target_heading))

        target_speed = float(target.get("hiz", 20.0))

        # 3. KAÇIŞ KONTROLÜ (3D MANEVRA EKLENDİ)
        escape, esc_reason = self._escape_check(dist, target_speed, angle_diff)

        if escape:
            # a) YATAY KAÇIŞ: 90 derece kır
            yaw1 = (target_heading + 90) % 360
            yaw2 = (target_heading - 90) % 360
            current_yaw = self._last_yaw_cmd if self._last_yaw_cmd else raw_yaw
            
            if abs(_normalize_deg(yaw1 - current_yaw)) < abs(_normalize_deg(yaw2 - current_yaw)):
                best_escape_yaw = yaw1
            else:
                best_escape_yaw = yaw2

            desired_yaw = self._smooth_yaw(best_escape_yaw)
            # Kaçış hızını rakip + marj ile sınırla, maks hız aşılmasın
            desired_speed = min(self.max, max(self.min, target_speed + 10.0))
            
            # b) DİKEY KAÇIŞ (3D MANEVRA)
            if "KAFA KAFAYA" in esc_reason:
                # Rakibin altına dal (Daha güvenlidir)
                desired_alt = max(10.0, target["z"] - 15.0)
                maneuver = "DALIŞ"
            else:
                # Hız tehdidi varsa yukarı kaç (Tırmanış)
                desired_alt = target["z"] + 20.0
                maneuver = "TIRMANIŞ"

            # Kaçışta hız limitini güvenli bir üst sınırla kısıtla
            desired_speed = min(self.max_escape_speed, max(self.min, desired_speed))

            return GuidanceCommand(
                desired_speed_ms=desired_speed,
                desired_yaw_deg=desired_yaw,
                desired_altitude_m=desired_alt,
                distance_to_target_m=dist,
                mode=f"{esc_reason} + {maneuver}"
            )

        # =====================================================
        # 🔽 NORMAL TAKİP MODU
        # =====================================================

        if angle_diff < 45:
            final_yaw_target = target_heading
            mode_yaw = " ARKA HİZA"
        else:
            final_yaw_target = raw_yaw
            mode_yaw = " DIREKT"

        desired_yaw = self._smooth_yaw(final_yaw_target)

        # İRTİFA: Tam hizalanma (Sıfır fark)
        desired_alt = target["z"] 

        # HIZ KONTROLÜ
        mode_speed = ""
        spd = self.min

        # Çok yakında sert fren (çarpışmayı azaltmak için)
        if dist <= self.hard_brake:
            spd = max(self.full_brake, self.min * 0.5)
            mode_speed = " SERT FREN"

        # Kafa kafaya yaklaşmada ekstra yavaşla
        elif angle_diff > 135 and dist < 25:
            spd = max(self.full_brake, min(self.max, target_speed * 0.4))
            mode_speed = " KAFA KAFAYA FREN"

        if dist <= self.brake:
            ratio = max(0.0, dist/self.brake)
            spd = target_speed * (0.5 + 0.3 * ratio)
            mode_speed = " FREN"

        elif dist <= self.cruise:
            ratio = (dist - self.brake) / max(1.0, self.cruise - self.brake)
            spd = target_speed - 2 + 4 * ratio
            mode_speed = " TAKIP"

        else:
            extra = self.k * (dist - self.cruise)
            spd = target_speed + extra
            mode_speed = " GAZ"

        final_speed = max(self.min, min(self.max, spd))

        return GuidanceCommand(
            desired_speed_ms=final_speed,
            desired_yaw_deg=desired_yaw,
            desired_altitude_m=desired_alt,
            distance_to_target_m=dist,
            mode=f"{mode_speed} | {mode_yaw}",
        )
