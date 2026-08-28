import argparse
import csv
import os
import struct
import sys

import matplotlib.pyplot as plt


# Windows终端的活动代码页可能不是中文代码页，强制UTF-8避免帮助/记录输出报错。
if hasattr(sys.stdout, "reconfigure"):
    sys.stdout.reconfigure(encoding="utf-8")
if hasattr(sys.stderr, "reconfigure"):
    sys.stderr.reconfigure(encoding="utf-8")


plt.rcParams["font.sans-serif"] = [
    "Microsoft YaHei", "SimHei", "PingFang SC", "Heiti SC",
    "Noto Sans CJK SC", "WenQuanYi Zen Hei", "DejaVu Sans",
]
plt.rcParams["axes.unicode_minus"] = False

MAGIC = 0xAA
TIME_WRAP_MS = 1 << 16
DWT_WRAP_CYCLES = 1 << 32

# 当前工程配置为168 MHz。固件时钟改变时可用--cpu-hz覆盖。
DEFAULT_CPU_HZ = 168_000_000

CONTROL_FIELDS = [
    "timestamp_cycle",
    "roll_angle_cdeg", "pitch_angle_cdeg", "yaw_angle_cdeg",
    "roll_angle_target_cdeg", "pitch_angle_target_cdeg",
    "roll_rate_target_ddps", "pitch_rate_target_ddps", "yaw_rate_target_ddps",
    "roll_rate_meas_ddps", "pitch_rate_meas_ddps", "yaw_rate_meas_ddps",
    "roll_p_term_duint", "pitch_p_term_duint", "yaw_p_term_duint",
    "roll_i_term_duint", "pitch_i_term_duint", "yaw_i_term_duint",
    "roll_d_term_duint", "pitch_d_term_duint", "yaw_d_term_duint",
    "roll_output_duint", "pitch_output_duint", "yaw_output_duint",
    "m1", "m2", "m3", "m4",
    "current_dA", "current_limit_permille", 
    "throttle",
    "limited_throttle",
    "control_dt_us",
    "active_imu",
    "fresh_imu_flags",
    "flags",
]

CONTROL_FIELDS_V2 = [
    "timestamp_cycle",
    "roll_angle_cdeg", "pitch_angle_cdeg", "yaw_angle_cdeg",
    "roll_angle_target_cdeg", "pitch_angle_target_cdeg",
    "yaw_angle_target_cdeg",
    "roll_rate_target_ddps", "pitch_rate_target_ddps", "yaw_rate_target_ddps",
    "roll_rate_meas_ddps", "pitch_rate_meas_ddps", "yaw_rate_meas_ddps",
    "roll_p_term_duint", "pitch_p_term_duint", "yaw_p_term_duint",
    "roll_i_term_duint", "pitch_i_term_duint", "yaw_i_term_duint",
    "roll_d_term_duint", "pitch_d_term_duint", "yaw_d_term_duint",
    "roll_output_duint", "pitch_output_duint", "yaw_output_duint",
    "m1", "m2", "m3", "m4",
    "current_dA", "current_limit_permille",
    "throttle", "limited_throttle", "control_dt_us",
    "active_imu", "fresh_imu_flags", "flags",
]

CONTROL_FIELDS_V3 = [
    "timestamp_cycle",
    "roll_angle_cdeg", "pitch_angle_cdeg", "yaw_angle_cdeg",
    "roll_angle_target_cdeg", "pitch_angle_target_cdeg", "yaw_angle_target_cdeg",
    "level_trim_roll_cdeg", "level_trim_pitch_cdeg",
    "mag_yaw_cdeg", "yaw_mag_innovation_cdeg", "mag_field_mG",
    "roll_rate_target_ddps", "pitch_rate_target_ddps", "yaw_rate_target_ddps",
    "roll_rate_meas_ddps", "pitch_rate_meas_ddps", "yaw_rate_meas_ddps",
    "roll_p_term_duint", "pitch_p_term_duint", "yaw_p_term_duint",
    "roll_i_term_duint", "pitch_i_term_duint", "yaw_i_term_duint",
    "roll_d_term_duint", "pitch_d_term_duint", "yaw_d_term_duint",
    "roll_output_duint", "pitch_output_duint", "yaw_output_duint",
    "m1", "m2", "m3", "m4",
    "current_dA", "current_limit_permille",
    "throttle", "limited_throttle", "control_dt_us",
    "active_imu", "fresh_imu_flags", "flags",
]

CONTROL_FIELDS_V4 = [
    "timestamp_cycle",
    "roll_angle_cdeg", "pitch_angle_cdeg", "yaw_angle_cdeg",
    "roll_angle_target_cdeg", "pitch_angle_target_cdeg", "yaw_angle_target_cdeg",
    "level_trim_roll_cdeg", "level_trim_pitch_cdeg",

    "mag_yaw_cdeg", "yaw_mag_innovation_cdeg", "mag_field_mG",
    "mag_field_reference_mG", "mag_field_ratio_permille",
    "mag_reject_reason",

    "roll_rate_target_ddps", "pitch_rate_target_ddps", "yaw_rate_target_ddps",
    "roll_rate_meas_ddps", "pitch_rate_meas_ddps", "yaw_rate_meas_ddps",

    "roll_p_term_duint", "pitch_p_term_duint", "yaw_p_term_duint",
    "roll_i_term_duint", "pitch_i_term_duint", "yaw_i_term_duint",
    "roll_d_term_duint", "pitch_d_term_duint", "yaw_d_term_duint",
    "roll_output_duint", "pitch_output_duint", "yaw_output_duint",

    "m1", "m2", "m3", "m4",
    "current_dA", "current_limit_permille",
    "throttle", "limited_throttle", "control_dt_us",
    "active_imu", "fresh_imu_flags", "flags",
]

MAG_CAL_SAMPLE_FIELDS = [
    "timestamp_cycle",
    "mag_x_gauss", "mag_y_gauss", "mag_z_gauss",
]

NAVIGATION_FIELDS = [
    "timestamp_cycle", "rmc_sequence",
    "gps_velocity_n_cms", "gps_velocity_e_cms",
    "est_velocity_n_cms", "est_velocity_e_cms",
    "accel_n_cms2", "accel_e_cms2",
    "accel_bias_n_cms2", "accel_bias_e_cms2",
    "velocity_target_n_cms", "velocity_target_e_cms",
    "nav_roll_target_cdeg", "nav_pitch_target_cdeg", "nav_yaw_cdeg",
    "controller_i_n_cms2", "controller_i_e_cms2",
    "gps_age_ms", "rmc_period_ms", "gps_hdop_centi",
    "gps_satellites", "nav_flags",
]

NAVIGATION_FIELDS_V2 = NAVIGATION_FIELDS + [
    "position_n_cm", "position_e_cm",
    "position_target_n_cm", "position_target_e_cm",
    "position_error_n_cm", "position_error_e_cm",
    "position_flags", "horizontal_mode_raw",
]

NAVIGATION_FIELDS_V3 = NAVIGATION_FIELDS + [
    "position_n_cm", "position_e_cm",
    "gps_position_n_cm", "gps_position_e_cm",
    "position_target_n_cm", "position_target_e_cm",
    "position_error_n_cm", "position_error_e_cm",
    "position_flags", "horizontal_mode_raw",
]

NAVIGATION_FIELDS_V4 = NAVIGATION_FIELDS_V3 + [
    "rmc_velocity_n_cms", "rmc_velocity_e_cms",
    "gps_position_velocity_n_cms", "gps_position_velocity_e_cms",
    "gps_velocity_source_raw",
]

NAVIGATION_FIELDS_V5 = NAVIGATION_FIELDS_V4 + [
    "position_control_phase_raw",
]


# type -> (记录总长度，payload struct格式，payload字段名)
# CONTROL payload与BB_ControlData_t的__packed布局一致：71 bytes，整帧73 bytes。
REC_SPECS = {
    0x01: (21, "<Hhhh HHHH bbb", [
        "time_ms", "roll_cdeg", "pitch_cdeg", "yaw_cdeg",
        "m1", "m2", "m3", "m4",
        "roll_target", "pitch_target", "yaw_target",
    ]),
    0x02: (5, "<HB", ["time_ms", "armed"]),
    0x03: (4, "<H", ["time_ms"]),
    0x04: (4, "<H", ["time_ms"]),
    0x05: (5, "<HB", ["time_ms", "new_active_imu"]),
    0x06: (
        73,
        "<I hhh hh hhh hhh hhh hhh hhh hhh HHHH h H H H H B B B",
        CONTROL_FIELDS,
    ),
    0x07: (
        75,
        "<I hhh hhh hhh hhh hhh hhh hhh hhh HHHH h H H H H B B B",
        CONTROL_FIELDS_V2,
    ),
    0x08: (
        85,
        "<I hhh hhh hh h h H hhh hhh hhh hhh hhh hhh HHHH h H H H H B B B",
        CONTROL_FIELDS_V3,
    ),
    0x09: (
        18,
        "<Ifff",
        MAG_CAL_SAMPLE_FIELDS,
    ),
    0x0A: (
        90,
        "<I hhh hhh hh h h H H H B "
        "hhh hhh hhh hhh hhh hhh "
        "HHHH h H H H H B B B",
        CONTROL_FIELDS_V4,
    ),
    0x0B: (
        48,
        "<II hhhhhhhhhhhhhhh HHH BB",
        NAVIGATION_FIELDS,
    ),
    0x0C: (
        62,
        "<II hhhhhhhhhhhhhhh HHH BB hhhhhh BB",
        NAVIGATION_FIELDS_V2,
    ),
    0x0D: (
        66,
        "<II hhhhhhhhhhhhhhh HHH BB hhhhhhhh BB",
        NAVIGATION_FIELDS_V3,
    ),
    0x0E: (
        75,
        "<II hhhhhhhhhhhhhhh HHH BB hhhhhhhh BB hhhh B",
        NAVIGATION_FIELDS_V4,
    ),
    0x0F: (
        76,
        "<II hhhhhhhhhhhhhhh HHH BB hhhhhhhh BB hhhh BB",
        NAVIGATION_FIELDS_V5,
    ),
}

REC_NAMES = {
    0x01: "MOTION", 0x02: "ARM_CHANGED", 0x03: "DUAL_FAULT",
    0x04: "VOLTAGE_FAULT", 0x05: "IMU_SWITCH",
    0x06: "CONTROL", 0x07: "CONTROL", 0x08: "CONTROL",
    0x09: "MAG_CAL_SAMPLE",
    0x0A: "CONTROL",
    0x0B: "NAVIGATION",
    0x0C: "NAVIGATION",
    0x0D: "NAVIGATION",
    0x0E: "NAVIGATION",
    0x0F: "NAVIGATION",
}

EVENT_STYLE = {
    "DUAL_FAULT": dict(color="#d62728", label="DUAL_FAULT"),
    "VOLTAGE_FAULT": dict(color="#ff7f0e", label="VOLTAGE_FAULT"),
    "IMU_SWITCH": dict(color="#1f77b4", label="IMU_SWITCH"),
}

MAG_REJECT_REASON_NAMES = {
    1 << 0: "INVALID_SAMPLE",
    1 << 1: "ABSOLUTE_FIELD",
    1 << 2: "FIELD_RATIO",
    1 << 3: "INNOVATION",
}


def decode_mag_reject_reason(value):
    if value == 0:
        return "NONE"

    names = [
        name for bit, name in MAG_REJECT_REASON_NAMES.items()
        if value & bit
    ]
    return "|".join(names) if names else f"UNKNOWN_0x{value:02X}"


def validate_specs():
    for rec_type, (rec_len, fmt, fields) in REC_SPECS.items():
        payload_len = struct.calcsize(fmt)
        if payload_len + 2 != rec_len:
            raise RuntimeError(
                f"记录0x{rec_type:02X}定义错误：payload={payload_len}，总长度={rec_len}"
            )
        if len(fields) != len(struct.unpack(fmt, bytes(payload_len))):
            raise RuntimeError(f"记录0x{rec_type:02X}字段数与struct格式不匹配")


validate_specs()


def add_control_engineering_values(rec):
    """保留原始定点值，并添加工程单位字段。"""
    for axis in ("roll", "pitch", "yaw"):
        rec[f"{axis}_angle_deg"] = rec[f"{axis}_angle_cdeg"] / 100.0
        rec[f"{axis}_rate_target_dps"] = rec[f"{axis}_rate_target_ddps"] / 10.0
        rec[f"{axis}_rate_meas_dps"] = rec[f"{axis}_rate_meas_ddps"] / 10.0
        for term in ("p", "i", "d"):
            rec[f"{axis}_{term}_term"] = rec[f"{axis}_{term}_term_duint"] / 10.0
        rec[f"{axis}_output_term"] = rec[f"{axis}_output_duint"] / 10.0

    rec["roll_angle_target_deg"] = rec["roll_angle_target_cdeg"] / 100.0
    rec["pitch_angle_target_deg"] = rec["pitch_angle_target_cdeg"] / 100.0
    if "yaw_angle_target_cdeg" in rec:
        rec["yaw_angle_target_deg"] = rec["yaw_angle_target_cdeg"] / 100.0
    if "level_trim_roll_cdeg" in rec:
        rec["level_trim_roll_deg"] = rec["level_trim_roll_cdeg"] / 100.0
        rec["level_trim_pitch_deg"] = rec["level_trim_pitch_cdeg"] / 100.0
        rec["mag_yaw_deg"] = rec["mag_yaw_cdeg"] / 100.0
        rec["yaw_mag_innovation_deg"] = rec["yaw_mag_innovation_cdeg"] / 100.0
        rec["mag_field_gauss"] = rec["mag_field_mG"] / 1000.0
    rec["current_a"] = rec["current_dA"] / 10.0
    rec["current_limit_pct"] = rec["current_limit_permille"] / 10.0
    rec["airmode_active"] = int(bool(rec["flags"] & (1 << 0)))
    rec["current_limiting"] = int(bool(rec["flags"] & (1 << 1)))
    if "mag_field_reference_mG" in rec:
        rec["mag_field_reference_gauss"] = (
                rec["mag_field_reference_mG"] / 1000.0
        )
        rec["mag_field_ratio"] = (
                rec["mag_field_ratio_permille"] / 1000.0
        )


def add_navigation_engineering_values(rec):
    for axis in ("n", "e"):
        rec[f"gps_velocity_{axis}_mps"] = (
            rec[f"gps_velocity_{axis}_cms"] / 100.0
        )
        rec[f"est_velocity_{axis}_mps"] = (
            rec[f"est_velocity_{axis}_cms"] / 100.0
        )
        rec[f"accel_{axis}_mps2"] = (
            rec[f"accel_{axis}_cms2"] / 100.0
        )
        rec[f"accel_bias_{axis}_mps2"] = (
            rec[f"accel_bias_{axis}_cms2"] / 100.0
        )
        rec[f"velocity_target_{axis}_mps"] = (
            rec[f"velocity_target_{axis}_cms"] / 100.0
        )
        rec[f"controller_i_{axis}_mps2"] = (
            rec[f"controller_i_{axis}_cms2"] / 100.0
        )

    rec["nav_roll_target_deg"] = rec["nav_roll_target_cdeg"] / 100.0
    rec["nav_pitch_target_deg"] = rec["nav_pitch_target_cdeg"] / 100.0
    rec["nav_yaw_deg"] = rec["nav_yaw_cdeg"] / 100.0
    rec["gps_hdop"] = rec["gps_hdop_centi"] / 100.0

    flags = rec["nav_flags"]
    rec["gps_velocity_valid"] = int(bool(flags & (1 << 0)))
    rec["horizontal_estimator_initialized"] = int(bool(flags & (1 << 1)))
    rec["horizontal_estimator_healthy"] = int(bool(flags & (1 << 2)))
    rec["velocity_hold_requested"] = int(bool(flags & (1 << 3)))
    rec["velocity_hold_active"] = int(bool(flags & (1 << 4)))
    rec["gps_correction_accepted"] = int(bool(flags & (1 << 5)))
    rec["imu_velocity_prediction_enabled"] = int(bool(flags & (1 << 6)))
    rec["gps_velocity_control_ready"] = int(bool(flags & (1 << 7)))

    if "position_n_cm" in rec:
        for axis in ("n", "e"):
            rec[f"position_{axis}_m"] = (
                    rec[f"position_{axis}_cm"] / 100.0
            )
            rec[f"position_target_{axis}_m"] = (
                    rec[f"position_target_{axis}_cm"] / 100.0
            )
            rec[f"position_error_{axis}_m"] = (
                    rec[f"position_error_{axis}_cm"] / 100.0
            )

        position_flags = rec["position_flags"]
        rec["gps_position_control_ready"] = int(bool(position_flags & (1 << 0)))
        rec["position_hold_requested"] = int(bool(position_flags & (1 << 1)))
        rec["position_hold_active"] = int(bool(position_flags & (1 << 2)))
        rec["position_controller_initialized"] = int(bool(position_flags & (1 << 3)))
        rec["position_control_enabled"] = int(bool(position_flags & (1 << 4)))

        if "gps_position_n_cm" in rec:
            rec["gps_position_n_m"] = rec["gps_position_n_cm"] / 100.0
            rec["gps_position_e_m"] = rec["gps_position_e_cm"] / 100.0
            rec["gps_position_correction_accepted"] = int(
                bool(position_flags & (1 << 5))
            )
            rec["gps_position_correction_rejected"] = int(
                bool(position_flags & (1 << 6))
            )
            rec["position_velocity_correction_applied"] = int(
                bool(position_flags & (1 << 7))
            )

        if "rmc_velocity_n_cms" in rec:
            for axis in ("n", "e"):
                rec[f"rmc_velocity_{axis}_mps"] = (
                        rec[f"rmc_velocity_{axis}_cms"] / 100.0
                )
                rec[f"gps_position_velocity_{axis}_mps"] = (
                        rec[f"gps_position_velocity_{axis}_cms"] / 100.0
                )

            rec["gps_velocity_source"] = {
                0: "RMC",
                1: "POSITION_WINDOW",
            }.get(rec["gps_velocity_source_raw"], "UNKNOWN")

        rec["horizontal_mode"] = {
            0: "MANUAL",
            1: "VELOCITY_HOLD",
            2: "POSITION_HOLD",
        }.get(rec["horizontal_mode_raw"], "UNKNOWN")

        if "position_control_phase_raw" in rec:
            rec["position_control_phase"] = {
                0: "INACTIVE",
                1: "MOVING",
                2: "BRAKING",
                3: "HOLD",
            }.get(rec["position_control_phase_raw"], "UNKNOWN")

def add_yaw_debug_engineering_values(rec):
    for key in ("roll", "pitch", "fused_yaw", "yaw_target", "yaw_error",
                "mag_yaw", "mag_innovation"):
        rec[f"{key}_deg"] = rec[f"{key}_cdeg"] / 100.0

    rec["yaw_rate_target_dps"] = rec["yaw_rate_target_ddps"] / 10.0
    rec["yaw_rate_meas_dps"] = rec["yaw_rate_meas_ddps"] / 10.0
    rec["yaw_output"] = rec["yaw_output_duint"] / 10.0
    rec["mag_x_gauss"] = rec["mag_x_mG"] / 1000.0
    rec["mag_y_gauss"] = rec["mag_y_mG"] / 1000.0
    rec["mag_z_gauss"] = rec["mag_z_mG"] / 1000.0
    rec["mag_field_gauss"] = rec["mag_field_mG"] / 1000.0
    rec["mag_field_ratio"] = rec["mag_field_ratio_permille"] / 1000.0
    rec["yaw_mode"] = "MANUAL" if rec["yaw_mode_raw"] else "HEADING_HOLD"
    rec["yaw_initialized"] = int(bool(rec["flags"] & (1 << 0)))
    rec["mag_new_since_last_log"] = int(bool(rec["flags"] & (1 << 1)))
    rec["armed"] = int(bool(rec["flags"] & (1 << 2)))


def add_level_trim_debug_engineering_values(rec):
    cdeg_keys = (
        "imu0_last_roll", "imu1_last_roll",
        "imu0_last_pitch", "imu1_last_pitch",
        "imu0_mean_roll", "imu1_mean_roll",
        "imu0_mean_pitch", "imu1_mean_pitch",
        "imu0_stddev_roll", "imu1_stddev_roll",
        "imu0_stddev_pitch", "imu1_stddev_pitch",
        "imu0_offset_roll", "imu1_offset_roll",
        "imu0_offset_pitch", "imu1_offset_pitch",
        "active_accel_raw_roll", "active_accel_raw_pitch",
        "active_accel_trimmed_roll", "active_accel_trimmed_pitch",
    )
    for key in cdeg_keys:
        rec[f"{key}_deg"] = rec[f"{key}_cdeg"] / 100.0

    for imu in (0, 1):
        rec[f"imu{imu}_accel_norm_g"] = rec[f"imu{imu}_accel_norm_mG"] / 1000.0
        rec[f"imu{imu}_gyro_abs_max_dps"] = (
            rec[f"imu{imu}_gyro_abs_max_ddps"] / 10.0
        )


    rec["trigger_holding"] = int(bool(rec["flags"] & (1 << 0)))
    rec["trigger_latched"] = int(bool(rec["flags"] & (1 << 1)))
    rec["trim_active"] = int(bool(rec["flags"] & (1 << 2)))
    rec["trim_ready"] = int(bool(rec["flags"] & (1 << 3)))
    rec["loaded_from_flash"] = int(bool(rec["flags"] & (1 << 4)))

def parse(path):
    with open(path, "rb") as f:
        data = f.read()

    records = []
    index = 0
    resync_count = 0

    while index < len(data):
        if data[index] != MAGIC:
            index += 1
            resync_count += 1
            continue
        if index + 1 >= len(data):
            break

        rec_type = data[index + 1]
        spec = REC_SPECS.get(rec_type)
        if spec is None:
            index += 1
            resync_count += 1
            continue

        rec_len, fmt, fields = spec
        if index + rec_len > len(data):
            break

        values = struct.unpack(fmt, data[index + 2:index + rec_len])
        rec = dict(zip(fields, values))
        rec["_record_index"] = len(records)
        rec["_file_offset"] = index
        rec["_type"] = REC_NAMES[rec_type]
        if rec_type in (0x0B, 0x0C, 0x0D, 0x0E, 0x0F):
            add_navigation_engineering_values(rec)
        if rec_type in (0x06, 0x07, 0x08, 0x0A):
            rec["control_version"] = {
                0x06: 1,
                0x07: 2,
                0x08: 3,
                0x0A: 4,
            }[rec_type]
            add_control_engineering_values(rec)

            if rec["control_version"] >= 2:
                rec["yaw_mode"] = (
                    "MANUAL" if rec["flags"] & (1 << 2) else "HEADING_HOLD"
                )

            if rec["control_version"] >= 3:
                rec["level_trim_ready"] = int(bool(rec["flags"] & (1 << 3)))
                rec["mag_correction_accepted"] = int(bool(rec["flags"] & (1 << 4)))
                rec["yaw_estimator_initialized"] = int(bool(rec["flags"] & (1 << 5)))

            if rec["control_version"] >= 4:
                rec["mag_reject_reason_text"] = decode_mag_reject_reason(
                    rec["mag_reject_reason"]
                )
        records.append(rec)
        index += rec_len

    return records, resync_count


def add_absolute_time(records, cpu_hz):
    """分别还原16-bit RTOS tick和32-bit DWT cycle的回绕。"""
    last_ms = None
    ms_wrap_count = 0
    last_cycle = None
    absolute_cycle = None
    first_control_cycle = None

    for rec in records:
        if "time_ms" in rec:
            raw_ms = rec["time_ms"]
            if last_ms is not None and raw_ms < last_ms - TIME_WRAP_MS // 2:
                ms_wrap_count += 1
            last_ms = raw_ms
            rec["_t_abs_ms"] = raw_ms + ms_wrap_count * TIME_WRAP_MS

        if "timestamp_cycle" in rec:
            raw_cycle = rec["timestamp_cycle"]
            if last_cycle is None:
                absolute_cycle = raw_cycle
                first_control_cycle = absolute_cycle
            else:
                absolute_cycle += (raw_cycle - last_cycle) % DWT_WRAP_CYCLES
            last_cycle = raw_cycle
            rec["_timestamp_cycle_abs"] = absolute_cycle
            rec["_t_control_s"] = (absolute_cycle - first_control_cycle) / cpu_hz

    return records


def export_csv(records, out_path):
    """导出全部异构记录；某类型不存在的字段在对应行留空。"""
    preferred = [
        "record_index", "file_offset", "record_type",
        "time_ms", "time_abs_ms", "timestamp_cycle", "timestamp_cycle_abs",
        "control_time_s",
    ]
    all_fields = set()
    rows = []

    for rec in records:
        row = {
            "record_index": rec["_record_index"],
            "file_offset": rec["_file_offset"],
            "record_type": rec["_type"],
        }
        row.update({key: value for key, value in rec.items() if not key.startswith("_")})
        if "_t_abs_ms" in rec:
            row["time_abs_ms"] = rec["_t_abs_ms"]
        if "_timestamp_cycle_abs" in rec:
            row["timestamp_cycle_abs"] = rec["_timestamp_cycle_abs"]
        if "_t_control_s" in rec:
            row["control_time_s"] = rec["_t_control_s"]
        all_fields.update(row)
        rows.append(row)

    fieldnames = [name for name in preferred if name in all_fields]
    fieldnames.extend(sorted(all_fields - set(fieldnames)))
    with open(out_path, "w", newline="", encoding="utf-8-sig") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)
    print(f"CSV已保存（{len(rows)}条完整记录）: {out_path}")


def plot_legacy_records(records, out_path):
    motion = [r for r in records if r["_type"] == "MOTION"]
    arm_events = [r for r in records if r["_type"] == "ARM_CHANGED"]
    fault_events = [r for r in records if r["_type"] in EVENT_STYLE]
    if not motion and not arm_events and not fault_events:
        return False

    fig, axes = plt.subplots(
        3, 1, figsize=(13, 9), sharex=True,
        gridspec_kw={"height_ratios": [2, 2, 1]},
    )
    ax_att, ax_motor, ax_arm = axes

    def mark_events(ax):
        seen = set()
        for rec in fault_events:
            t = rec["_t_abs_ms"] / 1000.0
            style = EVENT_STYLE[rec["_type"]]
            label = style["label"] if style["label"] not in seen else None
            seen.add(style["label"])
            ax.axvline(t, color=style["color"], linestyle="--", linewidth=0.9,
                       alpha=0.7, label=label)

    if motion:
        t = [r["_t_abs_ms"] / 1000.0 for r in motion]
        ax_att.plot(t, [r["roll_cdeg"] / 100.0 for r in motion], label="roll实际")
        ax_att.plot(t, [r["pitch_cdeg"] / 100.0 for r in motion], label="pitch实际")
        ax_att.plot(t, [r["yaw_cdeg"] / 100.0 for r in motion], label="yaw实际")
        ax_att.plot(t, [r["roll_target"] for r in motion], linestyle=":", label="roll目标")
        ax_att.plot(t, [r["pitch_target"] for r in motion], linestyle=":", label="pitch目标")
        ax_att.plot(t, [r["yaw_target"] for r in motion], linestyle=":", label="yaw目标")
    mark_events(ax_att)
    ax_att.set_ylabel("角度 (°)")
    ax_att.set_title("姿态：实际 vs 目标")
    if ax_att.get_legend_handles_labels()[0]:
        ax_att.legend(loc="upper right", fontsize=7, ncol=2)

    if motion:
        t = [r["_t_abs_ms"] / 1000.0 for r in motion]
        for key, color in zip(("m1", "m2", "m3", "m4"),
                              ("#d62728", "#ff7f0e", "#2ca02c", "#1f77b4")):
            ax_motor.plot(t, [r[key] for r in motion], color=color, linewidth=0.9, label=key)
    mark_events(ax_motor)
    ax_motor.set_ylabel("电机输出")
    ax_motor.set_title("四路电机输出")
    if ax_motor.get_legend_handles_labels()[0]:
        ax_motor.legend(loc="upper right", fontsize=7, ncol=4)

    if arm_events:
        t = [r["_t_abs_ms"] / 1000.0 for r in arm_events]
        armed = [r["armed"] for r in arm_events]
        ax_arm.step(t, armed, where="post", color="#2ca02c", linewidth=1.3, label="ARM状态")
        ax_arm.scatter(t, armed, s=16, color="#2ca02c", zorder=3)
    mark_events(ax_arm)
    ax_arm.set_yticks([0, 1])
    ax_arm.set_yticklabels(["DISARMED", "ARMED"])
    ax_arm.set_ylabel("解锁状态")
    ax_arm.set_xlabel("时间 (s)")
    if ax_arm.get_legend_handles_labels()[0]:
        ax_arm.legend(loc="upper right", fontsize=7)

    for ax in axes:
        ax.grid(True, alpha=0.2)
    fig.tight_layout()
    fig.savefig(out_path, dpi=150)
    print(f"旧格式图表已保存: {out_path}")
    return True


def plot_control_records(records, out_path):
    control = [r for r in records if r["_type"] == "CONTROL"]
    if not control:
        return False

    t = [r["_t_control_s"] for r in control]
    fig, axes = plt.subplots(3, 2, figsize=(15, 12), sharex=True)
    ax_att, ax_rate, ax_roll, ax_pitch, ax_yaw, ax_motor = axes.flat

    for axis, color in zip(("roll", "pitch", "yaw"),
                           ("#1f77b4", "#2ca02c", "#9467bd")):
        ax_att.plot(t, [r[f"{axis}_angle_deg"] for r in control], color=color,
                    linewidth=0.9, label=f"{axis}实际")
        target_key = f"{axis}_angle_target_deg"
        if all(target_key in rec for rec in control):
            ax_att.plot(t, [rec[target_key] for rec in control],
                        color=color, linestyle=":", linewidth=0.9,
                        label=f"{axis}目标")

    if all("mag_yaw_deg" in rec for rec in control):
        ax_att.plot(t, [rec["mag_yaw_deg"] for rec in control],
                    color="#7f7f7f", linewidth=0.7, alpha=0.65,
                    label="yaw原始Mag")

    ax_att.set_title("CONTROL姿态")
    ax_att.set_ylabel("角度 (°)")
    ax_att.legend(fontsize=7, ncol=2)

    for axis, color in zip(("roll", "pitch", "yaw"),
                           ("#1f77b4", "#2ca02c", "#9467bd")):
        ax_rate.plot(t, [r[f"{axis}_rate_meas_dps"] for r in control], color=color,
                     linewidth=0.9, label=f"{axis}实测")
        ax_rate.plot(t, [r[f"{axis}_rate_target_dps"] for r in control], color=color,
                     linestyle=":", linewidth=0.9, label=f"{axis}目标")
    ax_rate.set_title("Rate：实测 vs 目标")
    ax_rate.set_ylabel("角速度 (°/s)")
    ax_rate.legend(fontsize=7, ncol=2)

    for axis, ax in (("roll", ax_roll), ("pitch", ax_pitch), ("yaw", ax_yaw)):
        for term, style in (("p", "-"), ("i", "--"), ("d", ":"), ("output", "-.")):
            ax.plot(t, [r[f"{axis}_{term}_term"] for r in control],
                    linestyle=style, linewidth=0.9, label=term.upper())
        ax.set_title(f"{axis.capitalize()} Rate PID")
        ax.set_ylabel("Mixer unit")
        ax.legend(fontsize=7, ncol=4)

    for key, color in zip(("m1", "m2", "m3", "m4"),
                          ("#d62728", "#ff7f0e", "#2ca02c", "#1f77b4")):
        ax_motor.plot(t, [r[key] for r in control], color=color, linewidth=0.9, label=key)
    ax_motor.set_title("Mixer输出")
    ax_motor.set_ylabel("Motor command")
    ax_motor.legend(fontsize=7, ncol=4)

    for ax in axes.flat:
        ax.grid(True, alpha=0.2)
        ax.set_xlabel("CONTROL首帧起算时间 (s)")
    fig.tight_layout()
    fig.savefig(out_path, dpi=150)
    print(f"CONTROL图表已保存: {out_path}")
    return True


def plot_navigation_records(records, out_path):
    rows = [r for r in records if r["_type"] == "NAVIGATION"]
    if not rows:
        return False

    t = [r["_t_control_s"] for r in rows]
    has_position = all("position_n_m" in r for r in rows)

    fig, axes = plt.subplots(4, 2, figsize=(15, 15), sharex=True)
    (ax_vn, ax_ve,
     ax_accel, ax_angle,
     ax_pos_n, ax_pos_e,
     ax_quality, ax_flags) = axes.flat

    ax_vn.plot(t, [r["gps_velocity_n_mps"] for r in rows],
               linewidth=0.9, label="GPS Vn")
    ax_vn.plot(t, [r["est_velocity_n_mps"] for r in rows],
               linewidth=0.9, label="Estimator Vn")
    ax_vn.plot(t, [r["velocity_target_n_mps"] for r in rows],
               linestyle=":", linewidth=0.9, label="Target Vn")
    ax_vn.set_ylabel("m/s")
    ax_vn.set_title("North速度")
    ax_vn.legend(fontsize=8)

    ax_ve.plot(t, [r["gps_velocity_e_mps"] for r in rows],
               linewidth=0.9, label="GPS Ve")
    ax_ve.plot(t, [r["est_velocity_e_mps"] for r in rows],
               linewidth=0.9, label="Estimator Ve")
    ax_ve.plot(t, [r["velocity_target_e_mps"] for r in rows],
               linestyle=":", linewidth=0.9, label="Target Ve")
    ax_ve.set_ylabel("m/s")
    ax_ve.set_title("East速度")
    ax_ve.legend(fontsize=8)

    ax_accel.plot(t, [r["accel_n_mps2"] for r in rows], label="Accel N")
    ax_accel.plot(t, [r["accel_e_mps2"] for r in rows], label="Accel E")
    ax_accel.plot(t, [r["accel_bias_n_mps2"] for r in rows],
                  linestyle=":", label="Bias N")
    ax_accel.plot(t, [r["accel_bias_e_mps2"] for r in rows],
                  linestyle=":", label="Bias E")
    ax_accel.set_ylabel("m/s²")
    ax_accel.set_title("水平加速度与Bias")
    ax_accel.legend(fontsize=8, ncol=2)

    ax_angle.plot(t, [r["nav_roll_target_deg"] for r in rows],
                  label="Nav Roll target")
    ax_angle.plot(t, [r["nav_pitch_target_deg"] for r in rows],
                  label="Nav Pitch target")
    ax_angle.plot(t, [r["controller_i_n_mps2"] for r in rows],
                  linestyle=":", label="Velocity I-N")
    ax_angle.plot(t, [r["controller_i_e_mps2"] for r in rows],
                  linestyle=":", label="Velocity I-E")
    ax_angle.set_ylabel("deg / m/s²")
    ax_angle.set_title("Velocity Controller输出")
    ax_angle.legend(fontsize=8, ncol=2)

    if has_position:
        if all("gps_position_n_m" in r for r in rows):
            ax_pos_n.plot(
                t,
                [r["gps_position_n_m"] for r in rows],
                linewidth=0.7,
                alpha=0.55,
                label="Raw GPS N",
            )
            ax_pos_e.plot(
                t,
                [r["gps_position_e_m"] for r in rows],
                linewidth=0.7,
                alpha=0.55,
                label="Raw GPS E",
            )

        ax_pos_n.plot(t, [r["position_n_m"] for r in rows],
                      linewidth=1.0, label="Estimated N")
        ax_pos_e.plot(t, [r["position_e_m"] for r in rows],
                      linewidth=1.0, label="Estimated E")
        ax_pos_n.plot(t, [r["position_target_n_m"] for r in rows],
                      linestyle=":", label="Target N")
        ax_pos_n.plot(t, [r["position_error_n_m"] for r in rows],
                      linestyle="--", label="Error N")

        ax_pos_e.plot(t, [r["position_target_e_m"] for r in rows],
                      linestyle=":", label="Target E")
        ax_pos_e.plot(t, [r["position_error_e_m"] for r in rows],
                      linestyle="--", label="Error E")

        ax_pos_n.set_title("North位置")
        ax_pos_e.set_title("East位置")
        ax_pos_n.set_ylabel("m")
        ax_pos_e.set_ylabel("m")
        ax_pos_n.legend(fontsize=8)
        ax_pos_e.legend(fontsize=8)
    else:
        for ax in (ax_pos_n, ax_pos_e):
            ax.text(0.5, 0.5, "NAVIGATION V1无位置字段",
                    ha="center", va="center", transform=ax.transAxes)
            ax.set_title("Position Hold")

    ax_quality.plot(t, [r["gps_age_ms"] for r in rows], label="RMC age ms")
    ax_quality.plot(t, [r["rmc_period_ms"] for r in rows], label="RMC period ms")
    ax_quality.plot(t, [r["gps_hdop"] * 100.0 for r in rows],
                    linestyle=":", label="HDOP × 100")
    ax_quality.set_ylabel("ms / scaled")
    ax_quality.set_title("GPS更新率与质量")
    ax_quality.legend(fontsize=8)

    flag_specs = [
        ("gps_velocity_valid", "GPS velocity valid"),
        ("gps_velocity_control_ready", "GPS velocity ready"),
        ("horizontal_estimator_healthy", "Estimator healthy"),
        ("velocity_hold_active", "Velocity active"),
        ("gps_correction_accepted", "GPS accepted"),
    ]
    if has_position:
        flag_specs.extend([
            ("gps_position_control_ready", "GPS position ready"),
            ("position_hold_requested", "Position requested"),
            ("position_hold_active", "Position active"),
            ("position_control_enabled", "Position output enabled"),
        ])

    for key, label in flag_specs:
        ax_flags.step(t, [r[key] for r in rows], where="post", label=label)
    ax_flags.set_yticks([0, 1])
    ax_flags.set_title("Navigation状态")
    ax_flags.legend(fontsize=7, ncol=2)

    for ax in axes.flat:
        ax.grid(True, alpha=0.2)
        ax.set_xlabel("时间 (s)")

    fig.tight_layout()
    fig.savefig(out_path, dpi=150)
    print(f"Navigation图表已保存: {out_path}")
    return True

def plot_yaw_debug_records(records, out_path):
    rows = [r for r in records if r["_type"] == "YAW_DEBUG"]
    if not rows:
        return False

    t = [r["_t_control_s"] for r in rows]
    fig, axes = plt.subplots(4, 1, figsize=(14, 12), sharex=True)
    ax_heading, ax_error, ax_rate, ax_mag = axes

    ax_heading.plot(t, [r["fused_yaw_deg"] for r in rows], label="Fused Yaw")
    ax_heading.plot(t, [r["mag_yaw_deg"] for r in rows], label="Mag Yaw", alpha=0.75)
    ax_heading.plot(t, [r["yaw_target_deg"] for r in rows], ":", label="Heading Target")
    ax_heading.set_ylabel("deg")
    ax_heading.set_title("YawEstimator 与 Heading Hold")
    ax_heading.legend(fontsize=8, ncol=3)

    ax_error.plot(t, [r["mag_innovation_deg"] for r in rows], label="Mag innovation")
    ax_error.plot(t, [r["yaw_error_deg"] for r in rows], label="Heading error")
    rejected_t = [r["_t_control_s"] for r in rows if r["mag_result"] >= 3]
    rejected_v = [r["mag_innovation_deg"] for r in rows if r["mag_result"] >= 3]
    if rejected_t:
        ax_error.scatter(rejected_t, rejected_v, color="red", s=10, label="Mag rejected")
    ax_error.set_ylabel("deg")
    ax_error.legend(fontsize=8)

    ax_rate.plot(t, [r["yaw_rate_target_dps"] for r in rows], label="Yaw rate target")
    ax_rate.plot(t, [r["yaw_rate_meas_dps"] for r in rows], label="Yaw rate measured")
    ax_rate.plot(t, [r["yaw_output"] for r in rows], label="Yaw Rate PID output", alpha=0.75)
    ax_rate.set_ylabel("dps / output")
    ax_rate.legend(fontsize=8, ncol=3)

    for axis, color in zip(("x", "y", "z"), ("#d62728", "#2ca02c", "#1f77b4")):
        ax_mag.plot(t, [r[f"mag_{axis}_gauss"] for r in rows], color=color, label=f"M{axis}")
    ax_mag.plot(t, [r["mag_field_gauss"] for r in rows], color="black", label="|M|")
    ax_mag.set_ylabel("gauss")
    ax_mag.set_xlabel("时间 (s)")
    ax_mag.legend(fontsize=8, ncol=4)

    for ax in axes:
        ax.grid(True, alpha=0.2)
    fig.tight_layout()
    fig.savefig(out_path, dpi=150)
    print(f"Yaw专项图表已保存: {out_path}")
    return True


def plot_level_trim_debug_records(records, out_path):
    rows = [r for r in records if r["_type"] == "LEVEL_TRIM_DEBUG"]
    if not rows:
        return False

    t = [r["_t_control_s"] for r in rows]
    fig, axes = plt.subplots(3, 2, figsize=(15, 11), sharex=True)
    ax_state, ax_count, ax_sample, ax_result, ax_stddev, ax_apply = axes.flat

    ax_state.step(t, [r["state"] for r in rows], where="post", label="State")
    ax_state.step(t, [r["reset_count"] for r in rows], where="post", label="Reset count")
    ax_state.set_yticks([0, 1, 2])
    ax_state.set_yticklabels(["IDLE", "COLLECT", "SETTLE"])
    ax_state.set_title("Level Trim 状态")
    ax_state.legend(fontsize=8)

    ax_count.plot(t, [r["imu0_sample_count"] for r in rows], label="IMU0 count")
    ax_count.plot(t, [r["imu1_sample_count"] for r in rows], label="IMU1 count")
    ax_count.plot(t, [r["imu0_last_window_count"] for r in rows], ":", label="IMU0 last window")
    ax_count.plot(t, [r["imu1_last_window_count"] for r in rows], ":", label="IMU1 last window")
    ax_count.set_title("采样计数与清零前窗口")
    ax_count.legend(fontsize=8, ncol=2)

    for imu in (0, 1):
        ax_sample.plot(t, [r[f"imu{imu}_accel_norm_g"] for r in rows], label=f"IMU{imu} |a|")
    ax_sample.axhline(0.95, color="red", linestyle=":")
    ax_sample.axhline(1.05, color="red", linestyle=":")
    ax_sample.set_ylabel("g")
    ax_sample.set_title("Accel norm 静止门限")
    ax_sample.legend(fontsize=8)

    for imu in (0, 1):
        ax_result.plot(t, [r[f"imu{imu}_mean_roll_deg"] for r in rows], label=f"IMU{imu} Roll mean")
        ax_result.plot(t, [r[f"imu{imu}_mean_pitch_deg"] for r in rows], ":", label=f"IMU{imu} Pitch mean")
    ax_result.set_ylabel("deg")
    ax_result.set_title("候选 Trim 均值")
    ax_result.legend(fontsize=8, ncol=2)

    for imu in (0, 1):
        ax_stddev.plot(t, [r[f"imu{imu}_stddev_roll_deg"] for r in rows], label=f"IMU{imu} Roll std")
        ax_stddev.plot(t, [r[f"imu{imu}_stddev_pitch_deg"] for r in rows], ":", label=f"IMU{imu} Pitch std")
    ax_stddev.axhline(0.15, color="red", linestyle="--", label="0.15 deg limit")
    ax_stddev.set_ylabel("deg")
    ax_stddev.set_title("最终标准差门限")
    ax_stddev.legend(fontsize=8, ncol=2)

    ax_apply.plot(t, [r["active_accel_raw_roll_deg"] for r in rows], label="Raw Roll")
    ax_apply.plot(t, [r["active_accel_trimmed_roll_deg"] for r in rows], label="Trimmed Roll")
    ax_apply.plot(t, [r["active_accel_raw_pitch_deg"] for r in rows], ":", label="Raw Pitch")
    ax_apply.plot(t, [r["active_accel_trimmed_pitch_deg"] for r in rows], ":", label="Trimmed Pitch")
    ax_apply.set_ylabel("deg")
    ax_apply.set_title("Level Trim 实际应用前后")
    ax_apply.legend(fontsize=8, ncol=2)

    for ax in axes.flat:
        ax.grid(True, alpha=0.2)
        ax.set_xlabel("时间 (s)")
    fig.tight_layout()
    fig.savefig(out_path, dpi=150)
    print(f"Level Trim专项图表已保存: {out_path}")
    return True

def build_arg_parser():
    parser = argparse.ArgumentParser(description="解析STM32黑匣子BIN日志")
    parser.add_argument("path", help="LOGxxx.BIN文件路径")
    parser.add_argument(
        "--print-all", action="store_true",
        help="打印全部记录；默认只预览前20条（解析始终覆盖完整文件）",
    )
    parser.add_argument(
        "--csv", nargs="?", const="", default=None, metavar="PATH",
        help="导出全部记录到CSV；不指定PATH时自动生成*_analysis.csv",
    )
    parser.add_argument(
        "--cpu-hz", type=float, default=DEFAULT_CPU_HZ,
        help=f"DWT cycle换算频率，默认{DEFAULT_CPU_HZ}Hz",
    )
    parser.add_argument("--no-plot", action="store_true", help="不生成PNG图表")
    parser.add_argument("--no-show", action="store_true", help="保存图表但不打开窗口")
    return parser


def main():
    args = build_arg_parser().parse_args()
    if args.cpu_hz <= 0:
        raise SystemExit("--cpu-hz必须大于0")

    records, resync_count = parse(args.path)
    add_absolute_time(records, args.cpu_hz)
    print(
        f"共解析{len(records)}条记录，重同步次数{resync_count}"
        "（正常完整文件应为0，非0说明中间有数据损坏/写入中断）"
    )

    shown = records if args.print_all else records[:20]
    for rec in shown:
        print(rec)
    if not args.print_all and len(records) > 20:
        print(f"...其余{len(records) - 20}条省略；使用--print-all可显示全部")

    base_path = os.path.splitext(args.path)[0]
    if args.csv is not None:
        export_csv(records, args.csv or base_path + "_analysis.csv")

    plotted = False
    if not args.no_plot:
        if any(r["_type"] == "CONTROL" for r in records):
            plotted |= plot_control_records(records, base_path + "_analysis.png")
        elif any(r["_type"] == "MOTION" for r in records):
            plotted |= plot_legacy_records(records, base_path + "_analysis.png")
        plotted |= plot_yaw_debug_records(
            records, base_path + "_yaw_debug.png"
        )
        plotted |= plot_level_trim_debug_records(
            records, base_path + "_level_trim_debug.png"
        )
        plotted |= plot_navigation_records(
            records, base_path + "_velocity_analysis.png"
        )
        if not plotted:
            print("[图表跳过] 没有可画的记录")
        elif not args.no_show:
            try:
                plt.show()
            except Exception:
                pass


if __name__ == "__main__":
    main()
