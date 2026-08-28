import argparse
import csv
import os

import matplotlib.pyplot as plt
import numpy as np


MIN_REQUIRED_SAMPLES = 1000


def load_mag_samples(csv_path):
    samples = []
    with open(csv_path, "r", encoding="utf-8-sig", newline="") as f:
        for row in csv.DictReader(f):
            if row.get("record_type") != "MAG_CAL_SAMPLE":
                continue
            try:
                samples.append([
                    float(row["mag_x_gauss"]),
                    float(row["mag_y_gauss"]),
                    float(row["mag_z_gauss"]),
                ])
            except (KeyError, TypeError, ValueError):
                continue

    points = np.asarray(samples, dtype=np.float64)
    if points.ndim != 2 or points.shape[1:] != (3,):
        raise RuntimeError("CSV中没有可用的MAG_CAL_SAMPLE记录")
    points = points[np.all(np.isfinite(points), axis=1)]
    if len(points) < MIN_REQUIRED_SAMPLES:
        raise RuntimeError(
            f"有效样本只有{len(points)}个，至少需要{MIN_REQUIRED_SAMPLES}个"
        )
    return points


def fit_ellipsoid(points):
    """拟合 (m-bias)^T W (m-bias)=1，并返回bias和对称校正矩阵。"""
    x, y, z = points.T
    design = np.column_stack((
        x * x, y * y, z * z,
        2.0 * y * z, 2.0 * x * z, 2.0 * x * y,
        2.0 * x, 2.0 * y, 2.0 * z,
    ))

    params, _, rank, _ = np.linalg.lstsq(
        design, np.ones(len(points)), rcond=None
    )
    if rank < 9:
        raise RuntimeError("采样方向覆盖不足，椭球拟合矩阵不满秩")

    quad = np.array([
        [params[0], params[5], params[4]],
        [params[5], params[1], params[3]],
        [params[4], params[3], params[2]],
    ])
    linear = params[6:9]

    bias = -np.linalg.solve(quad, linear)
    scale = 1.0 + bias @ quad @ bias
    if not np.isfinite(scale) or abs(scale) < 1.0e-12:
        raise RuntimeError("椭球归一化失败：scale无效")

    shape = quad / scale
    eigenvalues, eigenvectors = np.linalg.eigh(shape)
    if np.any(eigenvalues <= 0.0) or not np.all(np.isfinite(eigenvalues)):
        raise RuntimeError("拟合结果不是正定椭球，请扩大三维方向覆盖并重新采样")

    condition = float(eigenvalues.max() / eigenvalues.min())
    if condition > 100.0:
        raise RuntimeError(f"椭球条件数过大({condition:.1f})，采样覆盖不可靠")

    sqrt_shape = eigenvectors @ np.diag(np.sqrt(eigenvalues)) @ eigenvectors.T

    # 保持Gauss量级：把单位球恢复为去偏置后样本的中位半径。
    target_field = float(np.median(np.linalg.norm(points - bias, axis=1)))
    correction = target_field * sqrt_shape
    return bias, correction, target_field, condition


def apply_calibration(points, bias, correction):
    return (correction @ (points - bias).T).T


def robust_fit(points):
    mask = np.ones(len(points), dtype=bool)

    for _ in range(3):
        bias, correction, field_ref, condition = fit_ellipsoid(points[mask])
        corrected = apply_calibration(points, bias, correction)
        ratio = np.linalg.norm(corrected, axis=1) / field_ref
        residual = np.abs(ratio - 1.0)

        used_residual = residual[mask]
        median = float(np.median(used_residual))
        mad = float(np.median(np.abs(used_residual - median)))
        threshold = min(0.20, max(0.03, median + 4.0 * 1.4826 * mad))
        new_mask = residual <= threshold

        if new_mask.sum() < max(MIN_REQUIRED_SAMPLES, int(0.70 * len(points))):
            raise RuntimeError("异常点过多，采样环境可能存在明显动态磁干扰")
        if np.array_equal(mask, new_mask):
            break
        mask = new_mask

    bias, correction, field_ref, condition = fit_ellipsoid(points[mask])
    corrected = apply_calibration(points, bias, correction)
    return bias, correction, field_ref, condition, mask, corrected


def check_coverage(corrected, mask):
    used = corrected[mask]
    codes = (
        (used[:, 0] >= 0.0).astype(np.uint8) |
        ((used[:, 1] >= 0.0).astype(np.uint8) << 1) |
        ((used[:, 2] >= 0.0).astype(np.uint8) << 2)
    )
    counts = np.bincount(codes, minlength=8)
    if np.any(counts == 0):
        missing = np.flatnonzero(counts == 0).tolist()
        raise RuntimeError(f"三维覆盖不完整，缺少八象限编号：{missing}")
    return counts


def format_report(points, bias, correction, field_ref, condition, mask, corrected):
    norms = np.linalg.norm(corrected[mask], axis=1)
    ratio_error = np.abs(norms / field_ref - 1.0)
    sym = (
        correction[0, 0], correction[0, 1], correction[0, 2],
        correction[1, 1], correction[1, 2], correction[2, 2],
    )

    return "\n".join((
        f"总样本数: {len(points)}",
        f"参与拟合: {int(mask.sum())}",
        f"剔除异常: {int((~mask).sum())}",
        f"椭球条件数: {condition:.6f}",
        f"校正后磁场参考: {field_ref:.9f} Gauss",
        f"模长相对误差 RMS: {np.sqrt(np.mean(ratio_error ** 2)) * 100.0:.3f}%",
        f"模长相对误差 P95: {np.percentile(ratio_error, 95) * 100.0:.3f}%",
        "",
        "/* soft_matrix_sym顺序：A00, A01, A02, A11, A12, A22 */",
        "static const MagCalibrationConfig_t s_mag_calibration =",
        "{",
        "    .bias = {" + ", ".join(f"{v:.9f}f" for v in bias) + "},",
        "    .soft_matrix_sym = {" + ", ".join(f"{v:.9f}f" for v in sym) + "},",
        f"    .field_reference_gauss = {field_ref:.9f}f,",
        "};",
    ))


def set_equal_3d(ax, points):
    mins = points.min(axis=0)
    maxs = points.max(axis=0)
    center = (mins + maxs) * 0.5
    radius = max((maxs - mins).max() * 0.5, 1.0e-6)
    ax.set_xlim(center[0] - radius, center[0] + radius)
    ax.set_ylim(center[1] - radius, center[1] + radius)
    ax.set_zlim(center[2] - radius, center[2] + radius)
    ax.set_box_aspect((1, 1, 1))


def save_plot(points, corrected, mask, field_ref, out_path):
    step = max(1, len(points) // 3000)
    fig = plt.figure(figsize=(15, 5))
    ax_raw = fig.add_subplot(131, projection="3d")
    ax_corr = fig.add_subplot(132, projection="3d")
    ax_norm = fig.add_subplot(133)

    ax_raw.scatter(*points[::step].T, s=3, alpha=0.45)
    ax_raw.set_title("Raw Mag ellipsoid")
    ax_raw.set_xlabel("X / G")
    ax_raw.set_ylabel("Y / G")
    ax_raw.set_zlabel("Z / G")
    set_equal_3d(ax_raw, points)

    ax_corr.scatter(*corrected[mask][::step].T, s=3, alpha=0.45)
    ax_corr.set_title("Corrected Mag sphere")
    ax_corr.set_xlabel("X / G")
    ax_corr.set_ylabel("Y / G")
    ax_corr.set_zlabel("Z / G")
    set_equal_3d(ax_corr, corrected[mask])

    norms = np.linalg.norm(corrected[mask], axis=1)
    ax_norm.hist(norms, bins=60, alpha=0.8)
    ax_norm.axvline(field_ref, color="red", linestyle="--", label="field reference")
    ax_norm.set_title("Corrected field norm")
    ax_norm.set_xlabel("Gauss")
    ax_norm.grid(True, alpha=0.2)
    ax_norm.legend()

    fig.tight_layout()
    fig.savefig(out_path, dpi=160)


def main():
    parser = argparse.ArgumentParser(
        description="对MAG_CAL_SAMPLE CSV执行Hard/Soft-Iron椭球拟合"
    )
    parser.add_argument("csv_path", help="main.py --csv生成的_analysis.csv")
    parser.add_argument("--out-prefix", default=None)
    args = parser.parse_args()

    points = load_mag_samples(args.csv_path)
    bias, correction, field_ref, condition, mask, corrected = robust_fit(points)
    octant_counts = check_coverage(corrected, mask)

    report = format_report(
        points, bias, correction, field_ref, condition, mask, corrected
    )
    report += "\n八象限样本数: " + ", ".join(str(v) for v in octant_counts) + "\n"

    prefix = args.out_prefix or os.path.splitext(args.csv_path)[0] + "_mag_fit"
    report_path = prefix + ".txt"
    plot_path = prefix + ".png"

    with open(report_path, "w", encoding="utf-8") as f:
        f.write(report)
    save_plot(points, corrected, mask, field_ref, plot_path)

    print(report)
    print(f"参数已保存: {report_path}")
    print(f"拟合图已保存: {plot_path}")


if __name__ == "__main__":
    main()
