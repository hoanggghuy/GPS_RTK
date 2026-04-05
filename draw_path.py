import argparse

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd


def geodetic_to_cartesian(lat, lon, lat_ref, lon_ref):
    R = 6371000
    phi = np.radians(lat)
    lambda_ = np.radians(lon)
    phi_ref = np.radians(lat_ref)
    lambda_ref = np.radians(lon_ref)

    x = R * (lambda_ - lambda_ref) * np.cos(phi_ref)
    y = R * (phi - phi_ref)
    return x, y


def detect_lat_lon_columns(df):
    if {"latitude", "longitude"}.issubset(df.columns):
        return "latitude", "longitude"
    if {"lat", "lon"}.issubset(df.columns):
        return "lat", "lon"
    raise ValueError(
        "Không tìm thấy cột tọa độ. Cần một trong 2 cặp: "
        "('latitude','longitude') hoặc ('lat','lon')."
    )


def load_track(file_path):
    df = pd.read_csv(file_path)
    lat_col, lon_col = detect_lat_lon_columns(df)
    track = df.dropna(subset=[lat_col, lon_col]).copy()
    track[lat_col] = pd.to_numeric(track[lat_col], errors="coerce")
    track[lon_col] = pd.to_numeric(track[lon_col], errors="coerce")
    track = track.dropna(subset=[lat_col, lon_col])
    if track.empty:
        raise ValueError("Không có điểm GPS hợp lệ để vẽ.")

    lat0 = track[lat_col].iloc[0]
    lon0 = track[lon_col].iloc[0]
    track["x"], track["y"] = geodetic_to_cartesian(track[lat_col], track[lon_col], lat0, lon0)
    return track


def main():
    parser = argparse.ArgumentParser(description="Vẽ quỹ đạo GPS từ file CSV.")
    parser.add_argument("--input", default="gps_rear_path.csv", help="Đường dẫn file CSV đầu vào")
    parser.add_argument("--output", default="gps_trajectory1.png", help="Tên file ảnh đầu ra")
    parser.add_argument("--show", action="store_true", help="Hiển thị biểu đồ trên màn hình")
    args = parser.parse_args()

    track = load_track(args.input)

    plt.figure(figsize=(10, 8))
    plt.plot(track["x"].values, track["y"].values, color="blue", linewidth=1.8, label="Trajectory")
    plt.scatter(track["x"].iloc[0], track["y"].iloc[0], color="green", marker="s", s=60, label="Start")
    plt.scatter(track["x"].iloc[-1], track["y"].iloc[-1], color="red", marker="o", s=60, label="End")
    plt.xlabel("X (meters)")
    plt.ylabel("Y (meters)")
    plt.title("GPS Trajectory")
    plt.grid(True)
    plt.axis("equal")
    plt.legend()
    plt.tight_layout()

    plt.savefig(args.output, dpi=150)
    print(f"Đã lưu ảnh quỹ đạo: {args.output}")

    if args.show:
        plt.show()
    else:
        plt.close()


if __name__ == "__main__":
    main()