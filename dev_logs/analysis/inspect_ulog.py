import pyulog
import os

dest_path = "/tmp/sliced_temp.ulg"
source_path = "/media/dorten/PX4_32G/log/2026-05-26/09_37_30.ulg"
chunk_size = 100 * 1024 * 1024 # 100 MB

with open(source_path, "rb") as sf:
    data = sf.read(chunk_size)
with open(dest_path, "wb") as df:
    df.write(data)

ul = pyulog.ULog(dest_path)
for name in ["vehicle_rates_setpoint", "vehicle_angular_velocity"]:
    ds = ul.get_dataset(name)
    print(f"\n{name} fields:")
    for k in ds.data.keys():
        print(f"- {k}: shape {ds.data[k].shape}, first 5: {ds.data[k][:5]}")

if os.path.exists(dest_path):
    os.remove(dest_path)
