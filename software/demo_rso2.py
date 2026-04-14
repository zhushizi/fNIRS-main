import numpy as np
import scipy.io as sio
import matplotlib.pyplot as plt

all_rso2_epoch_pct = []
for idx in range(1,18):
    data = sio.loadmat(f"{idx}_epo.mat", struct_as_record=False, squeeze_me=True)
    oxy_data = data["epo"].oxy.X   # 直接是 (409, 90, 60) 的 ndarray
    doxy_data = data["epo"].doxy.X   # 直接是 (409, 90, 60) 的 ndarray
    toxy_data = data["epo"].toxy.X   # 直接是 (409, 90, 60) 的 ndarray
    
    for i in range(oxy_data.shape[1]):
        for j in range(oxy_data.shape[2]):
            hbo = oxy_data[:, i, j]
            hbr = doxy_data[:, i, j]
            hbt = toxy_data[:, i, j]
            sum_abs_hbo = float(np.sum(np.abs(hbo))/len(hbo))
            sum_abs_hbr = float(np.sum(np.abs(hbr))/len(hbr))
            denom_epoch = sum_abs_hbo + sum_abs_hbr
            rso2_epoch_pct = 100.0 * sum_abs_hbo / denom_epoch
            all_rso2_epoch_pct.append(rso2_epoch_pct)

print(np.mean(np.array(all_rso2_epoch_pct)))

# t = np.arange(hbo.shape[0])
# fig, ax = plt.subplots(figsize=(10, 4))
# ax.plot(t, hbo, label="hbo", lw=1)
# ax.plot(t, hbr, label="hbr", lw=1)
# ax.plot(t, hbt, label="hbt", lw=1)
# ax.set_xlabel("sample")
# ax.set_ylabel("value")
# ax.legend()
# ax.grid(True, linestyle="--", alpha=0.6)
# fig.tight_layout()
# plt.show()