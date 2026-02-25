# fNIRS 数据处理链路详解（`fNIRS_processing.py`）

本文档面向“要改代码/接硬件/调参数”的使用场景，详细说明 `software/fNIRS_processing.py` 的完整处理链路。  
内容包括：算法流程、关键代码段、输入输出维度、默认参数、调参建议和常见坑点。

---

## 1. 一图看懂链路

```text
串口 64B 原始帧
  -> parse_packet
  -> capture_data -> all_groups.csv
  -> threshold_filter
  -> butter_lowpass_filter (1.0Hz, 4阶)
  -> sliding_window_rms (按 Emitter 切段)
  -> interleave_mode_blocks
  -> 重建时间轴 -> interleaved_output.csv
  -> process_csv_dataset
      -> combine_two_rows (660/940 配对)
      -> intensities_to_od_changes (OD)
      -> smart_bandpass (0.05~0.1Hz)
      -> mbll (HbO/HbR)
      -> cbsi (校正)
  -> processed_output.csv
```

---

## 2. 串口采集层

## 2.1 数据帧协议（当前代码假设）

- 固定帧长：`64` 字节
- 每帧包含 `8` 组
- 每组 `8` 字节：
  - `byte0`: Group ID
  - `byte1-2`: Short（uint16，大端）
  - `byte3-4`: Long1（uint16，大端）
  - `byte5-6`: Long2（uint16，大端）
  - `byte7`: Emitter 状态

## 2.2 关键代码段：包解析

```python
def parse_packet(data):
    ZERO_LEVEL = 2050
    parsed_data = np.zeros((8, 5), dtype=int)
    for i in range(8):
        offset = i * 8
        packet_identifier = data[offset]
        sensor_channel_1 = struct.unpack('>H', data[offset+1:offset+3])[0]
        sensor_channel_inv1 = 2 * ZERO_LEVEL - sensor_channel_1
        sensor_channel_2 = struct.unpack('>H', data[offset+3:offset+5])[0]
        sensor_channel_inv2 = 2 * ZERO_LEVEL - sensor_channel_2
        sensor_channel_3 = struct.unpack('>H', data[offset+5:offset+7])[0]
        sensor_channel_inv3 = 2 * ZERO_LEVEL - sensor_channel_3
        emitter_status = data[offset+7]
```

说明：

- 读取后做了 `2*ZERO_LEVEL - raw` 的反相映射。
- `parsed_data[i]` 最终结构是 `[gid, short, long1, long2, emitter]`。

## 2.3 关键代码段：采集写 CSV

```python
def capture_data(csv_filename, stop_on_enter=True):
    ...
    header = ["Time (s)"]
    for i in range(8):
        # 三路接收值 + 一路发射状态标签
        header += [f"G{i}_Short", f"G{i}_Long1", f"G{i}_Long2", f"G{i}_Emitter"]
    ...
    data = ser.read(64)
    if len(data) == 64:
        parsed_data = parse_packet(data)
```

输出文件：`all_groups.csv`

---

## 3. 预处理段（DataFrame 层）

输入：`all_groups.csv`  
输出：`interleaved_output.csv`

## 3.1 步骤 A：阈值滤波 `threshold_filter`

### 代码段

```python
suppressed_df[col] = np.where(
    (df[col] < lower_threshold) | (df[col] > upper_threshold),
    zero_level,
    df[col]
)
```

### 作用

- 将越界值直接替换为基线 `zero_level`，抑制异常点。

### 默认参数

- `lower_threshold=200`
- `upper_threshold=4000`
- `zero_level=2050`

### 注意

- `Emitter` 列一般通过 `exclude_columns` 排除，不参与该数值处理。

---

## 3.2 步骤 B：低通滤波 `butter_lowpass_filter`

### 代码段

```python
nyquist = 0.5 * fs
normal_cutoff = cutoff_hz / nyquist
b, a = butter(order, normal_cutoff, btype='low', analog=False)
filtered_df[col] = filtfilt(b, a, df[col])
```

### 作用

- 抑制高频噪声，保留慢变化。
- `filtfilt` 为零相位实现，不引入相位延迟。

### 默认参数

- `cutoff_hz=1.0`
- `order=4`
- `fs = 1.0 / mean(diff(Time))`

### 注意

- `fs` 由原始时间戳估算；若时间戳抖动明显，会影响截止频率的实际效果。

---

## 3.3 步骤 C：分段 RMS `sliding_window_rms`

### 代码段

```python
change_points = np.where(np.diff(emitter_reference) != 0)[0] + 1
segments = np.split(np.arange(len(df)), change_points)
...
rms_val = np.sqrt(np.mean(np.square(segment_data)))
df_rms.loc[segment_idx, col] = rms_val
```

### 作用

- 按 `Emitter` 状态变化切段。（对应了 660/940 的交替时隙）
- 每段对每个通道计算 RMS（均方根），并用该值回填整段。

### 默认参数

- `emitter_col="G0_Emitter"`
- `group_prefix="G"`
- `num_groups=8`
- `remove_dc=False`
- `split_segments_in_half=False`

### 注意

- 这不是固定窗口滑动平均，而是事件驱动分段。
- 分段质量强依赖 `Emitter` 切换是否稳定。

---

## 3.4 步骤 D：模式交错 `interleave_mode_blocks`

### 代码段

```python
df["group"] = (df[mode_col] != df[mode_col].shift()).cumsum()
...
while i < len(blocks) - 1:
    block1 = blocks[i].reset_index(drop=True)
    block2 = blocks[i + 1].reset_index(drop=True)
    ...
    all_rows.append(row1.copy())
    all_rows.append(row2.copy())
```

### 作用

- 将连续 mode 块成对交错。
- 为后续 “mode1=660 / mode2=940” 双波长配对做准备。

---

## 3.5 步骤 E：重建时间轴

### 代码段

```python
INCREMENT = 0.001
final_df.insert(0, "Time (s)", [i * INCREMENT for i in range(len(final_df))])
final_df["Time (s)"] = final_df["Time (s)"].round(3)
```

### 作用

- 丢弃原始时间戳后，强制用固定步长重建时间。

### 注意

- 该设计简化了后续处理，但会损失原始采样时间抖动信息。

---

## 4. 后处理段（矩阵层）

入口函数：`process_csv_dataset(input_csv, output_csv, ...)`  
输入：`interleaved_output.csv` # 每行33个=1+4*8 
输出：`processed_output.csv`

## 4.1 行配对为样本：`combine_two_rows`

### 代码段

```python
for g in range(8):
    short_660 = float(row_mode1[1 + 4*g])
    long1_660 = float(row_mode1[2 + 4*g])
    long2_660 = float(row_mode1[3 + 4*g])
    short_940 = float(row_mode2[1 + 4*g])
    long1_940 = float(row_mode2[2 + 4*g])
    long2_940 = float(row_mode2[3 + 4*g])
```

### 输出形状

- 单个样本长度：`48`（8组 × 每组6值）
- 样本矩阵：`samples.shape = (48, N)`（转置后）

---

## 4.2 OD 转换

### 代码段

```python
delta_od = nsp.intensities_to_od_changes(samples)
```

### 作用

- 从光强域转换到光密度变化（OD）域。

---

## 4.3 OD 带通滤波：`smart_bandpass`

### 代码段

```python
if fs > target_fs + 1:
    decim = int(round(fs / target_fs))
    data_ds = resample_poly(data, up=1, down=decim, axis=1)
...
sos = butter_bandpass_sos(lowcut, highcut, fs_ds, order)
data_bp = sosfiltfilt(sos, data_ds, axis=1, padtype="odd", padlen=3 * (order + 1))
...
if decim > 1:
    data_bp = resample_poly(data_bp, up=decim, down=1, axis=1)
```

### 作用

- 在时间轴做零相位带通。
- 高采样率时先降采样，滤波后再升采样，增强稳定性并降计算量。

### 默认参数

- `lowcut=0.05`
- `highcut=0.1`
- `order=4`
- `target_fs=20.0`

---

## 4.4 通道信息构建：`build_channel_info`

### 代码段

```python
physical_channels = [f"S{s}_D{d}" for s in range(1, 9) for d in (1, 2, 3)]
...
ch_wls.append(660.0); ch_dpfs.append(nsp.get_dpf(660.0, age))
...
ch_wls.append(940.0); ch_dpfs.append(nsp.get_dpf(940.0, age))
```

### 作用

- 生成 MBLL 所需元信息：通道名、波长、DPF、源探距离。

### 默认参数

- `age=22`
- `sd_short=0.6`
- `sd_long=3.5`

---

## 4.5 MBLL 反演

### 代码段

```python
delta_c, new_ch_names, new_ch_types = nsp.mbll(
    delta_od_filt,
    channel_names,
    ch_wls,
    ch_dpfs,
    ch_distances,
    unit='cm',
    table=molar_ext_coeff_table
)
```

### 作用（这一步在做什么）

- 将光密度变化 `delta_od_filt` 反演为血红蛋白浓度变化：
  - `HbO`（氧合血红蛋白）
  - `HbR`（脱氧血红蛋白）
- 这是从“光学量”到“生理量”的关键映射步骤。

### 数学模型（双波长、双组分）

对每个物理通道，在两个波长（本项目为 660/940）下有两条方程：

> 如果下面的 LaTeX 在你的编辑器中不显示，请直接看这段纯文本等价式：

```text
标量式:
ΔOD_λ = ( ε_(λ,HbO) * ΔHbO + ε_(λ,HbR) * ΔHbR ) * L * DPF_λ

矩阵式:
[ΔOD_660]   [ε_(660,HbO)*L*DPF_660   ε_(660,HbR)*L*DPF_660] [ΔHbO]
[ΔOD_940] = [ε_(940,HbO)*L*DPF_940   ε_(940,HbR)*L*DPF_940] [ΔHbR]

求解:
[ΔHbO]   -1 [ΔOD_660]
[ΔHbR] = A  [ΔOD_940]
```

\[
\Delta OD_{\lambda} =
\left(
\varepsilon_{\lambda,HbO}\Delta HbO +
\varepsilon_{\lambda,HbR}\Delta HbR
\right)\cdot L \cdot DPF_{\lambda}
\]

写成矩阵形式：

\[
\begin{bmatrix}
\Delta OD_{660}\\
\Delta OD_{940}
\end{bmatrix}
=
\underbrace{
\begin{bmatrix}
\varepsilon_{660,HbO}LDPF_{660} & \varepsilon_{660,HbR}LDPF_{660}\\
\varepsilon_{940,HbO}LDPF_{940} & \varepsilon_{940,HbR}LDPF_{940}
\end{bmatrix}
}_{A}
\begin{bmatrix}
\Delta HbO\\
\Delta HbR
\end{bmatrix}
\]

求解即：

\[
\begin{bmatrix}
\Delta HbO\\
\Delta HbR
\end{bmatrix}
=A^{-1}
\begin{bmatrix}
\Delta OD_{660}\\
\Delta OD_{940}
\end{bmatrix}
\]

工程实现通常用稳定求解（如伪逆/数值稳定算法），底层由 `nirsimple` 处理。

### 本项目传入 `mbll(...)` 的参数来源

- `delta_od_filt`：上一步带通滤波后的 OD 矩阵，形状约 `(48, N)`。
- `channel_names`：`build_channel_info` 构建，形如 `S1_D1, S1_D1, ...`（按 660/940 展开）。
- `ch_wls`：波长数组，交替为 `660, 940`。
- `ch_dpfs`：`nsp.get_dpf(波长, age)` 计算得到，受年龄影响。
- `ch_distances`：源探距离，`D1` 用 `sd_short`，`D2/D3` 用 `sd_long`。
- `table`：摩尔消光系数表，默认 `'wray'`。
- `unit`：距离单位，默认 `'cm'`。

### 为什么需要 `age`

- 年龄用于计算 `DPF`。
- `DPF` 描述光在组织中的等效路径修正（非直线传播），会影响反演量级和准确性。

### 输入输出形状与含义

- 输入：`delta_od_filt`，约 `(48, N)`  
  （48 = 24 物理通道 × 2 波长）
- 输出：
  - `delta_c`：浓度变化矩阵，约 `(48, N)`
  - `new_ch_names`：反演后通道名
  - `new_ch_types`：类型标签（`hbo` / `hbr`）

### 一个最小数值示例（便于理解）

设某一通道某时刻：

- \(\Delta OD_{660}=0.05\)
- \(\Delta OD_{940}=0.03\)

设系数矩阵（示意）：

\[
A=
\begin{bmatrix}
1.8 & 3.6\\
2.475 & 1.32
\end{bmatrix}
\]

则可反解得到一组 \(\Delta HbO,\Delta HbR\)（具体数值由矩阵求逆/求解得到）。  
这就是 `mbll(...)` 在每个时间点、每个通道上做的事情。

### 常见误区

- 误区 1：MBLL 是滤波器。  
  实际：MBLL 是“物理模型反演”，不是滤波。
- 误区 2：OD 后就直接是 HbO/HbR。  
  实际：OD 只是中间量，必须经 MBLL 才是浓度变化。
- 误区 3：`sd_short/sd_long`、`age` 可随意填。  
  实际：这些参数会直接影响结果量级与可解释性。

### 默认参数

- `molar_ext_coeff_table='wray'`
- `unit='cm'`

---

## 4.6 CBSI 校正

### 代码段

```python
delta_c_corr, corr_ch_names, corr_ch_types = nproc.cbsi(delta_c, new_ch_names, new_ch_types)
```

### 作用

- 对 MBLL 输出进行相关性约束校正。
- `delta_c_corr` 作为最终输出矩阵。

---

## 5. 文件级输入输出关系

## 5.1 `all_groups.csv`

- 来源：`capture_data`
- 内容：原始 ADC（Time + G0~G7 的 Short/Long1/Long2/Emitter）

## 5.2 `interleaved_output.csv`

- 来源：预处理 + 模式交错 + 重建时间轴
- 用途：作为 `process_csv_dataset` 输入

## 5.3 `processed_output.csv`

- 来源：OD -> 带通 -> MBLL -> CBSI
- 列结构：
  - `Time`
  - `{ChannelName}_{Type}`（`Type` 为 `hbo` / `hbr`）

---

## 6. 主流程对应代码（按执行顺序）

```python
if __name__ == '__main__':
    capture_data("all_groups.csv", stop_on_enter=True)
    df = pd.read_csv("all_groups.csv")

    time = df['Time (s)']
    dt = time.diff().mean()
    fs = 1.0 / dt

    if "Time (s)" in df.columns:
        df.drop(columns=["Time (s)"], inplace=True)

    exclude_cols = [c for c in df.columns if 'Emitter' in c]
    df = threshold_filter(df, exclude_columns=exclude_cols)
    df = butter_lowpass_filter(df=df, cutoff_hz=1.0, fs=fs, order=4, exclude_columns=exclude_cols)
    df = sliding_window_rms(df=df)

    final_df = interleave_mode_blocks(df, mode_col="G0_Emitter")
    ...
    final_df.to_csv("interleaved_output.csv", index=False)

    process_csv_dataset("interleaved_output.csv", "processed_output.csv")
```

---

## 7. 默认参数速查表

| 模块 | 参数 | 默认值 |
|---|---|---|
| 阈值滤波 | `lower_threshold` | `200` |
| 阈值滤波 | `upper_threshold` | `4000` |
| 阈值滤波 | `zero_level` | `2050` |
| 低通滤波 | `cutoff_hz` | `1.0` |
| 低通滤波 | `order` | `4` |
| 带通滤波 | `lowcut` | `0.05` |
| 带通滤波 | `highcut` | `0.1` |
| 带通滤波 | `order` | `4` |
| 带通滤波 | `target_fs` | `20.0` |
| MBLL | `age` | `22` |
| MBLL | `sd_short` | `0.6` |
| MBLL | `sd_long` | `3.5` |
| MBLL | `table` | `wray` |

---

## 8. 常见问题与排查

## 8.1 `processed_output.csv` 为空或行数很少

- 检查 `all_groups.csv` 是否有效采集。
- 检查 `process_csv_dataset` 中 `num_rows < 2` 条件。
- 检查 mode1/mode2 是否成对（奇偶行是否符合预期）。

## 8.2 滤波报错或输出异常

- 检查 `fs = 1/dt` 是否异常（例如 `dt` 太小或为 0）。
- 检查 `highcut < Nyquist` 是否满足。
- 检查时间列是否存在大量重复时间戳。

## 8.3 结果幅值明显不合理

- 确认 `sd_short/sd_long` 是否与硬件一致。
- 确认 DPF 计算所用 `age` 是否合理。
- 确认双波长配对顺序未错位。

---

## 9. 硬件适配改造建议（最小闭环）

如果硬件组数或每组接收通道数变化，建议按以下顺序改造：

1. `parse_packet` / `capture_data`：先对齐帧结构和列头。  
2. `sliding_window_rms`：对齐组数和列命名规则。  
3. `combine_two_rows`：对齐“每组字段数”和双波长拼接逻辑。  
4. `build_channel_info`：对齐通道数量、波长和源探距离。  
5. 最后验证 `processed_output.csv` 列数是否与预期一致。  

---

## 10. 相关函数索引

- 串口采集：`capture_data`
- 包解析：`parse_packet`
- 阈值滤波：`threshold_filter`
- 低通滤波：`butter_lowpass_filter`
- 分段 RMS：`sliding_window_rms`
- 模式交错：`interleave_mode_blocks`
- 双行配对：`combine_two_rows`
- 带通滤波：`butter_bandpass_sos`, `smart_bandpass`
- 反演主入口：`process_csv_dataset`

