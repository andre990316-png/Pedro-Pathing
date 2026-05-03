# 發射器子系統架構規格書 (Shooter Subsystem Specification)

本文件詳細記載了 2026 賽季發射器 (Shooter) 的系統架構、數學模型、變數定義以及現場調適的標準作業程序 (SOP)。
本架構專為「單飛輪 + 弧形軌道 (Hooded Shooter)」設計，將混亂的物理現實與純淨的數學模型分離，確保系統具備極高的可靠性與可維護性。

---

## 1. 系統架構總覽 (Architecture Overview)

系統被嚴格劃分為三個層級，確保職責分離：

1. **擬合物理層 (Physical Fitting Layer)**
   負責各式單位轉換 (例如 Ticks <-> RPM <-> in/sec)。透過現場實測的「雙參數 (轉換率與損失率)」來隱含並抵銷空氣阻力、摩擦打滑、Magnus 效應等難以純數學解析的現實物理因素。
2. **數學模型層 (Mathematical Model Layer)**
   負責純粹的拋物線運算。利用物理層提供的虛擬參數，計算出理想的發射仰角與所需的射彈初速。本層被完全獨立在 `ShooterCalculator.java` 靜態類別中。
3. **反饋控制層 (Feedback Control Layer)**
   負責硬體對接 (`Shooter.java`)。讀取感測器 (Encoder) 的真實反饋，根據當下的真實轉速，動態微調最終的伺服馬達 (仰角) 輸出，確保飛輪在加速 (Spin-up) 或受干擾時，系統仍能保持最佳的命中機率。

---

## 2. 參數與變量定義 (Constants & Variables)

所有常數皆集中於 `Config.java` 內的 `ShooterConfig` 類別中。

### 2.1 絕對硬體常數 (Absolute Hardware Constants)
這些數值取決於硬體規格，量測後通常不需要在比賽現場變更：
*   **`G` (重力加速度)**: `386.09` (in/s²)。
*   **`MAX_VELOCITY_IN_PER_SEC`**: 飛輪能提供的最大線速度極限。
*   **`MIN_PITCH_ANGLE_DEG` / `MAX_PITCH_ANGLE_DEG`**: 仰角伺服馬達的機械極限。
*   **`TICKS_PER_REVOLUTION`**: 馬達每圈的 Encoder Ticks 數 (例如 Bare Motor = 28)。
*   **`GEAR_RATIO`**: 馬達軸到飛輪軸的齒輪/皮帶傳動比 (直驅 = 1.0)。
*   **`FLYWHEEL_RADIUS_IN`**: 飛輪的實際半徑 (英吋)。

### 2.2 雙參數架構 - 場地動態擬合常數 (Two-Parameter Architecture)
只需調整這兩個參數，即可擬合所有的非理想物理因素：
*   **`ENERGY_CONVERSION_RATE` (能量轉換率)**:
    *   **意義**：飛輪轉速轉換為球體初速的效率。包含球體壓縮與軌道打滑的損耗。
    *   **特性**：通常介於 0.5 ~ 0.9 之間。於**近點無阻力測試**中求出。
*   **`ENERGY_LOSS_RATE` (能量損失率)**:
    *   **意義**：球體飛行過程中因空氣阻力與旋轉 (Magnus 效應) 造成的動能流失與提早下墜。
    *   **特性**：通常為微小的正數 (如 0.002)。透過平移虛擬目標點來欺騙數學模型。於**遠點測試**中求出。

### 2.3 數學模型與控制參數 (Math & Feedback Control)
*   **`IDEAL_INCIDENT_ANGLE_DEG` (理想入射角)**: 期望球體落入目標 (例如籃筐) 時的完美角度 (例如 60 度)。系統會優先以此角度反推所需初速。
*   **`RPM_ERROR_THRESHOLD` (轉速誤差容忍度)**: 當真實轉速與目標轉速的誤差大於此值時，仰角將維持在預設值，避免飛輪加速期間伺服馬達發生劇烈抖動。
*   **`PITCH_OFFSET_DEG` (仰角手動偏移)**: 供駕駛員在 Teleop 階段進行直覺的微調補償。

### 2.4 實時資料結構 (Real-time Data Structure)
*   **`ShooterPose`**: 統一存放 `x`, `y`, `z`, `yaw` 的資料結構，避免傳統 `double[]` 造成的 Index 混淆。

---

## 3. 核心運算流程 (Workflow & Formulas)

整個發射循環 (Loop) 嚴格遵循以下五個階段：

### 階段 I：物理層轉換 (RPM <-> 初速)
由於本系統採用「單飛輪 + 弧形軌道 (Hooded Shooter)」，球體一側貼緊飛輪，另一側與軌道摩擦滾動。
**物理公式：** 理論上，球體的出膛初速只有飛輪表面線速度的「一半」。
$$V_{theoretical\_exit} = \frac{V_{surface}}{2} = \frac{\frac{RPM}{60} \times 2\pi r}{2}$$
**最終轉換：** 將理論初速乘上 `ENERGY_CONVERSION_RATE` 來彌補擠壓與打滑。
$$V_{actual} = V_{theoretical\_exit} \times \text{ENERGY\_CONVERSION\_RATE}$$

### 階段 II & III：計算虛擬目標點 B' (補償空氣阻力)
為了解決空氣阻力與旋轉下墜，我們不修改標準拋物線公式，而是將實際目標 B 往後、往上平移，創造一個「虛擬目標 B'」。
令 $d$ 為目標水平距離，$h$ 為目標高度差：
$$d' = d \times (1 + \text{ENERGY\_LOSS\_RATE} \times d)$$
$$h' = h + (\text{ENERGY\_LOSS\_RATE} \times d^2)$$
這會迫使數學模型算出更大的初速，剛好抵銷飛行中的流失。

### 階段 IV：計算理想初速與前饋目標 RPM
已知射擊起點 A 與虛擬目標 B'，系統會嘗試以 `IDEAL_INCIDENT_ANGLE_DEG` ($\theta_{ideal}$) 作為拋物線仰角，反解出所需的完美初速 $V_{ideal}$：
$$V_{ideal} = \sqrt{ \frac{g \cdot (d')^2}{2 \cos^2(\theta_{ideal}) (d' \tan(\theta_{ideal}) - h')} }$$
算出 $V_{ideal}$ 後，透過階段 I 的反向公式求出「前饋目標 RPM」，並直接送入飛輪馬達的 PID 控制器。
*(若 $V_{ideal}$ 超出硬體極限，則約束放寬，強制回傳硬體最大容許初速)*

### 階段 V：反饋控制與真實仰角計算
飛輪馬達加速需要時間。系統會實時讀取飛輪的「真實 RPM」並轉換為「真實當下初速 $V_{actual}$」。
*   **穩定判定**：若 `|目標 RPM - 真實 RPM| > RPM_ERROR_THRESHOLD`，系統判定為「正在加速中 (Spinning up)」，此時仰角維持在理想初速對應的角度，防止伺服馬達亂跳。
*   **真實反饋**：若轉速達標，系統使用當下的 $V_{actual}$ 反向解一元二次方程式，求出當下真正能打中虛擬目標 B' 的「真實仰角 $\theta_{real}$」。
$$h' = d' \tan(\theta_{real}) - \frac{g (d')^2}{2 V_{actual}^2} (1 + \tan^2(\theta_{real}))$$
系統會自動選擇方程式的「高仰角解」，並限制在硬體容許的 `MIN` 與 `MAX` 範圍內，最後輸出給伺服馬達。

---

## 4. 現場調適標準作業程序 (Calibration SOP)

每當更換馬達、重印軌道 (Hood)、或換了一批耗損不同的球時，請按照以下兩步 SOP 重新校正雙參數：

### SOP 步驟 1：近距離測試 (求解 ENERGY_CONVERSION_RATE)
**目標：在忽略空氣阻力的近距離下，找出純粹的硬體傳輸效率。**
1. 將機器人移至距離目標極近的地方 (例如 24 吋)。
2. 暫時關閉自動仰角，將伺服馬達固定在 **45度角**。
3. 手動微調飛輪的 RPM 發射，直到球**剛好完美掉入目標**。記下這個轉速 `perfect_rpm`。
4. 呼叫 `ShooterCalculator.calibrateEnergyConversionRate(distance, height, perfect_rpm)`。
5. 將回傳的值填入 `Config.java` 的 `ENERGY_CONVERSION_RATE`。

### SOP 步驟 2：遠距離測試 (求解 ENERGY_LOSS_RATE)
**目標：在遠距離下，利用虛擬平移來補償空氣阻力與 Magnus 下墜效應。**
1. **必須確保 SOP 步驟 1 已經完成**，且轉換率已填寫正確。
2. 將機器人退至遠距離 (例如 72 吋以上)。
3. 開啟自動化數學模型 (恢復正常 Loop 運作) 並發射。
4. 觀察落點：
   * 若球**提早下墜 (落在目標前方)**，代表空氣阻力影響較大。
   * 請至 Dashboard 手動將 `ENERGY_LOSS_RATE` 逐步**調大** (建議每次增加 `0.001`)。
5. 反覆測試直到球在遠距離也能完美命中。將最終數值填入 `Config.java`。