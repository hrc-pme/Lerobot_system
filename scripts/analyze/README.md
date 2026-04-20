# LeRobot Dataset & Policy Analysis Tools

這個資料夾包含了我們用來診斷 **LeRobot Bimanual (雙臂) 任務模式崩潰 (Mode Collapse)** 與 **資料集變異數 (Dataset Variance)** 的一系列分析工具與腳本。

當你的模型在推論時發生「只會做其中一種動作 (例如永遠只動左手)」，或者「對環境特徵不敏感」時，請依照順序使用以下腳本來排查原因。

---

## 🛠️ 包含的腳本與功能介紹

### 1. `check_action_stats.py` (資料集動作統計分析)
*   **功能**：計算資料集中每一個關節 (Joint) 的動作極值 (Min/Max) 與變異數 (Variance)。
*   **邏輯**：神經網路 (如 ACT 等依賴 L1/MSE 損失的模型) 會嚴重被高變異數的資料牽著走。這個腳本幫助我們找出是否某一隻手 (例如左手 Joint 6-11) 的動作變異數遠大於另一隻手 (右手 Joint 0-5)。
*   **用法**：
    ```bash
    python3 scripts/analyze/check_action_stats.py
    ```

### 2. `analyze_visual_cues.py` (視覺特徵熱力圖分析)
*   **功能**：分析兩段不同的 Episode 初始晝面 (例如左手擦 vs 右手擦的擺設)，計算它們在神經網路眼裡的**像素級差異 (Pixel Difference Heatmap)**。
*   **邏輯**：如果兩張圖片在熱力圖上除了輕微雜訊外幾乎全為冷色調 (藍色)，代表「這兩個條件」在視覺上長得太像了，模型根本分辨不出現在身處哪一種任務情境。反之，如果紅色/黃色特徵明顯，代表模型看得到條件，只是在演算法計算 Loss 時被忽略了。
*   **用法**：(注意：須在 LeRobot 虛擬環境中執行)
    ```bash
    source /opt/venv/bin/activate
    python3 scripts/analyze/analyze_visual_cues.py
    ```
    *執行後會產出一張包含視角熱力對照的 `visual_cue_analysis.png`。*

### 3. `diagnose_policy.py` (神經網路策略診斷)
*   **功能**：這是一支將模型直接「解剖」的程式。它會讀取已經訓練好的 `pretrained_model` 中的權重，並畫出神經網路內建的 `MEAN_STD` Normalizer 分析圖。
*   **邏輯**：證明在訓練初期神經網路是否因為資料不平衡產生了偏差 (Bias)。它會餵給網路一組全零的空白假輸入，觀察模型在失去視覺條件下的「預設輸出」。如果預設輸出高度偏袒某一隻手，代表發生了嚴重的後驗坍塌 (Posterior Collapse)。
*   **用法**：(會生成 `norm_std_plot.png` 與 `inference_bias_plot.png`)
    ```bash
    python3 scripts/analyze/diagnose_policy.py
    ```

### 4. `analyze_dataset.py` (資料集整體品質與軌跡掃描)
*   **功能**：計算各個 parquet 檔案裡的總 Frame 數是否平均，並計算整體的空間變異數。主要是為了排除 `dataset.streaming` 順序問題或資料遺失問題。
*   **用法**：
    ```bash
    python3 scripts/analyze/analyze_dataset.py
    ```

---

## 💡 總結：模式崩潰診斷流程 (Troubleshooting Workflow)

如果你遇到雙臂機器人只會動單手的問題：
1. 先跑 `check_action_stats.py`，確認雙臂動作**資料變異數**是否有極端的不平衡 (例如左邊 5倍大於 右邊)。
2. 再跑 `analyze_visual_cues.py`，確認用來觸發切換兩隻手的**環境裝備 (Box, Colors, 桌布)** 足不足以在像素級產生強烈特徵。
3. 最後跑 `diagnose_policy.py`，從模型 checkpoint 裡證實它是不是已經放棄了變異小的手。
4. 如果以上皆是，代表遇到了數學最佳化的限制。解法為：**將 Dataset 依任務拆分成多個模型分別訓練 (最快)**，或是**修改底層演算法加入關節權重 Loss / Auxiliary Task**。

---

## 🚀 針對模式崩潰的解決與改進方案 (Solutions for Mode Collapse)

當你透過上述腳本證實模型因為「**動作變異數不平衡 (Action Variance Imbalance)**」與「**視覺條件微弱 (Weak Conditioning)**」而產生只動一隻手的崩潰現象時，請參考以下五條改進路線：

### 🔧 工程派解法 (工業界最快實用)

**方案一：拆分資料集訓練 (Dataset Splitting) [推薦程度：⭐⭐⭐⭐⭐]**
* **原理**：既然單一模型無法在一個大池子裡學會左右手切換，那就不要為難它。將矛盾的任務分開。
* **作法**：寫一個腳本將原本的資料集拆分為 `dataset_left_wipe` 和 `dataset_right_wipe`，分別訓練兩個單獨的 Policy (模型)。
* **推論端**：在 ROS 推論時，加入簡單的電腦視覺 (如 OpenCV 顏色追蹤) 來判斷桌上衛生紙盒的位置，自動決定要載入哪一個模型節點控制手臂。

**方案二：增強視覺標定 (Visual Anchors Enhancement) [推薦程度：⭐⭐⭐⭐]**
* **原理**：神經網路 (ResNet) 很容易忽略背景。強勢介入讓它「不得不看」。
* **作法**：在作為條件判斷的物體 (如衛生紙盒) 上貼上螢光膠帶、高對比顏色標籤，同時維持每次錄製有 5~10 公分的桌面隨機位移 (Domain Randomization)，強迫 CNN 提取該特徵。

### 🔬 演算法 / 研究派解法 (需要修改 LeRobot 核心)

**方案三：關節加權損失函數 (Joint-Weighted Loss) [推薦程度：⭐⭐⭐⭐]**
* **原理**：左手變異數是 5，右手是 1，模型為了偷懶只學左手。我們透過 Loss 大小強行把它們的權重拉平。
* **作法**：修改 `lerobot/policies/act/modeling_act.py` 中的 `forward` 方法計算 L1/MSE Loss 的地方。
* **概念**：給予一組 Weight Mask (例如右手關節 0-5 乘上 5.0，左手關節 6-11 乘上 1.0)。強迫梯度下降去重視變異較小但同樣重要的動作維度。

**方案四：關閉 CVAE 退化為確定性 Transformer [推薦程度：⭐⭐⭐]**
* **原理**：ACT 的 CVAE 具有「無視條件直接輸出平均行為」的偷懶特性 (後驗坍塌)。
* **作法**：在 training config 中，將 CVAE 關閉 (設定 `kl_weight = 0`，甚至移除 VAE 架構)。
* **概念**：讓模型失去「潛在空間隨機猜測」的能力，變成純粹的 Deterministic Vision-Action Transformer，強制它 100% 依賴影像中的線索來做出動作。

**方案五：輔助視覺任務損失 (Auxiliary Task Loss) [推薦程度：⭐⭐ (難度極高)]**
* **原理**：既然我們懷疑 ResNet 根本沒有學到「哪邊有盒子」，那我們給它一個考試！
* **作法**：在模型架構抽完 Image Features 後，外掛一個簡單的線性分類器。這個分類器的任務是預測「現在是任務A還是任務B (0 or 1)」。
* **概念**：將這個分類器的 CrossEntropy Loss 加進總 Loss。這會強迫神經網路先從畫面中找對了盒子位置 (分類正確)，再來生出精準的動作軌跡。