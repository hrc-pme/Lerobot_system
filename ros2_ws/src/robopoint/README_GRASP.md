# Autonomous Grasp Pipeline: Logic & Architecture

## 1. 問題分析 (Troubleshooting)

您遇到的錯誤：
```text
cv_bridge.core.CvBridgeError: Unknown encoding unchanged
```
**原因**：ROS 的 `cv_bridge` 函式庫中，`imgmsg_to_cv2` 函式並不支援 `unchanged` 這個參數值。雖然 OpenCV 的 `imread` 支援這個 flag，但在 ROS bridge 中，若要保持原始影像編碼 (例如深度圖的 `16UC1`)，應該使用 **`"passthrough"`**。

---

## 2. 系統邏輯 (System Logic)

我們將整個抓取流程模組化為四個層次，這也是標準機器人視覺應用架構：

### **Phase 1: 感知層 (Perception - CameraHandler)**
*   **職責**：持續接收來自 RealSense 的 RGB 影像與深度影像 (Aligned Depth)。
*   **關鍵技術**：
    *   `cv_bridge`: 將 ROS Image Message 轉為 OpenCV Numpy Array。
    *   **深度對齊**：這非常重要，確保 RGB 影像上的 (u,v) 座標與深度圖上的 (u,v) 是完全對應相同物理空間的。

### **Phase 2: 決策層 (Cognition - LLM/VLM)**
*   **職責**：理解場景，找出目標物。
*   **邏輯**：
    *   Input: RGB 影像。
    *   Processing: LLM (如 GPT-4, Gemini) 分析語意 "抓取紅色蘋果"。
    *   Output: 目標物的 2D Pixel 座標 center $(u_{pixel}, v_{pixel})$。

### **Phase 3: 空間轉換層 (Spatial Calculation - SpatialCalculator)**
*   **職責**：將 2D Pixel 轉換為機械手臂可達的 3D World 座標。
*   **數學原理**：
    1.  **2D -> 3D Camera Frame (Pinhole Model)**:
        $$Z_c = Depth(u,v)$$
        $$X_c = (u - c_x) \cdot Z_c / f_x$$
        $$Y_c = (v - c_y) \cdot Z_c / f_y$$
    2.  **3D Camera -> 3D World (TF2 Transformation)**:
        利用 TF Tree 查詢 `world` 到 `camera_link` 的轉換矩陣 $T_{world\_camera}$。
        $$P_{world} = T_{world\_camera} \cdot P_{camera}$$

### **Phase 4: 執行層 (Action - Planner/Executor)**
*   **職責**：規劃路徑並驅動手臂。
*   **工具**：MoveIt 2。
*   **邏輯**：設定 Target Pose (位置 $P_{world}$ + 姿態 Orientation)，執行反向運動學 (IK) 解算與軌跡規劃。

---

## 3. 模組化架構 (Package Structure)

我已為您將程式碼重構為標準 ROS 2 Python Package 結構，位於 `src/robopoint/robopoint/` 下：

```text
src/robopoint/
├── robopoint/
│   ├── __init__.py
│   ├── grasp_node.py        # [Main] 主程式 Entry Point，負責協調各模組
│   ├── camera_handler.py    # [Module] 處理影像訂閱與轉換 (修正了 encoding 問題)
│   ├── spatial_calculator.py# [Module] 處理 2D->3D 數學運算與 TF 變換
│   └── planner.py           # [Module] 負責與 MoveIt 溝通 (MoveIt Client)
├── setup.py                 # [Config] 註冊節點 execute point
└── package.xml
```

### 如何使用
1.  **編譯**：
    ```bash
    colcon build --packages-select robopoint --symlink-install
    source install/setup.bash
    ```
2.  **執行**：
    ```bash
    ros2 run robopoint grasp_node
    ```
