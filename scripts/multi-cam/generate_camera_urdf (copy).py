#!/usr/bin/env python3
import yaml
import os
import sys
from scipy.spatial.transform import Rotation as R

def main():
    # 嘗試尋找現有的 calibration_params.yaml
    possible_paths = [
        'scripts/multi-cam/config/calibration_params.yaml',
        'config/calibration_params.yaml',
        os.path.join(os.path.dirname(__file__), 'config/calibration_params.yaml')
    ]
    
    config_path = None
    for p in possible_paths:
        if os.path.exists(p):
            config_path = p
            break
            
    if not config_path:
        print("錯誤: 找不到 config/calibration_params.yaml")
        sys.exit(1)

    with open(config_path, 'r') as f:
        data = yaml.safe_load(f)

    if not data:
        print("錯誤: 校正檔案內沒有資料")
        sys.exit(1)

    print(f"找到校正資料: {config_path}")
    print("正在生成 URDF 檔案...")

    # 開始構建 URDF 內容
    urdf_content = '<?xml version="1.0"?>\n'
    urdf_content += '<robot name="realsense_multi_cam_system">\n\n'

    links = set()
    joints = []

    # D415 相機的粗略尺寸 (單位: 公尺)
    d415_box = '<box size="0.02 0.09 0.02"/>'

    for cam, params in data.items():
        parent_frame = params.get('parent_frame', 'base_link')
        # 如果 yaml 裡有 child_link_frame 就用，沒有就預設用相機名稱
        child_frame = params.get('child_link_frame', f"{cam}_link")
        
        pos = params['pos']
        quat = params['quat'] # [x, y, z, w]

        # 將 Quaternion 轉換為 URDF 使用的 RPY (Roll, Pitch, Yaw)
        r = R.from_quat(quat)
        roll, pitch, yaw = r.as_euler('xyz')

        # 收集 link 用於隨後生成
        links.add(parent_frame)
        links.add(child_frame)

        # 建立 Joint (連接 Parent 及 Child Camera)
        joints.append(f"""  <joint name="{parent_frame}_to_{child_frame}" type="fixed">
    <parent link="{parent_frame}"/>
    <child link="{child_frame}"/>
    <!-- x y z | r p y -->
    <origin xyz="{pos[0]:.6f} {pos[1]:.6f} {pos[2]:.6f}" rpy="{roll:.6f} {pitch:.6f} {yaw:.6f}"/>
  </joint>""")

    # 生成 Links (為每個 link 加上 D415 的形狀)
    urdf_content += "  <!-- ================= LINKS ================= -->\n"
    for link in links:
        urdf_content += f"""  <link name="{link}">
    <visual>
      <geometry>
        {d415_box}
      </geometry>
      <material name="realsense_gray">
        <color rgba="0.5 0.5 0.5 1.0"/>
      </material>
    </visual>
  </link>\n\n"""

    urdf_content += "  <!-- ================= JOINTS ================= -->\n"
    for joint in joints:
        urdf_content += joint + "\n\n"

    urdf_content += "</robot>\n"

    # 寫入檔案
    output_filename = "scripts/multi-cam/realsense_system.urdf"
    with open(output_filename, 'w') as f:
        f.write(urdf_content)

    print(f"成功！URDF 已生成至: {os.path.abspath(output_filename)}")
    print("您可以直接將此 .urdf 檔案導入 Unity 內 (搭配 URDF-Importer)，")
    print("然後將您的 Unity 相機物件掛載到對應的子節點 (如 camera_left_link) 下方即可！")

if __name__ == "__main__":
    main()
