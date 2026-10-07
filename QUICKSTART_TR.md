# Hızlı başlangıç — Ubuntu 24.04 / ROS 2 Jazzy

## 1. Çalışma alanını Linux tarafına alın

WSL kullanıyorsanız klasörü `~/arm_o7_digital_twin` gibi Linux dosya sistemindeki
bir konuma kopyalayın. Gazebo mesh yükleme ve `colcon` işlemleri `/mnt/c` altında
daha yavaş olabilir.

```bash
cd ~/arm_o7_digital_twin
bash scripts/install_dependencies.sh
bash scripts/build.sh
```

`build.sh`, `rosdep`, `colcon build`, proje doğrulayıcısı ve paket testlerini çalıştırır.

## 2. Terminal 1 — Gazebo

```bash
cd ~/arm_o7_digital_twin
bash scripts/terminal1_gazebo.sh
```

Gazebo açıldıktan ve dünya çalışmaya başladıktan sonra ikinci terminale geçin.

## 3. Terminal 2 — robot + MoveIt

```bash
cd ~/arm_o7_digital_twin
bash scripts/terminal2_robot.sh
```

Beklenen controller'lar:

```bash
ros2 control list_controllers -c /sim/controller_manager
```

Beklenen sonuçta dördü de `active` olmalı:

```text
joint_state_broadcaster
arm_controller
hand_controller
thumb_coupling_controller
```

Ek kontroller:

```bash
ros2 control list_hardware_interfaces -c /sim/controller_manager
ros2 topic hz /sim/joint_states
ros2 action list | grep follow_joint_trajectory
ros2 topic echo --once /twin/status
```

## 4. Sadece simülasyonda hareket testi

Üçüncü bir shell açabilir veya Terminal 2'de launch'ı `tmux` içinde çalıştırabilirsiniz:

```bash
cd ~/arm_o7_digital_twin
bash scripts/demo_sim_motion.sh
```

Alternatif olarak RViz MotionPlanning panelinde `arm` grubunu seçip planlayın ve
çalıştırın. `hand` grubu için collision-free `open`, `pregrasp`,
`four_finger_fist` ve `middle_finger` named state'leri kullanılabilir.
`four_finger_fist`, başparmak açıkken diğer dört parmağın PIP/DIP mimic
eklemleriyle tamamen kıvrıldığı el şeklidir; uygun kol/bilek yönelimiyle
thumbs-up işareti olur. `middle_finger` durumunda orta parmak açık, diğer üç uzun
parmak kapalı ve başparmak güvenli açık konumdadır. `contact_closed` parmak ucu
temasını temsil eder ve doğrudan OMPL hedefi olarak kullanılmamalıdır.

Hazır el jestlerini Terminal 3'ten sırayla göstermek için:

```bash
cd ~/arm_o7_digital_twin
bash scripts/demo_hand_gestures.sh
```

Beklenen son satır `HAND_GESTURE_DEMO=PASS` olur. Hareketler hem Gazebo hem
RViz'de görünür ve betik yalnız `/sim` action'ını kullanır.

Tüm kol/el named state'lerini MoveIt üzerinden sınayan yeterlilik testi:

```bash
bash scripts/run_qualification.sh 1 2>&1 | tee qualification.log
```

Beklenen son satır `DIGITAL_TWIN_QUALIFICATION=PASS` olur.

## 5. Montaj dönüşümünü ayarlama

O7 gerçek adaptöründe ofset veya yaw varsa Terminal 2 komutuna metre/radyan olarak
ekleyin:

```bash
bash scripts/terminal2_robot.sh \
  hand_mount_xyz:="0 0 0.012" \
  hand_mount_rpy:="0 0 1.57079632679"
```

Bu iki değer hem Gazebo hem MoveIt modeline aynı anda gider. Ölçülmeden tahminî
değerle fiziksel hareket yaptırmayın.

## 6. Sorun giderme

`/sim/controller_manager` bulunamıyorsa:

```bash
gz sim --versions
ros2 pkg prefix gz_ros2_control
ros2 topic echo --once /sim/robot_description
```

Mesh görünmüyorsa:

```bash
ros2 pkg prefix arm_o7_description
python3 tests/validate_project.py
```

Gazebo açılıyor fakat robot hareket etmiyorsa controller logunda
`gz_ros2_control/GazeboSimSystem` ve dört controller'ın `active` olduğunu kontrol edin.

WSL'de GUI sorunu varsa WSLg/OpenGL kurulumunu doğrulayın; önce headless dünya ile
controller ve topic testlerini tamamlamak daha kolaydır.
