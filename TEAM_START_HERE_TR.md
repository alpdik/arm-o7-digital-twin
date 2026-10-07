# Ekip kurulumu: ARM1.5 + LinkerHand O7 başlangıç paketi

Bu paket, ekipte doğrulanan **sağ LinkerHand O7** ile ARM1.5 kolun birleşik dijital
ikiz kaynaklarını içerir. Hazır bir `build/` veya `install/` klasörü içermez;
çünkü bu klasörler başka bilgisayarda yeniden üretilmelidir. Fiziksel robota çıkış
varsayılan olarak kapalıdır ve ilk hedef yalnız simülasyonu çalıştırmaktır.

Hedef sistem Ubuntu 24.04 ve ROS 2 Jazzy'dir. `install_dependencies.sh`, ROS 2'nin
resmî apt deposunun ve `/opt/ros/jazzy` kurulumunun hazır olduğunu varsayar; sıfırdan
ROS kurulumu yapmaz. ZIP'i komut satırından açacaksanız sistemde `unzip` yoksa önce
`sudo apt install unzip` çalıştırın veya Ubuntu'nun arşiv yöneticisini kullanın.

## 1. ZIP dosyasını Linux tarafına çıkarın

WSL2 kullanıyorsanız ZIP dosyasının Windows `Downloads` klasöründeki yolunu kendi
Windows kullanıcı adınıza göre düzenleyin:

```bash
cd ~
unzip /mnt/c/Users/<WINDOWS_KULLANICI_ADI>/Downloads/ARM1_5_O7_TEAM_STARTER_2026-10-07_v3.zip
cd ~/arm_o7_digital_twin
chmod +x scripts/*.sh scripts/*.py
```

Doğal Ubuntu kullanıyorsanız ZIP'i doğrudan home dizininize çıkarın ve yine
`~/arm_o7_digital_twin` klasörüne girin. Projeyi WSL'de `/mnt/c` altında
derlemeyin; Linux home dizini daha hızlı ve daha güvenilirdir.

## 2. Önce bilgisayar raporunu alın

Bu adım hiçbir paket kurmaz ve sistem ayarını değiştirmez:

```bash
cd ~/arm_o7_digital_twin
bash scripts/preflight_report.sh
```

Oluşan `preflight_report.txt` dosyasını ekip arkadaşına veya kullandığınız yapay
zekâ asistanına
gönderin. Özellikle şunlara bakılacak:

- Ubuntu sürümü: hedef 24.04
- ROS 2: hedef Jazzy
- WSL2/WSLg veya doğal Ubuntu
- CPU çekirdek sayısı ve RAM
- OpenGL `renderer` ve `Accelerated: yes/no`
- NVIDIA, AMD, Intel veya yazılım render durumu

GPU markası önceden varsayılmaz. Betikler WSLg'de D3D12 sürücüsünü kullanır,
adaptörü Mesa'ya otomatik seçtirir. Yalnız rapor birden fazla GPU gösterir ve yanlış
olanı seçerse çalıştırmadan önce örneğin şu kullanılabilir:

```bash
export ARM_O7_GPU_ADAPTER=NVIDIA
```

Intel veya AMD için değer ilgili kartın adından ayırt edici bir parça olabilir.
`llvmpipe` görülürse GPU zorlamadan önce raporu incelemek gerekir.

## 3. Kurun ve derleyin

Ön kontrol uygun olduğunda:

```bash
cd ~/arm_o7_digital_twin
bash scripts/install_dependencies.sh
bash scripts/build.sh 2>&1 | tee build.log
```

Beklenen ana sonuçlar:

```text
Summary: 5 packages finished
Summary: 28 tests, 0 errors, 0 failures, 0 skipped
```

Sayılar veya sonuçlar farklıysa ilerlemeden `build.log` dosyasının son 150 satırını
paylaşın:

```bash
tail -n 150 build.log
```

## 4. İki terminalde simülasyonu açın

Terminal 1:

```bash
cd ~/arm_o7_digital_twin
bash scripts/terminal1_gazebo.sh 2>&1 | tee terminal1_gazebo.log
```

Gazebo dünya penceresi açıldıktan sonra Terminal 2:

```bash
cd ~/arm_o7_digital_twin
bash scripts/terminal2_robot.sh 2>&1 | tee terminal2_robot.log
```

Zayıf bir bilgisayarda ilk controller testi için MoveIt/RViz geçici olarak
kapatılabilir:

```bash
ARM_O7_START_MOVEIT=false bash scripts/terminal2_robot.sh \
  2>&1 | tee terminal2_robot_light.log
```

Bu hafif mod yalnız tanılama içindir. Tam hedefe ulaşmak için daha sonra normal
Terminal 2 komutuyla MoveIt ve RViz de açılmalıdır.

VirtualBox kullanılıyorsa sanal makinenin görüntü ayarlarında 3D hızlandırmayı
açın ve mümkün olan CPU/RAM miktarını ayırın. Donanım markasını tahmin etmeyin;
gerçek renderer bilgisini `preflight_report.txt` içindeki `OpenGL renderer`
satırından kontrol edin. `llvmpipe` yazması yazılım render edildiğini gösterir.

## 5. Üçüncü terminalde kontrol edin ve hareket ettirin

```bash
cd ~/arm_o7_digital_twin
source /opt/ros/jazzy/setup.bash
source install/setup.bash

ros2 control list_controllers -c /sim/controller_manager
ros2 action list -t | grep follow_joint_trajectory
bash scripts/demo_combined_motion.sh 2>&1 | tee combined_motion_fist.log
bash scripts/demo_hand_gestures.sh 2>&1 | tee hand_gestures.log
```

Beklenen controller'ların tamamı `active` olmalıdır:

```text
joint_state_broadcaster
arm_controller
hand_controller
thumb_coupling_controller
```

Beklenen hareket testi sonucu:

```text
[outbound] ARM=SUCCEEDED HAND=SUCCEEDED
[return] ARM=SUCCEEDED HAND=SUCCEEDED
COMBINED_SIM_TEST=PASS
HAND_GESTURE_DEMO=PASS
```

Hareketler Gazebo ve RViz'de birlikte görünmelidir. Demolar yalnız `/sim`
action'larını kullanır; `/real` tarafına yayın yapmaz. Jest demosu sırasıyla eli
açar, `middle_finger` hareketini gösterir, tekrar açar, dört parmak yumruğu yapar
ve eli açık konuma döndürür.

Ardından MoveIt planlama ve yürütmeyi de kapsayan yeterlilik testini çalıştırın:

```bash
bash scripts/run_qualification.sh 1 2>&1 | tee qualification.log
```

Beklenen son satır:

```text
DIGITAL_TWIN_QUALIFICATION=PASS
```

JSON ve CSV kanıtları `results/digital_twin_qualification/` altında oluşturulur.
İlk döngü geçtikten sonra istenirse aynı testi `10` döngüyle tekrarlayın.

## Ulaşılması gereken ortak kontrol noktası

Kurulum tamamlandığında ekip arkadaşının sistemi şu durumda olmalıdır:

1. ARM1.5 ve sağ O7 tek robot olarak Gazebo'da görünür.
2. Aynı robot RViz MotionPlanning görünümünde görünür.
3. Dört controller aktiftir.
4. Kol ve el action sunucuları listelenir.
5. Kol ve el eşzamanlı gidip geri döner; birleşik demo `PASS` verir.
6. RViz named state'lerinde kol için `home`/`ready`/`left_demo`, el için `open`,
   `pregrasp`, `four_finger_fist` ve `middle_finger` kullanılabilir.
7. `run_qualification.sh 1` sonunda `DIGITAL_TWIN_QUALIFICATION=PASS` görülür.

## Bir yapay zekâ asistanıyla devam edecek kişi için

ZIP içindeki `AI_HANDOFF_PROMPT_TR.txt` metnini yeni konuşmaya ilk mesaj
olarak yapıştırın; ZIP'i ve ardından `preflight_report.txt` dosyasını ekleyin.
Kullandığınız asistandan modeli yeniden üretmesini değil bu paketi doğrulayıp
çalıştırmasını isteyin. Böylece O7 entegrasyonunu baştan ve farklı biçimde kurma
riski azalır. Bu dosya belirli bir yapay zekâ ürününe bağlı değildir.

## Lisans notu

LinkerHand O7 varlıkları Apache-2.0 lisansıyla gelir. ARM1.5 mesh/URDF arşivinde
bir lisans bulunmadı. Bu ZIP ekip içi teknik aktarım içindir; ARM1.5 dosyalarını
kamuya açık bir depoda yayımlamadan veya takım dışına dağıtmadan önce hak sahibinden
izin/lisans durumunu doğrulayın. Ayrıntı `LICENSES/ARM_ASSETS_NOTICE.txt` içindedir.
