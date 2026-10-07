# ARM1.5 + LinkerHand O7 dijital ikiz başlangıç projesi

Bu çalışma alanı, kullanıcı tarafından verilen 6 eksenli ARM1.5 modeli ile
`linker-bot/linkerhand-urdf` deposundaki **sağ O7** modelini tek bir ROS 2 robotu
olarak birleştirir. Hedef ortam Ubuntu 24.04, ROS 2 Jazzy, Gazebo Harmonic,
`gz_ros2_control` ve MoveIt 2'dir.

Bu teslim çalışır bir simülasyon/planlama temeli ve fail-closed dijital-ikiz komut
hakemi içerir. Fiziksel kol sürücüsü arşivde bulunmadığı için gerçek robota komut
çıkışı varsayılan olarak kapalıdır. Sağ/sol O7 seçimi de kesinleştirilmelidir;
fotoğrafa dayanarak sağ O7 varsayıldı.

## Hazır olanlar

- 6 ARM + 7 bağımsız O7 aktüatörü; O7'nin 10 mekanik bağlı eklemi `mimic`
- Düzeltilmiş mesh URI'leri, benzersiz link/joint adları ve zemine sabit robot kökü
- Gazebo Harmonic için `gz_ros2_control` ve iki trajectory controller
- MoveIt grupları: `arm`, `hand`, `arm_with_hand`
- Ham görsel meshlerden türetilmiş, çok daha hafif dışbükey collision meshleri
- Güvenli modlar: `SAFE_IDLE`, `SIM_ONLY`, `SHADOW`, `TWIN_COMMAND`, `REAL_ONLY`
- Heartbeat, deadman, stale-state, başlangıç farkı, takip hatası ve eklem limiti interlock'ları
- Gerçek O7 SDK'sı için ayrı, varsayılan kapalı adaptör paketi
- Ayrı fiziksel kol/el durumlarını tek sahipli `/real/joint_states` akışında birleştiren mux
- ROS kurulumu olmadan koşabilen statik doğrulama ve saf mantık testleri

## Paketler

| Paket | Görev |
|---|---|
| `arm_o7_description` | Birleşik Xacro/URDF, orijinal ve collision meshleri, ros2_control tanımı |
| `arm_o7_bringup` | Gazebo dünyası, spawn, controller sıralaması ve iki-terminal launch akışı |
| `arm_o7_moveit_config` | SRDF, KDL, OMPL, joint limits, controller ve RViz ayarları |
| `arm_o7_twin` | Sim/real komut hakemi, SHADOW aynalama ve güvenlik interlock'ları |
| `arm_o7_linkerhand_adapter` | O7'nin resmi ROS 2 SDK topic'leri ile canonical el arayüzü arasında adaptör |
| `arm_o7_glove` | 5DT veri eldiveniyle O7 elini simülasyonda sürme ve gerçek Ti5 eline aktarma |

## İlk kurulum

Projeyi WSL içindeki Linux home dizinine kopyalamak, `/mnt/c` üzerinde derlemekten
daha hızlı ve daha problemsizdir:

```bash
cd ~/arm_o7_digital_twin
bash scripts/install_dependencies.sh
bash scripts/build.sh
```

Hızlı kullanım için [QUICKSTART_TR.md](QUICKSTART_TR.md) dosyasını izleyin.
Projeyi başka bir ekip bilgisayarına kurmak için önce
[TEAM_START_HERE_TR.md](TEAM_START_HERE_TR.md) dosyasını izleyin. Paket donanım
markasına bağlı değildir; WSLg ekran kartını otomatik seçer ve gerekirse
`ARM_O7_GPU_ADAPTER` ile seçim yapılabilir.

## İki Ubuntu terminali

Terminal 1 — Gazebo dünya sunucusu, GUI ve tek yönlü `/clock` köprüsü:

```bash
cd ~/arm_o7_digital_twin
bash scripts/terminal1_gazebo.sh
```

Terminal 2 — robotu spawn et, controller'ları sırayla aç, MoveIt/RViz ve güvenli
hakemi başlat:

```bash
cd ~/arm_o7_digital_twin
bash scripts/terminal2_robot.sh
```

Simülasyon doğrulama hareketi:

```bash
cd ~/arm_o7_digital_twin
bash scripts/demo_sim_motion.sh
```

Bu demo yalnız `/sim/...` action'larına gider; hiçbir `/real/...` çıkışı üretmez.

`open`, `middle_finger` ve `four_finger_fist` el jestlerini sırayla göstermek için:

```bash
bash scripts/demo_hand_gestures.sh
```

Beklenen son satır `HAND_GESTURE_DEMO=PASS` olur.

İlk hareketten sonra yerel dijital ikiz yeterlilik testi:

```bash
cd ~/arm_o7_digital_twin
bash scripts/run_qualification.sh 1 2>&1 | tee qualification.log
```

Test; controller ve action sunucularını, MoveIt state-validity sonuçlarını,
`/joint_states` hızını, Gazebo gerçek-zaman oranını, MoveIt plan/yürütmesini ve
son eklem hatalarını birlikte ölçer. `home`, `ready`, `left_demo`, `open`,
`pregrasp`, `four_finger_fist` ve `middle_finger` hareketlerini kullanır. JSON ve
CSV kanıtları `results/digital_twin_qualification/` altında tarihli klasöre
yazılır. İlk tek döngü geçtikten sonra on tekrarlı regresyon çalıştırın:

```bash
bash scripts/run_qualification.sh 10 2>&1 | tee qualification_10_cycles.log
```

## Eldivenle kontrol (5DT eldiven + Ti5 el)

Terminal 1 ve Terminal 2 açıkken üçüncü terminalde:

```bash
cd ~/arm_o7_digital_twin
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch arm_o7_glove glove_teleop.launch.py
```

Açılan pencerede önce **Calibrate glove** ile eldiveni kalibre edin (açık el,
sonra yumruk); her açılışta zorunludur. Ardından **ON** eldivenle simülasyondaki
eli sürer. **Also move the REAL Ti5 hand** işaretliyse gerçek Ti5 el de
eldiveni izler. Cihaz izinleri için gereken udev kuralı, eşleme ve güvenlik
ayrıntıları [src/arm_o7_glove/README.md](src/arm_o7_glove/README.md) içindedir.

## Tasarım sınırı: simülasyon ile gerçek donanım

```text
Eldiven / operatör UI / onaylı plan
                 |
          /twin/joint_trajectory
                 |
       fail-closed komut hakemi
          /                 \
 /sim/*_controller    /real/*_controller
          \                 /
       sim ve real joint_states
                 |
        takip-hatası gözlemcisi
```

Gerçek ve sim joint state topic'leri birbirinin command topic'ine doğrudan
bağlanmaz. `SHADOW` modunda gerçek durum yalnız simülasyona yansır. `TWIN_COMMAND`
modunda aynı doğrulanmış referans iki backend'e ayrılır; sonuçlar ayrıca
karşılaştırılır. Böylece iki yönlü veri akışı bir pozitif geri-besleme döngüsüne
dönüşmez.

MoveIt/RViz bu ilk teslimde doğrudan `/sim` action controller'larını yürütür.
Fiziksel yürütme özellikle hakem topic'inden geçirilmiştir; MoveIt için izlenebilir
ve iptal edilebilir bir downstream action proxy, gerçek kol sürücüsü belli olduktan
sonra ikinci aşamada eklenmelidir.

## Bilinçli olarak etkinleştirilmeyenler

- `enable_real_output` varsayılanı `false`
- O7 adaptöründe `mapping_verified` varsayılanı `false`
- O7 adaptöründe `command_enabled` varsayılanı `false`
- Fiziksel kol için sahte bir sürücü yok
- Üretici tarafından verilmemiş hız/efor değerleri fiziksel limit kabul edilmiyor
- Ağ kaybında yalnız ROS düğümüne güvenilmiyor; sürücü watchdog'u ve fiziksel E-stop şart

## Bir sonraki aşama için gereken ölçü/bilgiler

1. Kolun marka/modeli, motor sürücüleri ve haberleşme protokolü (CAN, EtherCAT,
   Modbus/TCP, seri vb.).
2. Fiziksel eklem sırası, pozitif yönleri, encoder sıfırları, hız/ivme/efor sınırları.
3. O7'nin sağ mı sol mu olduğu, firmware/SDK sürümü ve CAN/RS485 seçimi.
4. `arm_flange -> hand_base_link` adaptör kalınlığı ve RPY dönüşümü. Başlangıç
   varsayımı `xyz="0 0 0"`, `rpy="0 0 0"`dır.
5. Eldivenin mesaj formatı, IMU modeli/frekansı, deadman düğmesi ve varsa parmak
   bükülme sensörleri.
6. Bağımsız fiziksel E-stop ve sürücü watchdog davranışı.

Jiroskop tek başına yalnız açısal hız/orientasyon sağlar; elin uzaydaki mutlak
konumunu veya tek tek parmak bükülmelerini güvenilir biçimde ölçmez. Ayrıntı için
[GLOVE_INTERFACE_TR.md](docs/GLOVE_INTERFACE_TR.md) dosyasına bakın.

## Doğrulama durumu

- Xacro hem `none` hem `sim` modunda açıldı.
- Oluşan URDF: 26 link, 25 joint, tek kök `world`, 13 komutlanabilir joint.
- URDF ağacı, mimic oranları, limitler, mesh URI/hashleri, ros2_control ve SRDF
  statik testten geçti.
- `arm_o7_twin` saf güvenlik testleri: 18/18 geçti.
- O7 adaptörünün eşleme/interpolasyon/mux saf mantık testleri: 10/10 geçti.
- Ubuntu 24.04 / ROS 2 Jazzy / WSL2 çalışma zamanı doğrulamasında 5 paket derlendi;
  28 test 0 hata ve 0 failure ile geçti.
- Gazebo ile RViz eşzamanlı joint-state hareketi, dört aktif controller ve birleşik
  kol-el demosunda `COMBINED_SIM_TEST=PASS` sonucu doğrulandı.

## Kaynak ve lisans

- LinkerHand O7: upstream commit
  `075cc7d42cc1e756bdcbece0fc069a0779fc5237`, Apache-2.0.
- ARM1.5 varlıkları kullanıcı arşivinden geldi; arşivde lisans yoktu.
- Ayrıntı: `src/arm_o7_description/SOURCE_ASSETS.md`, `LICENSES/` ve
  `ASSET_MANIFEST.sha256`.

Teknik akış ve topic sözleşmesi için [ARCHITECTURE_TR.md](docs/ARCHITECTURE_TR.md),
fiziksel devreye alma için [HARDWARE_INTEGRATION_TR.md](docs/HARDWARE_INTEGRATION_TR.md)
dosyalarını okuyun.

İki bilgisayar / Raspberry Pi arasında yalnız simülasyon komutu gönderme deneyi
için [NETWORK_TWO_HOSTS_TR.md](docs/NETWORK_TWO_HOSTS_TR.md), projeyi ekip içinde
GitHub'a yüklemek için [GITHUB_PUBLISH_TR.md](docs/GITHUB_PUBLISH_TR.md)
dosyalarını izleyin.
