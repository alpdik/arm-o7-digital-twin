# Fiziksel donanım entegrasyonu

## ARM1.5 kol

Verilen arşiv yalnız CAD URDF/STL içeriyordu. Üretici sürücüsü, encoder protokolü,
motor kontrol modu, transmission ve güvenlik limitleri yoktu. Bu yüzden fiziksel kol
için `ros2_control` plugin'i uydurulmadı.

Gerçek backend şu sözleşmeyi sağlamalı:

- `/real/arm_controller/follow_joint_trajectory`
- `/real/arm_controller/joint_trajectory`
- `/real/arm_joint_states` (yalnız altı kol eklemi)
- Joint adları `arm_joint_1` … `arm_joint_6`

Kaynak eşlemesi:

| CAD joint | Canonical joint | Eksen | Pozisyon aralığı (rad) |
|---|---|---|---|
| A | `arm_joint_1` | Z | -3.14 … 3.14 |
| B | `arm_joint_2` | X | -1.57 … 1.57 |
| C | `arm_joint_3` | X | -1.30 … 1.30 |
| D | `arm_joint_4` | Z | -3.14 … 3.14 |
| E | `arm_joint_5` | X | -1.30 … 1.30 |
| F | `arm_joint_6` | Z | -3.14 … 3.14 |

Encoder yönü/sıfırı fiziksel ölçümle doğrulanmadan bu eşleme komut için kullanılmamalı.

## LinkerHand O7

Resmi `linker-bot/linkerhand-ros2-sdk` O7'yi ve radian (`*_arc`) command/state
topic'lerini listeliyor. Ancak upstream README'nin ilan ettiği ortam Ubuntu 22.04 +
ROS 2 Humble'dır; Jazzy desteği upstream tarafından doğrulanmış değildir.

`arm_o7_linkerhand_adapter` şunları yapar:

- Canonical 7 joint sırasını SDK sırasına çevirir.
- Gerekirse sign/scale/offset uygular.
- Çıkışı resmi SDK'nın belirttiği en fazla 30 Hz sınırında örnekler.
- SDK state'ini `/real/hand_joint_states` biçimine geri çevirir.
- Kol ve el durumları tazeyse bunları birleştirip tek sahibi olduğu
  `/real/joint_states` topic'ini yayımlar.
- `mapping_verified=false`, enable/deadman kapalı ve timeout'lu başlar.

Başlatma:

```bash
source ~/arm_o7_digital_twin/install/setup.bash
ros2 launch arm_o7_linkerhand_adapter hardware_adapter.launch.py
```

Kol sürücünüz `/joint_states` yayımlıyorsa launch/remap ayarında bunu
`/real/arm_joint_states` adına taşıyın. Kol sürücüsü, el adaptörü ve mux'un üçünün
de `/real/joint_states` yayımlamasına izin vermeyin.

Teslimdeki sağ-el eşlemesi **geçicidir**: O7 URDF limitleri, resmî SDK'daki L7-R
arc sırası/aralıklarıyla normalize edilmiştir. Upstream dokümanı sağ-el URDF'sinin
değişeceğini ayrıca belirtiyor. Gerçek O7 model/firmware bilgisi ve tek-eklem
ölçümleriyle `config/o7_right.yaml` içindeki index, scale ve offset değerlerini
doğrulamadan `mapping_verified` veya `command_enabled` açılmamalıdır.

CAN açma, sistem parolası veya `sudo` işlemleri adaptörün görevi değildir. Bunlar
işletim sistemi/udev ve üreticinin kurulum prosedürüyle ayrı yönetilmelidir.

## Düşük hızda devreye alma sırası

1. Robot enerjisizken mekanik limitleri, yönleri ve el adaptörünü ölçün.
2. Fiziksel E-stop'u ve sürücü watchdog'unu bağımsız test edin.
3. Yalnız state okuyun; canonical joint isim/yön/sıfırlarını kayıtla karşılaştırın.
4. `SHADOW` modunda fiziksel hareketi Gazebo'da izleyin; real output kapalı kalsın.
5. Her joint'i tek tek, küçük adım ve düşük hızla test edin.
6. Gerçek hız/ivme/efor limitlerini URDF, MoveIt, controller ve arbiter katmanlarına
   aynı kaynak dosyadan taşıyın.
7. Başlangıç ve takip hata toleranslarını ölçülmüş gecikme/gürültüye göre daraltın.
8. Ancak risk değerlendirmesinden sonra hakemde `enable_real_output`, O7
   adaptöründe `mapping_verified` ve `command_enabled` değerlerini açın.

Gerçek controller bir trajectory'yi kabul ettikten sonra topic tabanlı hakemin yeni
komutları kesmesi o trajectory'yi zorunlu olarak iptal etmez. Üretim kullanımı için
downstream `FollowJointTrajectory` action proxy/cancel akışı ve sürücü seviyesinde
quick-stop uygulanmalıdır.
