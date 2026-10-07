# İki bilgisayar / Raspberry Pi ile ROS 2 ağ deneyi

## Amaç

İlk ağ deneyinde iki rol vardır:

```text
Gönderici cihaz                              Simülasyon cihazı
Raspberry Pi A                               Raspberry Pi B veya güçlü Ubuntu PC

network_sender_demo.sh                       Gazebo + ros2_control + MoveIt/RViz
        |                                                |
        +--- ROS 2 action / DDS ağı -------------------->+
                                                         |
                                             ARM1.5 + O7 hareketi
```

Gönderici cihaz eklem hedeflerini yollar. Simülasyon cihazındaki ROS 2
controller'ları hedefi kabul eder, Gazebo robotu hareket ettirir ve RViz
`/sim/joint_states` üzerinden aynı hareketi gösterir.

Bu ilk test yalnız `/sim` action'larına gider. Fiziksel robot çıkışı olan `/real`
kullanılmaz. Ayrıca bu doğrudan ROS 2/DDS testidir; DDS'nin varsayılan taşıması
çoğunlukla UDP/RTPS tabanlıdır. Araştırmanın özellikle TCP protokollerini
karşılaştırması gerekiyorsa TCP gateway/bridge sonraki aşamada ayrıca eklenmelidir.

## Donanım notu

ROS 2 Jazzy, Ubuntu 24.04'te hem 64-bit x86 hem 64-bit ARM'ı destekler. Gazebo
Harmonic'in resmî ana hedefi ise Ubuntu amd64'tür; ARM mimarileri best-effort
durumdadır. Bu nedenle Pi üzerinde Gazebo kurulması veya GUI performansı garanti
edilemez. İlk güvenilir düzen:

- Raspberry Pi A: yalnız komut gönderici
- x86_64 Ubuntu bilgisayar: Gazebo, controller, MoveIt ve RViz

Pi 5 gibi daha güçlü bir cihazda simülasyon denenebilir; önce
`scripts/preflight_report.sh` çıktısı alınmalıdır.

VirtualBox içindeki Ubuntu simülasyon cihazı olacaksa ağ adaptörü `NAT` yerine
`Bridged Adapter` seçilmelidir. Aksi hâlde fiziksel Raspberry Pi, sanal makinedeki
ROS 2 katılımcılarını keşfedemeyebilir.

## Ortak ağ ayarları

İki cihaz da aynı yerel ağa bağlanmalı ve aynı ROS 2 Jazzy dağıtımını kullanmalıdır.
Mümkünse ilk test Ethernet kablosuyla yapılmalıdır. Her iki cihazdaki her yeni
terminalde:

```bash
source /opt/ros/jazzy/setup.bash
cd ~/arm_o7_digital_twin
source scripts/network_env.sh
```

Betik şu ortak değerleri oluşturur:

- `ROS_DOMAIN_ID=42`: aynı deney odasındaki ROS 2 katılımcılarını gruplar.
- `ROS_LOCALHOST_ONLY=0`: iletişimi yalnız o bilgisayara hapsetmez.
- `ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET`: aynı alt ağdaki cihazları keşfeder.
- `RMW_IMPLEMENTATION=rmw_fastrtps_cpp`: iki tarafta aynı DDS uygulamasını seçer.

Multicast keşfi engelleniyorsa diğer cihazın IP'si elle verilebilir:

```bash
export ARM_O7_PEER_IP=192.168.1.52
source scripts/network_env.sh
```

IP adresini gerçek cihaz adresiyle değiştirin.

## 1. Keşif testi

Simülasyon cihazında:

```bash
ros2 multicast receive
```

Gönderici cihazda:

```bash
ros2 multicast send
```

Alıcı mesajı görürse multicast yolu çalışıyor demektir. Sonra simülasyon cihazında
Terminal 1 ve Terminal 2'yi, ortak ağ ortamını source ettikten sonra normal biçimde
başlatın.

## 2. Uzak action görünürlüğü

Gönderici cihazda:

```bash
ros2 action list -t | grep follow_joint_trajectory
```

Beklenen:

```text
/sim/arm_controller/follow_joint_trajectory
/sim/hand_controller/follow_joint_trajectory
```

Bu liste gelmiyorsa henüz hareket komutu göndermeyin. İki tarafta domain, RMW,
alt ağ, sanal makine ağ modu ve firewall ayarlarını karşılaştırın.

## 3. Ağ üzerinden doğrulanmış simülasyon demosu

Gönderici Raspberry Pi tam çalışma alanını derlemek zorunda değildir. ROS 2 Jazzy,
`control_msgs` ve repodaki iki network betiği yeterlidir:

```bash
cd ~/arm_o7_digital_twin
source /opt/ros/jazzy/setup.bash
source scripts/network_env.sh
bash scripts/network_sender_demo.sh 2>&1 | tee network_sender_demo.log
```

Beklenen sonuç:

```text
[outbound] ARM=SUCCEEDED HAND=SUCCEEDED
[return] ARM=SUCCEEDED HAND=SUCCEEDED
NETWORK_SIM_DEMO=PASS
```

Gazebo ve RViz'de kol ile elin eşzamanlı gidip geri döndüğü görülmelidir.
`elapsed_ms`, action gönderiminden iki hedefin sonuçlanmasına kadar gönderici
tarafında ölçülen yaklaşık toplam süredir; saf ağ gecikmesi değildir.

## Sonraki ağ aşaması

İlk test DDS erişimini kanıtladıktan sonra ağ ekibiyle şu katmanlar eklenebilir:

1. Komut ve joint-state mesajlarına sıra numarası ve kaynak zaman damgası
2. Tek yön, gidiş-dönüş ve jitter ölçümü
3. Paket kaybı, gecikme ve bant genişliği deneyleri
4. Eldiven/IMU verisini eklem hedefine dönüştüren sender node
5. `/twin/heartbeat`, `/twin/deadman` ve `/twin/joint_trajectory` üzerinden güvenli
   hakemli yol
6. Araştırma gerektiriyorsa DDS/UDP taban çizgisine ek olarak TCP gateway

Fiziksel `/real` çıkışı, sürücü protokolü ve limitler doğrulanmadan açılmamalıdır.
