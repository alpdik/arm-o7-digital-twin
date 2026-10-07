# ARM1.5 + LinkerHand O7 Dijital İkiz Çalışması

**İlerleme ve teknik kavramlar raporu — 11 Eylül 2026**

## Projenin amacı

ARM1.5 robot kolu ile LinkerHand O7 elinin Gazebo Harmonic, ROS 2 Jazzy,
MoveIt 2 ve RViz kullanılarak bir dijital ikizinin oluşturulması amaçlanmaktadır.
İlerleyen aşamada kol komutlarının jiroskop verisinden, el ve parmak komutlarının
ise sensörlü bir eldivenden alınması; hareketlerin dijital ikiz ile fiziksel
robot arasında çift yönlü ve eşzamanlı yürütülmesi hedeflenmektedir. Fiziksel
deneylerin ilk uygulama ortamı plastik manken olacaktır.

## Kullanılan ortam ve kaynaklar

- Ubuntu 24.04 LTS, WSL2
- ROS 2 Jazzy
- Gazebo Harmonic
- MoveIt 2 ve RViz
- ARM1.5 kol URDF ve mesh dosyaları
- LinkerHand O7 sağ el URDF ve mesh dosyaları
- NVIDIA RTX 5060 Laptop GPU; WSLg üzerinden D3D12 donanım hızlandırması

## Sistem nasıl çalışıyor?

Dijital ikiz tek bir program değildir. Birbirinin üzerine kurulan birkaç katman
birlikte çalışır:

```text
Jiroskop / IMU ──> kol komutu dönüştürücüsü ──> MoveIt Servo ──> arm_controller

Eldiven ─────────> parmak komutu dönüştürücüsü ───────────────> hand_controller
                                                                 │
                                                                 v
                                  Gazebo simülasyonu veya fiziksel robot
                                                                 │
                                                                 v
                                    joint_states geri bildirimi ──> RViz
```

Gazebo robotun fizik kurallarına göre nasıl davranacağını hesaplar. RViz,
robotun ROS 2 tarafından bilinen durumunu gösterir. MoveIt gidilecek hedefe
uygun bir yol planlar; MoveIt Servo ise jiroskop gibi sürekli değişen girdilerle
anlık kontrol sağlar. Controller'lar hesaplanan eklem komutlarını simülasyona
ve daha sonra gerçek motor sürücülerine uygular.

### URDF ve SRDF arasındaki fark

**URDF (Unified Robot Description Format / Birleşik Robot Tanımlama Biçimi),**
robotun fiziksel ve kinematik tarifidir. Linkleri, jointleri, eklem eksenlerini,
hareket sınırlarını, görsel ve collision meshlerini, kütleleri ve atalet
değerlerini içerir. Günlük hayattaki karşılığı bir insanın iskeleti ile teknik
resminin birleşimi gibidir: hangi kemiğin hangisine bağlı olduğunu ve eklemin
hangi yönde ne kadar dönebildiğini söyler.

**SRDF (Semantic Robot Description Format / Anlamsal Robot Tanımlama Biçimi),**
URDF ile oluşturulan bedenin MoveIt tarafından nasıl kullanılacağını tarif
eder. Planlama gruplarını, hazır pozları, end-effector bilgisini, passive
jointleri ve planlamada yok sayılacak güvenli collision çiftlerini içerir.
Günlük hayattaki karşılığı aracın teknik çizimi değil, kullanım kılavuzu ve
kayıtlı koltuk ayarlarıdır. `ready` veya `four_finger_fist` birer kayıtlı ayardır;
bu pozlara nasıl gidileceğini MoveIt mevcut konuma göre yeniden hesaplar.

Bu nedenle URDF'deki yanlış bir eklem ekseni robotun bedenini yanlış kurar;
SRDF'deki yanlış bir planning group ise doğru kurulmuş bedeni MoveIt'in yanlış
eklemlerle kullanmasına yol açar.

## Temel terimler ve anlaşılır karşılıkları

| Terim | Teknik anlamı | Günlük hayattan benzetme |
|---|---|---|
| Dijital ikiz | Gerçek sistemin durumunu ve davranışını sayısal ortamda temsil eden model | Uçuş simülatöründeki uçağın, gerçek uçağı sürekli temsil etmesi |
| ROS 2 | Robot yazılımlarının haberleşmesini ve birlikte çalışmasını sağlayan altyapı | Vücudun sinir sistemi ve şehirdeki posta ağı |
| Node (düğüm) | Tek bir görevi yapan ROS 2 programı | Fabrikadaki uzman bir işçi |
| Topic (konu) | Bir kaynaktan bir veya daha fazla dinleyiciye sürekli veri akışı | Sürekli yayın yapan radyo kanalı |
| Service (servis) | Bir isteğe tek cevap veren kısa işlem | Danışmaya soru sorup cevap almak |
| Action (eylem) | Zaman alan, durumu izlenebilen ve iptal edilebilen görev | Restoranda sipariş verip hazırlanmasını takip etmek |
| URDF | Robotun link, joint, geometri ve fiziksel özelliklerini tanımlayan dosya | İskelet ve teknik üretim çizimi |
| Xacro | Tekrarlı URDF parçalarını parametrelerle üreten şablon dili | Farklı ölçülerde ürün çıkaran kalıp |
| SRDF | MoveIt planning group, hazır poz ve anlamsal bilgilerini tanımlayan dosya | Kullanım kılavuzu ve kayıtlı koltuk pozisyonları |
| Link | Robotun rijit kabul edilen parçası | Kol kemiği veya parmak boğumu |
| Joint (eklem) | İki link arasındaki hareketli bağlantı | Dirsek veya kapı menteşesi |
| Mesh | Robot parçasının üç boyutlu yüzey modeli | Bir nesnenin dış kabuğu veya derisi |
| Collision geometry | Çarpışma hesabında kullanılan geometri | Bir nesnenin etrafına çizilen koruyucu hacim |
| TF / frame | Parçaların birbirine göre koordinat ilişkisi | Haritadaki konum ve yön tarifleri |
| Planning group | MoveIt'in birlikte planladığı eklem kümesi | Aynı hareket için çalışan kas grubu |
| Named state | Belirli eklem değerlerinin isimle kaydedilmiş hedef pozu | Otomobildeki hafızalı koltuk düğmesi |
| Trajectory (yörünge) | Eklem hedefleri ile bu hedeflere ulaşma zamanlarını içeren hareket yolu | Durakları ve süreleri belirlenmiş yolculuk planı |
| Controller | Hedef eklem değerlerini simülasyona veya motora uygulayan yazılım | Beynin emrini kasa taşıyan ve hareketi düzenleyen yapı |
| ros2_control | Controller'larla simülasyon veya donanım arasındaki standart katman | Farklı motorlara uyan standart şanzıman/ara bağlantı |
| MoveIt 2 | Kinematik, collision kontrolü ve hareket planlama sistemi | Engelleri dikkate alarak rota çizen navigasyon uygulaması |
| MoveIt Servo | Sürekli gelen hız, eklem veya uç-efektör komutlarını anlık harekete çeviren sistem | Hazır rota yerine direksiyonu canlı kullanmak |
| Gazebo | Fizik, yerçekimi ve temasları hesaplayan robot simülatörü | Robot için sanal deney laboratuvarı |
| RViz | ROS 2 verilerini ve robot durumunu görselleştiren arayüz | Aracın gösterge paneli; motorun kendisi değildir |
| Mimic joint | Başka bir eklemi belirli oranla otomatik takip eden eklem | Dişliyle ana mile bağlı ikinci mekanizma |
| Joint state | Eklem konumu, hızı veya kuvveti hakkındaki geri bildirim | Otomobil hız göstergesi ve sensör paneli |
| Namespace | Benzer isimleri farklı sistemler altında ayıran ön ek | Aynı isimli iki kişinin farklı departmanlarda kayıtlı olması |
| rosbag | ROS 2 mesajlarını zamanlarıyla kaydedip tekrar oynatabilen kayıt sistemi | Robotun kara kutusu |

## MoveIt, RViz ve hazır hareketlerin ilişkisi

SRDF'de `home`, `ready`, `open`, `pregrasp` veya `four_finger_fist` olarak
tanımladığımız öğeler, önceden kaydedilmiş tam hareketler değil hedef pozlardır.
RViz'de **Plan & Execute** seçildiğinde MoveIt güncel eklem durumunu okur,
collision ve limitleri değerlendirir ve o an için yeni bir trajectory üretir.

Doğrudan `/sim/.../follow_joint_trajectory` action'ına gönderdiğimiz test
komutları ise MoveIt planlayıcısını atlayarak controller'a gider. Bu komutlar
controller takibini ve Gazebo–RViz senkronizasyonunu sınamak için kullanılmıştır.

Jiroskop ve eldiven eklendiğinde hazır pozlar gereksiz olmayacaktır. Bunlar
kalibrasyon başlangıcı, teleoperasyona hazır olma, otomatik jest, hareketi
durdurma sonrası toparlanma ve görev sonu park pozları olarak kullanılacaktır.
Sürekli kullanıcı hareketi ise kayıtlı poz seçmek yerine sensörlerden üretilen
güncel komutlarla yürütülecektir.

Örnek hibrit görev akışı şöyledir:

```text
IDLE / bekleme
      ↓
CALIBRATION / jiroskop ve eldiven kalibrasyonu
      ↓
READY / MoveIt ile hazır poza geçiş
      ↓
TELEOP / jiroskop ve eldivenle sürekli kontrol
      ├── hazır jest çalıştır
      ├── belirli kol pozuna git
      ├── hareketi tut veya durdur
      └── tekrar kullanıcı kontrolüne dön
      ↓
OPEN + HOME / görevi bitir ve park et
```

## Tamamlanan çalışmalar

1. ARM1.5 kol modeli ile LinkerHand O7 sağ el modeli tek bir robot açıklamasında
   birleştirildi. Kol–el montaj dönüşümü parametreli hâle getirildi.
2. Beş ROS 2 paketinden oluşan çalışma alanı kuruldu. Son doğrulamada 28 testin
   tamamı hatasız geçti.
3. Gazebo için dünya, robot spawn işlemi, `gz_ros2_control` ve ROS–Gazebo saat
   köprüsü hazırlandı.
4. Kol, el ve joint-state controller'ları etkinleştirildi. Kol ve el için
   `FollowJointTrajectory` action sunucuları doğrulandı.
5. Robotun Gazebo ve RViz modelleri görünür hâle getirildi; joint-state akışı
   sayesinde iki arayüzdeki hareketler eşzamanlandı.
6. MoveIt 2 planlama grupları `arm`, `hand` ve `arm_with_hand` olarak tanımlandı.
   Kol için `home` ve `ready`; el için `open`, `pregrasp`,
   `four_finger_fist` ve `contact_closed` durumları oluşturuldu.
7. Kolun `home`–`ready` hareketleri ile doğrudan trajectory action komutları
   Gazebo ve RViz üzerinde başarıyla test edildi. RViz testleri için %60 hız
   ölçeklendirmesi uygun bulundu.
8. Elin tam kapanma hedefinde oluşan self-collision incelendi. Durum geçerlilik
   taramasında %64 kapanma geçerli, %65 kapanma geçersiz bulundu; güvenli
   yaklaşma durumu olarak %62 seviyesindeki `pregrasp` seçildi.
9. Sıfır eklem sınırındaki çok küçük sayısal sapmaların MoveIt başlangıç durumu
   hatasına yol açmaması için `open` durumu 0.001 rad tamponla tanımlandı.
10. Parmakların PIP/DIP uç eklemlerinin mimic oranları ölçüldü. Başparmak dâhil
    on takipçi eklemin tamamı beklenen URDF oranlarını sıfır ölçüm hatasıyla
    sağladı.
11. Başparmak açıkken diğer dört parmağın tamamen kapandığı
    `four_finger_fist` hareketi geçerlilik, controller ve mimic testlerinden
    başarıyla geçti. Bu şekil uygun kol/bilek yönelimiyle thumbs-up hareketinin
    el kısmını oluşturmaktadır.
12. Daha önce geçici bir path-tolerance hatası veren `thumb_cmc_roll` eklemi
    yeniden incelendi. 0.001–0.279 rad aralığındaki bütün hedefler sıfır izleme
    hatasıyla başarıyla tamamlandı; bu aralıkta kalıcı bir controller veya model
    arızası görülmedi.
13. Kol ve elin aynı anda hareket ettiği birleşik testte hem gidiş hem dönüş
    başarıyla tamamlandı: `COMBINED_SIM_TEST=PASS`.
14. Simülasyon ve ileride eklenecek gerçek robot yolları `/sim` ve `/real`
    namespace'leriyle ayrıldı. Mevcut test betikleri yalnızca `/sim` action'larına
    komut vermekte ve fiziksel robota veri göndermemektedir.
15. Kontrol mimarisi, jiroskop verisinin kolu; eldiven verisinin el ve parmakları
    yöneteceği şekilde netleştirildi. Önerilen girişler `/gyro/imu` ve
    `/glove/finger_curls` olarak belgelendi.

## Güncel durum

Dijital ikizin temel robot modeli, ROS 2 controller katmanı, MoveIt planlama
altyapısı ve Gazebo–RViz hareket senkronizasyonu çalışmaktadır. Kol ve dört
parmaklı yumruk hareketleri ayrı ayrı ve eşzamanlı olarak doğrulanmıştır.
Şu ana kadar fiziksel robotla hareket testi yapılmamıştır.

## Sonraki çalışmalar

### 1. Hareket ve jest kütüphanesini tamamlamak

Başparmak opozisyonunu içeren `pregrasp` yeniden sınanacak; thumbs-up, işaret
etme, pinch ve farklı kavrama biçimleri eklenecektir. Her hedef için MoveIt
geçerlilik, self-collision, controller takip ve Gazebo–RViz eşleşme testleri
yapılacaktır. Böylece eldiven henüz bağlı değilken modelin ulaşılabilir hareket
alanı doğrulanmış olacaktır.

### 2. Sürekli kol kontrolünü kurmak

Jiroskop/IMU mesajlarını alan bir `gyro_mapper` ROS 2 node'u geliştirilecektir.
Gürültü ve ani sıçramalar filtrelenecek; ölçümler MoveIt Servo'nun kabul ettiği
uç-efektör yönelimi, hız veya joint-jog komutlarına dönüştürülecektir. MoveIt
Servo, hazır rotayı oynatmak yerine aracı direksiyonla canlı sürmek gibi görev
yapacaktır.

Jiroskop tek başına güvenilir mutlak üç boyutlu konum vermez. Tam 6-DOF el pozu
istenirse yönelime ek olarak düğme, konum takipçisi, kamera veya başka bir
ölçüm kaynağı gerekebilir. Bu nedenle ilk deneyde gyro eksenlerinin hangi kol
hareketlerine karşılık geleceği açıkça tanımlanacaktır.

### 3. Eldiven–O7 eşlemesini kurmak

Bir `glove_mapper` node'u eldivenin parmak sensörlerini okuyacaktır. Her sensör
için açık ve kapalı kalibrasyon değerleri belirlenecek, değerler 0–1 aralığında
normalize edilecek ve O7'nin yedi aktif eklem hedefine dönüştürülecektir. PIP/DIP
uç eklemleri doğruladığımız mimic oranlarıyla otomatik olarak takip edecektir.
Filtreleme ve kısa süreli trajectory üretimi sayesinde hareketlerin titremeden
ve kopmadan akması sağlanacaktır.

### 4. Hibrit görev yöneticisini geliştirmek

Hazır MoveIt pozları ile canlı teleoperasyon arasında geçiş yapan bir ROS 2
state-machine/action node'u oluşturulacaktır. İlk sürümde `IDLE`, `CALIBRATION`,
`READY`, `TELEOP`, `HOLD`, `RECOVERY` ve `HOME` durumları bulunacaktır. Daha
karmaşık manken uygulamalarında yaklaşma, kullanıcı kontrolü, temas, bekleme ve
geri çekilme adımları MoveIt Task Constructor veya benzer bir görev planlayıcı
ile birleştirilebilecektir.

### 5. Ölçüm ve deney kaydı altyapısını eklemek

Sensör girdisi, üretilen hedef, Gazebo/robot geri bildirimi ve sistem durumu
ortak zaman damgalarıyla rosbag'e kaydedilecektir. Komut frekansı, uçtan uca
gecikme, jitter, paket kaybı ve eklem takip hatası otomatik olarak raporlanacaktır.
Bu kayıtlar hem networking ekibiyle karşılaştırma yapmak hem de ileride makale
hazırlamak için kullanılacaktır.

### 6. `/sim` ve `/real` entegrasyonunu tamamlamak

Önce bütün komutlar `/sim` yolunda Gazebo ile doğrulanacaktır. Networking ve
fiziksel robot ekipleri hazır olduğunda aynı yüksek seviyeli komut sözleşmesi
`/real` backend'e bağlanacaktır. Dijital ikizin gerçek bir ayna olması için
yalnızca gönderilen komutlar değil, gerçek robottan dönen eklem durumları da
Gazebo/RViz tarafına aktarılacaktır.

## Saklanan başlıca test kayıtları

- `build_four_finger_fist.log`
- `combined_motion_fist.log`
- `four_finger_mimic_check.log`
- `thumb_roll_range.log`
- `terminal1_gazebo.log`
- `terminal2_robot.log`
