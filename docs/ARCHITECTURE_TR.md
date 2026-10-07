# Dijital ikiz mimarisi

## Namespace ve veri sahipliği

| Arayüz | Simülasyon | Fiziksel |
|---|---|---|
| Controller manager | `/sim/controller_manager` | `/real/controller_manager` |
| Kol controller | `/sim/arm_controller` | `/real/arm_controller` |
| El controller | `/sim/hand_controller` | `/real/hand_controller` |
| Durum | `/sim/joint_states` | `/real/joint_states` |

Fiziksel tarafta durum sahipliği şöyledir:

```text
kol sürücüsü -> /real/arm_joint_states  \
                                         joint_state_mux -> /real/joint_states
O7 adaptörü  -> /real/hand_joint_states /
```

Birleşik `/real/joint_states` topic'inin tek yayıncısı mux olmalıdır. Hakemin
SHADOW ve takip-hatası mantığı, altı kol + yedi bağımsız el eklemini içeren bu
eksiksiz ve taze mesajı bekler.

Canonical komut girişi `/twin/joint_trajectory`dır. Bu topic'e yalnız bir komut
kaynağı sahip olmalıdır: eldiven adaptörü, operatör UI veya onaylı plan yürütücüsü.
MoveIt'in doğrudan `/sim` action yürütmesi ile hakem topic yürütmesini aynı anda
kullanmayın.

## Modlar

| Mod | Sim komutu | Real komutu | Amaç |
|---|---:|---:|---|
| `SAFE_IDLE` | hayır | hayır | Açılış, hata ve güvenli bekleme |
| `SIM_ONLY` | evet | hayır | Model/controller devreye alma |
| `SHADOW` | gerçek durumdan | asla | Fiziksel hareketi Gazebo'da gösterme |
| `TWIN_COMMAND` | evet | evet | Aynı güvenli referansı iki tarafa bölme |
| `REAL_ONLY` | hayır | evet | Kontrollü fiziksel devreye alma |

Aktif bir moddan diğerine doğrudan geçiş reddedilir. Önce `SAFE_IDLE` gerekir.
Deadman bırakılması, heartbeat/state timeout'u veya limit/takip hatası yeni komutları
keser ve arızayı kilitler.

## QoS ve zaman

- Command, heartbeat, deadman ve mode: reliable.
- Joint state: sensor-data profili; düşük gecikme, sınırlı queue.
- Mode/status: transient-local, son durum yeni aboneye ulaşır.
- Sim düğümleri `use_sim_time=true`; fiziksel sürücüler `false`.
- `/clock` yalnız Gazebo'dan ROS'a köprülenir.
- Farklı bilgisayarlar kullanılıyorsa duvar saatlerini NTP/chrony ile yakın tutun;
  fiziksel yürütmeye Gazebo timestamp'i kopyalamayın.

## Ağ

Tek bilgisayardaki iki Ubuntu terminalinde ek DDS ayarı gerekmez. Ayrı bilgisayarda:

1. Aynı güvenilir kablolu LAN ve aynı `ROS_DOMAIN_ID` kullanın.
2. Sim ve real topic'lerini mutlaka namespace'lerde tutun.
3. Multicast engelliyse seçilen RMW'nin statik peer/unicast ayarını yapın.
4. Komut ağı ile kurumsal/genel ağı ayırın; firewall yalnız gereken DDS trafiğini açsın.
5. Ağ kesilmesinde sürücünün kendi watchdog'u kontrollü duruş yapmalı; ROS düğümü
   fiziksel E-stop yerine geçmez.

## Neden joint state'i doğrudan command'e bağlamıyoruz?

İki yönlü topic kopyası gecikmeli pozitif geri besleme, oscillation ve iki controller'ın
aynı interface'i sahiplenmesi riskini doğurur. `TWIN_COMMAND` aynı referansı iki tarafa
gönderir; `sim_state` ve `real_state` yalnız gözlenip hata hesabında kullanılır.
`SHADOW` ise tek yönlü ve açıkça real→sim'dir.
