# Jiroskopla kol ve eldivenle O7 kontrol arayüzü

## Girdi ayrımı

- Ayrı jiroskop / IMU birimi kol hareket komutunun kaynağıdır.
- Eldiven, O7 el ve parmak hareket komutlarının kaynağıdır.
- Bu iki akış zaman damgalı olarak ayrı işlenir ve yalnız güvenlik hakeminden
  sonra aynı simülasyon ya da fiziksel backend'e gönderilir.

## Ölçülebilen ve ölçülemeyen

Bir IMU/jiroskop doğrudan açısal hız ve genellikle bir filtreyle orientasyon verir.
Tek başına:

- Elin mutlak XYZ konumunu uzun süre kararlı ölçemez; ivme integrasyonu hızla sürüklenir.
- Dört parmağın ve başparmağın ayrı bükülme açılarını ölçemez.

Bu nedenle kol tarafında önerilen sensör seti:

- Bilek orientasyonu: kalibre edilmiş IMU quaternion'u.
- Bilek XYZ: optik takip, UWB, kamera/AprilTag veya başka dış referans.
- Güvenlik: fiziksel, basılı tutulması gereken deadman düğmesi.

O7 tarafında eldiven, flex/Hall/encoder sensörleri ya da eldiven üreticisinin
eklem tahminiyle parmak bükülmelerini sağlamalıdır.

## Önerilen ROS sözleşmesi

```text
/gyro/imu                  sensor_msgs/Imu
/glove/finger_curls        sensor_msgs/JointState
/twin/deadman              std_msgs/Bool
/twin/heartbeat            std_msgs/Empty
```

Kol için jiroskoptan gelen filtrelenmiş IMU verisi `geometry_msgs/PoseStamped` veya
`geometry_msgs/TwistStamped`a dönüştürülüp MoveIt Servo'ya verilebilir. Parmak
sensörleri kalibrasyon eğrilerinden sonra canonical O7 joint açılarına çevrilip
`/twin/joint_trajectory` komutuna eklenir.

## Kalibrasyon sırası

1. Eldiven sabitken gyro bias ve quaternion normunu ölçün.
2. Kullanıcı nötr pozdayken `glove_frame -> robot_base` dönüşümünü kaydedin.
3. Hareket ölçeğini önce simülasyonda %10–20 ile sınırlayın.
4. Workspace clamp, tekillik yavaşlatma ve collision check'i etkinleştirin.
5. Deadman bırakıldığında 300 ms'den kısa sürede yeni komut akışının kesildiğini test edin.
6. Paket kaybı, sensör donması, NaN, quaternion norm hatası ve zaman aşımını test edin.

Eldiven mesaj formatı ve sensör modeli bilinmeden rastgele bir IMU→joint eşlemesi
eklemek güvenli değildir. İkinci aşamada gerçek mesaj örneği (`ros2 topic echo --once`)
ve kalibrasyon kaydıyla bu adapter tamamlanmalıdır.
