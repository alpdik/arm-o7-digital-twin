# Projeyi ekip için GitHub'a yükleme

## Neden private repository?

LinkerHand O7 kaynakları Apache-2.0 lisanslıdır. ARM1.5 URDF ve STL dosyaları ise
kullanıcı arşivinden gelmiş, arşivde açık bir lisans bulunmamıştır. Bu nedenle
kurumunuz model dosyalarını paylaşma hakkını doğrulayana kadar repository'yi
**private** oluşturun. Private repository de lisans izninin yerine geçmez; yalnız
istenmeyen genel erişimi önler.

## 1. GitHub'da boş repository oluşturun

GitHub'da `New repository` seçin:

- Repository name: `arm-o7-digital-twin`
- Visibility: `Private`
- README ekleme: kapalı
- `.gitignore` ekleme: kapalı
- License ekleme: kapalı

Projenin bu dosyaları zaten vardır. GitHub tarafında yeniden oluşturmak ilk push
sırasında gereksiz çakışma çıkarabilir.

## 2. Yerel projeyi Git repository yapın

Ubuntu terminalinde:

```bash
cd ~/arm_o7_digital_twin

git init -b main
git config user.name "AD SOYAD"
git config user.email "GITHUB_EPOSTA"

git status --short
git add .
git status --short
git diff --cached --stat
```

İkinci `git status` çıktısında şunlar bulunmamalıdır:

- `build/`, `install/`, `log/`
- `*.log`, video veya rosbag dosyaları
- `output/` altındaki ZIP/PDF çıktıları
- parola, token, Wi-Fi şifresi veya özel anahtar

Liste doğruysa ilk kayıt noktasını oluşturun:

```bash
git commit -m "Initial ARM1.5 and LinkerHand O7 digital twin"
```

## 3. GitHub'a bağlayıp gönderin

GitHub'ın boş repository sayfasında gösterilen URL'yi kullanın:

```bash
git remote add origin https://github.com/GITHUB_KULLANICI_ADI/arm-o7-digital-twin.git
git remote -v
git push -u origin main
```

GitHub parola kabul etmek yerine tarayıcıyla giriş, GitHub CLI veya kişisel erişim
tokenı isteyebilir. Tokenı hiçbir dosyaya yazmayın ve konuşmalara yapıştırmayın.

## 4. Arkadaşı ekleyin

Repository sayfasında `Settings` → `Collaborators` bölümünden arkadaşın GitHub
kullanıcı adını davet edin. Arkadaş kabul ettikten sonra:

```bash
git clone https://github.com/GITHUB_KULLANICI_ADI/arm-o7-digital-twin.git
cd arm-o7-digital-twin
```

Simülasyon bilgisayarı tam kurulumu yapar. Yalnız gönderici Raspberry Pi ise önce
`docs/NETWORK_TWO_HOSTS_TR.md` belgesini izler.

## Günlük çalışma düzeni

Yeni bir değişiklikten önce:

```bash
git pull --ff-only
```

Değişiklikten sonra:

```bash
git status --short
git add DOSYA_VEYA_KLASOR
git commit -m "Kısa ve açıklayıcı değişiklik özeti"
git push
```

`commit`, projenin tarihçesine çekilmiş fotoğraf gibidir. `push`, bu fotoğrafı
GitHub'a yollar. `pull`, ekip arkadaşlarının yeni fotoğraflarını bilgisayara alır.
