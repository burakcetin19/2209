# Claude Code — Otonom Uçak Taksi Simülasyonu: Ajan Görev Belgesi

> Bu belge doğrudan Claude Code'a beslenmek üzere yazılmıştır.
> Her görev bloğu bağımsız olarak doğrulanabilir ve sıralı şekilde uygulanmalıdır.
> Kod yazma yetkisi tamdır; ancak her görevin sonundaki **Doğrulama Kapısı** geçilmeden bir sonraki göreve geçilmez.

---

## GENEL KISITLAMALAR (Tüm Adımlar İçin Geçerli)

- PX4, ArduPilot veya herhangi bir uçuş kontrolcüsü kullanılmayacaktır.
- Araç havaya kalkmayacaktır; tüm hareket yerde, diferansiyel sürüş prensibiyle gerçekleşecektir.
- Araç modeli SDF formatında YAZILMAYACAKTIR. Format kararı Adım 1 içindeki Karar Kapısı K-1'de verilecektir.
- Tüm ROS2 node'ları Python ile yazılacaktır (ament_python).
- Simülasyon aracı olarak yalnızca Gazebo kullanılacaktır (Ignition/Harmonic veya Classic — K-1'de belirlenir).
- Her adımın sonunda terminalden doğrulanabilir bir çıktı üretilmelidir.

---

## ADIM 1 — Çalışma Alanı Kurulumu, Paket Yapısı ve Araç Modeli

### Bağlam
Bu adımın amacı; projenin iskeletini oluşturmak, ROS2 paketlerini tanımlamak ve simülatörde görünür, sensörlü bir araç modeli ortaya çıkarmaktır. Bu adım tamamlanmadan Adım 2 başlatılmamalıdır.

---

### GÖREV 1.1 — Sistem Ortamını Doğrula

**Ajan şunları kontrol etmelidir:**

- `ros2 --version` komutunu çalıştır ve ROS2 dağıtım adını (humble, iron, jazzy vb.) not et.
- `gazebo --version` veya `ign gazebo --version` komutlarından hangisinin çalıştığını sına.
- `python3 --version`, `colcon --version`, `xacro --version` komutlarının başarıyla döndüğünü doğrula.
- Eksik paket varsa `apt-get` ile kur; kurulum için sudo yetkisi gerekiyorsa kullanıcıya bildir ve devam etmeden önce onay bekle.

**Kurulması gereken paket grubu (ROS2 Humble için referans):**
`ros-humble-gazebo-ros-pkgs`, `ros-humble-xacro`, `ros-humble-robot-state-publisher`,
`ros-humble-joint-state-publisher-gui`, `ros-humble-rviz2`, `python3-colcon-common-extensions`

Farklı bir dağıtım tespit edilirse paket adlarındaki `humble` ön ekini tespit edilen dağıtım adıyla değiştir.

---

### KARAR KAPISI K-1 — Model Formatı ve Gazebo Sürümü

Bu karar kapısı GÖREV 1.1'in çıktısına göre çalışır. Ajan aşağıdaki tabloyu kullanarak kararını verir ve kararını terminale yazdırır:

| Tespit Edilen Ortam | Gazebo Sürümü | Kullanılacak Model Formatı |
|---|---|---|
| ROS2 Humble | Gazebo Classic 11 | URDF/XACRO + Gazebo Classic plugin'leri XACRO içinde |
| ROS2 Humble | Ignition Fortress veya üstü | URDF/XACRO + Gazebo Ignition plugin'leri XACRO içinde |
| ROS2 Iron / Jazzy | Ignition Harmonic | URDF/XACRO + Gazebo Harmonic plugin'leri XACRO içinde |

**Kural:** SDF araç modeli hiçbir koşulda yazılmayacaktır. Gazebo dünya dosyası (pist ortamı) Adım 3'te SDF olarak yazılabilir; bu kural yalnızca araç modelini kapsar.

Tespit edilen ortam bu tabloya uymuyorsa çalışmayı durdur ve kullanıcıya tespit edilen ortamı rapor ederek yönlendirme iste.

---

### GÖREV 1.2 — Çalışma Alanı Dizin Yapısını Oluştur

**Ajan şu yapıyı tam olarak oluşturmalıdır:**

```
~/taxi_ws/
└── src/
    ├── aircraft_taxi_description/    ← Araç modeli (URDF/XACRO)
    │   ├── urdf/
    │   ├── launch/
    │   ├── package.xml
    │   └── CMakeLists.txt
    │
    ├── aircraft_taxi_gazebo/         ← Dünya dosyaları ve simülasyon başlatıcı
    │   ├── worlds/
    │   ├── launch/
    │   ├── package.xml
    │   └── CMakeLists.txt
    │
    ├── aircraft_taxi_control/        ← Görüntü işleme + PID (Adım 4'te doldurulur)
    │   ├── aircraft_taxi_control/
    │   ├── package.xml
    │   └── setup.py
    │
    └── aircraft_taxi_safety/         ← Çarpışma engelleme (Adım 6'da doldurulur)
        ├── aircraft_taxi_safety/
        ├── package.xml
        └── setup.py
```

**Uygulama kuralları:**
- `aircraft_taxi_description` ve `aircraft_taxi_gazebo` paketleri `ament_cmake` build tipiyle oluşturulacaktır.
- `aircraft_taxi_control` ve `aircraft_taxi_safety` paketleri `ament_python` build tipiyle oluşturulacaktır.
- `ros2 pkg create` komutu kullanılacaktır; manuel `package.xml` yazılmayacaktır.
- `control` ve `safety` paketleri bu adımda sadece iskelet olarak oluşturulacak, içleri boş bırakılacaktır.

---

### GÖREV 1.3 — Araç XACRO Modelini Yaz

Bu görevde ajan, `aircraft_taxi_description/urdf/` dizinine `aircraft_taxi.urdf.xacro` dosyasını oluşturacaktır.

**Modelin içermesi zorunlu olan bileşenler ve gereksinimleri:**

**[A] Gövde (Fuselage)**
- Dikdörtgen prizma geometrisi, görsel boyut: yaklaşık 3m uzun, 0.8m geniş, 0.4m yüksek.
- Gerçekçi bir inertia ve kütle değeri atanmalıdır (60 kg referans).
- `chassis` adıyla tanımlanmalı, `base_link`'e fixed joint ile bağlanmalıdır.

**[B] Kanatlar ve Kuyruk**
- Yalnızca görsel `<visual>` geometrisi olacak, `<collision>` tanımı olmayacaktır.
- Kanatlar geniş ve ince bir kutu geometrisiyle temsil edilecektir.
- Kuyruk fini ayrı bir link olarak tanımlanacaktır.

**[C] Tekerlekler**
- İki adet tahrikli arka tekerlek (`left_wheel`, `right_wheel`): `continuous` joint tipi.
- Bir adet pasif burun tekeri (caster): `fixed` joint tipi, sürtünme katsayıları minimize edilecektir.
- Tekerlek yarıçapı 0.18m, genişliği 0.10m olacaktır.
- Her tekerlekde `<collision>` ve `<inertial>` tanımı bulunacaktır.

**[D] Ön Kamera**
- Aracın ön-alt bölgesine, zemine yaklaşık 60 derece bakış açısıyla konumlandırılacaktır.
- `camera_link` ve `camera_optical_frame` olmak üzere iki ayrı link tanımlanacaktır.
- `camera_optical_frame` için ROS standart optik frame yönelimi kullanılacaktır: `rpy="-pi/2 0 -pi/2"`.
- Kamera çözünürlüğü: 640x480, yenileme hızı: 30 Hz.

**[E] Lidar Sensörü**
- Aracın ön-üst bölgesine yerleştirilecektir.
- Tarama açısı: öne doğru ±90 derece (180 derece toplam).
- Menzil: 0.1m ile 15m arası, 360 örnek/tur.
- `lidar_link` adıyla tanımlanacaktır.

**[F] Gazebo Plugin'leri**
K-1 kararına göre doğru plugin isimleri kullanılacaktır. Her üç bileşen de XACRO dosyasının içine `<gazebo>` blokları olarak yazılacaktır:

- Diferansiyel sürüş plugin'i: namespace `/aircraft_taxi`, topic `cmd_vel`, odom yayını aktif.
- Kamera plugin'i: topic `/aircraft_taxi/image_raw` ve `/aircraft_taxi/camera_info`.
- Lidar plugin'i: topic `/aircraft_taxi/scan`, çıktı tipi `sensor_msgs/LaserScan`.

**[G] XACRO Parametreleri**
Gövde boyutları, tekerlek yarıçapı, tekerlek açıklığı ve sensör konumları dosyanın en üstünde `<xacro:property>` blokları olarak tanımlanacaktır. Hiçbir sayısal değer geometri bloklarına doğrudan yazılmayacaktır.

---

### GÖREV 1.4 — CMakeLists.txt Dosyalarını Güncelle

`aircraft_taxi_description` ve `aircraft_taxi_gazebo` paketlerinin `CMakeLists.txt` dosyaları şu dizinleri install edecek şekilde güncellenecektir: `urdf`, `meshes` (varsa), `launch`, `worlds` (varsa).

---

### GÖREV 1.5 — RViz2 Görselleştirme Launch Dosyasını Yaz

`aircraft_taxi_description/launch/display.launch.py` dosyası oluşturulacaktır.

**Launch dosyasının başlatması gereken node'lar:**
1. `robot_state_publisher` — XACRO dosyasını `Command(['xacro ', urdf_path])` ile işleyerek `robot_description` parametresine besleyecektir.
2. `joint_state_publisher_gui` — Tekerlek eklemlerini manuel test etmek için.
3. `rviz2` — Boş konfigürasyonla başlatılabilir; kullanıcı manuel olarak `RobotModel` display ekleyecektir.

---

### GÖREV 1.6 — İlk Derleme ve Paket Doğrulama

**Ajan sırasıyla şunları yapmalıdır:**

1. `~/taxi_ws` dizininden `colcon build` komutunu çalıştır.
2. Derleme hataları varsa hata mesajını analiz et ve düzelt; en fazla 2 deneme hakkı var, 2. denemede de hata alınırsa çalışmayı durdur ve kullanıcıya hata raporunu sun.
3. `source install/setup.bash` komutunu çalıştır.
4. `ros2 launch aircraft_taxi_description display.launch.py` komutunu çalıştır.

---

### ADIM 1 — DOĞRULAMA KAPISI

Bir sonraki adıma geçmeden önce aşağıdaki tüm maddeler karşılanmış olmalıdır:

- [ ] `colcon build` sıfır hata ile tamamlanmıştır.
- [ ] `ros2 pkg list | grep aircraft_taxi` komutu dört paketi listelemiştir.
- [ ] `ros2 run xacro xacro aircraft_taxi.urdf.xacro` komutu geçerli XML çıktısı üretmiştir (stderr boştur).
- [ ] RViz2 açılmış ve `RobotModel` eklendiğinde araç geometrisi (gövde + kanatlar + tekerlekler) görünmektedir.
- [ ] `ros2 run tf2_tools view_frames` çalıştırıldığında `base_link → chassis → camera_link → camera_optical_frame` zinciri ve `chassis → lidar_link` bağlantısı `frames.pdf` içinde mevcuttur.

Tüm maddeler geçilmeden Adım 2'ye başlanmayacaktır.

---

## ADIM 2 — Sürüş Kontrolü ve Gazebo Entegrasyonu

### Bağlam
Bu adımın amacı; aracı Gazebo'da yaşayan (spawn edilmiş), `cmd_vel` mesajlarına tepki veren, odometri yayımlayan çalışan bir simülasyon haline getirmektir. Bu adım, Adım 1'in Doğrulama Kapısı'nın geçildiği varsayımıyla başlar.

---

### GÖREV 2.1 — Temel Gazebo Dünya Dosyasını Oluştur

`aircraft_taxi_gazebo/worlds/` dizinine `empty_ground.world` adında minimal bir dünya dosyası oluşturulacaktır.

**Dünya dosyasının içermesi gerekenler:**
- Düz zemin düzlemi (ground plane).
- Güneş ışığı (sun).
- Gerçek zamanlı fizik motoru aktif, yerçekimi -9.81 m/s².
- Kamera, ışık, GUI gibi bileşenler minimal tutulacaktır; görsel şatafat gerekmez.

Bu dosya ileride Adım 3'te pist ortamıyla değiştirilecektir; bu yüzden yalnızca fonksiyonel minimum hedeflenmektedir.

---

### GÖREV 2.2 — Gazebo Spawn Launch Dosyasını Yaz

`aircraft_taxi_gazebo/launch/gazebo_spawn.launch.py` dosyası oluşturulacaktır.

**Launch dosyasının sırasıyla başlatması gereken bileşenler:**

1. **Gazebo sunucusu** — `empty_ground.world` dosyasıyla, `use_sim_time:=true` parametresiyle.
2. **`robot_state_publisher`** — XACRO modelini işleyerek `robot_description` yayımlayacak, `use_sim_time:=true`.
3. **`spawn_entity`** — `robot_description` topic'ini dinleyerek aracı Gazebo sahnesine ekleyecek, başlangıç konumu: `x=0, y=0, z=0.2` (tekerlek zeminde olsun diye hafif yükseltilmiş).
4. **`joint_state_publisher`** — GUI versiyonu değil, simülasyon versiyonu (`joint_state_publisher` paketi).

**Önemli:** Tüm node'larda `use_sim_time` parametresi `True` olarak geçilecektir.

---

### GÖREV 2.3 — Sürüş Kontrolünü Manuel Olarak Test Et

Bu görev doğrulama amaçlıdır, yeni bir dosya üretilmez.

**Ajan şu adımları uygulayacaktır:**

1. `ros2 launch aircraft_taxi_gazebo gazebo_spawn.launch.py` komutunu çalıştır.
2. Ayrı bir terminalde `ros2 topic list` çıktısını al ve şu topic'lerin varlığını doğrula:
   - `/aircraft_taxi/cmd_vel`
   - `/aircraft_taxi/odom`
   - `/aircraft_taxi/image_raw`
   - `/aircraft_taxi/scan`
3. Aracı hareket ettirmek için şu komutu çalıştır:
   `ros2 topic pub /aircraft_taxi/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.5}, angular: {z: 0.0}}" --once`
4. Gazebo sahnesinde aracın ileri hareket ettiğini görsel olarak doğrula.
5. `ros2 topic echo /aircraft_taxi/odom --once` komutunu çalıştır ve pozisyon verisinin değiştiğini doğrula.

---

### GÖREV 2.4 — Temel Sürüş Kontrolcüsü Node'unu Yaz

`aircraft_taxi_control` paketine `base_controller.py` adında bir Python node'u eklenecektir.

**Node'un sorumlulukları:**

- `/aircraft_taxi/cmd_vel` topic'ine `Twist` mesajı yayımlamak.
- Dışarıdan hedef hız ve dönüş açısı alabilmek için iki adet ROS2 parametresi tanımlamak: `linear_speed` (varsayılan: 0.5 m/s) ve `angular_speed` (varsayılan: 0.0 rad/s).
- Bu node doğrudan hareket algoritması içermez; yalnızca üst katmandan (çizgi takibi veya güvenlik) gelen `Twist` verisini ileride `/aircraft_taxi/cmd_vel`'e iletecek bir köprü görevi görecektir.
- Node başlatıldığında aracı durdurma mesajı (sıfır hız) yayımlamalıdır.

**Bağımlılık:** `geometry_msgs`, `rclpy` paketleri `package.xml` içine eklenecektir.

---

### GÖREV 2.5 — Keyboard Teleoperation Testi

Bu görev isteğe bağlı fakat önerilir. Ajan şu paketi kuracak ve test edecektir:

`ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args --remap cmd_vel:=/aircraft_taxi/cmd_vel`

Araç klavye ile kontrol edilebiliyorsa bu görev başarılı sayılır.

---

### ADIM 2 — DOĞRULAMA KAPISI

- [ ] `ros2 launch aircraft_taxi_gazebo gazebo_spawn.launch.py` hatasız başlamaktadır.
- [ ] Araç Gazebo sahnesinde görünmektedir ve zemine oturmaktadır (havada değil).
- [ ] `/aircraft_taxi/cmd_vel`, `/aircraft_taxi/odom`, `/aircraft_taxi/image_raw`, `/aircraft_taxi/scan` topic'lerinin tümü `ros2 topic list` çıktısında mevcuttur.
- [ ] Manuel `topic pub` komutuyla araç Gazebo içinde hareket etmektedir.
- [ ] `ros2 topic echo /aircraft_taxi/scan --once` komutu geçerli `LaserScan` verisi döndürmektedir (ranges dizisi boş değil).
- [ ] `ros2 topic echo /aircraft_taxi/image_raw --once` komutu geçerli görüntü verisi döndürmektedir (width: 640, height: 480).

Tüm maddeler geçilmeden Adım 3'e başlanmayacaktır.

---

## ADIM 3 — Simülasyon Ortamı: Pist Dünyası

### Bağlam
Bu adımın amacı; aracın üzerinde hareket edeceği, ortasında beyaz merkez çizgisi bulunan, kamera görüntüsünde çizgi takibinin çalışabileceği bir pist ortamı oluşturmaktır. Pist dünyası SDF formatında yazılacaktır (araç modeli değil, yalnızca dünya dosyası).

---

### GÖREV 3.1 — Pist Dokusu (Texture) Tasarımı

`aircraft_taxi_gazebo/worlds/materials/` dizin yapısı oluşturulacaktır.

**Ajan şunları yapmalıdır:**

1. Pist zemini için koyu gri (asfalt rengi) bir temel doku belirlenecektir. Gazebo'nun yerleşik `Gazebo/Grey` veya `Gazebo/DarkGrey` materyali kullanılabilir; özel doku dosyası gerekmez.
2. Merkez çizgisi için **ayrı bir bant geometrisi** kullanılacaktır. Doku dosyası yerine beyaz renkli ince ve uzun bir `box` modeli olarak pist zeminine yerleştirilecektir. Bu, OpenCV'nin algılaması için yeterince belirgin kontrast sağlar.
3. Merkez çizgisi boyutları: yaklaşık 100m uzun, 0.15m geniş, 0.005m yüksek (zemine çok yakın).

**Tasarım kısıtı:** Özel `.material` veya `.dae` dosyası gerektiren karmaşık doku işleme yapılmayacaktır. Gazebo'nun built-in renk/materyal sistemi kullanılacaktır.

---

### GÖREV 3.2 — Pist Dünya Dosyasını Yaz

`aircraft_taxi_gazebo/worlds/runway.world` adıyla yeni bir dünya dosyası oluşturulacaktır.

**Dünya dosyasının içermesi gereken modeller:**

**[A] Zemin**
- `ground_plane` plugin'i veya büyük düz bir `box` kullanılacaktır.
- Boyut: 200m x 50m, koyu gri renk.
- Zemin Z ekseni 0'da olacaktır.

**[B] Merkez Çizgisi**
- Beyaz renkli ince kutu (`box`), zemin üzerine yerleştirilecektir.
- `static` model olarak tanımlanacak (fizik motoru hesaplamayacak).
- Pist boyunca X ekseni üzerinde uzanacaktır.

**[C] Kenarlık Çizgileri (isteğe bağlı)**
- İki adet sarı kenarlık çizgisi merkez çizgisinin her iki yanına, aynı yöntemle eklenebilir.
- Bu elemanlar ileride ek görüntü işleme özelliği geliştirmeye imkan tanır.

**[D] Işıklandırma**
- Bir adet `directional` güneş ışığı, gölge olmadan (shadows: false), kamera görüntüsünde tutarlı aydınlatma için.

**[E] Engel Placeholder'ları**
- Pistin ileri bölgesine iki adet `box` engel yerleştirilecektir (Adım 6 için hazırlık).
- Engel boyutu yaklaşık 0.5m x 0.5m x 1m, kırmızı renkli, `static` model.
- Konumları pistin 20m ve 40m ilerisinde olacaktır.

---

### GÖREV 3.3 — Pist Launch Dosyasını Güncelle

`aircraft_taxi_gazebo/launch/gazebo_spawn.launch.py` dosyası güncellenecektir.

**Güncelleme:** Dünya dosyası parametresi `empty_ground.world`'den `runway.world`'e değiştirilecektir. Bunun dışında launch dosyasına dokunulmayacaktır.

Alternatif olarak `gazebo_runway.launch.py` adıyla yeni bir launch dosyası oluşturulabilir; bu durumda eski dosya silinmeyecek, her ikisi de korunacaktır.

---

### GÖREV 3.4 — Kamera Görüntüsünü Doğrula

**Ajan şu adımları uygulayacaktır:**

1. `ros2 launch aircraft_taxi_gazebo gazebo_spawn.launch.py` komutunu pist dünyasıyla başlat.
2. `ros2 run rqt_image_view rqt_image_view` komutunu çalıştır.
3. `/aircraft_taxi/image_raw` topic'ini seç.
4. Görüntüde zemin ile merkez çizgisinin kontrast oluşturduğunu gözle doğrula.

**Başarı kriteri:** Kamera görüntüsünde koyu zemin üzerinde açık renkli merkez çizgisi seçilebilir durumdadır. Bu, Adım 4'teki OpenCV threshold işleminin çalışabilmesi için zorunludur.

---

### ADIM 3 — DOĞRULAMA KAPISI

- [ ] `runway.world` dosyası Gazebo tarafından hatasız yüklenmektedir.
- [ ] Gazebo sahnesinde pist zemini, merkez çizgisi ve engel nesneleri görünmektedir.
- [ ] Araç pist başlangıcında spawn edilmekte ve zemine doğru oturmaktadır (çizgi üzerinde veya çizgiye yakın).
- [ ] `rqt_image_view` ile `/aircraft_taxi/image_raw` görüntülenmektedir ve görüntüde zemin-çizgi kontrastı gözle seçilebilmektedir.
- [ ] `ros2 topic echo /aircraft_taxi/scan --once` komutu engel nesnelerini menzil içinde göstermektedir (ranges dizisinde inf olmayan değerler mevcuttur).

---

## GENEL NOTLAR (Ajan İçin)

**Hata yönetimi:** Bir komut beklenen çıktıyı vermezse önce log dosyalarını (`~/.ros/log/`) incele. Gazebo ile ilgili hatalarda `gzserver` ve `gzclient` süreçlerini `pkill` ile temizleyip yeniden başlat.

**Namespace tutarlılığı:** Tüm topic'ler `/aircraft_taxi/` namespace'i altında olmalıdır. Plugin tanımlarında namespace ayarı gözden kaçırılırsa topic'ler kök seviyede yayımlanır ve sistem entegrasyonu bozulur.

**Sim time:** Gazebo çalışırken tüm node'larda `use_sim_time: true` parametresi aktif olmalıdır. Aksi takdirde zaman damgası uyumsuzluğu nedeniyle sensör verisi kaybolabilir.

**Sonraki adımlar için hazırlık:** Bu üç adım tamamlandığında sistem şu durumda olmalıdır: araç pist üzerinde spawn edilmiş, kamera çizgiyi görmekte, lidar engelleri taramakta, `cmd_vel` komutuyla hareket etmektedir. Adım 4 (görüntü işleme + PID) bu temel üzerine inşa edilecektir.
