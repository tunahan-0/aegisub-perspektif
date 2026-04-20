# Aegisub Perspektif Hesaplayıcı

Bu araç, dörtgen köşe koordinatlarını kullanarak ihtiyacımız olan Aegisub perspektif etiketlerini (\pos, \frz, \fscx, \fscy, \fax) hesaplamaya yarar. Hem grafik arayüzüyle hem de komut satırı üzerinden kullanılabilir.

## Kurulum

Versiyon sekmesinden .exe dosyasını indirip kullanabilirsiniz. Eğer .py dosyasını kullanmak istiyorsanız sisteminizde Python ve bazı Python kütüphaneleri yüklü olmalıdır, 'requirements.txt' dosyasında bulabilirsiniz.

## .py Dosyası İle Kullanım

### Grafik Arayuzu (GUI)

`python aegisub_perspective.py`

### Komut Satiri (CLI)
Arayüze girmeden direkt komut satırından da hesaplama yaptırabilirsiniz:

`python aegisub_perspective.py "x1,y1,x2,y2,x3,y3,x4,y4"`

İsteğe bağlı parametreler:
- `--ratio`: Hedef nesnenin en-boy oranı (Örn: 2.5)
- `--size`: Hedef nesnenin piksel boyutu (Örn: 1280x720)
- `--alignment`: Aegisub (\an) değeri (Varsayılan: 7)

## Notlar

- Koordinat giriş sırası sol üstten başlayıp sağ altta bitecek şekilde, yani saat yönünde olmalıdır: Sol üst, Sağ Üst, Sağ Alt, Sol Alt.
- Yazılım, Aegisub'daki eksen dönmelerini ve ölçek farklılıklarını otomatik olarak dengeler.
