# smart_cleaning_robot
cat <<EOF > README.md
# Akilli Supurge Robotu
KTUN – Robotiğe Giriş 
Final Uygulama Ödevi

## Proje Özeti
Bu projede ROS Noetic ve TurtleBot3 kullanılarak
ev ortamında çalışan akıllı bir süpürge robotu geliştirilmiştir.

Robot:
- SLAM ile harita çıkarır ve kaydeder
- Harita üzerinde AMCL ile lokalize olur
- Oda girişlerine gider
- QR ları okunmadan geçer 


## Kullanılan Teknolojiler
- ROS1 Noetic
- TurtleBot3
- Gazebo
- RViz
- Python (rospy)

## Klasör Yapısı
- launch/ : Simülasyon, SLAM, navigasyon ve görev yöneticisi launch dosyaları
- src/ : Görev yöneticisi, ev_tarama ve QR okuyucu node'ları
- config/ : Görev ve waypoint tanımları (gorev.yaml)
- haritalar/ : Kaydedilmiş harita dosyaları
- world/ : Ev ortamı ve QR yerleşimleri
- models/ : qr_salon,qr_mutfak,qr_yatak_odasi,qr_koridor
- rviz/ : navigasyon.rviz

## Çalıştırma (Özet)
1. Simülasyon başlat: `roslaunch akilli_supurge_robotu simulasyon.launch`  
2. SLAM başlat: `roslaunch akilli_supurge_robotu slam.launch`  
3. Navigation başlat: `roslaunch akilli_supurge_robotu navigasyon.launch`  
4. Görev yöneticisi: `roslaunch akilli_supurge_robotu gorev_yoneticisi.launch`  

Detaylı adımlar demo videosunda ve raporda gösterilmiştir.
EOF
