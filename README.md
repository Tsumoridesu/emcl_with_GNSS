# emcl_with_GNSS: GNSSにより重みをemclに実装
[amclにGNSSを組み込んだもの](https://github.com/midemig/gps_amcl
)のemcl版(の劣化かも？)

GNSSによる重み計算を追加済み，テストは一切していない

インストール方法もemclと一緒
## 追加のrosトピック


```gps_topic```:```nav_msgs::Odometry```GNSSのトピック名

  GNSSとIMUをセンサフュージョンしたのGNSS_odomみたいなものを使う

## 参考
[1]de Miguel MÁ, García F, Armingol JM. Improved LiDAR Probabilistic Localization for Autonomous Vehicles Using GNSS. Sensors. 2020; 20(11):3145. https://doi.org/10.3390/s20113145
[2]https://github.com/midemig/gps_amcl
