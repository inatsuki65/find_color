# ROSとOpenCVを使った色検出のプログラム

[![](https://img.youtube.com/vi/a0Ebx5SHyJU/0.jpg)](https://www.youtube.com/watch?v=a0Ebx5SHyJU)

※ 画像をクリックすると動画リンクに飛びます。


検出したい色はソースコード中のs_min,s_maxを調整することで変えられます。

## find_bgrではRGB色空間を使って青を検出してます。

** RGB色空間 とOpenCV **

  OpenCVでは、色の順番がBGR（青、緑、赤）になる。

## find_hsvではHSV色空間を使って青色を検出してます。

** HSV色空間 OpenCVで取り得る値の範囲 **

- H (Channel1)

 　　0～180の範囲。Hueは本来0～360だがOpenCVではH/2の値を示す。181～255の範囲は0からの循環に回される
- S (Channel2)

 　　0～255の範囲。255がS=1.0に相当。
- V (Channel3)

 　　0～255の範囲。255がV=1.0に相当。
