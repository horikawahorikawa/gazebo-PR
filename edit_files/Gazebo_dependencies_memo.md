# gazebo-PRのコンパイル方法
ROS-TMS ver4.0とgazebo6によるコンパイル方法を記す．

ROS-TMS ver5.はgazebo7との互換があるため，gazebo_rosのコンパイルはしなくてOK．

## 不要なパッケージの削除
ros-indigoをインストールした際に，インストールされるgazebo関連のパッケージを全てアンインストールしておく．


ros-indigoの互換バージョンがgazebo2のため，gazebo6のインストールの際にライブラリ等が衝突する．

## 元のgazebo6をコンパイルする  
コンパイル方法はHowtoBuild.mdに記している．

Gazebo６の依存ライブラリ
```
ignition-math2
sdformat3
protobuf-compiler
libprotobuf
libccd
```
\* 通常のgazeboをコンパイルした後に，gazeboの各種ライブラリにパスを通し忘れないように

## gazebo6のファイルの書き換えて，gazebo-PRに！！
元のgazebo6のフォルダ"gui"と"rendering"とedit_filesの"gui"と"rendering"に差し替えて再びコンパイルする．

edit_filesの依存ライブラリ
```
libv4l-dev
```

おそらくコンパイルは一度で通らない．これはoculus_pluginというライブラリが生成されるが，gazeboにリンク通っていないからである．なので，gazeboのライブラリフォルダに生成されたoculus_pluginを入れてください．

\* CmakeLists.txtをきちんと書けば一発で通ると思います

また，gtest関連でエラーを吐いて，コンパイルが通らないと思います．その時は```catkin/cmake/all.cmake```ファイルのgtestの箇所をコメントアウトすると通ります．
おそらくcatkin_makeによって生成されるgtestとgazebo6によって生成されるgtestで衝突しているためかと思います．

## gazebo_ros パッケージのコンパイル
githubからROSのバージョンに合うgazebo_rosのパッケージを探して，コンパイルする

\* ROSのパッケージをコンパイルするときは，```catkin/cmake/all.cmake```のコメントアウトしたgtestの箇所は元に戻してね

gazebo_ros パッケージの依存ライブラリ
```
ros-indigo-control-toolbox
ros-indigo-transmission-interface
ros-indigo-joint-limit-interface
```
