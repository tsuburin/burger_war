Linuxユーザー向けdocker-compose
===

## 概要

burger_warの環境を立ち上げるdocker-composeと、補助スクリプトです。

linuxマシンは持っているけど対応しているUbuntuではなかったり、環境を汚したくなかったり、だけど仮想マシンにするほどスペックの余裕もない、そういう人向けです。
intelのオンチップグラフィックス(Iris Plus 640)でhardware accelerationが効いた実績があるので、特にノートパソコン等に普段遣いのlinuxが入っている人は試してもらうといいかもしれません。

コンテナ内に入って作業するのではなく、ホスト環境に置かれたリポジトリをコンテナ内にマウントして、
ホストから通常のスクリプト実行のような感覚でコンテナ内のコマンドが実行できるような作りにしています。
[使用例](#使用例)と[コンテナ環境について](#コンテナ環境について)を読んでもらうとなんとなくわかるかもしれません。

CUIコマンドは`docker-compose exex`を介して行うので、普段使っているターミナル等でそのまま実行できます。
GUIレンダリングは`/tmp/X11-unix`と`/dev/dri`を共有してDRMで行います。
ただし、コンテナとホストのドライバの相性ではハードウェアアクセラレーションが効かずmesaのソフトウェアレンダリングにfallbackすることになります。

### 動作環境

ローカルでXサーバーを実行しているlinux環境で動くと思いますが、現状動作確認しているのは以下の環境です。

| OS        | CPU           | RAM  | GPU               | ハードウェアアクセラレーション | Real Time Factor | FPS |
|-----------|---------------|------|-------------------|--------------------------------|------------------|-----|
| Archlinux | Rydzen7 3700x | 32GB | Radeon RX 5500 XT | 無効                           | 1.0              | 17  |
| Archlinux | i7-7560U      | 8GB  | Iris Plus 640     | 有効                           | 0.85             | 44  |

確認方法：sim_with_judge.shとstart.shを実行し、Real Time FactorとFPSを確認  
数値のとり方は適当です、変動するのでだいたいの値です。

## 依存ソフトウェア

* Docker Composeのfile format 3.7以上に対応したDocker EngineとDocker Compose  
    https://docs.docker.com/compose/compose-file/compose-versioning/#compatibility-matrix などを参考にインストールしてください。  
    ディストリビューションのリポジトリ等は確認していないので、動作確認ができれば報告していただけるとありがたいです。
* xhost  
    コンテナからXサーバーへの接続を許可するために使用します。


## 使い方

以下の内容は特に断りがない限りこのディレクトリ（このREADME.mdと同じディレクトリ）で行うことを前提としています。

初めての人は、[準備](#準備)のあとすぐ[使用例](#使用例)まで飛ばしたほうがわかりやすいかもしれません。

### 準備

#### ユーザーID・グループIDの設定

初めて使う場合は、まず以下を実行してください。  
ホストのディレクトリをマウントする際に所有権を揃えるため、コンテナ内で使うユーザーIDとグループIDをホストと同一にします。

```sh
echo UID=$(id -u) >> .env
echo GID=$(id -g) >> .env
```

#### コンテナからXサーバーへの接続許可

Xサーバーはデフォルトではコンテナから接続できないようになっています。
許可を与えるために以下を実行してください。

```sh
xhost +local:
```

この設定はXサーバーを起動するごとにリセットされるので、その都度実行する必要があります。

---

### コンテナビルド

これはDockerfileやdocker-compose.yml等を編集したときなど、コンテナイメージに変更がある場合に実行してください。

```sh
docker-compose build
```

---

### コンテナ起動

コンテナ内でコマンドを実行する際はコンテナが起動している必要があります。以下で起動します。

```sh
docker-compose up -d
```

`-d`オプションを忘れるとコンテナは起動しますがターミナル上でそれ以上何も起きない状態になります。
その時は`Ctrl-C`で終了してやり直してください。

---

### コンテナ内でコマンド実行

コンテナ内で`burger`ユーザーとしてーとしてコマンドを実行します。
コマンドの実行ディレクトリは`/home/burger/`になります。

```sh
docker-compose exec burger COMMAND
```

これはコンテナが起動している状態で行ってください。

COMMANDはシェルを介さず実行されます。
そのため、`.bashrc`等が読み込まれず、`source /opt/ros/kinetic/setup.bash`や`source /home/burger/catkin_ws/devel/setup.bash`などが実行されていない状態でCOMMANDが実行されます。
それらの環境が必要な場合は、代わりに下記の`exec-*.sh`を使用してください。

#### 便利スクリプト集

`scripts/`内にホストからコンテナ内でコマンドを実行するためのスクリプトがあります。

それぞれのスクリプトはコマンドを実行する際に`.bashrc`を読み込むようになっています。

* `exec-*.sh`: どちらも与えられた引数をコンテナ内でコマンドとして実行しますが、実行ディレクトリが異なります。
    * `exec-in-home.sh`: homeディレクトリでコマンドを実行します。
    * `exec-here.sh`: リポジトリ内で実行した場合、現在のディレクトリに対応するコンテナ内のディレクトリでコマンドを実行します。
* `catkin_make.sh`: `/home/burger/catkin_ws`で`catkin_make`を実行します。引数はそのまま`catkin_make`の引数として渡されます。

`scripts/samples/sim_with_judge.sh`は公式配布スクリプトの`scripts/sim_with_judge.sh`と同等のコマンドを実行するスクリプト例です。  
ただし、`alacritty`は作者環境でのターミナルソフトです。各自の環境に合わせて置き換えてください。  
[使用例](#使用例)も参照してください。


---

### コンテナに入る

必要であればコンテナ内に入って作業することもできます。

```sh
docker-compose exec burger bash
```

コンテナから出るときは、`exit`コマンドもしくは`Ctrl-D`を使用します。

---

### コンテナ終了

```sh
docker-compose down 
```

---

### 使用例

この使用例は[メインREADME](../README.md)の`サンプルの実行`と対応しています。

前提として、

* [準備](#準備)が終わっている
* このリポジトリのパスは`$REPO`

とします。

以下のコマンドはすべてホスト側で行います。

ターミナル１
```sh
# コンテナ起動
cd $REPO/docker-compose
docker-compose up -d

# 必要であればcatkin_makeを実行
# 少なくとも初回は必要(コンテナ作成ではメインREADMEの5. makeは実行されないため)
scripts/catkin_make.sh

# 初回であればモデルデータの読み込みのためgazebo起動
scripts/exec-in-home.sh gazebo
# gazeboを手動で終了

# 公式配布のsim_with_judge.shと同等のコマンドを実行する
# ターミナルソフトをalacrittyとしているので、必要であれば書き換えてから実行してください
scripts/samples/sim_with_judge.sh
```

ターミナル２
```sh
# ロボット動作スクリプトを実行
cd $REPO
docker-compose/scripts/exec-here.sh bash scripts/start.sh
```

終了する際は、実行中のプロセスをCtrl-C等で止めたあと、いずれかのターミナルで、
```sh
cd $REPO/docker-compose
docker-compose down
```

---


## コンテナ環境について

[コンテナビルド](#コンテナビルド)を行うと、[メインREADME](../README.md)の`3. 依存ライブラリのインストール`まで完了したコンテナイメージが作られます。

コンテナ内にはホスト環境のuser ID, group IDと同じIDで`burger`ユーザーが作られます。
`catkin_ws`ディレクトリは`burger`ユーザーのホームディレクトリ直下に作成されます。

イメージ内のディレクトリは以下のようになっています。(関係ある部分のみ抜粋)

```
/home/burger/
  ├ catkin_ws/
  │  ├ src/
  │  │ └ CMakeLists.txt
  │  ├ build/
  │  ├ devel/
  │  └ .catkin_workspace
  └ .gazebo/
```

[コンテナ起動](#コンテナ起動)時に`/home/burger/catkin_ws/src/burger_war`としてこのリポジトリをまるごとマウントします。

コンテナは使い終わるたびに破棄するような使い方を想定しているのですが、いくつかのデータは継続して使えたほうが便利かと思い次回コンテナ起動時にも引き継ぐようにしています。
具体的には、以下のディレクトリです。

| パス                    | 説明                         |
|:------------------------|------------------------------|
| `/home/burger/.gazebo/` | gazeboのモデルデータ等       |
| `/home/burger/build`    | catkin_makeのbuild directory |
| `/home/burger/devel`    | catkin_makeのdevel directory |

コンテナ終了時にこれらのデータも削除したい場合は、`docker-compose down`のかわりに`docker-compose down -v`を実行してください。
