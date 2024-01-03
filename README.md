# smabo
スマートフォンが入るロボットsmaboに関するリポジトリです。

# smaboとは
smaboはスマートフォンが入るロボットであり、

- ナビゲーション (navigation)
- 動作計画 (motion planning)
- 画像処理 (image processing)
- 遠隔操作 (remote control)
- 深層学習 (deep learning)
- 二足歩行 (bipedal walking)

など様々なことができます。

https://github.com/akinami3/smabo/assets/151462572/efaf8d19-57ab-4605-b386-3ae562e6a525

本リポジトリでは、smaboに関係するコード等をまとめていく予定です。

<br>

# smaboの作り方、使い方
smaboの詳しい作り方については以下ブログにて解説しているので、ぜひご覧ください。

[スマホが入るロボットの作り方【smabo】](https://akinami3.com/smabo_summary)
![Alt text](./image/smabo_site_img.png)

<br>

本READMEでは、基本的な環境構築手順、使用法についてのみ解説します。
## smabo用ワークスペースの作成
以下コマンドで、smabo用のワークスペースを作成してください
```bash
mkdir -p smabo_ws/src
```

## リポジトリのクローン、パッケージのコピー
以下コマンドで、本リポジトリをcloneしてください。
```bash
cd ~/
```
```bash
git clone https://github.com/akinami3/smabo.git
```
<br>
次に、以下コマンドでsmabo用のパッケージをワークスペースのsrcディレクトリにコピーしてください

```bash
cp -r ~/smabo/smabo_pkgs ~/smabo_ws/src
```
<br>
smabo関連のパッケージ以外に、「unityをROS2で通信するために必要なパッケージ」である「ROS-TCP-Endpoint」もcloneします（スマホとラズパイの通信に使用します）。

```bash
cd ~/smabo_ws/src
```

```bash
git clone -b dev-ros2 https://github.com/Unity-Technologies/ROS-TCP-Endpoint.git
```

## パッケージのビルド
以下コマンドで、ワークスペースをビルドしてください
```bash
cd ~/smabo_ws
```
```bash
colcon build --symlink-install
```
<br>

次に、以下コマンドでros2コマンドからビルドしたパッケージを起動できるようにします。

```bash
source ~/smabo_ws/install/setup.bash
```

ターミナルを起動するたびに上記コマンドを実行するのは面倒なので、 `.bashrc` にも記述しておきます。

```bash
echo "source ~/smabo_ws/install/setup.bash" >> ~/.bashrc
```
## smaboアプリのインストール
リポジトリ内の「SmartPhoneRobot.apk」をスマホにダウンロードし、インストールしてください。

## smaboをROS2で通信
### ROS-TCP-Endpointの起動
以下コマンドで、unityをROSで通信するためのサーバーを起動します。

```bash
ros2 run ros_tcp_endpoint default_server_endpoint
```
### smaboアプリの起動
スマホ側で、インストールしたsmart_phone_robot（smabo用アプリ）を起動します。

アプリを起動すると、ipアドレスとポート番号を入力するオプション画面が表示されるので
- ip address : ラズパイ側のipアドレス
- port : 10000

と入力して「Connect」ボタンをクリックしてください。

ボタンをクリックし、しばらく待った後に「connected!!!!!!!」と表示されれば、接続が完了しています。

![Alt text](./gif/smabo_connection.gif)


> [!NOTE]
> この時、何度か試してみても「not connected」と表示され続ける場合は、**ラズパイとスマホが異なるネットワークに接続されている可能性がある**ので確認してみてください。

### smaboの表情を変更する
![Alt text](./gif/smabo_impression.gif)

以下コマンドで、smaboの表情を変更するノードが起動します。

```bash
ros2 run smabo_pkg impression_pub
```

ノードを起動したら、「0～5の間の整数」を入力することでsmaboの表情を変化させることが出来ます。

## スマホのセンサ情報をラズパイで取得する

### ジャイロ
![Alt text](./gif/smabo_gyro.gif)

以下コマンドで、スマホのジャイロの情報を取得するノードが起動します。

```bash
ros2 run smabo_pkg gyro_sub
```

### 加速度
以下コマンドで、スマホの加速度を取得するノードが起動します。

```bash
ros2 run smabo_pkg accel_sub
```

### ジャイロ
以下コマンドで、スマホのコンパス情報を取得するノードが起動します。

```bash
ros2 run smabo_pkg gyro_sub
```

