/**
 * Copyright (c) 2025 aNoken
 *
 * 回転しない寿司
 * 
 * このプログラムは、M5StickCとQwiic OTOSセンサーを使用して
 * RoverCを制御し、回転しない寿司の走行制御を実現します。
 * 
 * 機能:
 * - M5StickCのボタンAを押すと移動モードを切り替え (0:停止, 1:四角形, 2:三角形, 3:直線, 4:原点復帰)
 * - M5StickCのボタンBを押すとIMUをキャリブレーションし、位置トラッキングをリセット
 * - 自動的に目標地点に向かって移動
 */

#include <M5Unified.h>
#include "SparkFun_Qwiic_OTOS_Arduino_Library.h"
#include "Wire.h"

// RoverCのI2Cアドレス定義
#define ROVER_ADDRESS 0x38

// OTOSセンサーのインスタンス生成
QwiicOTOS myOtos;

/**
 * I2C通信でRoverCにバイト列を書き込む
 * 
 * @param addr I2Cアドレス
 * @param reg レジスタアドレス
 * @param buffer 送信データバッファ
 * @param length バッファの長さ
 */
void M5_RoverC_writeBytes(uint8_t addr, uint8_t reg, uint8_t *buffer, uint8_t length) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  for (int i = 0; i < length; i++) {
    Wire.write(buffer[i]);
  }
  Wire.endTransmission();
}

/**
 * I2C通信でRoverCにバイト列を書き込み、結果をデバッグ出力する
 * 
 * @param addr I2Cアドレス
 * @param reg レジスタアドレス
 * @param buffer 送信データバッファ
 * @param length バッファの長さ
 */
void M5_RoverC_writeBytes_debug(uint8_t addr, uint8_t reg, uint8_t *buffer, uint8_t length) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  for (int i = 0; i < length; i++) {
    Wire.write(buffer[i]);
  }
  uint8_t status = Wire.endTransmission();

  // 通信結果をシリアルに出力
  Serial.print("I2C Write Status (addr 0x");
  Serial.print(addr, HEX);
  Serial.print("): ");

  switch (status) {
    case 0:
      Serial.println("Success");
      break;
    case 1:
      Serial.println("Data too long");
      break;
    case 2:
      Serial.println("NACK on address");
      break;
    case 3:
      Serial.println("NACK on data");
      break;
    case 4:
      Serial.println("Other error");
      break;
    default:
      Serial.println("Unknown error");
      break;
  }
}

/**
 * RoverCの各モーターの速度を設定する
 * 
 * @param x X方向の速度成分 (-120〜120)
 * @param y Y方向の速度成分 (-120〜120)
 * @param z 回転方向の速度成分 (-120〜120)
 */
void M5_RoverC_setSpeed(int8_t x, int8_t y, int8_t z) {
  int8_t buffer[4];
  
  // 回転成分がある場合、X/Y成分を調整
  if (z != 0) {
    x = int(x * (100 - abs(z)) / 100);
    y = int(y * (100 - abs(z)) / 100);
  }
  
  // 各モーターの速度を計算（-120〜120の範囲に制限）
  buffer[0] = max(-120, min(120, y + x - z)); // 左前輪
  buffer[1] = max(-120, min(120, y - x + z)); // 右前輪
  buffer[3] = max(-120, min(120, y + x + z)); // 左後輪
  buffer[2] = max(-120, min(120, y - x - z)); // 右後輪
  
  // RoverCに速度データを送信
  uint8_t _addr = ROVER_ADDRESS;
  M5_RoverC_writeBytes_debug(_addr, 0x00, (uint8_t *)buffer, 4);
}

/**
 * セットアップ関数
 * デバイスの初期化とOTOSセンサーのセットアップを行う
 */
void setup() {
  // M5Stackの初期化
  auto cfg = M5.config();
  M5.begin(cfg);
  Serial.begin(115200);

  Serial.printf("M5StickC_Qwiic_OTOS_Example");
  
  // デバイス名を取得
  String name;
  String pins;
  switch (M5.getBoard()) {
    case m5::board_t::board_M5StickC:
      name = "M5StickC";
      break;
    default:
      name = "New Device";
      break;
  }
  
  // I2Cピン情報を取得
  pins = "[I2C] SDA:" + String(M5.Ex_I2C.getSDA()) +
         " SCL:" + String(M5.Ex_I2C.getSCL());

  Serial.println(name);
  Serial.println(pins);

  // I2C通信の初期化（SDA:0, SCL:26, クロック周波数:100kHz）
  Wire.begin(0, 26, 100000UL);

  // OTOSセンサーの接続確認
  while (myOtos.begin() == false) {
    M5.Log.printf("I2C not connected, check your wiring and I2C address!");
    delay(1000);
  }

  Serial.printf("OTOS connected!");
  Serial.printf("Ensure the OTOS is flat and stationary, then enter any key to calibrate the IMU");
  Serial.println("Calibrating IMU...");
  Serial.printf(" Calibrate the IMU,resetTracking");
  
  // IMUのキャリブレーションとトラッキングリセット
  myOtos.calibrateImu();
  myOtos.resetTracking();

  // モーターを停止状態に初期化
  M5_RoverC_setSpeed(0, 0, 0);
}

/**
 * メインループ関数
 * センサー値の読み取りと移動制御を繰り返し実行
 */
void loop() {
  // 状態変数の定義
  static int rover_flag = 0;       // 移動モード（0:停止, 1-4:移動パターン）
  static int rover_goal = 0;       // 現在の目標地点インデックス
  static float rover_x_goal = 0;   // X座標の目標値（mm）
  static float rover_y_goal = 0;   // Y座標の目標値（mm）
  static float rover_z_goal = 0;   // 角度の目標値（度）
  static unsigned long previousMillis = 0;
  
  // ボタン状態の更新
  M5.update();

  // センサーデータ構造体
  sfe_otos_pose2d_t pos;  // 位置
  sfe_otos_pose2d_t vel;  // 速度
  sfe_otos_pose2d_t acc;  // 加速度

  // OTOSセンサーからデータ取得
  myOtos.getPosition(pos);
  myOtos.getVelocity(vel);
  myOtos.getAcceleration(acc);
  
  // センサー値をミリメートル単位に変換（インチからミリメートルへ）
  int rover_x_now = pos.x * 25.4;  // 現在のX座標（mm）
  int rover_y_now = pos.y * 25.4;  // 現在のY座標（mm）
  int rover_z_now = pos.h;        // 現在の向き（度）

  // ボタンBが押された場合：センサーリセット
  if (M5.BtnB.wasReleased()) {
    Serial.printf(" Calibrate the IMU,resetTracking");
    myOtos.calibrateImu();
    myOtos.resetTracking();
    rover_goal = 0;
    rover_x_goal = 0;
    rover_y_goal = 0;
  }

  // ボタンAが押された場合：移動モード切替
  if (M5.BtnA.wasReleased()) {
    if (rover_flag == 0)
      rover_flag = 1;      // 四角形移動モード
    else if (rover_flag == 1)
      rover_flag = 2;      // 三角形移動モード
    else if (rover_flag == 2)
      rover_flag = 3;      // 直線移動モード
    else if (rover_flag == 3)
      rover_flag = 4;      // 原点復帰モード
    else
      rover_flag = 0;      // 停止モード
  }

  // 現在位置と目標位置の差分計算
  float dis_x = (rover_x_goal - rover_x_now);
  float dis_y = (rover_y_goal - rover_y_now);
  float dis_z = (rover_z_goal - rover_z_now);

  // 目標地点までの距離を計算
  float dis = sqrt(dis_x * dis_x + dis_y * dis_y + dis_z * dis_z);
  float dis_thresh = 50;  // 目標に到達したと判断する距離閾値（mm）

  // 四角形移動モード（モード1）での目標地点更新
  if ((dis_thresh >= dis) && (rover_flag == 1)) {
    rover_goal++;
    if (rover_goal > 3) rover_goal = 0;

    // 四角形の各頂点を順に目標に設定
    if (rover_goal == 0) {
      rover_y_goal = 0;      // 原点
      rover_x_goal = 0;
    }
    if (rover_goal == 1) {
      rover_y_goal = 300.0;  // 北
      rover_x_goal = 0;
    }
    if (rover_goal == 2) {
      rover_y_goal = 300.0;  // 北東
      rover_x_goal = 400.0;
    }
    if (rover_goal == 3) {
      rover_y_goal = 0.0;    // 東
      rover_x_goal = 400.0;
    }
  }

  // 三角形移動モード（モード2）での目標地点更新
  if ((dis_thresh >= dis) && (rover_flag == 2)) {
    rover_goal++;
    if (rover_goal > 3) rover_goal = 0;

    // 三角形パターンの各頂点を目標に設定
    if (rover_goal == 0) {
      rover_y_goal = 0;       // 原点
      rover_x_goal = 0;
    }
    if (rover_goal == 1) {
      rover_y_goal = 200.0;   // 北東
      rover_x_goal = 200.0;
    }
    if (rover_goal == 2) {
      rover_y_goal = 0.0;     // 原点
      rover_x_goal = 0.0;
    }
    if (rover_goal == 3) {
      rover_y_goal = 200.0;   // 北西
      rover_x_goal = -200.0;
    }
  }

  // 直線移動モード（モード3）での目標地点更新
  if ((dis_thresh >= dis) && (rover_flag == 3)) {
    rover_goal++;
    if (rover_goal > 2) rover_goal = 0;

    // 直線パターンの各端点を目標に設定
    if (rover_goal == 0) {
      rover_y_goal = 0;      // 原点
      rover_x_goal = 0;
    }
    if (rover_goal == 1) {
      rover_y_goal = 0.0;    // 東
      rover_x_goal = 400.0;
    }
  }

  // 原点復帰モード（モード4）
  if (rover_flag == 4) {
    rover_y_goal = 0.0;     // 原点を目標に設定
    rover_x_goal = 0.0;
  }

  // 制御ゲインの設定
  float z_gain = 0.80;  // 回転制御ゲイン
  float x_gain = 0.80;  // 位置制御ゲイン

  // PID制御的な位置制御計算（比例制御）
  float rover_x = -(rover_x_now - rover_x_goal) * x_gain;
  float rover_y = -(rover_y_now - rover_y_goal) * x_gain;
  float rover_z = (rover_z_goal - rover_z_now) * z_gain;

  // 速度制限（-100〜100の範囲に制限）
  if (rover_x < -100) rover_x = -100;
  if (rover_y < -100) rover_y = -100;
  if (rover_z < -100) rover_z = -100;

  if (rover_x > 100) rover_x = 100;
  if (rover_y > 100) rover_y = 100;
  if (rover_z > 100) rover_z = 100;

  // モード0なら停止、それ以外なら計算した速度で移動
  if (rover_flag == 0)
    M5_RoverC_setSpeed(0, 0, 0);
  else
    M5_RoverC_setSpeed((int)rover_x, (int)rover_y, (int)rover_z);

  // ディスプレイ表示の更新
  M5.Display.clear();
  M5.Display.startWrite();
  M5.Display.setCursor(0, 0);
  M5.Display.setTextSize(1);
  
  // 現在位置の表示
  M5.Display.printf("rx= %f \t", rover_x_now);
  M5.Display.printf("ry= %f \t", rover_y_now);
  M5.Display.printf("rz = %f \t\n", rover_z_now);
  
  // 目標位置の表示
  M5.Display.printf("gx= %f \t", rover_x_goal);
  M5.Display.printf("gy= %f \t", rover_y_goal);
  M5.Display.printf("gz = %f \t\n", rover_z_goal);
  
  // 現在の目標ポイントインデックス
  M5.Display.printf("cnt = %d \t\n", rover_goal);
  
  // 計算された速度指令値
  M5.Display.printf("tx= %f \t", rover_x);
  M5.Display.printf("ty= %f \t", rover_y);
  M5.Display.printf("tz = %f \t\n", rover_z);

  // センサーの位置情報（mm単位）
  M5.Display.printf("px= %+2.2f mm/s \t", pos.x * 25.4);
  M5.Display.printf("py= %+2.2f mm/s \t", pos.y * 25.4);
  M5.Display.printf("pw = %+2.2f deg/s \t\n", pos.h);
  
  // 目標までの距離
  M5.Display.printf("dis= %+2.2f \t", dis);
  
  // 現在のモード表示
  M5.Display.printf("rover_flag= %d \t", rover_flag);

  // モード1なら「stop」、それ以外なら「run」と表示（表示が逆？）
  if (rover_flag == 1)
    M5.Display.printf("stop \t\n");
  else
    M5.Display.printf("run \t\n");

  M5.Display.endWrite();

  // シリアルモニタにもセンサー値を出力
  Serial.printf("x = %+.2f mm \t", pos.x * 25.4);
  Serial.printf("y = %+.2f mm \t", pos.y * 25.4);
  Serial.printf("th= %+.2f deg \t", pos.h);
  Serial.println("");

  // 10ミリ秒待機
  delay(10);
}