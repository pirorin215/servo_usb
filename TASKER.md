# 照明 Tasker 連動 (Wi-Fiエッジ) — 設定手順

2026-09-11 復活。前回の LightEdge アプリ + light-api 方式 (9/11凍結・削除) を、
Tasker + Apache配信エンドポイント方式に置き換えた最小構成。
(同日夜の一時実装 light_server.py スタンドアロンサーバ 8879 は廃止済み)

```
スマホ Tasker ──(Wi-Fi接続/切断を検知)──▶ HTTP Request
    https://192.168.0.222/light-ir/?cmd=on    (自宅LAN内・到達すれば自宅確定)
    https://10.8.0.4/light-ir/?cmd=off        (WireGuard VPN経由・切断直後用)
        │ Apache 443 (mkcert portal.pem・ポータルと同一証明書)
~/www/light-ir/index.php  … _www 権限で直接スクリプト実行
        │ exec /bin/bash            (デバイス /dev/cu.usbmodemHIDEF1 が 666 のため
        │                             _wwwから直接シリアル書込可・キュー不要)
light_on.sh / light_off.sh … IR LED → 照明 (東芝NEC・ディスクリートコード)
display_on.sh / display_off.sh … サーボ → ディスプレイのオルタネートスイッチ
                                  (ONマクロはウェイクキー送出込み・2026-09-11追加)
```

## Mac側セットアップ (済み・参照)

- `~/www/light-ir/index.php` … 受付エンドポイント (独立Git・Apache DocumentRoot配下)
  - `GET /light-ir/?cmd=on|off|display_on|display_off` → 対応スクリプト実行
  - `GET /light-ir/` (cmdなし) → 死活確認 `{"ok":true}` / 不明cmdは404
- **認証なし** (LAN/VPN内のみ到達・家の「ローカルサービス無認証」方針準拠)
- ログ: `~/.cache/light-ir/events.log` (全リクエスト記録・JST)
- 常駐管理なし — Apache (brew services) に乗るので個別の死活/respawnは不要。
  一時停止したい時は `~/www/light-ir/` をリネーム等
- 2026-09-11 E2E合格済み: health / 実IR送信 rc=0 (_www直接書込) / 404

## 方向 (ON/OFFの割当)

**2026-09-11 実機設定で接続=ON / 切断=OFF に確定**
(スマホのWi-Fi OFF→照明消灯・ON→点灯を実証済み)。逆にしたくなった時は
Enter/Exit の on/off を入れ替えるだけでよい (Mac側は共通)。

## Tasker設定手順 (スマホ)

1. **Taskerを有効化**: 初回起動のガイドに沿って「すべての機能」を許可
2. **電池最適化を解除**: 端末設定 → アプリ → Tasker → 電池 → 「制限なし」
   (解除しないと画面OFF中のWi-Fiイベントが遅延/欠落する)
3. **Profile作成**: 「+」→ 状態(State) → ネット(Net) → **Wi-Fi Connected**
   - SSID / MAC / IP は**空欄のまま** (自宅判定は下記設計メモの通り)
4. **Enter Task (帰宅)**: アクション追加 → **HTTP Request** ×2
   1. URL: `https://192.168.0.222/light-ir/?cmd=display_on` (ディスプレイは
      起動に約10秒かかるので**先に**飛ばす)
   2. URL: `https://192.168.0.222/light-ir/?cmd=on` (照明)
   - Method: `GET`・Timeout: 10 (両方とも)
5. **Exit Task (外出)**: 戻るボタンで「Exit Taskを作成」
   1. **Wait** 15秒 (瞬断対策: 15秒以内に再接続するとTaskerがExit Taskを中断する)
   2. **HTTP Request** `https://10.8.0.4/light-ir/?cmd=off` (VPN経由)
   3. **HTTP Request** `https://192.168.0.222/light-ir/?cmd=off` (保険)
   4. **HTTP Request** `https://192.168.0.222/light-ir/?cmd=display_off`
      — Wi-Fi切断直後はWi-Fi経路が無いので、屆くのはモバイルデータ+WireGuard
      トンネルが上がっている場合のみ (本運用はVPN常時接続で実証済み)。
      失敗してもタスクは止まらない
6. **テスト**: クイック設定でWi-FiをOFF→ON → `tail -f ~/.cache/light-ir/events.log`
   に `client=<スマホIP>` が増える。照明IRはディスクリートコードなので
   点灯中のON送信は無害 (状態不变)

## 設計メモ

- **SSIDフィルタ不要の理由**: Android 13+ は SSID 取得に位置情報権限
  「常時許可」+ 端末の位置情報ON が必要 (前回 LightEdge が嵌った
  `<unknown ssid>` 問題)。本方式は「エンドポイントに屆く=自宅LAN内」という
  到達性で自宅判定するため位置情報に触れない。
- **VPN常時接続時の注意**: 外出先のWi-Fiに繋いでもトンネル経由で
  192.168.0.222 に屆いてしまう (誤発火)。防ぎたい場合は Profile に
  SSID条件を足す (その際は位置情報「常時許可」+位置情報ON が必要)。
- **証明書**: ポータルと同一の mkcert (SANに 192.168.0.222/.223, 10.8.0.4)。
  ポータルをスマホで開けた実績があればそのまま通る。
## バックアップ・機種変更時の引っ越し

Taskerの設定はプロファイル/タスク/変数ごとエクスポートでき、
Google Drive等に保存すれば新端末で復元できる (Tasker設定内の
バックアップ/エクスポート機能。Taskerは /sdcard/Tasker/ 配下にも
自動バックアップを書き出す)。

ただし**権限系はバックアップに含まれず、新端末で毎回手動で許可が必要**:

1. Taskerを有効化 (初回ガイドで「すべての機能」を許可)
2. 電池の「制限なし」 (画面OFF中のWi-Fiイベント用)
3. 位置情報「常に許可」 (SSID条件・Wi-Fi Nearを使う場合)
4. WireGuardの常時接続 (切断側をVPN経由で届かせる場合)
5. 動作確認: Wi-Fi切替で `~/.cache/light-ir/events.log` に
   `client=<スマホIP>` が出るか

**自動退避案 (未設定)**: スマホのSyncthingで /sdcard/Tasker/ を共有し
Mac側へ自動同期すれば「設定を変えたら自動で保護される」状態になる。
手動のGoogle Drive保存は「忘れる」リスクが残る分だけ劣る。

Mac側の復元はこのTASKER.mdと ~/www/light-ir/ (Git)・servo_usb/
(light_on/off.sh・display_on/off.sh・ファーム) があれば足りる。

## もっと早く反応させたい (Wi-Fi Near が反応しない時)

Taskerの **Wi-Fi Near**(近くのWi-Fi)はスキャン通知が出るのに反応しない場合、
ほぼ確実に**位置情報ゲート**が原因 (Android 10+ はスキャン結果の取得自体に
位置情報を要求する・SSID読み取りと同じ仕様):

1. 端末設定 → 位置情報 → **ON**
2. Tasker の権限 → 位置情報 → **「常に許可」**
3. (推奨) 開発者オプション → **「Wi-Fiスキャンのスロットル」を無効化** —
   画面OFF中のスキャンが「4回/2分」制限から解放され、検知も
   Wi-Fi自動接続も速くなる
4. Wi-Fi Near の設定で Minimum Signal Strength を最初は緩めに
   (誤検知したら強める)

正直言うと、Wi-Fi Near そのものが「接続より早く発火」する幅は小さい
(同じスキャン結果から両方起こるため・接続完了までの数秒分だけ早い)。
ただしスキャン頻度が上がることで**画面OFFのスマホが家のAPを見つけるまでの
時間自体が縮む**ので、体感速度はかなり改善しうる。ディスプレイの約10秒の
起動時間は「Enter Taskで display_on を先に飛ばす」で吸収するのが本命。

## 切断エッジの確実化

VPN常時接続運用なら切断側も屆く (2026-09-11 実機実証済み)。そうでない環境で
確実にするなら「Mac側でスマホのDHCP予約IPを定期監視し、数分間不通ならOFF」
方式が確実 (9/11撤去の旧light-api設計の「切断5分猶予auto_off」が同型)。

## 照明以外を連動させる

`index.php` の `$scripts` 連想配列に `'コマンド名' => 'スクリプトパス'` を
追記すると `?cmd=コマンド名` で叩けるようになる。Tasker側は Profile/Action
を増やすだけ。**ディスプレイ電源が第1号** (display_on/display_off —
2026-09-11追加・ファームに SEND_ON/SEND_OFF を追加実装)。
