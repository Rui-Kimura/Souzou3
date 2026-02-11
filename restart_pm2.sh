#!/bin/bash

while true; do
    echo "[INFO] pm2 restart all を実行しています..."
    
    # コマンドを実行
    pm2 restart all
    
    # 直前のコマンドの終了ステータスを確認 ($? が 0 なら成功)
    if [ $? -eq 0 ]; then
        echo "[SUCCESS] エラーなく再起動が完了しました。"
        break
    else
        echo "[ERROR] エラーが発生しました。5秒後に再試行します..."
        sleep 5
    fi
done