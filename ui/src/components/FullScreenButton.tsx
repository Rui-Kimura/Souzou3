import React, { useState, useEffect } from 'react';
import { IconButton, Tooltip } from '@mui/material';
import FullscreenIcon from '@mui/icons-material/Fullscreen';
import FullscreenExitIcon from '@mui/icons-material/FullscreenExit';

const FullscreenButton = () => {
  const [isFullscreen, setIsFullscreen] = useState(false);

  // フルスクリーンの切り替え処理
  const toggleFullscreen = () => {
    if (!document.fullscreenElement) {
      // フルスクリーン化 (ドキュメント全体)
      document.documentElement.requestFullscreen().catch((e) => {
        console.error(`Error attempting to enable fullscreen: ${e.message}`);
      });
    } else {
      // 解除
      if (document.exitFullscreen) {
        document.exitFullscreen();
      }
    }
  };

  // ブラウザのイベントを監視してstateを更新
  // (Escキーなどで解除された場合に対応するため)
  useEffect(() => {
    const handleFullscreenChange = () => {
      setIsFullscreen(!!document.fullscreenElement);
    };

    document.addEventListener('fullscreenchange', handleFullscreenChange);

    return () => {
      document.removeEventListener('fullscreenchange', handleFullscreenChange);
    };
  }, []);

  return (
    <Tooltip title={isFullscreen ? "フルスクリーン解除" : "フルスクリーン"}>
      <IconButton
        onClick={toggleFullscreen}
        aria-label="toggle fullscreen"
        size="large"
        sx={{
          position: 'fixed', // 画面上の固定位置
          bottom: 20,        // 下から20px
          right: 20,         // 右から20px
          zIndex: 1000,      // 他の要素より前面に表示
          backgroundColor: 'rgba(255, 255, 255, 0.8)', // 背景色（見やすくするため）
          '&:hover': {
            backgroundColor: 'rgba(255, 255, 255, 1)',
          },
          boxShadow: 3,      // 影をつけて浮かせる
        }}
      >
        {isFullscreen ? <FullscreenExitIcon fontSize="inherit" /> : <FullscreenIcon fontSize="inherit" />}
      </IconButton>
    </Tooltip>
  );
};

export default FullscreenButton;