'use client';

import { createTheme } from '@mui/material/styles';

export const theme = createTheme({
  palette: {
    primary: {
      main: '#4AACAA',       // メインカラー（トップバー、選択枠など）
      contrastText: '#ffffff',
    },
    secondary: {
      main: '#f50057',       // 移動アクションなどの強調ボタン用
    },
    divider: '#e0e0e0',      // 区切り線や枠線の色
    background: {
      default: '#f5f5f5',    // 全体の背景
      paper: '#ffffff',      // カードなどの背景
    },
  },
  components: {
    MuiButton: {
      styleOverrides: {
        root: {
          fontWeight: 'bold',
        },
      },
    },
  },
});