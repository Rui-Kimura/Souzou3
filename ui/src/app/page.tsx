"use client"
import { Box, Button, Paper, Typography } from "@mui/material";
import { useRouter } from 'next/navigation'

export default function Home() {
  const router = useRouter();
  //URLを引数とするクリックイベントハンドラ
  const handleClick = (url: string) => {
    router.push(url);
  }
  return (
    <Box sx={{ display: "flex", flexDirection: "column", alignItems: "center" }}>
      <Paper sx={{ width: "80%", p: 2 }}>
        <Box>
          <Typography variant="h6"sx={{p:2}}>メニュー</Typography>
          <Box sx={{ display: "flex",flexWrap: "wrap",gap:2}}>
            <Button onClick={() => handleClick('/automation')} variant="contained" color="primary" sx={{ width: "250px" }} >
              自動移動
            </Button>
            <Button onClick={() => handleClick('/controller')} variant="contained" color="primary" sx={{ width: "250px" }} >
              手動操作
            </Button>
            <Button onClick={() => handleClick('/map')} variant="contained" color="primary" sx={{ width: "250px" }} >
              マップ表示
            </Button>
            <Button onClick={() => handleClick('/')} variant="contained" color="primary" sx={{ width: "250px" }} >
              デモモード
            </Button>
            
          </Box>

        </Box>
      </Paper>

    </Box>
  );
}
