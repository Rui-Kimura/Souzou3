"use client";

import { useEffect, useState } from "react";
import {
  Card,
  Box,
  Typography,
  Container,
  Stack,
  Paper
} from "@mui/material";

export default function Page() {
  const TILE_SIZE = 20;
  const TILE_WIDTH = 50; // 50mm

  const [mapdata, setMapdata] = useState<[]>([]);
  const [position_x, setPositionX] = useState<number>(0);
  const [position_y, setPositionY] = useState<number>(0);
  const [position_angle, setPositionAngle] = useState<number>(0);

  const [target_x, setTargetX] = useState<number>(0);
  const [target_y, setTargetY] = useState<number>(0);
  const [target_angle, setTargetAngle] = useState<number>(0);

  const fetch_mapdata = async () => {
    try {
      const res = await fetch("/api/local/mapdata");
      const data = await res.json();
      setMapdata(data.mapdata);
    } catch (error) {
      console.error("Failed to fetch map data:", error);
    }
  };

  const fetch_position = async () => {
    try {
      const res = await fetch("/api/local/position");
      const data = await res.json();
      setPositionX(data.x);
      setPositionY(data.y);
      setPositionAngle(data.angle);
    } catch (error) {
      console.error("Failed to fetch position data:", error);
    }
  };

  const fetch_target_point = async () => {
    try {
      const res = await fetch("/api/local/target_point");
      const data = await res.json();
      setTargetX(data.x);
      setTargetY(data.y);
      setTargetAngle(data.angle);
    } catch (error) {
      console.error("Failed to fetch target point data:", error);
    }
  };

  const interval_function = async () => {
    await fetch_position();
    await fetch_target_point();
  }


  useEffect(() => {
    fetch_mapdata();
    const intervalId = setInterval(interval_function, 200);
    return () => clearInterval(intervalId);
  }, []);

  // 修正: Math.roundを削除し、正確なピクセル位置を計算 (滑らかな移動のため)
  const playerPixelX = (position_x / TILE_WIDTH) * TILE_SIZE;
  const playerPixelY = (position_y / TILE_WIDTH) * TILE_SIZE;
  
  // ターゲットはグリッド座標(整数)なのでそのまま
  const targetPixelX = target_x * TILE_SIZE;
  const targetPixelY = target_y * TILE_SIZE;

  const iconHalfWidth = TILE_SIZE / 3;
  const iconHeight = TILE_SIZE * 0.8;

  return (
    <Box
      sx={{
        minHeight: "100vh",
        bgcolor: "#f5f6fa",
        py: 4,
        display: "flex",
        flexDirection: "column",
        alignItems: "center",
      }}
    >
      <Container maxWidth="md" sx={{ display: 'flex', flexDirection: 'column', alignItems: 'center' }}>

        <Typography variant="h5" component="h1" sx={{ mb: 3, color: "#333", fontWeight: 'bold' }}>
          地図
        </Typography>

        <Card elevation={3} sx={{ overflow: "hidden", maxWidth: "100%" }}>
          <Box
            sx={{
              position: "relative",
              padding: "10px",
              overflow: "auto",
              bgcolor: "#fff",
              display: "flex",
              flexDirection: "column",
            }}
          >
            <Box
              sx={{
                position: "absolute",
                width: 0,
                height: 0,
                borderLeft: `${iconHalfWidth}px solid transparent`,
                borderRight: `${iconHalfWidth}px solid transparent`,
                borderBottom: `${iconHeight}px solid #e74c3c`,
                // 修正: playerPixelX/Yは既に中心座標相当(正確な位置)を指しているため、+10のオフセットを削除
                left: playerPixelX - iconHalfWidth,
                top: playerPixelY - (TILE_SIZE / 3 * 1.5),
                transform: `rotate(${position_angle + 90}deg)`,
                zIndex: 10,
                transition: "all 0.3s ease-out",
              }}
            />
            {/*ターゲットアイコン*/}
            <Box
              sx={{
                position: "absolute",
                width: 0,
                height: 0,
                borderLeft: `${iconHalfWidth}px solid transparent`,
                borderRight: `${iconHalfWidth}px solid transparent`,
                borderBottom: `${iconHeight}px solid #4400ffff`,
                // ターゲットはグリッドの左上座標なので、中心に合わせるために +10 する
                left: targetPixelX + 10 - iconHalfWidth,
                top: targetPixelY + 10 - (TILE_SIZE / 3 * 1.5),
                transform: `rotate(${target_angle + 90}deg)`,
                zIndex: 10,
              }}
            />
            {mapdata.map((rowString: string, rowIndex) => (
              <Box key={rowIndex} sx={{ display: "flex" }}>
                {rowString.split("").map((cellChar, colIndex) => (
                  <Box
                    key={`${rowIndex}-${colIndex}`}
                    sx={{
                      width: TILE_SIZE,
                      height: TILE_SIZE,
                      boxSizing: "border-box",
                      bgcolor: cellChar === "1" ? "#2c3e50" : "#ecf0f1",
                      border:
                        cellChar === "0"
                          ? "1px solid #bdc3c7"
                          : "1px solid #34495e",
                    }}
                  />
                ))}
              </Box>
            ))}
          </Box>
        </Card>

        <Paper
          variant="outlined"
          sx={{ mt: 2, p: 1, px: 2, bgcolor: 'rgba(255,255,255,0.6)', borderColor: 'transparent' }}
        >
          <Typography variant="body2" color="text.secondary">
            現在位置: <strong>X={position_x}</strong>, <strong>Y={position_y}</strong>, 角度={position_angle}°
          </Typography>
        </Paper>

        <Stack direction="row" spacing={3} sx={{ mt: 3 }}>
          <Stack direction="row" alignItems="center" spacing={1}>
            <Box
              sx={{
                width: 16,
                height: 16,
                bgcolor: "#2c3e50",
                borderRadius: 0.5,
              }}
            />
            <Typography variant="caption" color="text.secondary">
              壁 (1)
            </Typography>
          </Stack>

          <Stack direction="row" alignItems="center" spacing={1}>
            <Box
              sx={{
                width: 16,
                height: 16,
                bgcolor: "#ecf0f1",
                border: "1px solid #bdc3c7",
                borderRadius: 0.5,
              }}
            />
            <Typography variant="caption" color="text.secondary">
              通路 (0)
            </Typography>
          </Stack>

          <Stack direction="row" alignItems="center" spacing={1}>
            <Box
              sx={{
                width: 0,
                height: 0,
                borderLeft: "4px solid transparent",
                borderRight: "4px solid transparent",
                borderBottom: `${(TILE_SIZE / 3) * 1.5}px solid #e74c3c`,
              }}
            />
            <Typography variant="caption" color="text.secondary">
              現在位置
            </Typography>
          </Stack>
        </Stack>

      </Container>
    </Box>
  );
}