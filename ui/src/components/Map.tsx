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

export default function Map() {
  const TILE_SIZE = 20;
  const TILE_WIDTH = 50; // 50mm

  const [mapdata, setMapdata] = useState<string[]>([]);
  const [costmapData, setCostmapData] = useState<string[]>([]);
  
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

  const fetch_costmapdata = async () => {
    try {
      const res = await fetch("/api/local/costmapdata");
      const data = await res.json();
      setCostmapData(data.costmapdata || []);
    } catch (error) {
      console.error("Failed to fetch costmap data:", error);
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
  };

  useEffect(() => {
    fetch_mapdata();
    fetch_costmapdata();
    const intervalId = setInterval(interval_function, 200);
    return () => clearInterval(intervalId);
  }, []);

  const playerPixelX = (position_x / TILE_WIDTH) * TILE_SIZE;
  const playerPixelY = (position_y / TILE_WIDTH) * TILE_SIZE;

  const targetPixelX = (target_x / TILE_WIDTH) * TILE_SIZE;
  const targetPixelY = (target_y / TILE_WIDTH) * TILE_SIZE;

  // サイズ計算
  const iconHalfWidth = TILE_SIZE * 4.5;
  const iconWidth = iconHalfWidth * 2; // 全幅
  const iconHeight = TILE_SIZE * 0.8 * 4.5;

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
           {/* 【修正ポイント】
              borderではなく、四角いBoxをclip-pathで三角形に切り抜きます。
              これにより、要素の中心(center)が「三角形の重心付近」になり、
              回転させても軸がブレなくなります。
           */}
           
           {/* プレイヤーアイコン (赤) */}
            <Box
              sx={{
                position: "absolute",
                width: iconWidth,
                height: iconHeight,
                bgcolor: "#e74c3c", // 色はここで指定
                // 上向きの三角形に切り抜く
                clipPath: "polygon(50% 0%, 0% 100%, 100% 100%)",
                
                left: playerPixelX,
                top: playerPixelY,
                
                // 中心を基準に配置し、回転させる
                transform: `translate(-50%, -50%) rotate(${position_angle}deg)`,
                transformOrigin: "center center", 
                zIndex: 10,
                transition: "all 0.3s ease-out",
              }}
            />

            {/* ターゲットアイコン (青) */}
            <Box
              sx={{
                position: "absolute",
                width: iconWidth,
                height: iconHeight,
                bgcolor: "#4400ffff",
                clipPath: "polygon(50% 0%, 0% 100%, 100% 100%)",
                
                left: targetPixelX, 
                top: targetPixelY,
                
                transform: `translate(-50%, -50%) rotate(${target_angle}deg)`,
                transformOrigin: "center center",
                zIndex: 10,
              }}
            />
            
            {/* マップ描画 */}
            {mapdata.map((rowString: string, rowIndex) => (
              <Box key={rowIndex} sx={{ display: "flex" }}>
                {rowString.split("").map((cellChar, colIndex) => {
                  const isCost = costmapData[rowIndex]?.[colIndex] === "1";
                  return (
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
                        backgroundImage: isCost 
                          ? "repeating-linear-gradient(45deg, rgba(255, 0, 0, 0.15) 0, rgba(255, 0, 0, 0.15) 2px, transparent 2px, transparent 6px)" 
                          : "none",
                      }}
                    />
                  );
                })}
              </Box>
            ))}
          </Box>
        </Card>

        <Paper
          variant="outlined"
          sx={{ mt: 2, p: 1, px: 2, bgcolor: 'rgba(255,255,255,0.6)', borderColor: 'transparent' }}
        >
          <Typography variant="body2" color="text.secondary">
            現在位置: <strong>X={position_x.toFixed(0)}</strong>, <strong>Y={position_y.toFixed(0)}</strong>, 角度={position_angle.toFixed(0)}°
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
                width: 16,
                height: 16,
                bgcolor: "#ecf0f1",
                border: "1px solid #bdc3c7",
                borderRadius: 0.5,
                backgroundImage: "repeating-linear-gradient(45deg, rgba(255, 0, 0, 0.25) 0, rgba(255, 0, 0, 0.25) 2px, transparent 2px, transparent 6px)"
              }}
            />
            <Typography variant="caption" color="text.secondary">
              コスト領域
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