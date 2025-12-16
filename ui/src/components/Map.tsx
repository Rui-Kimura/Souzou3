"use client";

import { useEffect, useState } from "react";
import {
  Card,
  Box,
  Typography,
  Container,
  Stack,
  Paper,
  Popover,
  Button,
  Dialog,
  DialogTitle,
  DialogContent,
  TextField,
  DialogActions,
} from "@mui/material";

interface Point {
  name: string;
  x: number;
  y: number;
  angle: number;
}

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

  // --- 追加: タップ操作とダイアログ用State ---
  const [anchorEl, setAnchorEl] = useState<HTMLElement | null>(null);
  const [selectedGrid, setSelectedGrid] = useState<{r: number, c: number} | null>(null);
  const [isNameDialogOpen, setNameDialogOpen] = useState(false);
  const [pointName, setPointName] = useState("");

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

  // --- 追加: タイルクリック時の処理 ---
  const handleTileClick = (event: React.MouseEvent<HTMLElement>, rowIndex: number, colIndex: number, isWall: boolean, isCost: boolean) => {
    // 壁やコスト領域なら何もしない
    if (isWall || isCost) {
      return;
    }
    // ポップオーバーを表示
    setAnchorEl(event.currentTarget);
    setSelectedGrid({ r: rowIndex, c: colIndex });
  };

  // --- 追加: ポップオーバーを閉じる ---
  const handlePopoverClose = () => {
    setAnchorEl(null);
    setSelectedGrid(null);
  };

  // --- 追加: 「目標地点の追加」ボタンクリック ---
  const handleAddButtonClick = () => {
    setAnchorEl(null); // ポップオーバーを閉じる
    setPointName("");  // 名前入力をリセット
    setNameDialogOpen(true); // ダイアログを開く
  };

  // --- 追加: 保存処理 ---
  const handleSavePoint = async () => {
    if (!selectedGrid) return;

    // グリッド座標から物理座標(mm)へ変換
    // マスの中心座標にするため + TILE_WIDTH / 2 (25mm) を加算
    const physicalX = selectedGrid.c * TILE_WIDTH + TILE_WIDTH / 2.0;
    const physicalY = selectedGrid.r * TILE_WIDTH + TILE_WIDTH / 2.0;

    const payload: Point = {
      name: pointName || Date.now().toString(), // 空ならデフォルト名
      x: physicalX,
      y: physicalY,
      angle: 0.0, // タップ指定なので角度は一旦0度とします
    };

    try {
      const res = await fetch("/api/local/add_target_point", {
        method: "POST",
        headers: {
          "Content-Type": "application/json",
        },
        body: JSON.stringify(payload),
      });

      if (res.ok) {
        console.log("Saved successfully");
      } else {
        console.error("Failed to save point");
      }
    } catch (error) {
      console.error("Error saving point:", error);
    }

    setNameDialogOpen(false); // ダイアログを閉じる
    setSelectedGrid(null);
  };

  const playerPixelX = (position_x / TILE_WIDTH) * TILE_SIZE;
  const playerPixelY = (position_y / TILE_WIDTH) * TILE_SIZE;

  const targetPixelX = (target_x / TILE_WIDTH) * TILE_SIZE;
  const targetPixelY = (target_y / TILE_WIDTH) * TILE_SIZE;

  const iconHalfWidth = TILE_SIZE * 4.5;
  const iconWidth = iconHalfWidth * 2;
  const iconHeight = TILE_SIZE * 0.8 * 4.5;

  const openPopover = Boolean(anchorEl);

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
           {/* プレイヤーアイコン (赤) */}
            <Box
              sx={{
                position: "absolute",
                width: iconWidth,
                height: iconHeight,
                bgcolor: "#e74c3c",
                clipPath: "polygon(50% 0%, 0% 100%, 100% 100%)",
                
                left: playerPixelX,
                top: playerPixelY,
                
                transform: `translate(-50%, -50%) rotate(${position_angle}deg)`,
                transformOrigin: "center center", 
                zIndex: 10,
                transition: "all 0.3s ease-out",
                pointerEvents: "none", // アイコンがクリックを邪魔しないように
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
                pointerEvents: "none",
              }}
            />
            
            {/* マップ描画 */}
            {mapdata.map((rowString: string, rowIndex) => (
              <Box key={rowIndex} sx={{ display: "flex" }}>
                {rowString.split("").map((cellChar, colIndex) => {
                  const isCost = costmapData[rowIndex]?.[colIndex] === "1";
                  const isWall = cellChar === "1";
                  
                  return (
                    <Box
                      key={`${rowIndex}-${colIndex}`}
                      // --- 追加: クリックイベント ---
                      onClick={(e) => handleTileClick(e, rowIndex, colIndex, isWall, isCost)}
                      sx={{
                        width: TILE_SIZE,
                        height: TILE_SIZE,
                        boxSizing: "border-box",
                        bgcolor: isWall ? "#2c3e50" : "#ecf0f1",
                        border:
                          cellChar === "0"
                            ? "1px solid #bdc3c7"
                            : "1px solid #34495e",
                        backgroundImage: isCost 
                          ? "repeating-linear-gradient(45deg, rgba(255, 0, 0, 0.15) 0, rgba(255, 0, 0, 0.15) 2px, transparent 2px, transparent 6px)" 
                          : "none",
                        // --- 追加: クリック可能な場所はカーソルを変える ---
                        cursor: (!isWall && !isCost) ? "pointer" : "default",
                        "&:hover": (!isWall && !isCost) ? {
                          bgcolor: "#d6eaf8" // ホバー時のハイライト
                        } : {}
                      }}
                    />
                  );
                })}
              </Box>
            ))}
          </Box>
        </Card>

        {/* --- 追加: ポップオーバー (目標地点の追加ボタン) --- */}
        <Popover
          open={openPopover}
          anchorEl={anchorEl}
          onClose={handlePopoverClose}
          anchorOrigin={{
            vertical: 'bottom',
            horizontal: 'center',
          }}
          transformOrigin={{
            vertical: 'top',
            horizontal: 'center',
          }}
        >
          <Box sx={{ p: 1 }}>
            <Button size="small" variant="contained" onClick={handleAddButtonClick}>
              目標地点の追加
            </Button>
          </Box>
        </Popover>

        {/* --- 追加: 名前入力ダイアログ --- */}
        <Dialog open={isNameDialogOpen} onClose={() => setNameDialogOpen(false)}>
          <DialogTitle>地点名の入力</DialogTitle>
          <DialogContent>
            <TextField
              autoFocus
              margin="dense"
              label="地点名"
              type="text"
              fullWidth
              variant="standard"
              value={pointName}
              onChange={(e) => setPointName(e.target.value)}
            />
          </DialogContent>
          <DialogActions>
            <Button onClick={() => setNameDialogOpen(false)}>キャンセル</Button>
            <Button onClick={handleSavePoint} variant="contained">保存</Button>
          </DialogActions>
        </Dialog>

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